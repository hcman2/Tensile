################################################################################
#
# Copyright (C) 2016-2022 Advanced Micro Devices, Inc. All rights reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
################################################################################

include(CMakeParseArguments)

if(NOT DEFINED Tensile_ROOT)
    # Compute the installation prefix relative to this file.
    get_filename_component(Tensile_PREFIX "${CMAKE_CURRENT_LIST_FILE}" PATH)
    get_filename_component(Tensile_PREFIX "${Tensile_PREFIX}" PATH)

    if (WIN32)
        execute_process(COMMAND "${Tensile_PREFIX}/bin/TensileGetPath.exe" OUTPUT_VARIABLE Tensile_ROOT)
    else()
        execute_process(COMMAND "${Tensile_PREFIX}/bin/TensileGetPath" OUTPUT_VARIABLE Tensile_ROOT)
    endif()
endif()
list(APPEND CMAKE_MODULE_PATH "${Tensile_ROOT}/Source/cmake/")
list(APPEND CMAKE_MODULE_PATH "${Tensile_ROOT}/Source/")

if("HIP" IN_LIST Tensile_FIND_COMPONENTS)
    set(TENSILE_USE_HIP ON CACHE BOOL "Use HIP")
else()
    set(TENSILE_USE_HIP OFF CACHE BOOL "Use HIP")
endif()

if("LLVM" IN_LIST Tensile_FIND_COMPONENTS)
    set(TENSILE_USE_LLVM ON CACHE BOOL "Use LLVM")
else()
    set(TENSILE_USE_LLVM OFF CACHE BOOL "Use LLVM")
endif()

if("Client" IN_LIST Tensile_FIND_COMPONENTS)
    if(TENSILE_USE_HIP AND TENSILE_USE_LLVM)
        set(TENSILE_BUILD_CLIENT ON CACHE BOOL "Build Client")
    elseif(Tensile_FIND_REQUIRED_Client)
        message("Tensile client requires both Hip and LLVM.")
        set(Tensile_FOUND false)
    else()
        set(TENSILE_BUILD_CLIENT OFF CACHE BOOL "Build Client")
    endif()
else()
    set(TENSILE_BUILD_CLIENT OFF CACHE BOOL "Build Client")
endif()

if("STATIC_ONLY" IN_LIST Tensile_FIND_COMPONENTS)
    set(TENSILE_STATIC_ONLY ON CACHE BOOL "Disable exporting symbols from shared library.")
else()
    set(TENSILE_STATIC_ONLY OFF CACHE BOOL "Disable exporting symbols from shared library.")
endif()

add_subdirectory("${Tensile_ROOT}/Source" "Tensile")
include("${Tensile_ROOT}/Source/TensileCreateLibrary.cmake")

# Target is created for copying dependencies
function(TensileCreateCopyTarget
    Target_NAME
    Tensile_OBJECTS_TO_COPY
    Dest_PATH
    )

    file(MAKE_DIRECTORY "${Dest_PATH}")
    add_custom_target(
        ${Target_NAME} ALL
        COMMENT "${Target_NAME}: Copying tensile objects to ${Dest_PATH}"
        DEPENDS ${Tensile_OBJECTS_TO_COPY}
    )
    foreach(OBJECT_TO_COPY ${Tensile_OBJECTS_TO_COPY})
        add_custom_command(
            TARGET ${Target_NAME} PRE_BUILD
            COMMAND_EXPAND_LISTS
            COMMAND ${CMAKE_COMMAND} -E copy_if_different ${OBJECT_TO_COPY} ${Dest_PATH}
            DEPENDS ${OBJECT_TO_COPY}
        )
    endforeach()
endfunction()

# Output target: ${Tensile_VAR_PREFIX}_LIBRARY_TARGET. Ensures that the libs get built in Tensile_OUTPUT_PATH/library.
# Output symbol: ${Tensile_VAR_PREFIX}_ALL_FILES. List of full paths of all expected library files in manifest.
function(TensileCreateLibraryFiles
         Tensile_LOGIC_PATH
         Tensile_OUTPUT_PATH
         )

  # Boolean options
  set(options
       MERGE_FILES
       NO_MERGE_FILES
       SHORT_FILE_NAMES
       PRINT_DEBUG
       GENERATE_PACKAGE
       SEPARATE_ARCHITECTURES
       LAZY_LIBRARY_LOADING
       )

  # Single value settings
  set(oneValueArgs
       CODE_OBJECT_VERSION
       COMPILER
       COMPILER_PATH
       EMBED_KEY
       EMBED_LIBRARY
       LIBRARY_FORMAT
       TENSILE_ROOT
       VAR_PREFIX
       CPU_THREADS
       )

  # Multi value settings
  set(multiValueArgs
       ARCHITECTURE
       )

  cmake_parse_arguments(Tensile "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  if(Tensile_UNPARSED_ARGUMENTS)
    message(WARNING "Unrecognized arguments: ${Tensile_UNPARSED_ARGUMENTS}")
  endif()
  if(Tensile_KEYWORDS_MISSING_VALUES)
    message(WARNING "Malformed arguments: ${Tensile_KEYWORDS_MISSING_VALUES}")
  endif()

  # Parse incoming options
  if(Tensile_TENSILE_ROOT)
    set(Script "${Tensile_TENSILE_ROOT}/bin/TensileCreateLibrary")
  else()
    set(Script "${Tensile_ROOT}/bin/TensileCreateLibrary")
  endif()

  message(STATUS "Tensile script: ${Script}")

  # Older NO_MERGE_FILES flag overrides MERGE_FILES option.
  if(Tensile_NO_MERGE_FILES)
    set(Tensile_MERGE_FILES FALSE)
  endif()

  if(Tensile_MERGE_FILES)
    set(Options ${Options} "--merge-files")
  else()
    set(Options ${Options} "--no-merge-files")
  endif()

  if(Tensile_SEPARATE_ARCHITECTURES)
    set(Options ${Options} "--separate-architectures")
  endif()

  if(Tensile_LAZY_LIBRARY_LOADING)
    set(Options ${Options} "--lazy-library-loading")
  endif()

  if(Tensile_GENERATE_PACKAGE)
    set(Options ${Options} "--package-library")
  endif()

  if(Tensile_SHORT_FILE_NAMES)
    set(Options ${Options} "--short-file-names")
  else()
    set(Options ${Options} "--no-short-file-names")
  endif()

  if(Tensile_PRINT_DEBUG)
    set(Options ${Options} "--library-print-debug")
  else()
    set(Options ${Options} "--no-library-print-debug")
  endif()

  if(Tensile_EMBED_LIBRARY)
    set(Options ${Options} "--embed-library=${Tensile_EMBED_LIBRARY}")
  endif()

  if(Tensile_EMBED_KEY)
    set(Options ${Options} "--embed-library-key=${Tensile_EMBED_KEY}")
  endif()

  if(Tensile_CODE_OBJECT_VERSION)
    set(Options ${Options} "--code-object-version=${Tensile_CODE_OBJECT_VERSION}")
  endif()

  if(Tensile_COMPILER)
    set(Options ${Options} "--cxx-compiler=${Tensile_COMPILER}")
  endif()

  if(Tensile_COMPILER_PATH)
    set(Options ${Options} "--cmake-cxx-compiler=${Tensile_COMPILER_PATH}")
  endif()

  if(Tensile_CPU_THREADS)
    set(Options ${Options} "--jobs=${Tensile_CPU_THREADS}")
  endif()

  if(Tensile_LIBRARY_FORMAT)
    set(Options ${Options} "--library-format=${Tensile_LIBRARY_FORMAT}")
    if(Tensile_LIBRARY_FORMAT MATCHES "yaml")
        target_compile_definitions( TensileHost PUBLIC -DTENSILE_YAML=1)
    endif()
  endif()

  if(Tensile_ARCHITECTURE)
    string (REPLACE ";" "_" archString "${Tensile_ARCHITECTURE}")
    # uses _ separator to avoid cmake ; list interpretation, either ; or _ decoded in TensileCreateLibrary
    set(Options ${Options} "--architecture=${archString}")
  endif()

  if (WIN32)
    set(CommandLine ${VIRTUALENV_BIN_DIR}/${VIRTUALENV_PYTHON_EXENAME} ${Script} ${Options} ${Tensile_LOGIC_PATH} ${Tensile_OUTPUT_PATH} HIP)
  else()
    set(CommandLine ${Script} ${Options} ${Tensile_LOGIC_PATH} ${Tensile_OUTPUT_PATH} HIP)
  endif()
  message(STATUS "Tensile_CREATE_COMMAND: ${CommandLine}")

  if(Tensile_EMBED_LIBRARY)
      set(Tensile_EMBED_LIBRARY_SOURCE "${Tensile_OUTPUT_PATH}/library/${Tensile_EMBED_LIBRARY}.cpp")
  endif()

  if($ENV{TENSILE_SKIP_LIBRARY})
      message(STATUS "Skipping build of ${Tensile_OUTPUT_PATH}")
  else()

      if(NOT Tensile_VAR_PREFIX)
          set(Tensile_VAR_PREFIX TENSILE)
      endif()

      set(Tensile_MANIFEST_FILE_PATH "${Tensile_OUTPUT_PATH}/library/TensileManifest.txt")
      message(STATUS "Tensile_MANIFEST_FILE_PATH: ${Tensile_MANIFEST_FILE_PATH}")

      if($ENV{ENABLE_ADDRESS_SANITIZER})
        # Must populate LD_PRELOAD with ASAN runtime if ASAN is being used.
        # Find the ASAN RT with compiler and update env for Tensile call.
        execute_process(
          COMMAND ${CMAKE_CXX_COMPILER} --print-file-name=libclang_rt.asan-x86_64.so
          OUTPUT_VARIABLE ASAN_LIB_PATH
          COMMAND_ECHO STDOUT)
        string(STRIP ${ASAN_LIB_PATH} ASAN_LIB_PATH)
        set(CommandLine env LD_PRELOAD=${ASAN_LIB_PATH} ${CommandLine})
      endif()

      # Create the manifest file of the output libraries.
      set(Tensile_CREATE_MANIFEST_COMMAND ${CommandLine} "--generate-manifest-and-exit")
      execute_process(
        COMMAND ${Tensile_CREATE_MANIFEST_COMMAND}
        RESULT_VARIABLE Tensile_CREATE_MANIFEST_RESULT
        COMMAND_ECHO STDOUT)

      if(Tensile_CREATE_MANIFEST_RESULT OR (NOT EXISTS ${Tensile_MANIFEST_FILE_PATH}))
        message(FATAL_ERROR "Error creating Tensile library: ${Tensile_CREATE_MANIFEST_RESULT}")
      endif()

      # Defer the actual call of the TensileCreateLibraries to 'make' time as needed.
      # Read the manifest file and declare the files as expected output.
      file(STRINGS ${Tensile_MANIFEST_FILE_PATH} Tensile_MANIFEST_CONTENTS)
      add_custom_command(
        COMMENT "Generating Tensile Libraries"
        OUTPUT ${Tensile_EMBED_LIBRARY_SOURCE};${Tensile_MANIFEST_CONTENTS}
        COMMAND ${CommandLine}
      )

      set("${Tensile_VAR_PREFIX}_ALL_FILES" ${Tensile_MANIFEST_CONTENTS} PARENT_SCOPE)

      # Create a chained library build target.
      # We've declared the manifest contents as output of the custom
      # command above which builds the tensile libs. Now create a
      # target dependency on those files so that we force the custom
      # command to be invoked at build time, not cmake time.
      TensileCreateCopyTarget(
      "${Tensile_VAR_PREFIX}_LIBRARY_TARGET"
      "${Tensile_MANIFEST_CONTENTS}"
      "${Tensile_OUTPUT_PATH}/library"
    )

  endif()

  if(Tensile_EMBED_LIBRARY)

    add_library(${Tensile_EMBED_LIBRARY} ${Tensile_EMBED_LIBRARY_SOURCE})
    target_link_libraries(${Tensile_EMBED_LIBRARY} PUBLIC TensileHost)

  endif()

endfunction()

