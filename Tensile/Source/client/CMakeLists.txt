set(client_sources
    source/BenchmarkTimer.cpp
    source/CSVStackFile.cpp
    source/ClientProblemFactory.cpp
    source/ConvolutionProblem.cpp
    source/DataInitialization.cpp
    source/HardwareMonitor.cpp
    source/HardwareMonitorListener.cpp
    source/LibraryUpdateReporter.cpp
    source/MetaRunListener.cpp
    source/PerformanceReporter.cpp
    source/ProgressListener.cpp
    source/Reference.cpp
    source/ReferenceValidator.cpp
    source/ResultFileReporter.cpp
    source/ResultReporter.cpp
    source/SolutionIterator.cpp
    source/TimingEvents.cpp
    )

SET(Boost_USE_STATIC_LIBS ON)
#find_package(Boost COMPONENTS program_options REQUIRED)
find_package(Boost)
message("Boost_FOUND: " ${Boost_FOUND})
set(Boost_LIBRARIES "C:/local/boost_1_76_0/lib64-msvc-14.2/libboost_program_options-vc142-mt-x64-1_76.lib")
add_library(TensileClient STATIC ${client_sources})

set_target_properties(TensileClient
                      PROPERTIES
                      CXX_STANDARD 14
                      CXX_STANDARD_REQUIRED ON
                      CXX_EXTENSIONS OFF)

find_package(ROCmSMI QUIET)
if(NOT ROCmSMI_FOUND)
    #set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${Tensile_DIR}" "${Tensile_DIR}/../Source/cmake" "${CMAKE_HOME_DIRECTORY}/cmake")
    #find_package(ROCmSMI REQUIRED)
	message("Skip ROCmSMI module")
endif()

target_include_directories(TensileClient PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")# "${rocm_smi_root}/include")

message("Boost_LIBRARIES: " ${Boost_LIBRARIES})
message("Boost_LIBRARIES: " ${Boost_PROGRAM_OPTIONS_LIBRARY})
include_directories(${Boost_INCLUDE_DIR})

target_link_libraries(TensileClient PRIVATE TensileHost ${Boost_LIBRARIES} rocm_smi)
if(TENSILE_USE_OPENMP)
    target_link_libraries(TensileClient PRIVATE custom_openmp_cxx)
endif()

add_executable(tensile_client main.cpp)
set_target_properties(tensile_client
                      PROPERTIES
                      CXX_STANDARD 14
                      CXX_STANDARD_REQUIRED ON
                      CXX_EXTENSIONS OFF)

target_link_libraries(tensile_client PRIVATE TensileHost TensileClient ${Boost_LIBRARIES})
if(TENSILE_USE_OPENMP)
    target_link_libraries(tensile_client PRIVATE custom_openmp_cxx)
endif()

foreach(arch IN LISTS TENSILE_GPU_ARCHS)
    target_link_libraries(tensile_client PRIVATE "--amdgpu-target=${arch}")
endforeach(arch)
