################################################################################
# Copyright 2019-2020 Advanced Micro Devices, Inc. All rights reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell cop-
# ies of the Software, and to permit persons to whom the Software is furnished
# to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IM-
# PLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
# FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
# COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
# IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNE-
# CTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
################################################################################

import itertools
import os
import subprocess

from . import Common
from .Common import globalParameters
from .Parallel import CPUThreadCount

def cmake_path(os_path):
    return (os_path.replace("\\", "/") if (os.name == "nt") else os_path)

class CMakeEnvironment:
    def __init__(self, sourceDir, buildDir, **options):
        self.sourceDir = sourceDir
        self.buildDir  = buildDir
        self.options = options

    def generate(self):

        args = ['cmake']
        args += ['-G', 'Ninja'] if (os.name == 'nt') else []
        args += itertools.chain.from_iterable([ ['-D{}={}'.format(key, value)] for key,value in self.options.items()])
        args += [self.sourceDir]
        args = [cmake_path(arg) for arg in args]

        Common.print2(' '.join(args))
        with Common.ClientExecutionLock():
            subprocess.check_call(args, cwd=Common.ensurePath(self.buildDir))

    def build(self):
        args = [('ninja' if (os.name == "nt") else 'make'), f'-j{CPUThreadCount()}']
        Common.print2(' '.join(args))
        with Common.ClientExecutionLock():
            subprocess.check_call(args, cwd=self.buildDir)

    def builtPath(self, path, *paths):
        return os.path.join(self.buildDir, path, *paths)

def clientExecutableEnvironment(builddir=None):
    sourcedir = globalParameters["SourcePath"]
    if builddir is None:
        builddir = os.path.join(globalParameters["OutputPath"], globalParameters["ClientBuildPath"])
    builddir = Common.ensurePath(builddir)

    CxxCompiler = "clang++.exe" if ((os.name == "nt") and (globalParameters['CxxCompiler'] == "hipcc")) else globalParameters['CxxCompiler']
    CCompiler   = "clang.exe"   if ((os.name == "nt") and (globalParameters['CxxCompiler'] == "hipcc")) else globalParameters['CxxCompiler']

    options = {'CMAKE_BUILD_TYPE': globalParameters["CMakeBuildType"],
               'TENSILE_USE_MSGPACK': 'ON',
               'TENSILE_USE_LLVM': 'OFF' if (os.name == "nt") else 'ON',
               'Tensile_LIBRARY_FORMAT': globalParameters["LibraryFormat"],
               'CMAKE_CXX_COMPILER': os.path.join(globalParameters["ROCmBinPath"], CxxCompiler),
               'CMAKE_C_COMPILER': os.path.join(globalParameters["ROCmBinPath"], CCompiler)}

    return CMakeEnvironment(sourcedir, builddir, **options)


buildEnv = None

def getClientExecutable(builddir=None):
    if "PrebuiltClient" in globalParameters:
        return globalParameters["PrebuiltClient"]

    global buildEnv

    if buildEnv is None:
        buildEnv = clientExecutableEnvironment(builddir)
        buildEnv.generate()
        buildEnv.build()

    return buildEnv.builtPath("client/tensile_client")

