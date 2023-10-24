The boost software can be downloaded from https://www.boost.org/. The version 1.69 or higher is used.
Copy the boost directory to /source/Lib/boost.

Usage:

Windows sample for Visual Studio 2019 64 Bit:
cd build
cmake .. -DEXTENSION_CABAC_TRAINING=1 -G "Visual Studio 16 2019"

Windows sample for Visual Studio 2015 64 Bit:
cd build
cmake .. -DEXTENSION_CABAC_TRAINING=1 -G "Visual Studio 14 2015 Win64"

Linux Release Makefile sample:
cd build
cmake .. -DEXTENSION_CABAC_TRAINING=1 -DCMAKE_BUILD_TYPE=Release

Linux Debug Makefile sample:
cd build
cmake .. -DEXTENSION_CABAC_TRAINING=1 -DCMAKE_BUILD_TYPE=Debug

MACOSX Xcode sample:
cd build
cmake .. -DEXTENSION_CABAC_TRAINING=1 -G "Xcode"
