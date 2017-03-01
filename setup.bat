@echo off
WHERE git
IF %ERRORLEVEL% NEQ 0 (
	ECHO Please download git from https://git-scm.com/download/win
	start "" https://git-scm.com/download/win
	EXIT /B
)

WHERE cmake
IF %ERRORLEVEL% NEQ 0 (
	ECHO Please download CMake from https://cmake.org/download/
	start "" https://cmake.org/download/
	EXIT /B
)

IF NOT EXIST "3rdparty/libigl" (
    git clone https://github.com/smartgeometry-ucl/libigl.git 3rdparty/libigl
)

IF NOT EXIST "3rdparty/libigl/external/nanogui/ext" (
    cd 3rdparty/libigl
    git submodule update --init --recursive external/nanogui
    cd ../..
)
mkdir build
cd build
rm CMakeCache.txt
cmake -G "Visual Studio 14 2015 Win64" ..
cmake --build . --config RelWithDebInfo
RelWithDebInfo\iglFramework.exe ../3rdparty/libigl/tutorial/shared/bunny.off