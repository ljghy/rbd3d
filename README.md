# Rigidbody Dynamics in 3D

```
git clone --recursive https://github.com/ljghy/rbd3d.git
```

## Build and run

```
mkdir build
cd build
```

#### Windows MSVC

```
cmake ..
cmake --build . --config Release
.\test\Release\test_viewer.exe
```

#### Windows MinGW

```
cmake -G"MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release ..
mingw32-make
.\test\test_viewer.exe
```

#### Linux GCC

```
cmake -DCMAKE_BUILD_TYPE=Release ..
make 
export MESA_GL_VERSION_OVERRIDE=3.3
./test/test_viewer
```
