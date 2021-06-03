# CNRS_DS

# Building and running the system

```
cd build
cmake ..
make -j
```

To run the file
```
cd build
./CNRS_DS
```

# Using in a CMake project

```cmake
find_package(CNRS_DS REQUIRED)

#...

target_link_libraries(MyTarget PUBLIC CNRS_DS::CNRS_DS)
```
