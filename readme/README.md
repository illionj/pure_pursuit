# README.md
- zh_CN [简体中文](/readme/README_ZH.md)
- en [English](README.md)

# 0. Introduction
The pure pursuit algorithm is a simple path tracking algorithm. Its advantage lies in its simplicity and low computational cost, with no high requirements for path quality. 

However, it has many drawbacks, especially in high-speed or curve scenarios, where its performance is not ideal.

Common implementations, such as Autoware, often focus on adjusting the lookahead distance. On straight paths, the lookahead distance should be large; if it is too small, it causes the vehicle's front end to jitter. In curves, the lookahead distance should be small; if it is too large, it results in insufficient steering force.

We found that by introducing a gain factor after computing the steering angle of the front wheels, the algorithm’s performance could be significantly improved:
```c++
  double steer = std::atan(curvature * veh_info.wheel_base);
  // steering_angle_ratio is crucial
  steer = adjustAngle(steer * steering_angle_ratio);
```

This principle provides an adjustment mechanism similar to a PID controller for pure pursuit, enabling quick vehicle attitude adjustments. Once the adjustment is made, the next frame's actual front wheel angle will decrease rapidly. The gain value used here is small. When the vehicle is on a straight path or a path with small curvature, where the front wheel angle is close to zero, applying the gain adjustment does not affect its performance.

+ Excellent and realistic performance
+ Supports reverse driving (input speed < 0)
+ Smooth angular velocity, simple interpolation while searching for target points
+ Fast computation
+ A large number of configurable parameters



# 1. Usage
## 1.1 From Source
```CMakeLists.txt
# Assuming the project is in the third_party folder, add the following to the current project's CMakeLists.txt
add_subdirectory("third_party/pure_pursuit" EXCLUDE_FROM_ALL)
add_executable(demo "demo.cc")
target_link_libraries(demo PRIVATE pure_pursuit)
```

## 1.2 Installation
```bash
cd pure_pursuit
# The default installation path is ./build/install. The path can be set with CMAKE_INSTALL_PREFIX
cmake -B build
cmake --build build --target install
```
The `./build/install` directory will look like this:
```bash
$ tree
.
├── include
│   └── pure_pursuit.h
└── lib
    ├── cmake
    │   └── pure_pursuit
    │       ├── pure_pursuitConfig.cmake
    │       ├── pure_pursuitConfigVersion.cmake
    │       ├── pure_pursuitTargets.cmake
    │       └── pure_pursuitTargets-relwithdebinfo.cmake
    └── libpure_pursuit.a

4 directories, 6 files
```

After installation, add the following to your project's CMakeLists.txt:
```cmake
find_package(pure_pursuit 0.1 REQUIRED
             PATHS "${CMAKE_SOURCE_DIR}/third_party/puresuitlib/build/install"  # Local install directory
             NO_DEFAULT_PATH)        # Optional: Only search here, skip system paths
add_executable(demo demo.cc)
target_link_libraries(demo PRIVATE pure_pursuit::pure_pursuit)
```

# 2. Local Testing
```bash
cd pure_pursuit
./build_and_test.sh
```

# 3. Test Results
```bash
Number of iterations: 185
millis: 6 ms
micros: 6192 μs
nanos: 6192948 ns
float ms: 6.192948 ms
```

<!-- To display multiple images side by side, you can use tables like this -->
| Path Information        | Heading Angle        
| --------------------- | ------------------ 
| ![Path](/scripts/path.png) | ![Yaw](/scripts/yaw.png) 

| Angular Velocity        | Angular Acceleration        
| --------------------- | -------------------- 
| ![Path](/scripts/yaw_rate.png) | ![Yaw](/scripts/yaw_accel.png) 

