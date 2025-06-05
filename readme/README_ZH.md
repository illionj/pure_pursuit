# README.md
- zh_CN [简体中文](/readme/README_ZH.md)
- en [English](README.md)

# 0.介绍
pure_pursuit 纯追踪算法属于简单的路径追踪算法,它的优点在于原理简单,计算量低,对路径质量要求不高.

同样,它的缺点也很多,比如在高速场景或者弯道场景下表现效果不佳.

参考常见实现方案,比如autoware等,它们的着眼点往往在于视距调整,在直道上视距应该大,过小会导致车头抖动.而在弯道上,视距应该小,过大会导致转弯力度不足.

而我发现,在完成计算前轮转角计算后,为其增加一个增益系数,可以显著提升其效果:
```c++
  double steer = std::atan(curvature * veh_info.wheel_base);
  //streeing_angle_ratio is crucial
  steer = adjustAngle(steer * streeing_angle_ratio);
```
它的原理就是为pure_pursuit提供类似PID的调整手段,当施加增益后可以快速调整车辆姿态,调整完成后会让下一帧的真实前轮转角迅速变小.一个小的数值进行增益数值也不大.当车辆行驶在直道/曲率较小的路段中,其前轮转角接近于0,进行增益调整也不影响其表现.

+ 极佳且真实的表现效果
+ 支持倒车(输入速度<0)
+ 角速度平滑,寻找目标点时会进行简单插值
+ 快速计算
+ 大量参数自由配置


# 1.使用
## 1.1 源码方式
```CMakeLists.txt
# 假设项目位于third_party文件夹下,在当前项目CMakeLists.txt中添加以下内容
add_subdirectory("third_party/pure_pursuit" EXCLUDE_FROM_ALL)
add_executable(demo "demo.cc")
target_link_libraries(demo PRIVATE pure_pursuit)
```

## 1.2 安装方式
```bash
cd pure_pursuit
#默认安装路径为 ./build/install,路径使用CMAKE_INSTALL_PREFIX进行设置
cmake -B build
cmake --build build --target install
```
./build/install文件如下:
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
完成编译安装后,在当前项目CMakeLists.txt中添加以下内容
```

find_package(pure_pursuit 0.1 REQUIRED
             PATHS "${CMAKE_SOURCE_DIR}/third_party/puresuitlib/build/install"  # 本地安装目录
             NO_DEFAULT_PATH)        # 可选：只查这里，跳过系统路径
add_executable(demo demo.cc)
target_link_libraries(demo PRIVATE pure_pursuit::pure_pursuit)
```

# 2.本地测试
```bash
# 顶层编译会自动生成测试demo
cd pure_pursuit
cmake -B build
cmake --build build
# 执行测试demo和可视化脚本
./build/bin/demo && python scripts/plot.py 
```

# 3.测试结果
```bash
计算轮次: 185 次
millis: 6 ms
micros: 6192 μs
nanos : 6192948 ns
float ms: 6.192948 ms
```

<!-- 如果想并排展示多张图，可以用表格这样排版 -->
| 路径信息          | 航向角            
| -------------- | -------------- 
| ![Path](/scripts/path.png) | ![Yaw](/scripts/yaw.png) 

| 角速度            | 角加速度            
| -------------- | -------------- 
| ![Path](/scripts/yaw_rate.png) | ![Yaw](/scripts/yaw_accel.png) 

