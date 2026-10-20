接入步骤：
1. 在顶层 CMakeLists.txt 的 target_sources 中加入五个 .cpp。当前 LIBXR_USER_SOURCES_GLOB 只匹配 User/*.cpp，不会递归包含这些文件。
2. 新建底盘适配任务，订阅 /cmd/chassis、四个电机、IMU、裁判系统和电容话题。
3. 将话题数据转换为 Chassis::Command 和 Chassis::Feedback。
4. 以固定周期调用：

```
   chassis.set_command(command);
   chassis.set_feedback(feedback);
   const auto& output = chassis.update(dt);
```

5. 将 output.motor_torque[4] 转成对应电机 CAN 指令。
6. 用 chassis_params.hpp 填充 Chassis::Config；速度单位使用 m/s、rad/s，角度使用弧度，转矩使用 N·m。