# 任务生成框架

本目录采用“配置描述运行方式、生成器创建任务框架、业务代码实现任务内容”的设计。

`task.toml` 用于描述任务如何运行；CMake 构建时，`generate_task.py` 会读取该配置，
生成 FreeRTOS/LibXR 任务创建和循环框架。业务代码只需在普通 `.cpp` 文件中实现生成的
`TaskHooks<...>::loop()` 特化。

生成文件位于：

```text
<构建目录>/generated/task/
├── task_generated.hpp
└── task_generated.cpp
```

这些文件由构建系统自动生成，不应手动修改。

## 触发模式

每个任务必须且只能选择一种触发方式。

### 周期任务

通过 `period_us` 设置任务周期：

```toml
period_us = 1000
```

当前 FreeRTOS 后端要求周期不小于 1000 微秒，并且必须是 1000 微秒的整数倍。

生成的任务循环会按固定周期休眠，并在每次唤醒后更新：

- `context.dt_s`：与上一次执行之间的实际时间，单位为秒。
- `context.now_us`：当前时间，单位为微秒。
- `context.iteration`：任务已经执行的次数。

### 事件任务

通过 `trigger_sem` 指定一个全局 `LibXR::Semaphore`：

```toml
trigger_sem = "imu_update_sem"
```

对应的信号量需要在业务代码中定义：

```cpp
LibXR::Semaphore imu_update_sem;
```

生成的任务会阻塞等待该信号量，在信号量被释放后执行一次 `loop()`。

`period_us` 和 `trigger_sem` 不能同时配置，也不能同时省略。

## 初始化 Hook

普通任务不需要实现初始化函数。如果配置：

```toml
custom_init = true
```

业务代码还需要实现：

```cpp
void generated::task::TaskHooks<
    generated::task::example_task_tag>::init(
        void* p1, void* p2, void* p3)
{
    // 任务启动时执行一次
}
```

## 任务守护服务

设置 `daemon_timeout_ms` 后，生成器会为任务注册一个守护服务：

```toml
daemon_timeout_ms = 100
```

此时 `LoopContext::daemon_service` 不为空，业务循环应在成功完成关键工作后喂狗：

```cpp
if (context.daemon_service != nullptr)
{
    context.daemon_service->feed();
}
```

任务超过配置时间没有喂狗时，守护线程会调用弱符号函数：

```cpp
extern "C" void generated_task_timeout_hook(const char* task_name);
```

应用可以覆盖该函数，实现电机卸力、报警或进入安全模式等项目相关处理。

## 优先级和兼容选项

LibXR 后端支持以下优先级名称：

- `IDLE`
- `LOW`
- `MEDIUM`
- `HIGH`
- `REALTIME`

也可以直接填写 FreeRTOS 整数优先级。`options` 是为其他 RTOS 后端预留的字段，
当前 LibXR/FreeRTOS 后端只接受 `0`。

## 新增任务示例

首先在 `task.toml` 中添加配置：

```toml
[[tasks]]
name = "telemetry_task"
priority = "LOW"
stack_size = 2048
entry_function = "telemetry_thread_entry"
daemon_timeout_ms = 100
parameters = ["nullptr", "nullptr", "nullptr"]
options = 0
delay_start = "IMMEDIATE"
period_us = 10000
```

然后新建业务源文件并实现循环：

```cpp
#include "task_generated.hpp"

void generated::task::TaskHooks<
    generated::task::telemetry_task_tag>::loop(
        const generated::task::LoopContext& context)
{
    if (context.daemon_service != nullptr)
    {
        context.daemon_service->feed();
    }

    // 遥测业务逻辑
}
```

最后将新的业务 `.cpp` 文件加入 `CMakeLists.txt`。修改 `task.toml` 后再次编译，
CMake 会自动重新生成任务代码。
