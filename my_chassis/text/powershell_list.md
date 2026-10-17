你现在的状态是：

- 10 个模块仓库都已经拉取到 `Modules/scu-hotpot/`，并由 `xrobot.lock` 固定版本。
- 当前真正实例化的只有：
  - `RTTLogger`
  - `BMI088`
- 电机、CanManager、VQF、遥控器、裁判系统、板间通信虽然已经下载，但还没有实例化。
- 基础工程已经可以正常构建。

下面是一份适用于你当前工程的完整命令手册。所有命令默认先进入项目根目录：

```powershell
Set-Location D:\stm32_workSpace\preparation_for_the_27RMUL\my_chassis
```

# 一、先理解“拉取模块”和“实例化模块”

`Modules/modules.yaml` 决定“项目允许使用哪些模块仓库”：

```yaml
modules:
  - scu-hotpot/BMI088@xrobot-v1
  - scu-hotpot/DJIMotor@xrobot-v1
  - scu-hotpot/CanManager@xrobot-v1
```

`User/xrobot.yaml` 决定“实际创建哪些对象”：

```yaml
modules:
  - module: scu-hotpot/BMI088
    id: bmi088
    args: ...
```

所以：

```text
module add/setup = 下载模块代码
instance add/gen = 创建模块对象
```

当前模块已全部下载，但只有 RTTLogger 和 BMI088 被实例化。

# 二、日常最常用命令

查看整个 XRobot 状态：

```powershell
xrobot -C . describe
```

它可以检查：

- `xrobot_main.hpp` 是否过期；
- 模块版本是否和锁文件一致；
- 硬件容器中有哪些对象；
- 模块构造参数；
- 当前有哪些实例；
- 硬件参数有哪些候选对象。

修改 `User/xrobot.yaml` 后重新生成：

```powershell
xrobot -C . gen -c User/xrobot.yaml
```

日常 Debug 编译：

```powershell
cmake --build --preset Debug --parallel
```

最常见的日常流程就是：

```powershell
xrobot -C . gen -c User/xrobot.yaml
cmake --build --preset Debug --parallel
```

# 三、查看模块支持哪些参数

例如查看电机模块：

```powershell
xrobot -C . module show scu-hotpot/DJIMotor
```

其他模块：

```powershell
xrobot -C . module show scu-hotpot/BMI088
xrobot -C . module show scu-hotpot/DMMotor
xrobot -C . module show scu-hotpot/LKMotor
xrobot -C . module show scu-hotpot/CanManager
xrobot -C . module show scu-hotpot/VQFModule
xrobot -C . module show scu-hotpot/RemoteControl
xrobot -C . module show scu-hotpot/RefereeSystem
xrobot -C . module show scu-hotpot/BtoBOnCan
```

在添加实例前，建议先执行一次 `module show`，确认当前锁定版本的真实构造参数。

# 四、创建普通模块实例

例如创建一个 DJI 电机：

```powershell
xrobot -C . instance add scu-hotpot/DJIMotor --id chassis_lf
```

它会在 `User/xrobot.yaml` 中加入一个使用默认参数的实例。

修改参数：

```powershell
xrobot -C . instance set chassis_lf args.id 1
xrobot -C . instance set chassis_lf args.actual_total_ratio 19.0f
xrobot -C . instance set chassis_lf args.command_timeout_ms 100
```

修改 C++ 字符串时，需要保留里面的双引号：

```powershell
xrobot -C . instance set `
  chassis_lf `
  args.topic_name `
  '"/chassis/left_front/feedback"'
```

修改枚举：

```powershell
xrobot -C . instance set `
  chassis_lf `
  args.type `
  DJIMotor::MotorType::GM6020
```

具体枚举值以这里的输出为准：

```powershell
xrobot -C . module show scu-hotpot/DJIMotor
```

配置完成后：

```powershell
xrobot -C . gen -c User/xrobot.yaml
cmake --build --preset Debug --parallel
```

# 五、创建模板模块实例

有些模块带 C++ 模板参数，例如 VQF 和 LKMotor。

创建 VQF：

```powershell
xrobot -C . instance add `
  scu-hotpot/VQFModule `
  --id vqf `
  --template-arg float
```

默认话题名称刚好可以接 BMI088：

```text
bmi088_accl
bmi088_gyro
```

生成并编译：

```powershell
xrobot -C . gen -c User/xrobot.yaml
cmake --build --preset Debug --parallel
```

创建 LKMotor：

```powershell
xrobot -C . instance add `
  scu-hotpot/LKMotor `
  --id lk_motor_1 `
  --template-arg 14
```

其中 `14` 是编码器位数模板参数，必须根据电机型号确认。

# 六、修改已有实例

修改 BMI088 目标温度：

```powershell
xrobot -C . instance set `
  bmi088 `
  args.target_temperature `
  45.0f
```

修改 BMI088 任务栈：

```powershell
xrobot -C . instance set `
  bmi088 `
  args.task_stack_depth `
  3072
```

修改嵌套结构体字段：

```powershell
xrobot -C . instance set bmi088 args.pid_param.k 1.0f
xrobot -C . instance set bmi088 args.pid_param.p 0.1f
xrobot -C . instance set bmi088 args.pid_param.i 0.0f
xrobot -C . instance set bmi088 args.pid_param.d 0.0f
```

修改后统一执行：

```powershell
xrobot -C . gen -c User/xrobot.yaml
cmake --build --preset Debug --parallel
```

# 七、删除或重命名实例

删除实例：

```powershell
xrobot -C . instance remove chassis_lf
```

重命名实例：

```powershell
xrobot -C . instance rename chassis_lf chassis_left_front
```

重命名时，XRobot 会同时修改配置中对该实例的引用。

操作完成后：

```powershell
xrobot -C . gen -c User/xrobot.yaml
cmake --build --preset Debug --parallel
```

# 八、添加一个还没下载的新模块

先添加模块请求：

```powershell
xrobot -C . module add owner/ModuleName@branch
```

例如：

```powershell
xrobot -C . module add scu-hotpot/SomeModule@xrobot-v1
```

拉取模块、解析依赖、更新锁文件：

```powershell
xrobot -C . setup
```

查看接口：

```powershell
xrobot -C . module show scu-hotpot/SomeModule
```

创建实例：

```powershell
xrobot -C . instance add `
  scu-hotpot/SomeModule `
  --id some_module
```

生成并编译：

```powershell
xrobot -C . gen -c User/xrobot.yaml
cmake --preset Debug
cmake --build --preset Debug --parallel
```

新增模块会改变 CMake 模块清单，因此建议重新执行一次 CMake 配置。

# 九、删除不再需要的模块仓库

必须先删除该模块的所有实例：

```powershell
xrobot -C . instance remove some_module
```

再移除模块请求：

```powershell
xrobot -C . module remove owner/ModuleName
```

然后更新模块目录和锁文件：

```powershell
xrobot -C . setup
```

重新配置和编译：

```powershell
cmake --preset Debug
cmake --build --preset Debug --parallel
```

注意：当前 `Modules/modules.yaml` 请求了全部 10 个模块，所以即使某个模块没有实例化，它也会参与 CMake 配置和源文件编译，只是最终没有使用的代码通常会被链接器回收。

如果你确认长期不使用某个模块，可以从请求列表中移除，以减少依赖和编译时间。

# 十、更新模块版本

更新所有模块：

```powershell
xrobot -C . setup --update
```

只更新指定模块：

```powershell
xrobot -C . setup --update scu-hotpot/BMI088
```

更新模块后，XRobot 会同步配置字段并重新生成入口。然后执行：

```powershell
cmake --preset Debug
cmake --build --preset Debug --parallel
```

正式项目中不要随意更新全部模块。建议先提交当前可用版本，再更新。

# 十一、严格按照锁文件恢复模块

刚克隆工程，或者换电脑后：

```powershell
git submodule update --init --recursive
xrobot -C . setup --frozen
```

含义：

- `git submodule update`：恢复 LibXR 和 LibHT。
- `xrobot setup --frozen`：严格按照 `xrobot.lock` 恢复业务模块，不更新版本。

没有网络、但 `Modules/` 内容已经完整时：

```powershell
xrobot -C . setup --frozen --offline
```

# 十二、模块接口升级后同步配置

如果模块新增、删除或重命名了构造参数：

```powershell
xrobot -C . sync -c User/xrobot.yaml
```

它会：

- 添加新参数及其默认值；
- 删除模块已经移除的参数；
- 保留仍然有效的用户配置。

然后重新生成：

```powershell
xrobot -C . gen -c User/xrobot.yaml
```

更新模块时，通常直接使用：

```powershell
xrobot -C . setup --update
```

因为 `setup --update` 会同时执行配置同步。

# 十三、格式化和检查 XRobot YAML

格式化：

```powershell
xrobot -C . format -c User/xrobot.yaml
```

只检查、不修改：

```powershell
xrobot -C . format --check -c User/xrobot.yaml
```

修改 YAML 后如果遇到缩进或结构问题，可以先执行格式检查。

# 十四、只修改 xrobot.yaml 后怎么办

适用场景：

- 添加/删除模块实例；
- 修改电机 ID；
- 修改话题名；
- 修改 PID；
- 修改任务优先级或栈；
- 修改 CanManager 列表。

执行：

```powershell
xrobot -C . gen -c User/xrobot.yaml
cmake --build --preset Debug --parallel
```

不需要运行 `libxr parse` 或 `libxr gen`。

# 十五、修改 libxr_config.yaml 后怎么办

适用场景：

- 修改 UART/SPI 缓冲区大小；
- 修改 DMA section；
- 修改 CAN 队列深度；
- 选择终端串口；
- 修改软件定时器栈和优先级。

执行：

```powershell
libxr gen `
  -i .\.config.yaml `
  -o .\User\app_main.cpp `
  --libxr-config .\User\libxr_config.yaml `
  --xrobot

xrobot -C . gen -c User/xrobot.yaml

cmake --build --preset Debug --parallel
```

这里通常不需要 `libxr parse`，因为 `.ioc` 没变。

# 十六、修改硬件容器后怎么办

“硬件容器”主要由以下两部分决定：

```text
.config.yaml
User/libxr_config.yaml
```

最终生成到：

```text
User/app_main.cpp
```

如果只是修改 `User/libxr_config.yaml`，按照上一节执行。

如果手工修改 `User/app_main.cpp`，只能改：

```cpp
/* User Code Begin ... */
/* User Code End ... */
```

修改用户区域后建议执行：

```powershell
xrobot -C . gen -c User/xrobot.yaml
cmake --build --preset Debug --parallel
```

不要手工修改生成区中的：

```cpp
STM32SPI
STM32UART
STM32CANFD
XR_REGISTER
```

这些内容下次执行 `libxr gen` 会被覆盖。

# 十七、修改 CubeMX 配置后的完整流程

适用场景：

- 增加或删除外设；
- 修改 GPIO 标签；
- 修改 SPI/UART/CAN；
- 修改 DMA；
- 修改定时器/PWM；
- 修改引脚；
- 修改中断；
- 修改 FreeRTOS。

首先在 CubeMX 中保存 `.ioc` 并 Generate Code。

也可以尝试命令行生成：

```powershell
libxr stm32 cubemx-gen -d .
```

但日常使用 CubeMX GUI 生成更直观。

生成后检查两个容易被覆盖的地方。

检查 HAL tick 优先级：

```powershell
Select-String `
  -Path .\Core\Inc\stm32h7xx_hal_conf.h `
  -Pattern 'TICK_INT_PRIORITY'
```

应为：

```c
#define TICK_INT_PRIORITY (0UL)
```

检查默认任务入口：

```powershell
Select-String `
  -Path .\Core\Src\freertos.c `
  -Pattern 'app_main\(\)'
```

应存在：

```c
app_main();
```

然后执行完整再生成流程：

```powershell
libxr parse -d .

libxr gen `
  -i .\.config.yaml `
  -o .\User\app_main.cpp `
  --libxr-config .\User\libxr_config.yaml `
  --xrobot

xrobot -C . sync -c User/xrobot.yaml
xrobot -C . gen -c User/xrobot.yaml

cmake --preset Debug --fresh
cmake --build --preset Debug --parallel
```

各命令作用：

1. `libxr parse`：重新读取 `.ioc`，更新硬件描述。
2. `libxr gen`：重新生成硬件容器和注册对象。
3. `xrobot sync`：同步模块参数结构。
4. `xrobot gen`：根据新的硬件容器重新绑定模块。
5. `cmake --fresh`：重新读取 CubeMX 生成的源文件和 CMake。
6. `cmake --build`：编译。

如果你重命名了 GPIO 标签，例如：

```text
BMI088_GYRO_CS → IMU_GYRO_CS
```

那么 `xrobot gen` 会报告旧硬件名不存在。此时修改引用：

```powershell
xrobot -C . instance set `
  bmi088 `
  args.cs_gyro `
  IMU_GYRO_CS

xrobot -C . gen -c User/xrobot.yaml
```

# 十八、只修改普通 C/C++ 业务代码后怎么办

例如修改：

```text
Core/Src/*.c
User/*.cpp 的 User Code 区域
你自己新增的 App/*.cpp
```

只需要：

```powershell
cmake --build --preset Debug --parallel
```

如果新增了源文件但 CMake 没有自动收录，需要在 `CMakeLists.txt` 中加入：

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    App/my_file.cpp
)
```

然后：

```powershell
cmake --preset Debug
cmake --build --preset Debug --parallel
```

`User/*.cpp` 已通过 `LIBXR_USER_SOURCES_GLOB` 自动收录。

# 十九、Debug 和 Release 构建

配置 Debug：

```powershell
cmake --preset Debug
```

编译 Debug：

```powershell
cmake --build --preset Debug --parallel
```

配置 Release：

```powershell
cmake --preset Release
```

编译 Release：

```powershell
cmake --build --preset Release --parallel
```

生成文件位于：

```text
build/Debug/my_chassis.elf
build/Debug/my_chassis.hex
build/Debug/my_chassis.bin

build/Release/my_chassis.elf
build/Release/my_chassis.hex
build/Release/my_chassis.bin
```

# 二十、什么时候使用 --fresh

普通代码变化不用 `--fresh`。

以下情况建议使用：

- CubeMX 重新生成工程；
- 修改 CMakeLists；
- 修改 `cmake/LibXR.CMake`；
- 添加或删除模块；
- 修改工具链；
- CMake 缓存异常；
- 文件明明存在但 CMake 没识别。

命令：

```powershell
cmake --preset Debug --fresh
cmake --build --preset Debug --parallel
```

# 二十一、清理后重新编译

清理目标文件：

```powershell
cmake --build --preset Debug --target clean
```

再重新配置和构建：

```powershell
cmake --preset Debug --fresh
cmake --build --preset Debug --parallel
```

通常不需要手工删除整个 `build` 目录。

# 二十二、编译错误时增加详细输出

```powershell
cmake --build --preset Debug --verbose
```

它会显示完整编译器参数、头文件路径、宏和链接命令。

XRobot 入口过期：

```text
xrobot_main.hpp is stale
```

解决：

```powershell
xrobot -C . gen -c User/xrobot.yaml
cmake --build --preset Debug --parallel
```

模块锁不一致：

```powershell
xrobot -C . setup --frozen
```

硬件名找不到：

```powershell
xrobot -C . describe
```

查看实际注册名称和候选对象，然后修改对应实例参数。

# 二十三、构建结果检查

查看固件大小：

```powershell
arm-none-eabi-size .\build\Debug\my_chassis.elf
```

确认入口存在：

```powershell
arm-none-eabi-nm -C .\build\Debug\my_chassis.elf |
  Select-String ' app_main$|StartDefaultTask'
```

检查 DMA 段：

```powershell
arm-none-eabi-objdump `
  -h .\build\Debug\my_chassis.elf |
  Select-String 'dma_buffer'
```

当前正确结果应类似：

```text
.dma_buffer  00000140  38000000
```

查看链接 map：

```powershell
Select-String `
  -Path .\build\Debug\my_chassis.map `
  -Pattern 'dma_buffer'
```

# 二十四、换电脑或重新克隆后的初始化

推荐克隆方式：

```powershell
git clone --recurse-submodules <你的仓库地址>
Set-Location .\my_chassis
```

如果已经普通克隆：

```powershell
git submodule update --init --recursive
```

恢复 XRobot 模块：

```powershell
xrobot -C . setup --frozen
```

生成硬件代码：

```powershell
libxr parse -d .

libxr gen `
  -i .\.config.yaml `
  -o .\User\app_main.cpp `
  --libxr-config .\User\libxr_config.yaml `
  --xrobot

xrobot -C . gen -c User/xrobot.yaml
```

构建：

```powershell
cmake --preset Debug --fresh
cmake --build --preset Debug --parallel
```

# 二十五、Git 日常检查

查看状态：

```powershell
git status
```

查看源码变化：

```powershell
git diff
```

查看暂存变化：

```powershell
git diff --cached
```

查看子模块版本：

```powershell
git submodule status --recursive
```

固定当前 LibXR/LibHT 版本：

```powershell
git add .gitmodules
git add Middlewares/Third_Party/LibXR
git add lib/libht
```

不要在 LibXR、LibHT 和 `Modules/scu-hotpot/*` 内直接开发业务代码。否则后续更新依赖时很难维护。

# 最后给你一个场景速查表

| 发生的变化 | 需要执行 |
|---|---|
| 修改普通 C/C++ 代码 | `cmake --build --preset Debug --parallel` |
| 修改 `User/xrobot.yaml` | `xrobot gen` → build |
| 添加/删除/修改实例 | `xrobot gen` → build |
| 修改 `User/libxr_config.yaml` | `libxr gen` → `xrobot gen` → build |
| 修改 CubeMX `.ioc` | CubeMX Generate → `libxr parse` → `libxr gen` → `xrobot sync` → `xrobot gen` → CMake fresh build |
| 添加新模块仓库 | `xrobot module add` → `xrobot setup` → `instance add` → `xrobot gen` → build |
| 更新模块版本 | `xrobot setup --update` → CMake configure/build |
| 恢复锁定模块 | `xrobot setup --frozen` |
| 入口提示 stale | `xrobot gen` → build |
| 修改 CMake | `cmake --preset Debug --fresh` → build |
| 换电脑/重新克隆 | `git submodule update --init --recursive` → `xrobot setup --frozen` → configure/build |

你日常最常用的四条基本就是：

```powershell
xrobot -C . describe
xrobot -C . gen -c User/xrobot.yaml
cmake --preset Debug
cmake --build --preset Debug --parallel
```

而修改 CubeMX 后，记住这一整套：

```powershell
libxr parse -d .

libxr gen `
  -i .\.config.yaml `
  -o .\User\app_main.cpp `
  --libxr-config .\User\libxr_config.yaml `
  --xrobot

xrobot -C . sync -c User/xrobot.yaml
xrobot -C . gen -c User/xrobot.yaml

cmake --preset Debug --fresh
cmake --build --preset Debug --parallel
```