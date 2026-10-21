# my_sentry PowerShell 命令手册

本文档适用于当前多目标工程结构，并以以下工具版本为基准：

- XRobot CLI 1.0.0
- LibXR CLI 6.0.0
- CMake 3.22 及以上版本
- PowerShell 7 或 Windows PowerShell 5.1

当前固件目标有 `my_chassis` 和 `my_gimbal`。CMake 在仓库根目录统一构建两个目标；每个目标的 STM32CubeMX 工程位于 `hal`，XRobot 配置位于 `xr`。

## 1. 初始化当前 PowerShell 会话

先进入仓库根目录：

```powershell
Set-Location 'D:\stm32_workSpace\preparation_for_the_27RMUL\my_sentry'
```

选择要操作的固件目标。操作底盘时使用：

```powershell
$Target = 'my_chassis'
$TargetDir = Join-Path $PWD "src\targets\$Target"
$HalDir = Join-Path $TargetDir 'hal'
$XrDir = Join-Path $TargetDir 'xr'
```

操作云台时只需把第一行改为：

```powershell
$Target = 'my_gimbal'
```

检查工具版本和目录：

```powershell
xrobot --version
libxr --version
cmake --version

Test-Path $HalDir
Test-Path $XrDir
```

> 下文所有命令均假设当前目录是仓库根目录，并且已经定义 `$Target`、`$TargetDir`、`$HalDir` 和 `$XrDir`。

## 2. 目录与工具的对应关系

| 内容 | 当前路径 | 使用的工具 |
|---|---|---|
| CubeMX 工程与 HAL 源码 | `src/targets/<目标>/hal` | CubeMX、`libxr` |
| XRobot 模块清单 | `src/targets/<目标>/xr/Modules/modules.yaml` | `xrobot module`、`xrobot setup` |
| XRobot 实例配置 | `src/targets/<目标>/xr/User/xrobot.yaml` | `xrobot instance`、`xrobot gen` |
| LibXR 用户配置 | `src/targets/<目标>/xr/User/libxr_config.yaml` | `libxr gen` |
| 生成的硬件容器 | `src/targets/<目标>/xr/User/app_main.cpp` | `libxr gen` |
| 生成的模块入口 | `src/targets/<目标>/xr/User/xrobot_main.hpp` | `xrobot gen`、`xrobot setup` |
| 最终固件 | `build/<配置>/firmware` | CMake |

XRobot 会从 `-C` 指定的位置向上查找最近的 `Modules/modules.yaml`。因此当前工程必须使用 `-C $XrDir`，不能再在仓库根目录使用 `xrobot -C .`。

## 3. 日常构建

首次构建或 CMake 配置发生变化：

```powershell
cmake --preset Debug
cmake --build --preset Debug --parallel
```

普通 C/C++ 代码修改后：

```powershell
cmake --build --preset Debug --parallel
```

只构建一个目标：

```powershell
cmake --build --preset Debug --target "${Target}_firmware" --parallel
```

构建 Release：

```powershell
cmake --preset Release
cmake --build --preset Release --parallel
```

构建产物位于：

```text
build/Debug/firmware/my_chassis.elf
build/Debug/firmware/my_chassis.hex
build/Debug/firmware/my_chassis.bin
build/Debug/firmware/my_chassis.map

build/Debug/firmware/my_gimbal.elf
build/Debug/firmware/my_gimbal.hex
build/Debug/firmware/my_gimbal.bin
build/Debug/firmware/my_gimbal.map
```

底盘参数、任务表和 RTT Topic Monitor 的生成器已经接入 CMake，修改相应 TOML 或生成脚本后直接构建即可，无需手动运行 Python：

```text
src/config/chassis/chassis.toml
src/targets/my_chassis/task/task.toml
src/targets/my_chassis/xr/User/debug/topic_monitor.toml
```

## 4. XRobot 日常操作

查看所选目标的完整状态：

```powershell
xrobot -C $XrDir describe
```

查看模块接口和真实构造参数：

```powershell
xrobot -C $XrDir module show scu-hotpot/DJIMotor
xrobot -C $XrDir module show scu-hotpot/BMI088
xrobot -C $XrDir module show scu-hotpot/CanManager
```

修改 `xrobot.yaml` 后重新生成模块入口并构建：

```powershell
xrobot -C $XrDir gen -c User/xrobot.yaml
cmake --build --preset Debug --target "${Target}_firmware" --parallel
```

如果只操作默认的 `User/xrobot.yaml`，可以省略 `-c User/xrobot.yaml`：

```powershell
xrobot -C $XrDir gen
```

### 添加和修改实例

添加普通模块实例：

```powershell
xrobot -C $XrDir instance add scu-hotpot/DJIMotor --id chassis_motor5
```

添加带模板参数的实例；每个模板参数都要单独写一个 `--template-arg`：

```powershell
xrobot -C $XrDir instance add scu-hotpot/VQFModule `
  --id vqf `
  --template-arg float
```

修改数值、枚举和嵌套字段：

```powershell
xrobot -C $XrDir instance set chassis_motor5 args.id 5
xrobot -C $XrDir instance set chassis_motor5 args.actual_total_ratio 19.0f
xrobot -C $XrDir instance set chassis_motor5 args.type DJIMotor::MotorType::M3508
xrobot -C $XrDir instance set bmi088 args.pid_param.p 0.1f
```

设置 C++ 字符串时，传给 XRobot 的 YAML 值本身必须保留双引号：

```powershell
xrobot -C $XrDir instance set `
  chassis_motor5 `
  args.topic_name `
  '"/fb/motor/chassis_motor5"'
```

删除或重命名实例：

```powershell
xrobot -C $XrDir instance remove chassis_motor5
xrobot -C $XrDir instance rename chassis_motor1 chassis_left_front
```

重命名时 XRobot 会同步修改该配置中对实例 ID 的引用。完成增删改后统一执行：

```powershell
xrobot -C $XrDir gen
cmake --build --preset Debug --target "${Target}_firmware" --parallel
```

### 同步和格式化配置

模块接口升级后，把新增参数的默认值写入配置并移除已经失效的参数：

```powershell
xrobot -C $XrDir sync -c User/xrobot.yaml
xrobot -C $XrDir gen -c User/xrobot.yaml
```

格式化配置：

```powershell
xrobot -C $XrDir format -c User/xrobot.yaml
```

只检查格式，不改文件：

```powershell
xrobot -C $XrDir format --check -c User/xrobot.yaml
```

## 5. XRobot 模块仓库管理

`Modules/modules.yaml` 表示允许使用哪些模块，`User/xrobot.yaml` 表示实际创建哪些模块实例。下载模块不等于创建实例。

添加模块请求并解析依赖：

```powershell
xrobot -C $XrDir module add scu-hotpot/SomeModule@xrobot-v1
xrobot -C $XrDir setup
```

删除模块前应先删除其全部实例，然后执行：

```powershell
xrobot -C $XrDir module remove scu-hotpot/SomeModule
xrobot -C $XrDir setup
```

严格按照 `xrobot.lock` 恢复模块，适合克隆仓库后使用：

```powershell
xrobot -C $XrDir setup --frozen
```

模块代码已经完整存在、本次不允许联网时：

```powershell
xrobot -C $XrDir setup --frozen --offline
```

更新全部模块或指定模块：

```powershell
xrobot -C $XrDir setup --update
xrobot -C $XrDir setup --update scu-hotpot/BMI088
```

`setup --update` 会重新解析版本、更新锁文件、同步配置并重新生成入口。更新后重新配置并构建：

```powershell
cmake --preset Debug --fresh
cmake --build --preset Debug --parallel
```

> 两个目标各有独立的 `xrobot.lock` 和模块目录。需要同时恢复或更新两个目标时，要分别对两个 `xr` 目录执行命令。

同时严格恢复底盘和云台模块：

```powershell
'my_chassis', 'my_gimbal' | ForEach-Object {
  xrobot -C (Join-Path $PWD "src\targets\$_\xr") setup --frozen
}
```

## 6. 修改 LibXR 配置后的生成流程

只修改了 `xr/User/libxr_config.yaml`，而 `.ioc` 和 `hal/.config.yaml` 没有变化时：

```powershell
libxr gen `
  -i (Join-Path $HalDir '.config.yaml') `
  -d $HalDir `
  -o (Join-Path $XrDir 'User\app_main.cpp') `
  --libxr-config (Join-Path $XrDir 'User\libxr_config.yaml') `
  --xrobot

xrobot -C $XrDir gen
cmake --build --preset Debug --target "${Target}_firmware" --parallel
```

`app_main.cpp` 是生成文件。需要保留的手写内容只能放在文件标出的 User Code 区域；其余硬件对象和注册代码会在下次 `libxr gen` 时覆盖。

## 7. 修改 CubeMX 配置后的完整流程

在 STM32CubeMX 中保存 `.ioc` 并 Generate Code。也可以用命令行调用 CubeMX：

```powershell
libxr stm32 cubemx-gen -d $HalDir
```

CubeMX 生成完成后，依次重新解析硬件、生成 LibXR 容器、同步 XRobot 配置并构建：

```powershell
libxr parse `
  -d $HalDir `
  -o (Join-Path $HalDir '.config.yaml')

libxr gen `
  -i (Join-Path $HalDir '.config.yaml') `
  -d $HalDir `
  -o (Join-Path $XrDir 'User\app_main.cpp') `
  --libxr-config (Join-Path $XrDir 'User\libxr_config.yaml') `
  --xrobot

xrobot -C $XrDir sync -c User/xrobot.yaml
xrobot -C $XrDir gen -c User/xrobot.yaml

cmake --preset Debug --fresh
cmake --build --preset Debug --parallel
```

CubeMX 可能覆盖手工改动。生成后至少检查 FreeRTOS 默认任务仍会调用 `app_main()`：

```powershell
Select-String `
  -Path (Join-Path $HalDir 'Core\Src\freertos.c') `
  -Pattern 'app_main\(\)'
```

同时检查 HAL tick 优先级的实际值；不同目标未必相同，不要用脚本盲目改成固定数字：

```powershell
Select-String `
  -Path (Join-Path $HalDir 'Core\Inc\stm32h7xx_hal_conf.h') `
  -Pattern 'TICK_INT_PRIORITY'
```

如果 CubeMX 重命名了 GPIO、SPI、UART、CAN 或 PWM 对象，`xrobot gen` 会指出旧硬件名不存在。先用以下命令查看候选对象，再修改对应实例字段：

```powershell
xrobot -C $XrDir describe
xrobot -C $XrDir instance set bmi088 args.cs_gyro IMU_GYRO_CS
xrobot -C $XrDir gen
```

## 8. CMake 清理、重配和诊断

以下情况建议使用 `--fresh`：

- CubeMX 重新生成了工程；
- 修改了根目录或目标目录的 `CMakeLists.txt`；
- 修改了工具链或 `cmake/` 中的平台集成；
- 添加、删除或更新了 XRobot 模块；
- CMake 缓存与当前源码结构不一致。

重新配置并构建：

```powershell
cmake --preset Debug --fresh
cmake --build --preset Debug --parallel
```

调用 clean 目标后重建：

```powershell
cmake --build --preset Debug --target clean
cmake --build --preset Debug --parallel
```

显示完整编译器和链接器命令：

```powershell
cmake --build --preset Debug --verbose
```

列出根构建可用目标：

```powershell
cmake --build --preset Debug --target help
```

## 9. 检查固件

把固件路径保存到变量：

```powershell
$Elf = Join-Path $PWD "build\Debug\firmware\$Target.elf"
$Map = Join-Path $PWD "build\Debug\firmware\$Target.map"
```

查看段大小：

```powershell
arm-none-eabi-size $Elf
```

确认入口符号存在：

```powershell
arm-none-eabi-nm -C $Elf |
  Select-String ' app_main$|StartDefaultTask'
```

检查 DMA 段和链接 map：

```powershell
arm-none-eabi-objdump -h $Elf |
  Select-String 'dma_buffer'

Select-String -Path $Map -Pattern 'dma_buffer'
```

不是所有目标都必须定义 `.dma_buffer`；应以该目标的链接脚本和硬件配置为准。

## 10. 克隆后初始化

克隆仓库及 Git 子模块：

```powershell
$RepositoryUrl = '请替换为仓库地址'
git clone --recurse-submodules $RepositoryUrl my_sentry
Set-Location .\my_sentry
```

如果已经完成普通克隆：

```powershell
git submodule update --init --recursive
```

分别恢复两个目标的 XRobot 模块：

```powershell
'my_chassis', 'my_gimbal' | ForEach-Object {
  xrobot -C (Join-Path $PWD "src\targets\$_\xr") setup --frozen
}
```

仅当生成文件缺失或硬件配置需要重新生成时，再针对相应目标执行第 6 或第 7 节的 LibXR 流程。最后构建：

```powershell
cmake --preset Debug --fresh
cmake --build --preset Debug --parallel
```

## 11. Git 日常检查

```powershell
git status --short
git diff
git diff --cached
git submodule status --recursive
```

`lib/LibXR` 和 `lib/libht` 是 Git 子模块；`src/targets/*/xr/Modules/` 是 XRobot 管理的模块目录。业务代码应放在项目自己的 `src/`、`task/` 或 `xr/User/` 等目录，不要直接在依赖目录中开发。

## 12. 场景速查表

| 发生的变化 | 应执行的命令 |
|---|---|
| 普通 C/C++ 代码 | `cmake --build --preset Debug --parallel` |
| 底盘 TOML、任务表或 Topic Monitor | 直接构建，CMake 会自动生成 |
| `xr/User/xrobot.yaml` | `xrobot -C $XrDir gen`，然后构建目标 |
| 添加、删除或修改 XRobot 实例 | `xrobot instance ...`，再 `xrobot gen` 和构建 |
| 模块接口发生变化 | `xrobot sync`，再 `xrobot gen` 和构建 |
| `xr/User/libxr_config.yaml` | `libxr gen`，再 `xrobot gen` 和构建 |
| CubeMX `.ioc` 或 HAL 外设配置 | CubeMX Generate Code，再 `libxr parse`、`libxr gen`、`xrobot sync`、`xrobot gen`、完整重配构建 |
| 添加、删除或更新 XRobot 模块 | `xrobot setup`，再 `cmake --preset Debug --fresh` 和构建 |
| CMake 或工具链文件 | `cmake --preset Debug --fresh`，再构建 |

## 13. 常见错误

### 找不到 `Modules/modules.yaml`

原因通常是仍在仓库根目录执行旧命令：

```powershell
xrobot -C . describe
```

改为：

```powershell
xrobot -C $XrDir describe
```

### `xrobot_main.hpp is stale`

```powershell
xrobot -C $XrDir gen
cmake --build --preset Debug --target "${Target}_firmware" --parallel
```

### 模块目录与锁文件不一致

```powershell
xrobot -C $XrDir setup --frozen
```

### XRobot 找不到硬件对象

```powershell
xrobot -C $XrDir describe
```

根据输出中的实际候选名称修正 `User/xrobot.yaml`，再执行 `xrobot gen`。

### 生成或编译时路径错误

先确认当前目录和目标变量：

```powershell
Get-Location
$Target
$HalDir
$XrDir
Test-Path (Join-Path $XrDir 'Modules\modules.yaml')
Test-Path (Join-Path $HalDir '.config.yaml')
```
