硬件参数

遥控器

话题在线监测

```powershell
openocd `
  -f interface/cmsis-dap.cfg `
  -c "transport select swd" `
  -f target/stm32h7x.cfg `
  -c "adapter speed 1000" `
  -c "program D:/stm32_workSpace/preparation_for_the_27RMUL/my_sentry/build/Debug/firmware/my_gimbal.elf verify reset exit"
```

```powershell
openocd `
  -f interface/cmsis-dap.cfg `
  -c "transport select swd" `
  -f target/stm32h7x.cfg `
  -c "adapter speed 1000" `
  -c "program D:/stm32_workSpace/preparation_for_the_27RMUL/my_sentry/build/Debug/firmware/my_chassis.elf verify reset exit"
```

```powershell
openocd `
  -f interface/cmsis-dap.cfg `
  -c "transport select swd" `
  -f target/stm32h7x.cfg `
  -c "adapter speed 1000" `
  -c "program C:/Users/29820/Downloads/gimbal.elf verify reset exit"
```

```powershell
  openocd `
  -f interface/cmsis-dap.cfg `
  -c "transport select swd" `
  -f target/stm32h7x.cfg `
  -c "adapter speed 1000" `
  -c "program C:/Users/29820/Downloads/chassis.elf verify reset exit"
```


```powershell
  openocd `
  -f interface/cmsis-dap.cfg `
  -c "transport select swd" `
  -f target/stm32h7x.cfg `
  -c "adapter speed 1000" `
  -c "program D:/ShaoBing/steering_sentry-main-a2746ccc4aef2de4d204291df405944f465e870a/build/gimbal-firmware/firmware/gimbal.elf verify reset exit"
```

  首次编译或需要完全重新编译：
```powershell
Set-Location "D:\ShaoBing\steering_sentry-main-a2746ccc4aef2de4d204291df405944f465e870a"

$env:ZEPHYR_BASE = "D:\Zephyr\zephyrproject\deps\zephyr"
$env:ZEPHYR_SDK_INSTALL_DIR = "D:\Zephyr\zephyr-sdk-1.0.1"
$env:PATH = "C:\Users\29820\.zinstaller\tools\cmake\bin;C:\Users\29820\.zinstaller\tools\ninja;C:\Users\29820\.zinstaller\tools\dtc\usr\bin;C:\Users\29820\.zinstaller\tools\gperf\bin;C:\Users\29820\.zinstaller\.venv\Scripts;$env:PATH"

& "C:\Users\29820\.zinstaller\.venv\Scripts\west.exe" build `
  --pristine always `
  -b mc_02 `
  -d build/gimbal-firmware `
  . `
  -- `
  -DAPP_TARGET=gimbal `
  -DAPP_BUILD_BOTH=OFF `
  -DPython3_EXECUTABLE="C:/Users/29820/.zinstaller/.venv/Scripts/python.exe" `
  -DAPP_EIGEN_INCLUDE_DIR="D:/ShaoBing/steering_sentry-main-a2746ccc4aef2de4d204291df405944f465e870a/.cache/deps/eigen-3.4.0" `
  -DCONFIG_BUILD_OUTPUT_BIN=y `
  -DCONFIG_BUILD_OUTPUT_HEX=y
```

以后修改代码或 TOML 参数后，可以增量编译：
```powershell
& "C:\Users\29820\.zinstaller\.venv\Scripts\west.exe" build -d build/gimbal-firmware
```
