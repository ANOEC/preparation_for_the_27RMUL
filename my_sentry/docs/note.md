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

openocd `
  -f interface/cmsis-dap.cfg `
  -c "transport select swd" `
  -f target/stm32h7x.cfg `
  -c "adapter speed 1000" `
  -c "program D:/stm32_workSpace/preparation_for_the_27RMUL/my_sentry/build/Debug/firmware/my_chassis.elf verify reset exit"