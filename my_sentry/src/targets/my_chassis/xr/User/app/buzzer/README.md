接入时，在 app_main.cpp 的用户代码区添加：

```
#include "app/buzzer/you_buzzer.hpp"
```

然后在 /* User Code Begin 3 */ 中使用已经存在的 pwm_tim12_ch2：

```
static UserApp::Buzzer::YouBuzzer you_buzzer(pwm_tim12_ch2);
```

// 需要播放时调用

```
you_buzzer.Play();
```

需要停止时：

```
you_buzzer.Stop();
```

播放中再次调用 Play() 会从头重新播放。