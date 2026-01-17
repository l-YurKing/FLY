#ifndef __BSP_BUZZER_H
#define __BSP_BUZZER_H

/*
* 使用说明：
* 已对外提供了3个接口：UserLed1、UserLed2、UserBuzzer
* 使用示例：想开启LED1灯
*    pLedInterface_t led1 = &UserLed1;
*    led1 -> on();  //开启LED1
*/

typedef struct {
    void (*on)(void);
    void (*off)(void);
    void (*toggle)(void);
}BuzzerInterface_t,*pBuzzeInterface_t;

extern BuzzerInterface_t UserBuzzer;

#endif /* __BSP_BUZZER_H */

