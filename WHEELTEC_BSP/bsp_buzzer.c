#include "bsp_buzzer.h"
#include "gpio.h"

static void BuzzerOn(void)
{
    HAL_GPIO_WritePin(User_BUZZER_GPIO_Port,User_BUZZER_Pin,GPIO_PIN_SET);
}

static void BuzzerOff(void)
{
    HAL_GPIO_WritePin(User_BUZZER_GPIO_Port,User_BUZZER_Pin,GPIO_PIN_RESET);
}

static void Buzzeroggle(void)
{
    HAL_GPIO_TogglePin(User_BUZZER_GPIO_Port,User_BUZZER_Pin);
}

BuzzerInterface_t UserBuzzer = {
    .on = BuzzerOn,
    .off = BuzzerOff,
    .toggle = Buzzeroggle
};

