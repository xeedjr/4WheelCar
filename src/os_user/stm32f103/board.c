/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "hal.h"

/**
 * @brief   Driver default configuration.
 */
/*const SerialConfig sd1_config = {
  UBRR2x_F(115200),
  USART_CHAR_SIZE_8
};
*/

//64kHz
#define PERIOD  250U

PWMConfig pwm3cfg = {
  16000000,                            /* PWM frequency.         */
  PERIOD,                           /* PWM period.            */
  NULL,                             /* TODO: comment.         */
  {
    {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* PWM channel 1 actived. */
    {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* PWM channel 2 actived. */
    {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* PWM channel 3 actived. */
  },
};

/**
 * @brief   PAL setup.
 * @details Digital I/O ports static configuration as defined in @p board.h.
 *          This variable is used by the HAL when initializing the PAL driver.
 */
#if HAL_USE_PAL || defined(__DOXYGEN__)
const PALConfig pal_default_config =
{
  {VAL_GPIOAODR, VAL_GPIOACRL, VAL_GPIOACRH},
  {VAL_GPIOBODR, VAL_GPIOBCRL, VAL_GPIOBCRH},
  {VAL_GPIOCODR, VAL_GPIOCCRL, VAL_GPIOCCRH},
  {VAL_GPIODODR, VAL_GPIODCRL, VAL_GPIODCRH},
  {VAL_GPIOEODR, VAL_GPIOECRL, VAL_GPIOECRH},
};
#endif

/*
 * Early initialization code.
 * This initialization must be performed just after stack setup and before
 * any other initialization.
 */
void __early_init(void) {

  stm32_clock_init();
}

/*
 * Board-specific initialization code.
 */
void boardInit(void) {
	  palSetLineMode(LINE_TB66_PWMA, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
	  palSetLineMode(LINE_TB66_AIN2, PAL_MODE_OUTPUT_PUSHPULL);
	  palSetLineMode(LINE_TB66_AIN1, PAL_MODE_OUTPUT_PUSHPULL);
	  palSetLineMode(LINE_TB66_PWMB, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
	  palSetLineMode(LINE_TB66_BIN2, PAL_MODE_OUTPUT_PUSHPULL);
	  palSetLineMode(LINE_TB66_BIN1, PAL_MODE_OUTPUT_PUSHPULL);
	  palSetLineMode(LINE_TB66_STBY, PAL_MODE_OUTPUT_PUSHPULL);
}
