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

#ifndef _BOARD_H_
#define _BOARD_H_

#include "hal.h"

/*
 * Setup for the Arduino Mega board.
 */

/*
 * Board identifier.
 */
#define BOARD_ARDUINO_MEGA
#define BOARD_NAME "Arduino Mega"


/*
 * IO lines assignments.
 */
#define LINE_LED1   PAL_LINE(IOPORT2, 7U)

#define ENCODE_LEFT_LINE    PAL_LINE(IOPORT2, 5U)
#define ENCODE_RIGHT_LINE   PAL_LINE(IOPORT2, 6U)

#define LINE_TB66_PWMA   PAL_LINE(IOPORT5, 3U)
#define LINE_TB66_PWMB   PAL_LINE(IOPORT5, 4U)  
#define LINE_TB66_AIN1   PAL_LINE(IOPORT1, 0U)
#define LINE_TB66_AIN2   PAL_LINE(IOPORT1, 1U)
#define LINE_TB66_BIN1   PAL_LINE(IOPORT1, 2U)
#define LINE_TB66_BIN2   PAL_LINE(IOPORT1, 3U)
#define LINE_TB66_STBY   PAL_LINE(IOPORT1, 4U)
#define PWM_CHAN_TB66_A  ((pwmchannel_t)0)
#define PWM_CHAN_TB66_B  ((pwmchannel_t)1)

#define DEBUG_UART_RX_LINE    PAL_LINE(IOPORT5, 0U)
#define DEBUG_UART_TX_LINE    PAL_LINE(IOPORT5, 1U)
#define DEBUG_UART_DRIVE      SD1

#define RASPBERY_UART_RX_LINE    PAL_LINE(IOPORT4, 2U)
#define RASPBERY_UART_TX_LINE    PAL_LINE(IOPORT4, 3U)
#define RASPBERY_UART_DRIVE       SD2


/*
 * Port A setup.
 * All inputs with pull-ups.
 */
#define VAL_DDRA    0x00
#define VAL_PORTA   0xFF

/*
 * Port B setup.
 * All inputs except PB7 which has a LED connected.
 */
#define VAL_DDRB    0x80
#define VAL_PORTB   0xFF

/*
 * Port C setup.
 * All inputs with pull-ups.
 */
#define VAL_DDRC    0x00
#define VAL_PORTC   0xFF

/*
 * Port D setup.
 * All inputs with pull-ups.
 */
#define VAL_DDRD    0x00
#define VAL_PORTD   0xFF

/*
 * Port E setup.
 * All inputs except PE1 (Serial TX0).
 */
#define VAL_DDRE    0x02
#define VAL_PORTE   0xFF

/*
 * Port F setup.
 * All inputs with pull-ups.
 */
#define VAL_DDRF    0x00
#define VAL_PORTF   0xFF

/*
 * Port G setup.
 * All inputs with pull-ups.
 */
#define VAL_DDRG    0x00
#define VAL_PORTG   0xFF

/*
 * Port H setup.
 * All inputs with pull-ups.
 */
#define VAL_DDRH    0x00
#define VAL_PORTH   0xFF

/*
 * Port J setup.
 * All inputs with pull-ups.
 */
#define VAL_DDRJ    0x00
#define VAL_PORTJ   0xFF

/*
 * Port K setup.
 * All inputs with pull-ups.
 */
#define VAL_DDRK    0x00
#define VAL_PORTK   0xFF

/*
 * Port L setup.
 * All inputs with pull-ups.
 */
#define VAL_DDRL    0x00
#define VAL_PORTL   0xFF


#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
