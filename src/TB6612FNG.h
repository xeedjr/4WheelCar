#pragma once

#include "hal.h"


// 64kHz  for FCPU 16MHz
// #define PERIOD  250U
// 
// static PWMConfig pwm3cfg = {
//   F_CPU,                            /* PWM frequency.         */ 
//   PERIOD,                           /* PWM period.            */
//   NULL,                             /* TODO: comment.         */
//   {
//     {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* PWM channel 1 actived. */
//     {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* PWM channel 2 actived. */
//     {PWM_OUTPUT_ACTIVE_HIGH, NULL}, /* PWM channel 3 actived. */
//   },
// };
//
//	  palSetLineMode(LINE_TB66_PWMA, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
//	  palSetLineMode(LINE_TB66_AIN2, PAL_MODE_OUTPUT_PUSHPULL);
//	  palSetLineMode(LINE_TB66_AIN1, PAL_MODE_OUTPUT_PUSHPULL);
//	  palSetLineMode(LINE_TB66_PWMB, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
//	  palSetLineMode(LINE_TB66_BIN2, PAL_MODE_OUTPUT_PUSHPULL);
//	  palSetLineMode(LINE_TB66_BIN1, PAL_MODE_OUTPUT_PUSHPULL);
//	  palSetLineMode(LINE_TB66_STBY, PAL_MODE_OUTPUT_PUSHPULL);

class TB6612FNG  {
public:
  enum Mode {
    kShortBrake,
    kCCW,
    kCW,
    kStop,
    kStandby
  };
  enum Channels {
    kA = 0,
    kB = 1,
    kCount = 2,
  };

private:
  struct Channel {
    ioline_t pwm, in1, in2;
    PWMDriver *pwmp;
    pwmchannel_t channel;
  };
  struct Channel chn[Channels::kCount];
  ioline_t stby;

  void _set_drive_speed(Channels ch, float pwm);

public:
  TB6612FNG() {};
  void init (ioline_t pwma, ioline_t ain2, ioline_t ain1,
                ioline_t pwmb, ioline_t bin2, ioline_t bin1,
                ioline_t stby,
                PWMDriver *apwmp, pwmchannel_t achannel,
                PWMDriver *bpwmp, pwmchannel_t bchannel);
                
  void drive(Channels ch, Mode mode, float pwm);
};
