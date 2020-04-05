#include "TB6612FNG.h"

/// Copy because Chibios  calculate with overflow for uint16_t
#define TB6612FNG_PWM_FRACTION_TO_WIDTH(pwmp, denominator, numerator)                 \
          ((pwmcnt_t)((((uint32_t)(pwmp)->period) *                                 \
          (uint32_t)(numerator)) / (uint32_t)(denominator)))

void TB6612FNG::init(ioline_t pwma, ioline_t ain2, ioline_t ain1,
                        ioline_t pwmb, ioline_t bin2, ioline_t bin1,
                        ioline_t stby, 
                        PWMDriver *apwmp, pwmchannel_t achannel,
                        PWMDriver *bpwmp, pwmchannel_t bchannel)
{
  chn[kA].pwm = pwma;
  chn[kA].in2 = ain2;          
  chn[kA].in1 = ain1;
  chn[kA].pwmp = apwmp;
  chn[kA].channel = achannel;

  chn[kB].pwm = pwmb;
  chn[kB].in2 = bin2;
  chn[kB].in1 = bin1;
  chn[kB].pwmp = bpwmp;
  chn[kB].channel = bchannel;

  this->stby = stby;

  palSetLineMode(pwma, PAL_MODE_OUTPUT_PUSHPULL);
  palSetLineMode(ain2, PAL_MODE_OUTPUT_PUSHPULL);
  palSetLineMode(ain1, PAL_MODE_OUTPUT_PUSHPULL);
  palSetLineMode(pwmb, PAL_MODE_OUTPUT_PUSHPULL);
  palSetLineMode(bin2, PAL_MODE_OUTPUT_PUSHPULL);
  palSetLineMode(bin1, PAL_MODE_OUTPUT_PUSHPULL);
  palSetLineMode(stby, PAL_MODE_OUTPUT_PUSHPULL);
  
  palClearLine(stby);
  palClearLine(pwma);
  palClearLine(ain2);
  palClearLine(ain1);
  palClearLine(pwmb);
  palClearLine(bin2);
  palClearLine(bin1);
  
  
  pwmDisableChannel(apwmp, achannel);
  pwmDisableChannel(bpwmp, bchannel);
}

void TB6612FNG::_set_drive_speed(Channels ch, float pwm)
{
  if ((pwm > 100.0) || (pwm < 0.0)) {
    /// error
    chSysHalt(__FUNCTION__);
  }
  pwmEnableChannel(chn[ch].pwmp, chn[ch].channel, TB6612FNG_PWM_FRACTION_TO_WIDTH(chn[ch].pwmp, (10000), (pwm*100.0)));
}
  
void TB6612FNG::drive(Channels ch, Mode mode, float pwm)
{
  if ((pwm > 100.0) || (pwm < 0.0)) {
    /// error
    chSysHalt(__FUNCTION__);
  }
    
  palSetLine(stby);
  
  switch (mode) {
    case kCW:
      palSetLine(chn[ch].in1);
      palClearLine(chn[ch].in2);
      _set_drive_speed(ch, pwm);
    break;
    case kCCW:
      palClearLine(chn[ch].in1);
      palSetLine(chn[ch].in2);
      _set_drive_speed(ch, pwm);
    break;
    case kShortBrake:
      pwmDisableChannel(chn[ch].pwmp, chn[ch].channel);
      palClearLine(chn[ch].pwm);
      palSetLine(chn[ch].in1);
      palSetLine(chn[ch].in2);
    break;
  }
}  