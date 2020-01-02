// This is the library for the TB6612 that contains the class Motor and all the
// functions
#include <SparkFun_TB6612.h>
#include "board.h"

const float max_rad_per_s = TWO_PI*1.5;

// these constants are used to allow you to make your motor configuration 
// line up with function names like forward.  Value can be 1 or -1
const int offsetA = 1;
const int offsetB = 1;

// Initializing motors.  The library will allow you to initialize as many
// motors as you have memory for.  If you are using functions like forward
// that take 2 motors as arguements you can either write new functions or
// call the function more than once.
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

//radians per second
void Set_MotorLeft_RadialSpeed(float RadPerS){
     float speed = RadPerS * 255 / max_rad_per_s;
     motor2.drive(speed,0);
}
void Set_MotorRight_RadialSpeed(float RadPerS){
     float speed = RadPerS * 255 / max_rad_per_s;
     motor1.drive(speed,0);
}

void loop_motor()
{

 
}
