/*
 * MotorInterface.h
 *
 *  Created on: 10 лют. 2021 р.
 *      Author: Bogdan
 */

#pragma once

namespace motor {

class MotorInterface {
public:
    virtual void wheel_position_cb(double*, uint8_t) = 0;
};

} /* namespace motor */

