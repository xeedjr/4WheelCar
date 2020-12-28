/*
 * IMUInterface.h
 *
 *  Created on: 28 груд. 2020 р.
 *      Author: Bogdan
 */

#ifndef IMUINTERFACE_H_
#define IMUINTERFACE_H_

class IMUInterface {
public:

    virtual void update_data(float, float, float) = 0;
};

#endif /* IMUINTERFACE_H_ */
