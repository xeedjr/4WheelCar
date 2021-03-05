#ifndef BUSINESS_LOGIC_H_
#define BUSINESS_LOGIC_H_

#include "BusinessLogicBase.h"
#include "orion_protocol/orion_minor.h"
#include <stdint.h>
#include <stdbool.h>
#include <cstddef>
#include "qpcpp.h"
#include "Motor.h"
#include "SensorsInterface.h"
#include "MotorInterface.h"

#define COMMAND_BUFFER_SIZE (1024)

namespace business_logic
{

typedef struct
{
    int32_t alpha;
    int32_t beta;
    int32_t gamma;
} ImuData;

typedef struct
{
    int32_t left;
    int32_t right;
} EncodersData;

typedef struct
{
    float left;
    float right;
} USData;

typedef struct
{
    float left;
    float right;
} WheelPosData;

typedef struct
{
    float left;
    float right;
} WheelSpeedData;

struct Event : public QP::QEvt {
    union
    {
        ImuData imu;
        EncodersData encoders;
        USData us_data;
        WheelPosData wheel_pos_data;
        WheelSpeedData wheel_speed_data;
    } data;

    Event(QP::QSignal const s) : QEvt(s) {};
};

class BusinessLogic : public BusinessLogicBase, public sensors::SensorsInterface, public motor::MotorInterface
{
public:
    BusinessLogic(orion::Minor *minor);
    BusinessLogic() = delete;
    BusinessLogic& operator=(const BusinessLogic& object) = delete;
    BusinessLogic(const BusinessLogic& object) = delete;

    void startAO();

    virtual void update_Sensors_cb(float, float, float);
    virtual void us_sensor_cb(float*, uint8_t);
    virtual void tof_sensors_cb(float*, uint8_t);

    virtual void wheel_position_cb(double*, uint8_t);
    virtual void wheel_curr_speed_cb(double*, uint8_t);


    void setEncoders(int32_t left, int32_t right);

    void sendNewCommandEvent();

    void setMotor(motor::Motor *motor);

protected:
    virtual void setImuHandler(QP::QEvt const * e);
    virtual void setEncodersHandler(QP::QEvt const * e);
    virtual void commandHandler();
    virtual void setUSHandler(QP::QEvt const * e);
    virtual void setWheelsPosHandler(QP::QEvt const * e);
    virtual void setWheelsSpeedHandler(QP::QEvt const * e);
    virtual void heartBeat(QP::QEvt const * e);

    void process_handshake_receive(void);
    void process_set_commands_receive(void);
    void process_set_pid_receive(void);

private:
    uint8_t stack[3*1024];    ///< QP AO Stack
    QP::QEvt const *queueSto[10];    ///< QP AO message queue

    orion::Minor *minor_ = NULL;
    motor::Motor *motor_ = NULL;

    int32_t encoder_left_ = 0;
    int32_t encoder_right_ = 0;

    int32_t imu_angle_alpha_ = 0;
    int32_t imu_angle_beta_ = 0;
    int32_t imu_angle_gamma_ = 0;

    /// US distance in cm.
    float us_left = 0;
    float us_right = 0;

    /// Wheel current position in -+ radians.
    float wheel_pos_left = 0;
    float wheel_pos_right = 0;

    float wheel_speed_left = 0;
    float wheel_speed_right = 0;

    uint8_t command_buffer_[COMMAND_BUFFER_SIZE];
    std::size_t command_size_ = 0;
};

} // namespace business_logic

#endif /* BUSINESS_LOGIC_H_ */
