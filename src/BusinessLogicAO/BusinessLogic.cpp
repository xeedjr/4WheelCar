#include "BusinessLogic.h"
#include <assert.h>
#include "protocol.h"
#include "qpcpp.h"
#include "logging.h"

namespace business_logic {

BusinessLogic::BusinessLogic(orion::Minor *minor): BusinessLogicBase(), minor_(minor)
{
    setAttr(QP::TASK_NAME_ATTR, "BL");
}

void BusinessLogic::startAO() {
    start(10U, // priority
                 queueSto, Q_DIM(queueSto),
#ifndef WIN32
                 stack, sizeof(stack)); // no stack
#else
                 nullptr, 0); // no stack
#endif
}

void BusinessLogic::update_Sensors_cb(float y, float p, float r)
{
    auto event = Q_NEW(Event, BL_SET_IMU_SIG);
    event->data.imu.alpha = (int32_t)(y * 1000);
    event->data.imu.beta = (int32_t)(p * 1000);
    event->data.imu.gamma = (int32_t)(r * 1000);
    POST(event, this);
}

void BusinessLogic::us_sensor_cb(float* data, uint8_t num) {
    auto event = Q_NEW(Event, BL_SET_US_SIG);
    event->data.us_data.left = data[0];
    event->data.us_data.right = data[1];
    POST(event, this);
}

void BusinessLogic::tof_sensors_cb(float* data, uint8_t num) {

}

void BusinessLogic::wheel_position_cb(double* data, uint8_t num) {
    auto event = Q_NEW(Event, BL_SET_WHEEL_POS_SIG);
    event->data.wheel_pos_data.left = data[0];
    event->data.wheel_pos_data.right = data[1];
    POST(event, this);
}

void BusinessLogic::wheel_curr_speed_cb(double* data, uint8_t num) {
    auto event = Q_NEW(Event, BL_SET_WHEEL_SPEED_SIG);
    event->data.wheel_speed_data.left = data[0];
    event->data.wheel_speed_data.right = data[1];
    POST(event, this);
}

void BusinessLogic::setEncoders(int32_t left, int32_t right)
{
    auto event = Q_NEW(Event, BL_SET_ENCODERS_SIG);
    event->data.encoders.left = left;
    event->data.encoders.right = right;
    POST(event, this);
}

void BusinessLogic::sendNewCommandEvent()
{
    auto event = Q_NEW_FROM_ISR(Event, BL_COMMAND_SIG);
    POST_FROM_ISR(event, nullptr, this);
}

void BusinessLogic::setMotor(motor::Motor *motor)
{
    assert(nullptr != motor);
    this->motor_ = motor;
}

void BusinessLogic::process_handshake_receive(void)
{
    assert(nullptr != this->command_buffer_);
    assert(this->command_size_ >= sizeof(orion::CommandHeader));
    orion::CommandHeader * command_header = reinterpret_cast<orion::CommandHeader*>(this->command_buffer_);
    assert(carmen_hardware::MessageType::Handshake ==
            (carmen_hardware::MessageType)command_header->common.message_id);
    carmen_hardware::HandshakeResult handshake_result;
    handshake_result.header.common.sequence_id = command_header->common.sequence_id;
    LOG_DEBUG("BusinessLogic::process_handshake_receive Sending response to handshake\n");
    // TODO: Add code to validate that protocol versions coincide else send error code e.g. minor.validate method
    this->minor_->sendResult((uint8_t*)&handshake_result, sizeof(handshake_result));
}

void BusinessLogic::setImuHandler(QP::QEvt const * e)
{
    auto event = reinterpret_cast<Event const *>(e);

    assert(nullptr != event);
    this->imu_angle_alpha_ = event->data.imu.alpha; 
    this->imu_angle_beta_ = event->data.imu.beta; 
    this->imu_angle_gamma_ = event->data.imu.gamma; 
}

void BusinessLogic::setEncodersHandler(QP::QEvt const * e)
{
    auto event = reinterpret_cast<Event const *>(e);

    assert(nullptr != event);
    this->encoder_left_ = event->data.encoders.left; 
    this->encoder_right_ = event->data.encoders.right; 
}

void BusinessLogic::setUSHandler(QP::QEvt const * e)
{
    auto event = reinterpret_cast<Event const *>(e);
    assert(nullptr != event);

    us_left = event->data.us_data.left;
    us_right = event->data.us_data.right;
}

void BusinessLogic::setWheelsPosHandler(QP::QEvt const * e)
{
    auto event = reinterpret_cast<Event const *>(e);
    assert(nullptr != event);

    wheel_pos_left = event->data.wheel_pos_data.left;
    wheel_pos_right = event->data.wheel_pos_data.right;

}

void BusinessLogic::setWheelsSpeedHandler(QP::QEvt const * e)
{
    auto event = reinterpret_cast<Event const *>(e);
    assert(nullptr != event);

    wheel_speed_left = event->data.wheel_speed_data.left;
    wheel_speed_right = event->data.wheel_speed_data.right;
}

void BusinessLogic::heartBeat(QP::QEvt const * e)
{
    auto event = reinterpret_cast<Event const *>(e);

    LOG_DEBUG("BL Heart Beat 5sec.\n");
}

void BusinessLogic::process_set_commands_receive(void)
{
    assert(nullptr != this->command_buffer_);
    assert(this->command_size_ >= sizeof(carmen_hardware::SetCommandsCommand));
    carmen_hardware::SetCommandsCommand * command =
            reinterpret_cast<carmen_hardware::SetCommandsCommand*>(this->command_buffer_);
    assert(carmen_hardware::MessageType::SetCommands ==
            (carmen_hardware::MessageType)command->header.common.message_id);

    this->motor_->SetSpeedL(command->left_cmd / 1000.0);
    this->motor_->SetSpeedR(command->right_cmd / 1000.0);

    carmen_hardware::SetCommandsResult reply;
    reply.header.common.sequence_id = command->header.common.sequence_id;

    reply.imu_angle_alpha = this->imu_angle_alpha_;
    reply.imu_angle_beta = this->imu_angle_beta_;
    reply.imu_angle_gamma = this->imu_angle_gamma_;

    reply.ultra_sonic_left = this->us_left * 1000;
    reply.ultra_sonic_right = this->us_right * 1000;

    reply.wheel_pos_left = this->wheel_pos_left * 1000;
    reply.wheel_pos_right = this->wheel_pos_right * 1000;

    reply.wheel_vel_left = this->wheel_speed_left * 1000;
    reply.wheel_vel_right = this->wheel_speed_right * 1000;

    // TODO: Add code to validate that protocol versions coincide else send error code e.g. minor.validate method
    this->minor_->sendResult((uint8_t*)&reply, sizeof(reply));
}

void BusinessLogic::process_set_pid_receive(void)
{
    assert(nullptr != this->command_buffer_);
    assert(this->command_size_ >= sizeof(carmen_hardware::SetPIDCommand));
    carmen_hardware::SetPIDCommand * command =
            reinterpret_cast<carmen_hardware::SetPIDCommand*>(this->command_buffer_);
    assert(carmen_hardware::MessageType::SetPID ==
            (carmen_hardware::MessageType)command->header.common.message_id);

    carmen_hardware::SetPIDResult reply;
    reply.header.common.sequence_id = command->header.common.sequence_id;

    // TODO: Implement
    // reply.result = this->motor_->setPid(command->left_p, command->left_i, command->left_d, command->right_p, command->right_i, command->right_d);

    // TODO: Add code to validate that protocol versions coincide else send error code e.g. minor.validate method
    this->minor_->sendResult((uint8_t*)&reply, sizeof(reply));
}

void BusinessLogic::commandHandler()
{
    LOG_DEBUG("BusinessLogic::commandHandler Insider command handler\n");

    if (this->minor_->receiveCommand(this->command_buffer_, COMMAND_BUFFER_SIZE, this->command_size_))
    {
        LOG_DEBUG("BusinessLogic::commandHandler Command received\n");
        assert(nullptr != this->command_buffer_);
        assert(this->command_size_ >= sizeof(orion::CommandHeader));
        orion::CommandHeader * command_header = reinterpret_cast<orion::CommandHeader*>(this->command_buffer_);
        switch (static_cast<carmen_hardware::MessageType>(command_header->common.message_id))
        {
            case carmen_hardware::MessageType::Handshake:
                LOG_DEBUG("BusinessLogic::commandHandler Hanshake received\n");
                process_handshake_receive();
                break;
            case carmen_hardware::MessageType::SetCommands:
                process_set_commands_receive();
                break;
            case carmen_hardware::MessageType::SetPID:
                process_set_pid_receive();
                break;
            default:
                assert(false);
        }
    }
}

} // namespace business_logic
