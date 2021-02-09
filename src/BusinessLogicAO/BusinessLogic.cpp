#include "BusinessLogic.h"
#include <assert.h>
#include "protocol.h"
#include "qpcpp.h"

namespace business_logic {

BusinessLogic::BusinessLogic(orion::Minor *minor): BusinessLogicBase(), minor_(minor)
{
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

void BusinessLogic::update_data(float alpha, float beta, float gamma)
{
	auto event = Q_NEW(Event, BL_SET_IMU_SIG);
	event->data.imu.alpha = (int32_t)(alpha * 1000);
	event->data.imu.beta = (int32_t)(beta * 1000);
	event->data.imu.gamma = (int32_t)(gamma * 1000);
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
	auto event = Q_NEW(Event, BL_COMMAND_SIG);
	POST(event, this);
}

void BusinessLogic::setMotor(motor::Motor *motor)
{
	assert(NULL != motor);
	this->motor_ = motor;
}

void BusinessLogic::process_handshake_receive(void)
{
    assert(NULL != this->command_buffer_);
    assert(this->command_size_ >= sizeof(orion::CommandHeader));
    orion::CommandHeader * command_header = reinterpret_cast<orion::CommandHeader*>(this->command_buffer_);
    assert(carmen_hardware::MessageType::Handshake ==
            (carmen_hardware::MessageType)command_header->common.message_id);
    carmen_hardware::HandshakeResult handshake_result;
    handshake_result.header.common.sequence_id = command_header->common.sequence_id;
    // TODO: Add code to validate that protocol versions coincide else send error code e.g. minor.validate method
    this->minor_->sendResult((uint8_t*)&handshake_result, sizeof(handshake_result));
}

void BusinessLogic::setImuHandler(Event const* event)
{
    assert(NULL != event);
    this->imu_angle_alpha_ = event->data.imu.alpha; 
    this->imu_angle_beta_ = event->data.imu.beta; 
    this->imu_angle_gamma_ = event->data.imu.gamma; 
}

void BusinessLogic::setEncodersHandler(Event const* event)
{
    assert(NULL != event);
    this->encoder_left_ = event->data.encoders.left; 
    this->encoder_right_ = event->data.encoders.right; 
}

void BusinessLogic::process_set_commands_receive(void)
{
    assert(NULL != this->command_buffer_);
    assert(this->command_size_ >= sizeof(carmen_hardware::SetCommandsCommand));
    carmen_hardware::SetCommandsCommand * command =
            reinterpret_cast<carmen_hardware::SetCommandsCommand*>(this->command_buffer_);
    assert(carmen_hardware::MessageType::SetCommands ==
            (carmen_hardware::MessageType)command->header.common.message_id);

    this->motor_->SetSpeedL(command->left_cmd / 1000.0);
    this->motor_->SetSpeedR(command->right_cmd / 1000.0);

    carmen_hardware::SetCommandsResult reply;
    reply.header.common.sequence_id = command->header.common.sequence_id;
    reply.encoder_left = this->encoder_left_;
    reply.encoder_right = this->encoder_right_;
    reply.imu_angle_alpha = this->imu_angle_alpha_;
    reply.imu_angle_beta = this->imu_angle_beta_;
    reply.imu_angle_gamma = this->imu_angle_gamma_;
    // TODO: Add code to validate that protocol versions coincide else send error code e.g. minor.validate method
    this->minor_->sendResult((uint8_t*)&reply, sizeof(reply));
}

void BusinessLogic::process_set_pid_receive(void)
{
    assert(NULL != this->command_buffer_);
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
    if (this->minor_->receiveCommand(this->command_buffer_, COMMAND_BUFFER_SIZE, this->command_size_))
    {
        assert(NULL != this->command_buffer_);
        assert(this->command_size_ >= sizeof(orion::CommandHeader));
        orion::CommandHeader * command_header = reinterpret_cast<orion::CommandHeader*>(this->command_buffer_);
        switch (static_cast<carmen_hardware::MessageType>(command_header->common.message_id))
        {
            case carmen_hardware::MessageType::Handshake:
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
