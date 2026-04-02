#include "ODrive.hpp"

ODrive::ODrive(uint8_t devId) : deviceId(devId) {
    dutyCycleNow = 0;
    avgMotorCurrent = 0;
    erpm = 0;
    WattHours = 0;
    tempFET = 0;
    tempMotor = 0;
    avgInputCurrent = 0;
    inpVoltage = 0;
}

void ODrive::feedMsg(twai_message_t& msg) {
    uint32_t node = msg.identifier >> 5;
    uint32_t cmd = msg.identifier & 0x1F;
    if (node != deviceId) return;

    if (cmd == 0x17) {
        memcpy(&inpVoltage, &msg.data[0], 4);
        memcpy(&avgInputCurrent, &msg.data[4], 4);
    }
}

void ODrive::requestVoltage() {
    twai_message_t request;
    request.identifier = (deviceId << 5) | 0x17;
    request.data_length_code = 0;
    request.rtr = 1;
    twai_transmit(&request, pdMS_TO_TICKS(10));
}

// void ODrive::set_duty(float duty) {
//     uint32_t set_value = duty * 100000;
//     uint8_t buffer[4];
//     buffer[0] = (set_value >> 24) & 0xFF;
//     buffer[1] = (set_value >> 16) & 0xFF;
//     buffer[2] = (set_value >> 8) & 0xFF;
//     buffer[3] = set_value & 0xFF;
//     uint32_t canId = 0b00000 | deviceId;
// //    CAN0.sendMsgBuf(canId, 1, 4, buffer);

// }

// void ODrive::set_current(float current) {
//     uint32_t set_value = current * 1000;
//     uint8_t buffer[4];
//     buffer[0] = (set_value >> 24) & 0xFF;
//     buffer[1] = (set_value >> 16) & 0xFF;
//     buffer[2] = (set_value >> 8) & 0xFF;
//     buffer[3] = set_value & 0xFF;
//     uint32_t canId = 0x00000100 | deviceId;
// //    CAN0.sendMsgBuf(canId, 1, 4, buffer);
// }

// void ODrive::set_erpm(float eerpm) {
//     uint32_t set_value = eerpm * 30;
//     uint8_t buffer[4];
//     buffer[0] = (set_value >> 24) & 0xFF;
//     buffer[1] = (set_value >> 16) & 0xFF;
//     buffer[2] = (set_value >> 8) & 0xFF;
//     buffer[3] = set_value & 0xFF;
//     uint32_t canId = 0x00000300 | deviceId;
//     // CAN0.sendMsgBuf(canId, 1, 4, buffer);
// }