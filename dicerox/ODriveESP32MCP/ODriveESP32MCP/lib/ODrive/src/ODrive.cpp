#include "ODrive.hpp"

MCP2515* ODrive::_configuredBuses[4] = {nullptr, nullptr, nullptr, nullptr};
uint8_t  ODrive::_configuredBusCount = 0;

ODrive::ODrive(uint8_t devId, MCP2515* canBus) : deviceId(devId), _canBus(canBus) {
    busCurrent          = 0;
    busVoltage          = 0;
    iqSetpoint          = 0;
    iqMeasured          = 0;
    encEstPos           = 0;
    encEstVel           = 0;
    encShadowCount      = 0;
    encCountCPR         = 0;
    senlessEstPos       = 0;
    senlessEstVel       = 0;
    axisError           = 0;
    currentState        = 0;
    motorErrorFlag      = 0;
    encoderErrorFlag    = 0;
    controllerErrorFlag = 0;
    trajectoryDone      = 0;
    motorError          = 0;
    encoderError        = 0;
    sensorlessError     = 0;
    controllerError     = 0;
}

bool ODrive::_isBusConfigured() {
    for (uint8_t i = 0; i < _configuredBusCount; i++) {
        if (_configuredBuses[i] == _canBus) return true;
    }
    return false;
}

void ODrive::_markBusConfigured() {
    if (_configuredBusCount < 4) {
        _configuredBuses[_configuredBusCount++] = _canBus;
    }
}

bool ODrive::begin(CAN_SPEED speed, CAN_CLOCK clock) {
  if (_isBusConfigured()) {
        return true; 
    }

  _canBus->reset();
    if (_canBus->setBitrate(speed, clock) != MCP2515::ERROR_OK) {
        return false;
    }
    if (_canBus->setNormalMode() != MCP2515::ERROR_OK) {
        return false;
    }

    _markBusConfigured();
    return true;
}

void ODrive::feedMsg(const can_frame &frame) {
    uint32_t standardId = frame.can_id & 0x7FF;
    uint32_t node = standardId >> 5;
    uint32_t cmd = standardId & 0x1F;
    
    if (node != deviceId) return;
    
    switch (cmd) {
        case 0x01: {
            memcpy(&axisError, &frame.data[0], 4);
            currentState        = frame.data[4];
            motorErrorFlag      = bitRead(frame.data[5], 0);
            encoderErrorFlag    = bitRead(frame.data[6], 0);
            controllerErrorFlag = bitRead(frame.data[7], 0);
            trajectoryDone      = bitRead(frame.data[7], 7);
            break;
        }
        case 0x03: {
            memcpy(&motorError, &frame.data[0], 8);
            break;
        }
        case 0x04: {
            memcpy(&encoderError, &frame.data[0], 4);
            break;
        }
        case 0x05: {
            memcpy(&sensorlessError, &frame.data[0], 4);
            break;
        }
        case 0x09: {
            memcpy(&encEstPos, &frame.data[0], 4);
            memcpy(&encEstVel, &frame.data[4], 4);
            break;
        }
        case 0x14: {
            memcpy(&iqSetpoint, &frame.data[0], 4);
            memcpy(&iqMeasured, &frame.data[4], 4);
            break;
        }
        case 0x15: {
            memcpy(&senlessEstPos, &frame.data[0], 4);
            memcpy(&senlessEstVel, &frame.data[4], 4);
            break; 
        }
        case 0x17: {
            memcpy(&busVoltage, &frame.data[0], 4);
            memcpy(&busCurrent, &frame.data[4], 4);
            break;
        }
        case 0x1D: {
            memcpy(&controllerError, &frame.data[0], 4);
            break;
        }
        default: {
            break;
        }
    }
}

void ODrive::setAxisState(uint32_t state) {
    can_frame frame;
    frame.can_id = _makeCanId(0x07);
    frame.can_dlc = 4;
    memcpy(frame.data, &state, 4);
    _canBus->sendMessage(&frame);
}

void ODrive::setVelocity(float velocity, float torqueFF) {
    can_frame frame;
    frame.can_id = _makeCanId(0x0D);
    frame.can_dlc = 8;
    memcpy(&frame.data[0], &velocity, 4);
    memcpy(&frame.data[4], &torqueFF, 4);
    _canBus->sendMessage(&frame);
}

void ODrive::setRPMs(int rpm, float torqueFF) {
    float ts = rpm / 60;
    setVelocity(ts, torqueFF);
}
void ODrive::setControllerMode(uint32_t controlMode, uint32_t inputMode) {
    can_frame frame;
    frame.can_id = _makeCanId(0x0B);
    frame.can_dlc = 8;
    memcpy(&frame.data[0], &controlMode, 4);
    memcpy(&frame.data[4], &inputMode, 4);
    _canBus->sendMessage(&frame);
}

void ODrive::clearErrors() {
    can_frame frame;
    frame.can_id = _makeCanId(0x18);
    frame.can_dlc = 0;
    _canBus->sendMessage(&frame);
}

void ODrive::Estop() {
    _sendEmptyFrame(0x02);
}

void ODrive::requestVoltage() {
    _sendEmptyFrame(0x17, true);
}

void ODrive::_sendEmptyFrame(uint8_t cmdId, bool rtr) {
    can_frame frame;
    frame.can_id = _makeCanId(cmdId);
    if (rtr) frame.can_id |= CAN_RTR_FLAG;
    frame.can_dlc = 8;
    _canBus->sendMessage(&frame);
}

float ODrive::getEpos()    { return encEstPos; }
float ODrive::getEvel()    { return encEstVel; }
float ODrive::getErpms()   { return encEstVel * 60; }
float ODrive::getVoltage() { return busVoltage; }
float ODrive::getCurrent() { return busCurrent; }
