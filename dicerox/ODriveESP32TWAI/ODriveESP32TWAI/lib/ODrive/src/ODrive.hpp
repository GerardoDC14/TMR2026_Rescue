#ifndef ODRIVE_H
#define ODRIVE_H

#include <driver/twai.h>
#include <Arduino.h>
#include <stdint.h>

class ODrive {
private:
    uint8_t deviceId;

public:
    float     busCurrent;
    float     busVoltage;
    float     iqSetpoint;
    float     iqMeasured;

    float     encEstPos;
    float     encEstVel;
    float     encShadowCount;
    float     encCountCPR;
    float     senlessEstPos;
    float     senlessEstVel;
    
    uint32_t  axisError;
    uint8_t   currentState;
    bool      motorErrorFlag;
    bool      encoderErrorFlag;
    bool      controllerErrorFlag;
    bool      trajectoryDone;
    uint64_t  motorError;
    uint32_t  encoderError;
    uint32_t  sensorlessError;
    uint32_t  controllerError;

    ODrive(uint8_t devId);

    void feedMsg(twai_message_t& msg);
    void setRpm(float rpm);
    void requestVoltage();
    void Estop();

    float getEpos();
    float getEvel();
    float getVoltage();
    float getCurrent();
};

#endif
