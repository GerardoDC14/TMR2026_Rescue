
#ifndef ODRIVE_H
#define ODRIVE_H

#include <driver/twai.h>
#include <Arduino.h>

class ODrive {
private:
    uint8_t deviceId;

public:
    float dutyCycleNow;
    float avgMotorCurrent;
    float erpm;
    float WattHours;
    float tempFET;
    float tempMotor;
    float avgInputCurrent;
    float inpVoltage;

    ODrive(uint8_t devId);

    void feedMsg(twai_message_t& msg);
    void set_duty(float duty);
    void set_current(float current);
    void set_erpm(float eerpm);
    void requestVoltage();

private:
    void get_frame();
    float process_data_frame(char datatype, unsigned char byte1, unsigned char byte2);
    int hex2int(char buf[]);
};

#endif
