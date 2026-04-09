#ifndef ODRIVE_MCP_H
#define ODRIVE_MCP_H

#include <Arduino.h>
#include <mcp2515.h>
#include <stdint.h>

class ODrive {
public:
    enum AxisState : uint32_t {
        UNDEFINED = 0,
        IDLE = 1,
        STARTUP_SEQUENCE = 2,
        FULL_CALIBRATION_SEQUENCE = 3,
        MOTOR_CALIBRATION = 4,
        ENCODER_INDEX_SEARCH = 5,
        ENCODER_OFFSET_CALIBRATION = 6,
        CLOSED_LOOP_CONTROL = 8,
        LOCKIN_SPIN = 9,
        ENCODER_DIR_FIND = 10,
        HOMING = 11,
        ENCODER_HALL_POLARITY_CALIBRATION = 12,
        ENCODER_HALL_PHASE_CALIBRATION = 13
    };

    enum ControlMode : uint32_t {
        VOLTAGE_CONTROL = 0,
        TORQUE_CONTROL = 1,
        VELOCITY_CONTROL = 2,
        POSITION_CONTROL = 3
    };

    enum InputMode : uint32_t {
        INACTIVE = 0,
        PASSTHROUGH = 1,
        VEL_RAMP = 2,
        POS_FILTER = 3,
        MIX_CHANNELS = 4,
        TRAP_TRAJ = 5,
        TORQUE_RAMP = 6,
        MIRROR = 7,
        TUNING = 8
    };

private:
    uint8_t deviceId;
    MCP2515* _canBus;

    static MCP2515* _configuredBuses[4]; 
    static uint8_t  _configuredBusCount;
    bool _isBusConfigured();
    void _markBusConfigured();

    void _sendEmptyFrame(uint8_t cmdId, bool rtr = false);
    uint32_t _makeCanId(uint8_t cmdId) { return (deviceId << 5) | (cmdId & 0x1F); }

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

    ODrive(uint8_t devId, MCP2515* canBus);
    bool begin(CAN_SPEED speed = CAN_1000KBPS, CAN_CLOCK clock = MCP_8MHZ);

    void feedMsg(const can_frame &frame);

    void setAxisState(uint32_t state);
    void setControllerMode(uint32_t controlMode, uint32_t inputMode);
    void setVelocity(float velocity, float torqueFF = 0.0f);
    void clearErrors();
    void Estop(); 

    void requestVoltage();

    float getEpos();
    float getEvel();
    float getVoltage();
    float getCurrent();
};

#endif
