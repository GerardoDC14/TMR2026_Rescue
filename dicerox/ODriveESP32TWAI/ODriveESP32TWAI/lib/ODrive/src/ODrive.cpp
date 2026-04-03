#include "ODrive.hpp"

ODrive::ODrive(uint8_t devId) : deviceId(devId) {
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

void ODrive::feedMsg(twai_message_t& msg) {
  uint32_t node = msg.identifier >> 5;
  uint32_t cmd = msg.identifier & 0x1F;
  if (node != deviceId) return;
    
  switch (cmd) {
    case 0x01:{
      memcpy(&axisError, &msg.data[0], 4);
      currentState        = msg.data[4];
      motorErrorFlag      = bitRead(msg.data[5], 0);
      encoderErrorFlag    = bitRead(msg.data[6], 0);
      controllerErrorFlag = bitRead(msg.data[7], 0);
      trajectoryDone      = bitRead(msg.data[7], 7);
      break;
    }

    case 0x03: {
      memcpy(&motorError, &msg.data[0], 8);
      break;
    }

    case 0x04: {
      memcpy(&encoderError, &msg.data[0], 4);
      break;
    }
    case 0x05: {
      memcpy(&sensorlessError, &msg.data[0], 4);
      break;
    }

    case 0x09: {
      memcpy(&encEstPos, &msg.data[0], 4);
      memcpy(&encEstVel, &msg.data[4], 4);
      break;
    }

    case 0x14: {
      memcpy(&iqSetpoint, &msg.data[0], 4);
      memcpy(&iqMeasured, &msg.data[4], 4);
      break;
    }
    case 0x15: {
      memcpy(&senlessEstPos, &msg.data[0], 4);
      memcpy(&senlessEstVel, &msg.data[4], 4);
    }

    case 0x17: {
      memcpy(&busVoltage, &msg.data[0], 4);
      memcpy(&busCurrent, &msg.data[4], 4);
      break;
    }
    case 0x1D: {
      memcpy(&controllerError, &msg.data[0], 4);
      break;
    }
    default:{break;}
  }
}

void ODrive::requestVoltage() {
  twai_message_t request;
  request.identifier = (deviceId << 5) | 0x17;
  request.data_length_code = 0;
  request.rtr = 1;
  twai_transmit(&request, pdMS_TO_TICKS(10));
}

void ODrive::Estop(){
  twai_message_t request;
  request.identifier = (deviceId<<5) | 0x02;
  request.data_length_code = 0;
  request.rtr = 1;
  twai_transmit(&request, pdMS_TO_TICKS(10));
}
