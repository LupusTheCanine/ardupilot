#pragma once

#include "SIM_I2CDevice.h"

#include <AP_Common/Bitmask.h>

namespace SITL {

class BH1750: public I2CDevice 
{
public:

    BH1750();

protected:
    
    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override;

    void update(const class Aircraft &aircraft) override;

private:
    enum class Command : uint8_t {
        POWER_DOWN  = 0x00,
        POWER_ON    = 0x01,
        RESET       = 0x07,
        CONT_H_MODE = 0x10,
        CONT_H2_MODE= 0x11,
        CONT_L_MODE = 0x12,
        ONCE_H_MODE = 0x20,
        ONCE_H2_MODE= 0x21,
        ONCE_L_MODE = 0x22,

        SET_MTREG_H= 0x40,
        SET_MTREG_L= 0x60,
    }; //MTreg handled separately as command and data are integrated in one byte
       //0b01000abc
       //0b011defgh
    enum class Mode : uint8_t {
        IDLE=0,
        CONT_H_MODE=1,
        CONT_H2_MODE,
        CONT_L_MODE,
        ONCE_H_MODE=11,
        ONCE_H2_MODE,
        ONCE_L_MODE,
    };
    enum class State : uint8_t { 
        COLD,
        COLD_WAIT,
        POWER_OFF,
        POWER_ON,
        CONVERSION_START,
        CONVERSION,
    };
           uint16_t raw=0;
    uint8_t MTreg=69;
    State state = State::COLD;
    Mode mode = Mode::IDLE; 
    uint32_t conversion_end_us;
    uint32_t command_start_us;
    uint32_t conversion_time_us=120000;
    float gain=1.2;
    void setMTregH(uint8_t command);
    void setMTregL(uint8_t command);


};




}

