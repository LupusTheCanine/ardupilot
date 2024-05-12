#include "SIM_BH1750.h"

#include <SITL/SITL.h>
#include <math.h>


using namespace SITL;

#include <AP_HAL/AP_HAL.h>
extern const AP_HAL::HAL& hal;

BH1750::BH1750(): 
    I2CDevice()
{
}

void BH1750::update(const class Aircraft &Aircraft)
{
    hal.console->printf("LS_update\n");
    const uint32_t now_us = AP_HAL::micros();
    switch (state) {
    case State::COLD:
        raw=0;
        MTreg=69;
        state=State::COLD_WAIT;
        command_start_us=now_us;
        break;
    case State::COLD_WAIT:
       if (now_us-command_start_us>2000){
        state=State::POWER_OFF;
       }
    break;
    case State::POWER_OFF:
    break;
    case State::POWER_ON:
    break;
    case State::CONVERSION_START:
        if(mode==Mode::CONT_L_MODE or mode==Mode::ONCE_L_MODE){
            conversion_end_us=now_us+MTreg*88+1500;  //worst case scenario is 24 ms for MTreg=254 1.5ms for data processing
        }else{
            conversion_end_us=now_us+MTreg*702+1500; //worst case scenario is 180ms for MTreg=254 1.5ms for data processing
            //TODO: add source and parameters to change actual conversion time to simulate different sensors
        }
        state=State::CONVERSION;
        FALLTHROUGH;
    case State::CONVERSION:
    if (now_us > conversion_end_us) {
        raw=raw>1000?0:raw+1;
        if (mode == Mode::CONT_H2_MODE or mode == Mode::ONCE_H2_MODE)
        {
            uint32_t val=2*(uint32_t)raw;
            raw=val>((uint16_t)-1)?((uint16_t)-1):val;
        }
        if (mode>=Mode::ONCE_H_MODE && mode<=Mode::ONCE_L_MODE)
        {
            state=State::POWER_OFF;
            mode=Mode::IDLE;
            break;
        }else
        {
            state=State::CONVERSION_START;
        };
    };
    break;
    default:
        AP_HAL::panic("Invalid state");
        break;
    }
}
int BH1750::rdwr(I2C::i2c_rdwr_ioctl_data *&data)
{
    hal.console->printf("LS_RDWR\n");
    if (data->nmsgs>1){
        AP_HAL::panic("RW not supported");
        return 0;
    }
    struct I2C::i2c_msg &msg=data->msgs[0];
    if (msg.flags==0) //receive command
    {
        if (msg.len != 1) {
            AP_HAL::panic("bad command length %d",msg.len);
            return 0;
        }    
        const Command cmd = (Command)msg.buf[0];
        if (state<State::POWER_OFF)
        {
            hal.console->printf("Command (0x%02x) received while uninitialized.\n",(unsigned)cmd);
            return -1;
        }
        switch (cmd)
        {
        case Command::POWER_DOWN:
            state=State::POWER_OFF;
            break;
        case Command::POWER_ON:
            if (state==State::POWER_OFF){
                state=State::POWER_ON;
                mode=Mode::IDLE;
            }
            break;
        case Command::RESET:
            raw=0;
            break;
        case Command::CONT_H_MODE:
            mode=Mode::CONT_H_MODE;
            state=State::CONVERSION_START;
            break;
        case Command::CONT_H2_MODE:
            mode=Mode::CONT_H2_MODE;
            state=State::CONVERSION_START;
            break;
        case Command::CONT_L_MODE:
            mode=Mode::CONT_L_MODE;
            state=State::CONVERSION_START;
            break;
        
        case Command::ONCE_H_MODE:
            mode=Mode::ONCE_H_MODE;
            state=State::CONVERSION_START;
            break;
        case Command::ONCE_H2_MODE:
            mode=Mode::ONCE_H2_MODE;
            state=State::CONVERSION_START;
            break;
        case Command::ONCE_L_MODE:
            mode=Mode::ONCE_L_MODE;
            state=State::CONVERSION_START;
            break;
        default:
            if(((uint8_t)cmd&0b11111000)==(uint8_t)Command::SET_MTREG_H){
                MTreg=(MTreg&0b00011111)&((uint8_t)cmd<<5);
            }else if(((uint8_t)cmd&0b11100000)==(uint8_t)Command::SET_MTREG_L){
                MTreg=(MTreg&0b11100000)&((uint8_t)cmd&0b000111111);
            }else{
                AP_HAL::panic(" unknown command (ox%02x)",(unsigned)cmd);
            }
            break;
        }
    }else if(msg.flags==I2C_M_RD)
    {
        if(msg.len!=2){
            AP_HAL::panic("Unexpected read length=%u",msg.len);
            return 0;
        }
        msg.buf[0]=(uint8_t)(raw>>8);
        msg.buf[1]=(uint8_t)(raw%256);
    }
    return 0;
}