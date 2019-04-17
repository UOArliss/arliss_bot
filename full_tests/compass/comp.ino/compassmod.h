#ifndef _COMPASSMOD_H
#define _COMPASSMOD_H

//#include <Arduino.h>

 enum reg_addr
    {
      CTRL_REG1_A       = 0x20, // DLH, DLM, DLHC
      CTRL_REG2_A       = 0x21, // DLH, DLM, DLHC
      CTRL_REG3_A       = 0x22, // DLH, DLM, DLHC
      CTRL_REG4_A       = 0x23, // DLH, DLM, DLHC
      CTRL_REG5_A       = 0x24, // DLH, DLM, DLHC
      CTRL_REG6_A       = 0x25, // DLHC
      HP_FILTER_RESET_A = 0x25, // DLH, DLM
      REFERENCE_A       = 0x26, // DLH, DLM, DLHC
      STATUS_REG_A      = 0x27, // DLH, DLM, DLHC

      OUT_X_L_A         = 0x28,
      OUT_X_H_A         = 0x29,
      OUT_Y_L_A         = 0x2A,
      OUT_Y_H_A         = 0x2B,
      OUT_Z_L_A         = 0x2C,
      OUT_Z_H_A         = 0x2D,

      FIFO_CTRL_REG_A   = 0x2E, // DLHC
      FIFO_SRC_REG_A    = 0x2F, // DLHC

      INT1_CFG_A        = 0x30, // DLH, DLM, DLHC
      INT1_SRC_A        = 0x31, // DLH, DLM, DLHC
      INT1_THS_A        = 0x32, // DLH, DLM, DLHC
      INT1_DURATION_A   = 0x33, // DLH, DLM, DLHC
      INT2_CFG_A        = 0x34, // DLH, DLM, DLHC
      INT2_SRC_A        = 0x35, // DLH, DLM, DLHC
      INT2_THS_A        = 0x36, // DLH, DLM, DLHC
      INT2_DURATION_A   = 0x37, // DLH, DLM, DLHC

      CLICK_CFG_A       = 0x38, // DLHC
      CLICK_SRC_A       = 0x39, // DLHC
      CLICK_THS_A       = 0x3A, // DLHC
      TIME_LIMIT_A      = 0x3B, // DLHC
      TIME_LATENCY_A    = 0x3C, // DLHC
      TIME_WINDOW_A     = 0x3D, // DLHC

      CRA_REG_M         = 0x00, // DLH, DLM, DLHC
      CRB_REG_M         = 0x01, // DLH, DLM, DLHC
      MR_REG_M          = 0x02, // DLH, DLM, DLHC

      SR_REG_M          = 0x09, // DLH, DLM, DLHC
      IRA_REG_M         = 0x0A, // DLH, DLM, DLHC
      IRB_REG_M         = 0x0B, // DLH, DLM, DLHC
      IRC_REG_M         = 0x0C, // DLH, DLM, DLHC

      WHO_AM_I_M        = 0x0F, // DLM

      TEMP_OUT_H_M      = 0x31, // DLHC
      TEMP_OUT_L_M      = 0x32, // DLHC


      // dummy addresses for registers in different locations on different devices;
      // the library translates these based on device type
      // value with sign flipped is used as index into translated_regs array

      OUT_X_H_M         = -1,
      OUT_X_L_M         = -2,
      OUT_Y_H_M         = -3,
      OUT_Y_L_M         = -4,
      OUT_Z_H_M         = -5,
      OUT_Z_L_M         = -6,
      // update dummy_reg_count if registers are added here!

      // device-specific register addresses

      DLH_OUT_X_H_M     = 0x03,
      DLH_OUT_X_L_M     = 0x04,
      DLH_OUT_Y_H_M     = 0x05,
      DLH_OUT_Y_L_M     = 0x06,
      DLH_OUT_Z_H_M     = 0x07,
      DLH_OUT_Z_L_M     = 0x08,

      DLM_OUT_X_H_M     = 0x03,
      DLM_OUT_X_L_M     = 0x04,
      DLM_OUT_Z_H_M     = 0x05,
      DLM_OUT_Z_L_M     = 0x06,
      DLM_OUT_Y_H_M     = 0x07,
      DLM_OUT_Y_L_M     = 0x08,

      DLHC_OUT_X_H_M    = 0x03,
      DLHC_OUT_X_L_M    = 0x04,
      DLHC_OUT_Z_H_M    = 0x05,
      DLHC_OUT_Z_L_M    = 0x06,
      DLHC_OUT_Y_H_M    = 0x07,
      DLHC_OUT_Y_L_M    = 0x08,

      D_OUT_X_L_M       = 0x08,
      D_OUT_X_H_M       = 0x09,
      D_OUT_Y_L_M       = 0x0A,
      D_OUT_Y_H_M       = 0x0B,
      D_OUT_Z_L_M       = 0x0C,
      D_OUT_Z_H_M       = 0x0D
    };

    /*maybe change to explicit 16 bit int*/
    

    typedef struct device{
      int timeout;

      int accel_readings[3];
      int mag_readings[3];
      int cal_max[3];
      int cal_min[3];

    } device;

    const device* init_device()
    
    void calibrate(int** min, int**max);

    void write_acc_reg(byte register, byte value);
    byte read_acc_reg(byte reg);

    void write_mag_reg(byte register, byte value);
    byte read_mag_reg(int register);

    void write_reg(byte register, byte value);
    byte read_reg(int register);

    void read_acc();
    void read_mag();
    void read();

    float compass_heading(int** f);



#endif