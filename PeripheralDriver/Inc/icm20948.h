

#ifndef INC_ICM20948_H_
#define INC_ICM20948_H_

#include <stdint.h>
#include "stm32f4xx.h"
//The slave address of the ICM-20948 is b110100X which is 7 bits long. The LSB bit of the 7-bit address is determined by
//the logic level on pin AD0. This allows two ICM-20948s to be connected to the same I2C bus. When used in this
//configuration, the address of the one of the devices should be b1101000 (pin AD0 is logic low) and the address of the
//other should be b1101001 (pin AD0 is logic high).
//TODO check if this is correct.
#define ICM20948_ADDR (0x68 << 1) // SLAVE ADDRESS of ICM20948(left shift by 1)


//Register addresses for registers that we will be using
#define REG_BANK_SEL  (0x7F) //Register Bank Select
#define USER_BANK_0		(0x00)
#define USER_BANK_1		(0x10)
#define USER_BANK_2		(0x20)
#define USER_BANK_3		(0x30)

#define REG_BANK_SEL_0 (0x00)
#define REG_BANK_SEL_1 (0x10)
#define REG_BANK_SEL_2  (0x20)

//Choosing what to turn on
#define PWR_MGMT_1 		(0x06)
#define PWR_MGMT_2		(0x07)
#define LP_CONFIG       (0x05)

//user Control
#define USER_CTRL       (0x03)

//choose whether or not to enable to disable alignment of Output Data Register
#define ODR_ALIGN_EN (0x03)

//Gyro configurations
#define ZG_OFFS_USRH   (0x00)
#define ZG_OFFS_USRL    (0x00)
#define GYRO_CONFIG_1   (0x01)
#define GYRO_CONFIG_2   (0x02)
#define GYRO_SMPLRT_DIV (0x00)

//configuration data that we are using
// User Control Bits in register
// 7:DMP_EN = 0(Disable DMP), 6:FIFO_EN = 0(Disable FIFO), 5:I2C_MST_EN = 1(Enable master control for magnetometer), 4:I2C_IF_DIS = 0
// 3:DMP_RST = 0, 2:SRAM_RST = 0, 1:I2C_MST_RST = 0, 0:Reserved = 0

static const uint8_t USER_CTRL_VAL = 0x20; // 0b00100000


// Low Power Configuration
// 7:Reserved = 0, 6:I2C_MST_CYCLE = 1(Operate I2C master in duty cycled mode), 5:ACCEL_CYCLE = 0,
// 4:GYRO_CYCLE = 0 [3:0]:Reserved = 0000
static const uint8_t LP_CONFIG_VAL = 0x40; // 0b01000000

// Power Management 1 Set Bits
// 7:Device reset = 0, 6:SLEEP = 0(Wake device up from sleep mode), 5:LP_EN = 0(Low power disable), 4:Reserved = 0
// 3:TEMP_DIS = 1(Disable temp sensor), [2:0]:CLKSEL = 1(Auto select clock source for full functionality)
static const uint8_t PWR_MGMT_1_VAL = 0x09; // 0b00001001


//Power Management 2
// Power Management 2 Set Bits
// [7:6]:Reserved, [5:3]:DISABLE_ACCEL = 000(Accel only Z axis off), [2:0]:DISABLE_GYRO = 110(Gyro only Z axis on)
static const uint8_t PWR_MGMT_2_VAL = 0x3E; // 0b00111110* 0x3E*

// Interrupt Enable 1 Set Bits
// [7:1]:Reserved, 0:RAW_DATA_0_RDY_EN = 0(Disable Data Ready Interrupt)
static const uint8_t INT_ENABLE_1_VAL = 0x00;// 0b00000000

//Interrupt Enable 2
// Interrupt Enable 1 Set Bits
// [7:5]:Reserved, [4:0]:FIFO_OVERFLOW_EN = 00000(Disable Interrupt for FIFO overflow)
static const uint8_t INT_ENABLE_2_VAL = 0x00;// 0b00000000

// ODR Alignment Set Bits
// [7:1]:Reserved = 0000000, 0:ODR_ALIGN_EN = 1(Disable ODR Alignment)
static const uint8_t ODR_ALIGN_EN_VAL = 0x00; // 0b00000000

// Gyro Z axis offset cancellation Set Bits
// [7:0]:Upper bits of the X accelerometer offset cancellation = 00000000
static const uint8_t ZG_OFFS_USRH_VAL = 0x00; // 0b00000000

// Gyro Z axis offset cancellation Set Bits
// [7:0]:Lower bits of the X accelerometer offset cancellation = 00000000
static const uint8_t ZG_OFFS_USRL_VAL = 0x00; // 0b00000000

// GYRO_CONFIG_1 Set Bits
// [7:6]:Reserved = 00, [5:3]:GYRO_DLPFCFG = 111, [2:1]:GYRO_FS_SEL = 00(250dps), 0:GYRO_FCHOICE = 1(Enable Gyro DLPF)
static const uint8_t GYRO_CONFIG_1_VAL = 0x39; // 0b00111001

// GYRO_CONFIG_2 Set Bits
// [7:6]:Reserved = 00, [5:3]:XYZGYRO_CTEN = 000(All gyro self test disable, [2:0]:GYRO_AVGCFG = 000(1x Averaging)
static const uint8_t GYRO_CONFIG_2_VAL = 0x00; // 0b00000000

// Gyro Sample Rate Divider Set Bits
// [7:0]:GYRO_SMPLRT_DIV = 00000000(divide by 1 - full frequency 1125hz)
static const uint8_t GYRO_SMPLRT_DIV_VAL = 0x08; // 0b00001000


// R/W FIFO
static const uint8_t ACCEL_XOUT_H = 0x2D; //MSB(8bit) of 16-bit Acceleration(X)
static const uint8_t ACCEL_XOUT_L = 0x2E; //LSB(8bit) of 16-bit Acceleration(X)
static const uint8_t ACCEL_YOUT_H = 0x2F; //MSB(8bit) of 16-bit Acceleration(Y)
static const uint8_t ACCEL_YOUT_L = 0x30; //LSB(8bit) of 16-bit Acceleration(Y)
static const uint8_t ACCEL_ZOUT_H = 0x31; //MSB(8bit) of 16-bit Acceleration(Z)
static const uint8_t ACCEL_ZOUT_L = 0x32; //LSB(8bit) of 16-bit Acceleration(Z)
static const uint8_t GYRO_XOUT_H = 0x33; //MSB(8bit) of 16-bit Gyro Rotation(X)
static const uint8_t GYRO_XOUT_L = 0x34; //LSB(8bit) of 16-bit Gyro Rotation(X)
static const uint8_t GYRO_YOUT_H = 0x35; //MSB(8bit) of 16-bit Gyro Rotation(Y)
static const uint8_t GYRO_YOUT_L = 0x36; //LSB(8bit) of 16-bit Gyro Rotation(Y)
static const uint8_t GYRO_ZOUT_H = 0x37; //MSB(8bit) of 16-bit Gyro Rotation(Z)
static const uint8_t GYRO_ZOUT_L = 0x38; //LSB(8bit) of 16-bit Gyro Rotation(Z)


int IMU_Init(I2C_HandleTypeDef* hi2c_Addr);
int IMU_Read(int16_t* Data);
int Magnetometer_Read(uint8_t reg, uint8_t* Data);
int Magnetometer_Write(uint8_t reg, uint8_t value);


#endif /* INC_ICM20948_H_ */
