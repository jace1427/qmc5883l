#include <hardware/i2c.h>
#include <hardware/regs/pads_bank0.h>
#include <pico/binary_info.h>
#include <pico/error.h>
#include <pico/stdio.h>
#include <stdint.h>

// The qmc struct
typedef struct {
  uint8_t ODR, RNG, OSR;
  float magnetic_declination;

  struct {
    int16_t x, y, z;
  } data;

  struct {
    int status;
    int16_t x, y, z;
  } calibrated;

  struct {
    float x, y, z;
  } offsets;

  struct {
    float x, y, z;
  } scales;

} QMC5883L;

// Functions
int qmc5883l_get_azimuth(QMC5883L *qmc);
int qmc5883l_read_data(QMC5883L *qmc);
void qmc5883l_calibrate(QMC5883L *qmc);
int qmc5883l_init(QMC5883L *qmc, const int odr, const int rng, const int osr,
                  const int degrees, const uint8_t minutes);

// The QMC5883L registers and settings
// default address is OD: 0001101
#define ADDRESS 0x0D
#define CHIP_ID 0xFF

#define REG_XOUT_LSB 0x00
#define REG_XOUT_MSB 0x01
#define REG_YOUT_LSB 0x02
#define REG_YOUT_MSB 0x03
#define REG_ZOUT_LSB 0x04
#define REG_ZOUT_MSB 0x05
#define REG_STATUS_1 0x06
#define REG_TOUT_LSB 0x07
#define REG_TOUT_MSB 0x08
#define REG_CONTROL_1 0x09
#define REG_CONTROL_2 0x0A
#define REG_SET_RESET 0x0B
// 0x0C is reserved
#define REG_CHIP_ID 0x0D

// REG_STATUS_1 settings
#define STAT_DRDY 0x00000001
#define STAT_OVL 0x00000010
#define STAT_DOR 0b00000100

// REG_CONTROL_1 settings
#define MODE_CONT 0b00000001
#define MODE_STBY 0b00000000
#define ODR_10Hz 0b00000000
#define ODR_50Hz 0b00000100
#define ODR_100Hz 0b00001000
#define ODR_200Hz 0b00001100
#define RNG_2G 0b00000000
#define RNG_8G 0b00010000
#define OSR_512 0b00000000
#define OSR_256 0b01000000
#define OSR_128 0b10000000
#define OSR_64 0b11000000

// REG_CONTROL_2 settings
#define SOFT_RST 0b10000000
#define INT_ENB 0b00000001
