#include "qmc5883l.h"
#include <hardware/gpio.h>
#include <math.h>
#include <pico/error.h>
#include <pico/time.h>
#include <stdint.h>
#include <stdio.h>

/*! Attempt to write val to reg.
 *  /ingroup qmc5883l
 *
 * /param reg The byte address of the register to write to.
 * /param val The byte of data to be written.
 * /return PICO_OK on success, PICO_ERROR_GENERIC on failure
 */
int _qmc5883l_write_one_byte(const uint8_t reg, const uint8_t val) {
  const uint8_t buf[2] = {reg, val};
  int ret = i2c_write_blocking(i2c_default, ADDRESS, buf, 2, false);
  return ret == 2 ? PICO_OK : PICO_ERROR_GENERIC;
}

/*! Attempt to read the byte stored in reg.
 *  /ingroup qmc5883l
 *
 * /param reg The byte address of the register to write to.
 * /param final_val A pointer to store the byte read.
 * /return PICO_OK on success, PICO_ERROR_GENERIC on failure
 */
int _qmc5883l_read_one_byte(const uint8_t reg, uint8_t *final_val) {
  // In order to read a reg, first send the reg, then read
  int ret = PICO_OK;
  ret |= i2c_write_blocking(i2c_default, ADDRESS, &reg, 1, true);
  ret |= i2c_read_blocking(i2c_default, ADDRESS, final_val, 1, false);
  return ret == 1 ? PICO_OK : PICO_ERROR_GENERIC;
}

/*! Attempt to read two bytes stored in lsb and msb.
 *  /ingroup qmc5883l
 *
 * /param lsb The byte address of the first register to read from.
 * /param msb The byte address of the second register to read from.
 * /param final_val A pointer to store the two bytes read.
 * /return PICO_OK on success, PICO_ERROR_GENERIC on failure
 */
int _qmc5883l_read_two_bytes(const uint8_t lsb, const uint8_t msb,
                             int16_t *final_val) {
  uint8_t raw_lsb;
  uint8_t raw_msb;
  int ret = PICO_OK;
  ret |= _qmc5883l_read_one_byte(lsb, &raw_lsb);
  ret |= _qmc5883l_read_one_byte(msb, &raw_msb);
  *final_val = (raw_msb << 8) | raw_lsb;
  return ret;
}

int qmc5883l_get_azimuth(QMC5883L *qmc) {
  int16_t x, y, z;
  if (qmc->calibrated.is_calibrated) {
    x = qmc->calibrated.x;
    y = qmc->calibrated.y;
    z = qmc->calibrated.z;
  } else {
    x = qmc->data.x;
    y = qmc->data.y;
    z = qmc->data.z;
  }

  double heading = (atan2(y, x) * 180.0) / M_PI;
  heading += qmc->magnetic_declination;
  if (heading <= 0) {
    heading += 360;
  }
  return (int)heading;
}

/*! Calculated new x, y, and z values using offsets and scales.
 *  /ingroup qmc5883l
 *
 * /param qmc A pointer to the qmc struct.
 */
void _qmc5883l_apply_calibration(QMC5883L *qmc) {
  qmc->calibrated.x = (int16_t)((qmc->data.x - qmc->offsets.x) * qmc->scales.x);
  qmc->calibrated.y = (int16_t)((qmc->data.y - qmc->offsets.y) * qmc->scales.y);
  qmc->calibrated.z = (int16_t)((qmc->data.z - qmc->offsets.z) * qmc->scales.z);
}

/*! Reads the x, y, and z data from the QMC5883L.
 *  /ingroup qmc5883l
 *
 * /param qmc A pointer to the qmc struct.
 * /return PICO_OK on success, PICO_ERROR_GENERIC on failure
 */
int qmc5883l_read_data(QMC5883L *qmc) {
  uint8_t status;
  int ret = _qmc5883l_read_one_byte(REG_STATUS_1, &status);
  // timeout after ~1/10th of a second
  for (int i = 0; i < 100; i++) {
    if (ret != PICO_OK) {
      return ret;
    } else if (status & STAT_OVL) {
      if (qmc->verbose) {
        char *msg = qmc->RNG == RNG_2G ? "Consider switching to RNG_8G.\n" : "";
        printf("Magnetic sensor overflow!\n");
        printf("%s", msg);
      }
    } else if (status & STAT_DOR || status & STAT_DRDY) {
      ret |= _qmc5883l_read_two_bytes(REG_XOUT_LSB, REG_XOUT_MSB, &qmc->data.x);
      ret |= _qmc5883l_read_two_bytes(REG_YOUT_LSB, REG_YOUT_MSB, &qmc->data.y);
      ret |= _qmc5883l_read_two_bytes(REG_ZOUT_LSB, REG_ZOUT_MSB, &qmc->data.z);
      if (qmc->verbose && ret != PICO_OK) {
        printf("qmc5883l_read_data: %d\n", ret);
      }
      if (qmc->calibrated.is_calibrated) {
        _qmc5883l_apply_calibration(qmc);
      }
      return PICO_OK;
    } else {
      sleep_ms(1);
    }
  }
  return PICO_ERROR_TIMEOUT;
}

/*! Given max's and min's, calculated and store the offsets and scales.
 *  /ingroup qmc5883l
 *
 * /param qmc A pointer to the qmc struct.
 * /param x_max The max x reading from our calibration.
 * /param x_min The min x reading from our calibration.
 * /param y_max The max y reading from our calibration.
 * /param y_min The min y reading from our calibration.
 * /param z_max The max z reading from our calibration.
 * /param z_min The min z reading from our calibration.
 */
void qmc5883l_set_calibration(QMC5883L *qmc, int16_t x_max, int16_t x_min,
                              int16_t y_max, int16_t y_min, int16_t z_max,
                              int16_t z_min) {

  qmc->offsets.x = (float)(x_max + x_min) / 2;
  qmc->offsets.y = (float)(y_max + y_min) / 2;
  qmc->offsets.z = (float)(z_max + z_min) / 2;

  float x_avg_delta = (float)(x_max - x_min) / 2;
  float y_avg_delta = (float)(y_max - y_min) / 2;
  float z_avg_delta = (float)(z_max - z_min) / 2;
  float avg_delta = (x_avg_delta + y_avg_delta + z_avg_delta) / 3;

  qmc->scales.x = avg_delta / x_avg_delta;
  qmc->scales.y = avg_delta / y_avg_delta;
  qmc->scales.z = avg_delta / z_avg_delta;

  qmc->calibrated.is_calibrated = 1;
  if (qmc->verbose) {
    printf("Offsets: [%0.3f, %0.3f, %0.3f]\n", qmc->offsets.x, qmc->offsets.y,
           qmc->offsets.z);
    printf("Scales: [%0.3f, %0.3f, %0.3f]\n", qmc->scales.x, qmc->scales.y,
           qmc->scales.z);
  }
}

/*! Clear our x, y, and z offsets and scales.
 *  /ingroup qmc5883l
 *
 * /param qmc A pointer to the qmc struct.
 */
void _qmc5883l_clear_calibration(QMC5883L *qmc) {
  qmc->offsets.x = 0.0f;
  qmc->offsets.y = 0.0f;
  qmc->offsets.z = 0.0f;
  qmc->scales.x = 1.0f;
  qmc->scales.y = 1.0f;
  qmc->scales.z = 1.0f;
}

/*! Calibrates the QMC5883L.
 *  /ingroup qmc5883l
 *
 * /param qmc A pointer to the qmc struct.
 */
void qmc5883l_calibrate(QMC5883L *qmc) {
  _qmc5883l_clear_calibration(qmc);
  int16_t x_max, x_min, y_max, y_min, z_max, z_min;

  uint32_t start_time = to_ms_since_boot(get_absolute_time());
  while ((to_ms_since_boot(get_absolute_time()) - start_time) < 10000) {
    qmc5883l_read_data(qmc);
    if (x_min > qmc->data.x) {
      x_min = qmc->data.x;
    }
    if (x_max < qmc->data.x) {
      x_max = qmc->data.x;
    }
    if (y_min > qmc->data.y) {
      y_min = qmc->data.y;
    }
    if (y_max < qmc->data.y) {
      y_max = qmc->data.y;
    }
    if (z_min > qmc->data.z) {
      z_min = qmc->data.z;
    }
    if (z_max < qmc->data.z) {
      z_max = qmc->data.z;
    }
  }

  printf("qmc5883l_calibrate(qmc, %d, %d, %d, %d, %d, %d);\n", x_min, x_max,
         y_min, y_max, z_min, z_max);
  qmc5883l_set_calibration(qmc, x_min, x_max, y_min, y_max, z_min, z_max);
}

/*! Parses the given settings into constants, and stores them into a qmc struct.
 *  /ingroup qmc5883l
 *
 * /param qmc A pointer to the qmc struct.
 * /param odr The Output Data Rate
 * /param rng The Range
 * /param osr The Over Sample Rate
 * /param degrees
 * /param minutes
 * /param verbose
 *
 * /return PICO_OK on success, PICO_ERROR_INVALID_ARG on failure
 */
int _qmc5883L_init_struct(QMC5883L *qmc, const int odr, const int rng,
                          const int osr, const int degrees,
                          const uint8_t minutes, const int verbose) {
  switch (odr) {
  case 10:
    qmc->ODR = ODR_10Hz;
    break;
  case 50:
    qmc->ODR = ODR_50Hz;
    break;
  case 100:
    qmc->ODR = ODR_100Hz;
    break;
  case 200:
    qmc->ODR = ODR_200Hz;
    break;
  default:
    return PICO_ERROR_INVALID_ARG;
  }

  switch (rng) {
  case 2:
    qmc->RNG = RNG_2G;
    break;
  case 8:
    qmc->RNG = RNG_8G;
    break;
  default:
    return PICO_ERROR_INVALID_ARG;
  }

  switch (osr) {
  case 64:
    qmc->OSR = OSR_64;
    break;
  case 128:
    qmc->OSR = OSR_128;
    break;
  case 256:
    qmc->OSR = OSR_256;
    break;
  case 512:
    qmc->OSR = OSR_512;
    break;
  default:
    return PICO_ERROR_INVALID_ARG;
  }

  if (degrees != 0 && minutes != 0) {
    qmc->magnetic_declination = degrees + (float)minutes / 60;
  } else {
    qmc->magnetic_declination = 0.0f;
  }

  if (verbose) {
    qmc->verbose = true;
    while (1) {
      printf("Welcome to Verbose mode!\n");
      printf("Ready to continue? y/n:\n");
      char input = getchar();
      if (input == 'y') {
        break;
      }
    }
    printf("Provided values:\nodr: %d, rng: %d, osr: %d, degrees: %d, minutes: "
           "%d, verbose: %d\n",
           odr, rng, osr, degrees, minutes, verbose);
    printf("Stored values:\nodr: %d, rng: %d, osr: %d, magnetic_declination: "
           "%f, verbose: %d\n",
           qmc->ODR, qmc->RNG, qmc->OSR, qmc->magnetic_declination,
           qmc->verbose);
  } else {
    qmc->verbose = false;
  }

  qmc->data.x = 0;
  qmc->data.y = 0;
  qmc->data.z = 0;

  qmc->calibrated.is_calibrated = false;
  qmc->calibrated.x = 0;
  qmc->calibrated.y = 0;
  qmc->calibrated.z = 0;

  qmc->scales.x = 0;
  qmc->scales.y = 0;
  qmc->scales.z = 0;

  return PICO_OK;
}

/*! Resets the QMC5883L, and sets it to continuous data read mode.
 *  /ingroup qmc5883l
 *
 * /param qmc A pointer to the qmc struct.
 *
 * /return PICO_OK on success, PICO_ERROR_GENERIC on failure
 */
int _qmc5883l_init_internal(QMC5883L *qmc) {
  sleep_ms(1000); // Delay to help QMC boot up

  // get and check ChipID
  uint8_t chipID;
  if (qmc->verbose && _qmc5883l_read_one_byte(REG_CHIP_ID, &chipID)) {
    printf("Error getting ChipID!\n");
  } else if (qmc->verbose && chipID != CHIP_ID) {
    printf("Wrong ChipID: 0x%x!\n", chipID);
  }

  int ret = PICO_OK;
  // Soft reset - restore default value of all registers
  ret |= _qmc5883l_write_one_byte(REG_CONTROL_2, SOFT_RST);
  // Disable interrupt pin
  ret |= _qmc5883l_write_one_byte(REG_CONTROL_2, INT_ENB);
  // Define SET/RESET period (it's just recommended ig)
  ret |= _qmc5883l_write_one_byte(REG_SET_RESET, 0x01);
  // Set operation mode
  ret |= _qmc5883l_write_one_byte(REG_CONTROL_1,
                                  MODE_CONT | qmc->ODR | qmc->RNG | qmc->OSR);
  if (qmc->verbose) {
    printf("_qmc5883l_init_internal: %d", ret);
  }
  return ret;
}

/*! Readies the Pico and the QMC5883L for I2C communication.
 *  /ingroup qmc5883l
 *
 * /param qmc A pointer to the qmc struct.
 * /param odr The Output Data Rate (default: 200)
 * /param rng The Range (default: 8)
 * /param osr The Over Sample Rate (default: 512)
 *
 * /return PICO_OK on success, PICO_ERROR_GENERIC on failure
 */
int qmc5883l_init(QMC5883L *qmc, const int odr, const int rng, const int osr,
                  const int degrees, const uint8_t minutes, const int verbose) {
  // init stdio
  bool stdio = stdio_init_all();
  if (!stdio) {
    return PICO_ERROR_GENERIC;
  }

  // ready gpio pins
  i2c_init(i2c_default, 100 * 1000);
  gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
  gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

  // Make the I2C pins available to picotool
  bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN,
                             GPIO_FUNC_I2C));

  if (_qmc5883L_init_struct(qmc, odr, rng, osr, degrees, minutes, verbose) !=
      PICO_OK) {
    return PICO_ERROR_INVALID_ARG;
  }
  return _qmc5883l_init_internal(qmc);
}
