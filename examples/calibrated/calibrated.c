#include "qmc5883l.h"
#include <pico/time.h>
#include <stdio.h>

/* Example usage for the qmc5883l library.
 *
 * Gets the raw x, y, and z readings from the magnetometer, and prints them
 * every half second.
 */
int main(void) {
  QMC5883L qmc;
  sleep_ms(10000);
  int ret = qmc5883l_init(&qmc, 10, 8, 512, 15, 5, 1);
  if (ret == PICO_OK) {
    printf("\ninit succeeded!!!\n");
  } else {
    while (true) {
      printf("\ninit failed :(\n");
      sleep_ms(1000);
    }
  }

  printf("Calibration will begin in 5 seconds, please wiggle the QMC all "
         "around!\n");
  for (int i = 5; i > 0; i--) {
    printf("%d\n", i);
    sleep_ms(1000);
  }

  printf("Calibrating...\n");
  qmc5883l_calibrate(&qmc);
  // qmc5883l_set_calibration(&qmc, -8227, 4280, -3220, 8695, -5385, 7427);
  printf("Well Done!\n");

  while (true) {
    printf("Ready to read data? y/n:\n");
    char input = getchar();
    if (input == 'y') {
      break;
    }
  }

  int heading;
  while (true) {
    qmc5883l_read_data(&qmc);
    printf("Raw         - [%d, %d, %d]\n", qmc.data.x, qmc.data.y, qmc.data.z);
    printf("Calibrated  - [%d, %d, %d]\n", qmc.calibrated.x, qmc.calibrated.y,
           qmc.calibrated.z);
    heading = qmc5883l_get_azimuth(&qmc);
    printf("Heading     - %d\n", heading);
    sleep_ms(500);
  }

  return 0;
}
