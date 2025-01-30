#include "qmc5883l.h"
#include <pico/time.h>
#include <stdio.h>

/* Example usage for the qmc5883l library.
 * @return 1 if failure, 0 on success
 */
int main() {
  QMC5883L qmc;
  int success = qmc5883l_init(&qmc, 200, 8, 512, 0, 0);
  if (success) {
    printf("\ninit succeeded!!!\n");
  } else {
    while (1) {
      printf("\ninit failed :(\n");
      sleep_ms(1000);
    }
  }

  while (1) {
    printf("Ready to calibrate? y/n/q: \n");
    char input = getchar();
    if (input == 'y') {
      printf("Calibrating in 5 seconds....\n");
      sleep_ms(5000);
      printf("Wiggle the Pico!!!!\n");
      qmc5883l_calibrate(&qmc);
      printf("Nice job!!\n");
      printf("Settings found:\n");
      printf("Offsets: (%0.2f, %0.2f, %0.2f)\n", qmc.offsets.x, qmc.offsets.y,
             qmc.offsets.z);
      printf("Scales: (%0.2f, %0.2f, %0.2f)\n", qmc.scales.x, qmc.scales.y,
             qmc.scales.z);
      printf("Ready to read data? y/n: \n");
      input = getchar();
      if (input == 'y') {
        break;
      }
    } else if (input == 'q') {
      break;
    }
  }

  while (1) {
    qmc5883l_read_data(&qmc);
    int heading = qmc5883l_get_azimuth(&qmc);
    printf("%d   [%d, %d, %d]\n", heading, qmc.calibrated.x, qmc.calibrated.y,
           qmc.calibrated.z);
    sleep_ms(500);
  }
  return 0;
}
