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
  int ret = qmc5883l_init(&qmc, 10, 2, 512, 0, 0, 1);
  if (ret == PICO_OK) {
    printf("\ninit succeeded!!!\n");
  } else {
    while (true) {
      printf("\ninit failed :(\n");
      sleep_ms(1000);
    }
  }

  while (true) {
    qmc5883l_read_data(&qmc);
    printf("[%d, %d, %d]\n", qmc.data.x, qmc.data.y, qmc.data.z);
    sleep_ms(500);
  }

  return 0;
}
