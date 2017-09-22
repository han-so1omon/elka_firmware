#include <i2cdev.h>
#include <mpu6050.h>

static uint8_t devAddr;
static I2C_TypeDef *I2Cx;
static uint8_t buffer[14];
static bool isInit;

int main(int argc, char **argv) {
  /*
bool i2cdevReadByte(I2C_TypeDef *I2Cx, uint8_t devAddress, uint8_t memAddress,
                    uint8_t *data);
                    */
  mpu6050GetDeviceID();

  i2cdevReadBits(I2Cx, devAddr, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH,
        buffer);

  return 0;
}
