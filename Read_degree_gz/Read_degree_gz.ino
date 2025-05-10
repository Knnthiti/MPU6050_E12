#include "MPU6050_E12.h"

#define I2C_SCL_Pin 22
#define I2C_SDA_Pin 23
#define freq_MPU6050 100

MPU6050 _MPU6050(I2C_SDA_Pin, I2C_SCL_Pin, freq_MPU6050);

long Part_Time = 0;

float Degree[3] = { 0 };

uint8_t X = 0;


void setup() {
  _MPU6050.MPU_init();
  _MPU6050.gyro_calib();

  Serial.begin(115200);
}

void loop() {
  if ((millis() - Part_Time) > 10) {
    Part_Time = millis();
    _MPU6050.Degree(Degree);

    Serial.print(Degree[0]);
    Serial.print(" , ");
    Serial.print(Degree[1]);
    Serial.print(" , ");
    Serial.print(Degree[2]);
    Serial.println(" ");

    // Serial.println(_MPU6050.mpuData_Offset.gz);
  }
}
