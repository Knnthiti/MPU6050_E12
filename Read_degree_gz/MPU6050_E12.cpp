#include "MPU6050_E12.h"

void MPU6050 :: MPU_i2c_writeReg8(uint8_t reg, uint8_t data8) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(data8);
  Wire.endTransmission();
}

void MPU6050 :: MPU_init() {
  //data_sheet : https://cdn.sparkfun.com/datasheets/Sensors/Accelerometers/RM-MPU-6000A.pdf

  // power management register 0X6B we should write all 0's to wake the sensor up
  MPU_i2c_writeReg8(REG_PWRMGMT_1, 0x00);

  // Set DATA RATE of 8KHz by writing SMPLRT_DIV register
  MPU_i2c_writeReg8(REG_SMPRT_DIV, 0);

  // Accel 44Hz - Gyro 42Hz
  MPU_i2c_writeReg8(REG_DLPFCONF, 0x03);

  // Set accelerometer configuration in ACCEL_CONFIG Register
  // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ± 16g
  MPU_i2c_writeReg8(REG_ACCLCONF, 3 << 3);

  // Set Gyroscopic configuration in GYRO_CONFIG Register
	// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ± 500 ̐/s
  MPU_i2c_writeReg8(REG_GYROCONF, 0x01);
}

void MPU6050 :: MPU_get_Accelerometer(){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);  // Starting register for accelerometer
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6);

  mpuData.Ax = Wire.read() << 8 | Wire.read();
  mpuData.Ay = Wire.read() << 8 | Wire.read();
  mpuData.Az = Wire.read() << 8 | Wire.read();
}

void MPU6050 :: MPU_get_gyro(){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_GYRO_XOUT_H);  // Starting register for accelerometer
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6);

  mpuData.gx = Wire.read() << 8 | Wire.read();
  mpuData.gy = Wire.read() << 8 | Wire.read();
  mpuData.gz = Wire.read() << 8 | Wire.read();

}

void MPU6050 :: gyro_calib(){
  float Offset_gx ,Offset_gy ,Offset_gz = 0;

  for(uint16_t i = 0 ; i < 2000 ; i++){
    MPU_get_gyro();
    Offset_gx += mpuData.gx;
    Offset_gy += mpuData.gy;
    Offset_gz += mpuData.gz;

    // mpuData_Offset.gx += mpuData.gx;
    // mpuData_Offset.gx += mpuData.gy;
    // mpuData_Offset.gx += mpuData.gz;
    delay(1);
  }

  mpuData_Offset.gx = Offset_gx/2000.0f;
  mpuData_Offset.gy = Offset_gy/2000.0f;
  mpuData_Offset.gz = Offset_gz/2000.0f;
}

void MPU6050 :: Degree(){
  MPU_get_gyro();

  Angular.Deg_x += (float)((mpuData.gx - mpuData_Offset.gx)/ (131.0f * freq_MPU6050));
  Angular.Deg_y += (float)((mpuData.gy - mpuData_Offset.gy)/ (131.0f * freq_MPU6050));
  Angular.Deg_z += (float)((mpuData.gz - mpuData_Offset.gz)/ (131.0f * freq_MPU6050));
}

void MPU6050 :: Radian(){
  MPU_get_gyro();

  Angular.Rad_x = (float)((mpuData.gx - mpuData_Offset.gx)/ (131.0f * freq_MPU6050)*Degree_to_Radian);
  Angular.Rad_y = (float)((mpuData.gy - mpuData_Offset.gy)/ (131.0f * freq_MPU6050)*Degree_to_Radian);
  Angular.Rad_z = (float)((mpuData.gz - mpuData_Offset.gz)/ (131.0f * freq_MPU6050)*Degree_to_Radian);
}

void MPU6050 :: Degree(float _Degree[]){
  MPU_get_gyro();

  _Degree[0] += (float)((mpuData.gx - mpuData_Offset.gx)/ (131.0f * freq_MPU6050));
  _Degree[1] += (float)((mpuData.gy - mpuData_Offset.gy)/ (131.0f * freq_MPU6050));
  _Degree[2] += (float)((mpuData.gz - mpuData_Offset.gz)/ (131.0f * freq_MPU6050));
}

void MPU6050 :: Radian(float _Radian[]){
  MPU_get_gyro();

  _Radian[0] = (float)((mpuData.gx - mpuData_Offset.gx)/ (131.0f * freq_MPU6050)*Degree_to_Radian);
  _Radian[1] = (float)((mpuData.gy - mpuData_Offset.gy)/ (131.0f * freq_MPU6050)*Degree_to_Radian);
  _Radian[2] = (float)((mpuData.gz - mpuData_Offset.gz)/ (131.0f * freq_MPU6050)*Degree_to_Radian);
}