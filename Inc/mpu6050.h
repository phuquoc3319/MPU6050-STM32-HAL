#ifmdef __MPU6050_H
#define __MPU6050_H


#define   SMPLRT_DIV      0x19   //0x07(125Hz)
#define   CONFIG          0x1A   //0x06(5Hz)
#define   GYRO_CONFIG     0x1B   //0x18(不自检，2000deg/s)
#define   ACCEL_CONFIG    0x1C   //0x01(不自检，2G，5Hz)
#define   ACCEL_XOUT_H    0x3B
#define   ACCEL_XOUT_L    0x3C
#define   ACCEL_YOUT_H    0x3D
#define   ACCEL_YOUT_L    0x3E
#define   ACCEL_ZOUT_H    0x3F
#define   ACCEL_ZOUT_L    0x40
#define   TEMP_OUT_H      0x41
#define   TEMP_OUT_L      0x42
#define   GYRO_XOUT_H     0x43
#define   GYRO_XOUT_L     0x44   
#define   GYRO_YOUT_H     0x45
#define   GYRO_YOUT_L     0x46
#define   GYRO_ZOUT_H     0x47
#define   GYRO_ZOUT_L     0x48
#define   USER_CTRL       0x6A
#define   PWR_MGMT_1      0x6B
#define   WHO_AM_I        0x75

#define mpu6050 0xD0  //Dia chi cua cam bien mpu6050 co 7 bit doi sang trai 1 bit de set read or write


#endif
