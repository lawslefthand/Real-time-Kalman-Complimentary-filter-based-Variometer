/*
 * mpu6050_port.h
 *
 *  Created on: Oct 9, 2025
 *      Author: danba
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

/*
 * mpu6050_hal.h
 *
 * Created on: Oct 9, 2025
 * Author: YourName
 */

#ifndef MPU6050_HAL_H_
#define MPU6050_HAL_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define MPU6050_ADDR        (0x68 << 1)
#define MPU9250_MAG_ADDRESS 0x0C

#define WHO_AM_I_REG        0x75
#define PWR_MGMT_1_REG      0x6B
#define SMPLRT_DIV_REG      0x19
#define GYRO_CONFIG_REG     0x1B
#define ACCEL_CONFIG_REG    0x1C
#define CONFIG_REG          0x1A   // <— DLPF, EXT_SYNC settings
#define ACCEL_XOUT_H_REG    0x3B
#define GYRO_XOUT_H_REG     0x43



#define CNTLA 0x0A
#define INT_PIN_CFG 0x37


extern I2C_HandleTypeDef hi2c1;



float mpu_roll_pitch_calibration_accel(int rp);

float mpu_roll_pitch_calibration_gyro(int rp);


extern float Gx, Gy, Gz;

extern float mag_x, mag_y, mag_z;

extern float mag_adj_x, mag_adj_y, mag_adj_z;

// ------------------ Initialization ------------------
void mpu_init(void);

// ------------------ Accelerometer ------------------
float mpu_accel_read(int ret);
float mpu_roll_pitch_read_accel(int ret);
float mpu_accel_calibration(int axis);

// ------------------ Gyroscope ------------------
float mpu_gyro_read(int ret);
float mpu_roll_pitch_read_gyro(int ret, float dt);
float mpu_gyro_calibration(int axis);

// ------------------ Calibration (roll/pitch) ------------------
float mpu_roll_pitch_calibration_accel(int rp);
float mpu_roll_pitch_calibration_gyro(int rp);

//kALMAN
typedef struct {
    float Q_angle;
    float Q_bias;
    float R_measure;

    float angle;
    float bias;
    float P[2][2];
} Kalman_t;

double Kalman_get_angle(Kalman_t *Kalman,
                        double newAngle,
                        double newRate,
                        double dt);
void mpu_get_kalman_angles(float *roll, float *pitch);

#endif /* MPU6050_HAL_H_ */

#endif /* INC_MPU6050_H_ */
