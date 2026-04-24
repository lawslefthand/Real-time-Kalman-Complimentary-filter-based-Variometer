#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include "mpu6050.h"

extern float calibration_const_global_roll_accel;
extern float calibration_const_global_pitch_accel;

extern float calibration_const_global_gx;
extern float calibration_const_global_gy;
extern float calibration_const_global_gz;

// LPF defines
//float rollraw = 0.0f;
//float pitchraw =0.0f;

//float Rollraw = 0.0f;
//float Pitchraw =0.0f;

float Roll = 0.0f;
float Pitch = 0.0f;

extern I2C_HandleTypeDef hi2c3;

// R_measure controls accel trust: low = trust accel, high = trust gyro. - aryan
// Increase R_measure as RPM/throttle increases to prevent thrust-induced tilt.
// Typical range: 0.3 (motors off) → 1.0–2.0 (in flight), never below 0.1 or above ~5.

// angle = how much we trust the reading of gyro if too low we trust its reading if high we dont - aryan
// bias is how much we do not trust gyro , low = bias changes slowly , high bias changes continuously

Kalman_t KalmanX = { .Q_angle = 0.001f, .Q_bias = 0.003f, .R_measure = 1.25f, };

Kalman_t KalmanY = { .Q_angle = 0.001f, .Q_bias = 0.003f, .R_measure = 0.80f, };

void Kalman_Init(Kalman_t *K) {
	K->angle = 0.0f;
	K->bias = 0.0f;

	K->P[0][0] = 1.0f;
	K->P[0][1] = 0.0f;
	K->P[1][0] = 0.0f;
	K->P[1][1] = 1.0f;
}

void mpu_init(void) {

	uint8_t who = 0;

	HAL_I2C_DeInit(&hi2c3);
	HAL_I2C_Init(&hi2c3);
	HAL_I2C_Mem_Read(&hi2c3,
	MPU6050_ADDR,
	WHO_AM_I_REG, 1, &who, 1, 100);

	printf("WHO_AM_I = 0x%02X\n", who);

	uint8_t data;

	data = 0x00;
	HAL_I2C_DeInit(&hi2c3);
	HAL_I2C_Init(&hi2c3);
	HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 50);

	data = 0x00;
	HAL_I2C_DeInit(&hi2c3);
	HAL_I2C_Init(&hi2c3);
	HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, 50);

	data = 0x00;
	HAL_I2C_DeInit(&hi2c3);
	HAL_I2C_Init(&hi2c3);
	HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, 50);

	data = 0x05;
	HAL_I2C_DeInit(&hi2c3);
	HAL_I2C_Init(&hi2c3);
	HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, CONFIG_REG, 1, &data, 1, 100);

	data = 0x03;
	HAL_I2C_DeInit(&hi2c3);
	HAL_I2C_Init(&hi2c3);
	HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 100);

	Kalman_Init(&KalmanX);
	Kalman_Init(&KalmanY);
}

float mpu_accel_read(int ret) {
	uint8_t Rec_Data[6];
	int16_t ax, ay, az;
	float Ax, Ay, Az;

	HAL_I2C_DeInit(&hi2c3);
	HAL_I2C_Init(&hi2c3);
	HAL_I2C_Mem_Read(&hi2c3, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 50);

	ax = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
	ay = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
	az = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);

	Ax = ax / 16384.0f;
	Ay = ay / 16384.0f;
	Az = az / 16384.0f;

	if (ret == 0)
		return Ax;
	else if (ret == 1)
		return Ay;
	else if (ret == 2)
		return Az;
	else
		return 0.0f;
}

float mpu_roll_pitch_read_accel(int ret) {
	uint8_t Rec_Data[6];
	int16_t ax, ay, az;
	float Ax, Ay, Az;

	HAL_I2C_DeInit(&hi2c3);
	HAL_I2C_Init(&hi2c3);
	HAL_I2C_Mem_Read(&hi2c3, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 50);

	ax = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
	ay = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
	az = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);

	Ax = ax / 16384.0f;
	Ay = ay / 16384.0f;
	Az = az / 16384.0f;

	Roll = atan2f(Ay, sqrtf(Ax * Ax + Az * Az)) * 180.0f / M_PI;
	Pitch = atan2f(-Ax, sqrtf(Ay * Ay + Az * Az)) * 180.0f / M_PI;

	if (ret == 0)
		return Roll;
	else if (ret == 1)
		return Pitch;
	else
		return 0.0f;
}

float mpu_roll_pitch_calibration_accel(int rp) {
	float r = 0, p = 0;
	for (int i = 0; i < 2000; i++) {
		r += mpu_roll_pitch_read_accel(0);
		p += mpu_roll_pitch_read_accel(1);
	}
	float rc = r / 2000.0f;
	float pc = p / 2000.0f;
	if (rp == 0)
		return rc;
	else if (rp == 1)
		return pc;
	else
		return 0.0f;
}

float mpu_roll_pitch_calibration_gyro(int rp) {
	float r = 0, p = 0;
	for (int i = 0; i < 2000; i++) {
		r += mpu_roll_pitch_read_gyro(0, 0.001f);
		p += mpu_roll_pitch_read_gyro(1, 0.001f);
	}
	float rc = r / 2000.0f;
	float pc = p / 2000.0f;
	if (rp == 0)
		return rc;
	else if (rp == 1)
		return pc;
	else
		return 0.0f;
}

float mpu_gyro_calibration(int axis) {
	float gx = 0, gy = 0, gz = 0;
	for (int i = 0; i < 2000; i++) {
		gx += mpu_gyro_read(0);
		gy += mpu_gyro_read(1);
		gz += mpu_gyro_read(2);
		HAL_Delay(1);
	}
	float ox = gx / 2000.0f;
	float oy = gy / 2000.0f;
	float oz = gz / 2000.0f;
	if (axis == 0)
		return ox;
	else if (axis == 1)
		return oy;
	else if (axis == 2)
		return oz;
	else
		return 0.0f;
}

float mpu_accel_calibration(int axis) {
	float ax = 0, ay = 0, az = 0;
	for (int i = 0; i < 2000; i++) {
		ax += mpu_accel_read(0);
		ay += mpu_accel_read(1);
		az += mpu_accel_read(2);
		HAL_Delay(1);
	}
	float ox = ax / 2000.0f;
	float oy = ay / 2000.0f;
	float oz = az / 2000.0f;
	if (axis == 0)
		return ox;
	else if (axis == 1)
		return oy;
	else if (axis == 2)
		return oz;
	else
		return 0.0f;
}

float mpu_gyro_read(int ret) {
	uint8_t Rec_Data[6];
	int16_t gx, gy, gz;
	float Gx, Gy, Gz;

	HAL_I2C_DeInit(&hi2c3);
	HAL_I2C_Init(&hi2c3);
	HAL_I2C_Mem_Read(&hi2c3, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 50);

	gx = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
	gy = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
	gz = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);

	Gx = gx / 131.0f;
	Gy = gy / 131.0f;
	Gz = gz / 131.0f;

	if (ret == 0)
		return Gx;
	else if (ret == 1)
		return Gy;
	else if (ret == 2)
		return Gz;
	else
		return 0.0f;
}

float mpu_roll_pitch_read_gyro(int ret, float dt) {
	static float roll = 0.0f;
	static float pitch = 0.0f;

	float Gx = mpu_gyro_read(0);
	float Gy = mpu_gyro_read(1);

	roll += Gx * dt;
	pitch += Gy * dt;

	if (ret == 0)
		return roll;
	else if (ret == 1)
		return pitch;
	else
		return 0.0f;
}
// 1 D Kalman Filter with Bias Calculation
double Kalman_get_angle(Kalman_t *Kalman, double newAngle, double newRate,
		double dt) {
	//Prediction Stage
	double rate = newRate - Kalman->bias;
	Kalman->angle += dt * rate;
	// Covariance prediction
	Kalman->P[0][0] += dt
			* (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0]
					+ Kalman->Q_angle);
	Kalman->P[0][1] -= dt * Kalman->P[1][1];
	Kalman->P[1][0] -= dt * Kalman->P[1][1];
	Kalman->P[1][1] += Kalman->Q_bias * dt;

	// Innovation
	double y = newAngle - Kalman->angle;  // Angle diff

	// Innovation Covariance
	double S = Kalman->P[0][0] + Kalman->R_measure; // Estimate error

	// Kalman Gain
	double K[2]; // Kalman gain - A 2x1 matrix
	K[0] = Kalman->P[0][0] / S;
	K[1] = Kalman->P[1][0] / S;

	// Update Angle
	Kalman->angle += K[0] * y;
	Kalman->bias += K[1] * y;
	//Update Covarience
	double tempA = Kalman->P[0][0];
	double tempB = Kalman->P[0][1];

	Kalman->P[0][0] -= K[0] * tempA;
	Kalman->P[0][1] -= K[0] * tempB;
	Kalman->P[1][0] -= K[1] * tempA;
	Kalman->P[1][1] -= K[1] * tempB;

	return Kalman->angle;
};

void mpu_get_kalman_angles(float *roll, float *pitch) {
	static uint32_t lastTick = 0;
	uint32_t now = HAL_GetTick();
	float dt = (now - lastTick) / 1000.0f;
	lastTick = now;

	/* Accel angles (degrees) */
	float accRoll = mpu_roll_pitch_read_accel(0);
			//- calibration_const_global_roll_accel;
	float accPitch = mpu_roll_pitch_read_accel(1);
			//- calibration_const_global_pitch_accel;

	/* Gyro rates (deg/sec) */
	float gyroRollRate = mpu_gyro_read(0); // - calibration_const_global_gx;
	float gyroPitchRate = mpu_gyro_read(1); //- calibration_const_global_gy;

	/* Kalman filter */
	*roll = Kalman_get_angle(&KalmanX, accRoll, gyroRollRate, dt);
	*pitch = Kalman_get_angle(&KalmanY, accPitch, gyroPitchRate, dt);
}
