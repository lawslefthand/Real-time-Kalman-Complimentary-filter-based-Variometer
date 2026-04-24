/*
 * bmp280.c
 */

#include "bmp280.h"
#include "math.h"

extern I2C_HandleTypeDef hi2c1;

static uint8_t calib_loaded = 0;

static uint16_t calib_T1;
static int16_t  calib_T2, calib_T3;

static uint16_t dig_P1;
static int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;


/* ================= I2C LOW LEVEL ================= */

void bmp_hal_i2c_write(uint8_t reg_addr, uint8_t value) {
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Write(&hi2c1, BMP280_I2C_ADDR,
                              reg_addr, I2C_MEMADD_SIZE_8BIT,
                              &value, 1, 50);

    if (status != HAL_OK) {
        HAL_I2C_DeInit(&hi2c1);
        HAL_I2C_Init(&hi2c1);
        bmp_i2c_setup();

        HAL_I2C_Mem_Write(&hi2c1, BMP280_I2C_ADDR,
                          reg_addr, I2C_MEMADD_SIZE_8BIT,
                          &value, 1, 50);
    }
}


uint8_t bmp_hal_i2c_read(uint8_t reg_addr) {
    uint8_t read_value = 0;
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(&hi2c1, BMP280_I2C_ADDR,
                             reg_addr, I2C_MEMADD_SIZE_8BIT,
                             &read_value, 1, 50);

    if (status != HAL_OK) {
        HAL_I2C_DeInit(&hi2c1);
        HAL_I2C_Init(&hi2c1);
        bmp_i2c_setup();

        HAL_I2C_Mem_Read(&hi2c1, BMP280_I2C_ADDR,
                         reg_addr, I2C_MEMADD_SIZE_8BIT,
                         &read_value, 1, 50);
    }

    return read_value;
}


/* ================= CALIB ================= */

static void bmp_load_calib(void) {
    if (calib_loaded) return;

    uint8_t buf[24];
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(&hi2c1, BMP280_I2C_ADDR,
                             0x88, I2C_MEMADD_SIZE_8BIT,
                             buf, 24, 50);

    if (status != HAL_OK) {
        HAL_I2C_DeInit(&hi2c1);
        HAL_I2C_Init(&hi2c1);
        bmp_i2c_setup();

        HAL_I2C_Mem_Read(&hi2c1, BMP280_I2C_ADDR,
                         0x88, I2C_MEMADD_SIZE_8BIT,
                         buf, 24, 50);
    }

    calib_T1 = (buf[1] << 8) | buf[0];
    calib_T2 = (buf[3] << 8) | buf[2];
    calib_T3 = (buf[5] << 8) | buf[4];

    dig_P1 = (buf[7]  << 8) | buf[6];
    dig_P2 = (buf[9]  << 8) | buf[8];
    dig_P3 = (buf[11] << 8) | buf[10];
    dig_P4 = (buf[13] << 8) | buf[12];
    dig_P5 = (buf[15] << 8) | buf[14];
    dig_P6 = (buf[17] << 8) | buf[16];
    dig_P7 = (buf[19] << 8) | buf[18];
    dig_P8 = (buf[21] << 8) | buf[20];
    dig_P9 = (buf[23] << 8) | buf[22];

    calib_loaded = 1;
}


/* ================= TEMP ================= */

double temperature(int x) {
    bmp_load_calib();

    uint8_t tbuf[3];
    signed long raw_temperature;
    double var1, var2;
    uint32_t t_fine;
    double final_temp;

    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(&hi2c1, BMP280_I2C_ADDR,
                             0xFA, I2C_MEMADD_SIZE_8BIT,
                             tbuf, 3, 1);

    if (status != HAL_OK) {
        HAL_I2C_DeInit(&hi2c1);
        HAL_I2C_Init(&hi2c1);
        bmp_i2c_setup();

        HAL_I2C_Mem_Read(&hi2c1, BMP280_I2C_ADDR,
                         0xFA, I2C_MEMADD_SIZE_8BIT,
                         tbuf, 3, 1);
    }

    raw_temperature = (tbuf[0] << 12) | (tbuf[1] << 4) | (tbuf[2] >> 4);

    var1 = (((raw_temperature / 16384.0) - (calib_T1 / 1024.0)) * calib_T2);
    var2 = (((raw_temperature / 131072.0) - (calib_T1 / 8192.0)) *
            ((raw_temperature / 131072.0) - (calib_T1 / 8192.0)) *
            calib_T3);

    t_fine = (uint32_t)(var1 + var2);
    final_temp = t_fine / 5120.0;

    if (x == 1) return t_fine;
    if (x == 0) return final_temp;
    return 0;
}


/* ================= PRESSURE ================= */

double pressure(void) {
    bmp_load_calib();

    uint8_t pbuf[3];
    signed long raw_pressure;
    double var1, var2, p;
    uint32_t t_fine;

    t_fine = (uint32_t)temperature(1);

    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(&hi2c1, BMP280_I2C_ADDR,
                             0xF7, I2C_MEMADD_SIZE_8BIT,
                             pbuf, 3, 1);

    if (status != HAL_OK) {
        HAL_I2C_DeInit(&hi2c1);
        HAL_I2C_Init(&hi2c1);
        bmp_i2c_setup();

        HAL_I2C_Mem_Read(&hi2c1, BMP280_I2C_ADDR,
                         0xF7, I2C_MEMADD_SIZE_8BIT,
                         pbuf, 3, 1);
    }

    raw_pressure = (pbuf[0] << 12) | (pbuf[1] << 4) | (pbuf[2] >> 4);

    var1 = ((double)t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * dig_P6 / 32768.0;
    var2 = var2 + var1 * dig_P5 * 2.0;
    var2 = (var2 / 4.0) + (dig_P4 * 65536.0);
    var1 = (dig_P3 * var1 * var1 / 524288.0 + dig_P2 * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * dig_P1;

    if (var1 == 0.0) return 0;

    p = 1048576.0 - raw_pressure;
    p = (p - (var2 / 4096.0)) * 6250.0 / var1;
    var1 = dig_P9 * p * p / 2147483648.0;
    var2 = p * dig_P8 / 32768.0;
    p = p + (var1 + var2 + dig_P7) / 16.0;

    return p;
}


/* ================= SETUP ================= */

void bmp_i2c_setup(void) {

    bmp_hal_i2c_write(0xF5, 0x00);
    bmp_hal_i2c_write(0xF4, 0x27);
}


/* ================= ALTITUDE ================= */

float altitude_calc(void) {
    static float refPressure = 0.0f;

    float pressure_hPa = pressure() / 100.0f;
    float tempC = temperature(0);

    if (refPressure == 0.0f) {
        refPressure = pressure_hPa;
    }

    float altitude =
        (powf(refPressure / pressure_hPa, 1.0f / 5.257f) - 1.0f) *
        (tempC + 273.15f) / 0.0065f;

    return altitude;
}
