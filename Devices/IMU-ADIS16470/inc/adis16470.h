#ifndef __ADIS16470_H__
#define __ADIS16470_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "usart.h"
#include <stdint.h>

#define ADIS16470_IMU_UART      huart1
#define ADIS16470_PC_UART       huart7
#define ADIS16470_IMU_FREQUENCY_HZ   100U

#if ADIS16470_IMU_FREQUENCY_HZ > 255U
#error "ADIS16470_IMU_FREQUENCY_HZ must fit in uint8_t"
#endif

#define ADIS16470_UART_TX_TIMEOUT_MS 100U
#define ADIS16470_RX_LINE_BUFFER_SIZE 192U
#define ADIS16470_TX_TEXT_BUFFER_SIZE 128U
#define ADIS16470_SCALE_DENOMINATOR 2147483648LL
#define ADIS16470_ACCEL_SCALE_NUMERATOR 40000L
#define ADIS16470_GYRO_SCALE_NUMERATOR 216000L
#define ADIS16470_FIXED_DECIMAL_SCALE 1000000LL

typedef struct
{
    int32_t z_accel_output;
    int32_t y_accel_output;
    int32_t x_accel_output;
    int32_t z_gyro_output;
    int32_t y_gyro_output;
    int32_t x_gyro_output;
} ADIS16470_RAWIMUSA_Data;

typedef struct
{
    int64_t z_accel;
    int64_t y_accel;
    int64_t x_accel;
    int64_t z_gyro;
    int64_t y_gyro;
    int64_t x_gyro;
} ADIS16470_IMUSA_Data;

typedef struct
{
    int32_t yaw;
    int32_t pitch;
    int32_t roll;
} ADIS16470_YPR_Data;

typedef struct
{
    float angle;
    float bias;
    float rate;
    float p00;
    float p01;
    float p10;
    float p11;
} ADIS16470_KalmanFilter;

typedef uint8_t UCHAR;
typedef int32_t INT;
typedef uint32_t ULONG;

void ADIS16470_Init(void);
void ADIS16470_Start(void);
void ADIS16470_Stop(void);
void ADIS16470_ProcessReceivedData(void);
void ADIS16470_ProcessYPRData(const ADIS16470_IMUSA_Data *imu_data, ADIS16470_YPR_Data *ypr_data);
void ADIS16470_SendYPRData(const ADIS16470_YPR_Data *data);
// ULONG CalculateCRC32(UCHAR *szBuf, INT iSize);

#ifdef __cplusplus
}
#endif

#endif
