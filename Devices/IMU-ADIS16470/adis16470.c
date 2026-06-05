#include "adis16470.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const float ADIS16470_RAD_TO_DEG = 57.2957795131f;
static const float ADIS16470_KALMAN_Q_ANGLE = 0.001f;
static const float ADIS16470_KALMAN_Q_BIAS = 0.003f;
static const float ADIS16470_KALMAN_R_MEASURE = 0.03f;

static uint8_t adis16470_imu_rx_byte;
static uint8_t adis16470_is_running;
static char adis16470_rx_line_buffer[ADIS16470_RX_LINE_BUFFER_SIZE];
static char adis16470_ready_line_buffer[ADIS16470_RX_LINE_BUFFER_SIZE];
static uint16_t adis16470_rx_line_index;
static volatile uint8_t adis16470_line_ready;
static float adis16470_yaw_deg;
static float adis16470_pitch_deg;
static float adis16470_roll_deg;
static uint32_t adis16470_ypr_last_tick;
static uint8_t adis16470_ypr_initialized;
static ADIS16470_KalmanFilter adis16470_pitch_kalman;
static ADIS16470_KalmanFilter adis16470_roll_kalman;

// ??????????
static void ADIS16470_ResetKalman(ADIS16470_KalmanFilter *filter, float initial_angle)
{
    filter->angle = initial_angle;
    filter->bias = 0.0f;
    filter->rate = 0.0f;
    filter->p00 = 0.0f;
    filter->p01 = 0.0f;
    filter->p10 = 0.0f;
    filter->p11 = 0.0f;
}

// ??????????????????
static float ADIS16470_UpdateKalman(ADIS16470_KalmanFilter *filter, float measured_angle, float gyro_rate, float dt)
{
    float innovation;
    float innovation_covariance;
    float kalman_gain0;
    float kalman_gain1;
    float p00_temp;
    float p01_temp;

    filter->rate = gyro_rate - filter->bias;
    filter->angle += dt * filter->rate;

    filter->p00 += dt * ((dt * filter->p11) - filter->p01 - filter->p10 + ADIS16470_KALMAN_Q_ANGLE);
    filter->p01 -= dt * filter->p11;
    filter->p10 -= dt * filter->p11;
    filter->p11 += ADIS16470_KALMAN_Q_BIAS * dt;

    innovation = measured_angle - filter->angle;
    innovation_covariance = filter->p00 + ADIS16470_KALMAN_R_MEASURE;
    kalman_gain0 = filter->p00 / innovation_covariance;
    kalman_gain1 = filter->p10 / innovation_covariance;

    filter->angle += kalman_gain0 * innovation;
    filter->bias += kalman_gain1 * innovation;

    p00_temp = filter->p00;
    p01_temp = filter->p01;

    filter->p00 -= kalman_gain0 * p00_temp;
    filter->p01 -= kalman_gain0 * p01_temp;
    filter->p10 -= kalman_gain1 * p00_temp;
    filter->p11 -= kalman_gain1 * p01_temp;

    return filter->angle;
}

// ??IMU??????
static void ADIS16470_EnableUARTIRQ(UART_HandleTypeDef *huart)
{
#if defined(USART1)
    if (huart->Instance == USART1)
    {
        HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
        return;
    }
#endif
#if defined(USART2)
    if (huart->Instance == USART2)
    {
        HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
        return;
    }
#endif
#if defined(USART3)
    if (huart->Instance == USART3)
    {
        HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART3_IRQn);
        return;
    }
#endif
#if defined(UART4)
    if (huart->Instance == UART4)
    {
        HAL_NVIC_SetPriority(UART4_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(UART4_IRQn);
        return;
    }
#endif
#if defined(UART5)
    if (huart->Instance == UART5)
    {
        HAL_NVIC_SetPriority(UART5_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(UART5_IRQn);
        return;
    }
#endif
#if defined(USART6)
    if (huart->Instance == USART6)
    {
        HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART6_IRQn);
        return;
    }
#endif
#if defined(UART7)
    if (huart->Instance == UART7)
    {
        HAL_NVIC_SetPriority(UART7_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(UART7_IRQn);
        return;
    }
#endif
#if defined(UART8)
    if (huart->Instance == UART8)
    {
        HAL_NVIC_SetPriority(UART8_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(UART8_IRQn);
        return;
    }
#endif
#if defined(UART9)
    if (huart->Instance == UART9)
    {
        HAL_NVIC_SetPriority(UART9_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(UART9_IRQn);
        return;
    }
#endif
#if defined(USART10)
    if (huart->Instance == USART10)
    {
        HAL_NVIC_SetPriority(USART10_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART10_IRQn);
        return;
    }
#endif
}

// UART????????????????????????
static void ADIS16470_UART_IRQHandler(USART_TypeDef *instance)
{
    if (ADIS16470_IMU_UART.Instance == instance)
    {
        HAL_UART_IRQHandler(&ADIS16470_IMU_UART);
    }
}

// ???????????????
static void ADIS16470_ResetReceiveLine(void)
{
    adis16470_rx_line_index = 0U;
    adis16470_rx_line_buffer[0] = '\0';
    adis16470_line_ready = 0U;
}

// ???????????????????????????????????
static void ADIS16470_PushReceivedByte(uint8_t byte)
{
    if ((byte == '\r') || (byte == '\n'))
    {
        adis16470_rx_line_buffer[adis16470_rx_line_index] = '\0';

        if ((adis16470_rx_line_index > 0U) && (adis16470_line_ready == 0U))
        {
            memcpy(adis16470_ready_line_buffer, adis16470_rx_line_buffer, adis16470_rx_line_index + 1U);
            adis16470_line_ready = 1U;
        }

        adis16470_rx_line_index = 0U;
        return;
    }

    if (adis16470_rx_line_index < (ADIS16470_RX_LINE_BUFFER_SIZE - 1U))
    {
        adis16470_rx_line_buffer[adis16470_rx_line_index] = (char)byte;
        adis16470_rx_line_index++;
    }
    else
    {
        ADIS16470_ResetReceiveLine();
    }
}

// ?????????PC???
static void ADIS16470_SendPCText(const char *text)
{
    HAL_UART_Transmit(&ADIS16470_PC_UART, (uint8_t *)text, (uint16_t)strlen(text), ADIS16470_UART_TX_TIMEOUT_MS);
}

// ????????IMU?
static void ADIS16470_SendCommand(const char *cmd, uint16_t len)
{
    HAL_UART_Transmit(&ADIS16470_IMU_UART, (uint8_t *)cmd, len, ADIS16470_UART_TX_TIMEOUT_MS);
}

// ????????????????IMU??????????
static void ADIS16470_StartWithFrequency(uint8_t *frequency)
{
    const char *cmd;

    switch (*frequency)
    {
    case 100U:
        cmd = "log rawimusa ontime 0.01\r\n";
        break;
    case 50U:
        cmd = "log rawimusa ontime 0.02\r\n";
        break;
    case 20U:
        cmd = "log rawimusa ontime 0.05\r\n";
        break;
    case 10U:
        cmd = "log rawimusa ontime 0.1\r\n";
        break;
    case 5U:
        cmd = "log rawimusa ontime 0.2\r\n";
        break;
    case 1U:
        cmd = "log rawimusa ontime 1\r\n";
        break;
    default:
        cmd = "log rawimusa ontime 0.01\r\n";
        break;
    }

    ADIS16470_SendCommand(cmd, (uint16_t)strlen(cmd));
}

// ??ASCII?????????
static uint8_t ADIS16470_SkipAsciiField(const char **cursor)
{
    const char *comma = strchr(*cursor, ',');

    if (comma == NULL)
    {
        return 0U;
    }

    *cursor = comma + 1U;
    return 1U;
}

// ??ASCII??????????????
static uint8_t ADIS16470_ParseLongAsciiField(const char **cursor, int32_t *value)
{
    char *end;
    long parsed = strtol(*cursor, &end, 10);

    if (end == *cursor)
    {
        return 0U;
    }

    *value = (int32_t)parsed;

    if (*end == ',')
    {
        *cursor = end + 1U;
        return 1U;
    }

    if ((*end == '*') || (*end == '\r') || (*end == '\n') || (*end == '\0'))
    {
        *cursor = end;
        return 1U;
    }

    return 0U;
}

// ??RAWIMUSA????????IMU??
static uint8_t ADIS16470_ParseRAWIMUSA(const char *sentence, ADIS16470_RAWIMUSA_Data *data)
{
    const char *cursor;
    const char *rawimusa;
    ADIS16470_RAWIMUSA_Data parsed_data;

    if ((sentence == NULL) || (data == NULL))
    {
        return 0U;
    }

    memset(data, 0, sizeof(*data));
    memset(&parsed_data, 0, sizeof(parsed_data));

    if (!adis16470_is_running)
    {
        return 0U;
    }

    rawimusa = strstr(sentence, "RAWIMUSA");
    if (rawimusa == NULL)
    {
        return 0U;
    }

    cursor = strchr(rawimusa, ';');
    if (cursor == NULL)
    {
        return 0U;
    }
    cursor++;

    if (!ADIS16470_SkipAsciiField(&cursor)) return 0U; /* Week */
    if (!ADIS16470_SkipAsciiField(&cursor)) return 0U; /* Seconds into week */
    if (!ADIS16470_SkipAsciiField(&cursor)) return 0U; /* IMU Status */
    
    if (!ADIS16470_ParseLongAsciiField(&cursor, &parsed_data.z_accel_output)) return 0U;
    if (!ADIS16470_ParseLongAsciiField(&cursor, &parsed_data.y_accel_output)) return 0U;
    if (!ADIS16470_ParseLongAsciiField(&cursor, &parsed_data.x_accel_output)) return 0U;
    if (!ADIS16470_ParseLongAsciiField(&cursor, &parsed_data.z_gyro_output)) return 0U;
    if (!ADIS16470_ParseLongAsciiField(&cursor, &parsed_data.y_gyro_output)) return 0U;
    if (!ADIS16470_ParseLongAsciiField(&cursor, &parsed_data.x_gyro_output)) return 0U;

    *data = parsed_data;

    return 1U;
}

// ???IMU??????????????????6???
static int64_t ADIS16470_ScaleRawToFixed6(int32_t raw, int32_t numerator)
{
    int64_t scaled_num = (int64_t)raw * (int64_t)numerator * ADIS16470_FIXED_DECIMAL_SCALE;

    if (scaled_num >= 0)
    {
        return (scaled_num + (ADIS16470_SCALE_DENOMINATOR / 2LL)) / ADIS16470_SCALE_DENOMINATOR;
    }

    return (scaled_num - (ADIS16470_SCALE_DENOMINATOR / 2LL)) / ADIS16470_SCALE_DENOMINATOR;
}

// ????????????????????6?????????
static void ADIS16470_WriteFixed6Text(char *buffer, uint16_t size, int64_t value)
{
    int64_t abs_value = (value < 0) ? -value : value;

    snprintf(buffer, size, "%s%lld.%06lld",
             (value < 0) ? "-" : "",
             (long long)(abs_value / ADIS16470_FIXED_DECIMAL_SCALE),
             (long long)(abs_value % ADIS16470_FIXED_DECIMAL_SCALE));
}

// ????IMU?????IMU??
static void ADIS16470_ProcessIMUSAData(const ADIS16470_RAWIMUSA_Data *raw_data, ADIS16470_IMUSA_Data *imu_data)
{
    imu_data->z_accel = ADIS16470_ScaleRawToFixed6(raw_data->z_accel_output, ADIS16470_ACCEL_SCALE_NUMERATOR);
    imu_data->y_accel = ADIS16470_ScaleRawToFixed6(-raw_data->y_accel_output, ADIS16470_ACCEL_SCALE_NUMERATOR);
    imu_data->x_accel = ADIS16470_ScaleRawToFixed6(raw_data->x_accel_output, ADIS16470_ACCEL_SCALE_NUMERATOR);
    imu_data->z_gyro = ADIS16470_ScaleRawToFixed6(raw_data->z_gyro_output, ADIS16470_GYRO_SCALE_NUMERATOR);
    imu_data->y_gyro = ADIS16470_ScaleRawToFixed6(-raw_data->y_gyro_output, ADIS16470_GYRO_SCALE_NUMERATOR);
    imu_data->x_gyro = ADIS16470_ScaleRawToFixed6(raw_data->x_gyro_output, ADIS16470_GYRO_SCALE_NUMERATOR);
}

// ?IMU?????PC???
static void ADIS16470_SendIMUSAData(const ADIS16470_IMUSA_Data *data)
{
    char z_accel[16];
    char y_accel[16];
    char x_accel[16];
    char z_gyro[16];
    char y_gyro[16];
    char x_gyro[16];
    char tx_buffer[ADIS16470_TX_TEXT_BUFFER_SIZE];
    int len;

    ADIS16470_WriteFixed6Text(z_accel, sizeof(z_accel), data->z_accel);
    ADIS16470_WriteFixed6Text(y_accel, sizeof(y_accel), data->y_accel);
    ADIS16470_WriteFixed6Text(x_accel, sizeof(x_accel), data->x_accel);
    ADIS16470_WriteFixed6Text(z_gyro, sizeof(z_gyro), data->z_gyro);
    ADIS16470_WriteFixed6Text(y_gyro, sizeof(y_gyro), data->y_gyro);
    ADIS16470_WriteFixed6Text(x_gyro, sizeof(x_gyro), data->x_gyro);

    len = snprintf(tx_buffer, sizeof(tx_buffer), 
                   "%s,%s,%s,%s,%s,%s\r\n",
                   z_accel,
                   y_accel,
                   x_accel,
                   z_gyro,
                   y_gyro,
                   x_gyro);

    if ((len > 0) && (len < (int)sizeof(tx_buffer)))
    {
        ADIS16470_SendPCText(tx_buffer);
    }
}

// ???????-180?180????
static float ADIS16470_WrapAngleDeg(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }

    while (angle < -180.0f)
    {
        angle += 360.0f;
    }

    return angle;
}

// ????????????????????6???
static int32_t ADIS16470_DegToFixed6(float value)
{
    if (value >= 0.0f)
    {
        return (int32_t)((value * (float)ADIS16470_FIXED_DECIMAL_SCALE) + 0.5f);
    }

    return (int32_t)((value * (float)ADIS16470_FIXED_DECIMAL_SCALE) - 0.5f);
}

// ????????????
static void ADIS16470_ResetYPR(void)
{
    adis16470_yaw_deg = 0.0f;
    adis16470_pitch_deg = 0.0f;
    adis16470_roll_deg = 0.0f;
    adis16470_ypr_last_tick = 0U;
    adis16470_ypr_initialized = 0U;
    ADIS16470_ResetKalman(&adis16470_pitch_kalman, 0.0f);
    ADIS16470_ResetKalman(&adis16470_roll_kalman, 0.0f);
}

// ??IMU????????
void ADIS16470_ProcessYPRData(const ADIS16470_IMUSA_Data *imu_data, ADIS16470_YPR_Data *ypr_data)
{
    float x_accel;
    float y_accel;
    float z_accel;
    float x_gyro;
    float y_gyro;
    float z_gyro;
    float pitch_acc_deg;
    float roll_acc_deg;
    float dt;
    uint32_t now;

    if ((imu_data == NULL) || (ypr_data == NULL))
    {
        return;
    }

    x_accel = imu_data->x_accel;
    y_accel = imu_data->y_accel;
    z_accel = imu_data->z_accel;
    x_gyro = imu_data->x_gyro;
    y_gyro = imu_data->y_gyro;
    z_gyro = imu_data->z_gyro;

    roll_acc_deg = atan2f(y_accel, z_accel) * ADIS16470_RAD_TO_DEG;
    pitch_acc_deg = atan2f(-x_accel, sqrtf((y_accel * y_accel) + (z_accel * z_accel))) * ADIS16470_RAD_TO_DEG;
    now = HAL_GetTick();

    if (!adis16470_ypr_initialized)
    {
        adis16470_yaw_deg = 0.0f;
        adis16470_pitch_deg = pitch_acc_deg;
        adis16470_roll_deg = roll_acc_deg;
        ADIS16470_ResetKalman(&adis16470_pitch_kalman, pitch_acc_deg);
        ADIS16470_ResetKalman(&adis16470_roll_kalman, roll_acc_deg);
        adis16470_ypr_last_tick = now;
        adis16470_ypr_initialized = 1U;
    }
    else
    {
        dt = (float)(now - adis16470_ypr_last_tick) / 1000.0f;
        adis16470_ypr_last_tick = now;

        if (dt <= 0.0f)
        {
            dt = 1.0f / (float)ADIS16470_IMU_FREQUENCY_HZ;
        }

        adis16470_yaw_deg += z_gyro * dt;
        adis16470_roll_deg = ADIS16470_UpdateKalman(&adis16470_roll_kalman, roll_acc_deg, x_gyro, dt);
        adis16470_pitch_deg = ADIS16470_UpdateKalman(&adis16470_pitch_kalman, pitch_acc_deg, y_gyro, dt);
        adis16470_yaw_deg = ADIS16470_WrapAngleDeg(adis16470_yaw_deg);
    }

    ypr_data->yaw = ADIS16470_DegToFixed6(adis16470_yaw_deg);
    ypr_data->pitch = ADIS16470_DegToFixed6(adis16470_pitch_deg);
    ypr_data->roll = ADIS16470_DegToFixed6(adis16470_roll_deg);
}

// ?????????PC???
void ADIS16470_SendYPRData(const ADIS16470_YPR_Data *data)
{
    char yaw[16];
    char pitch[16];
    char roll[16];
    char tx_buffer[ADIS16470_TX_TEXT_BUFFER_SIZE];
    int len;

    ADIS16470_WriteFixed6Text(yaw, sizeof(yaw), data->yaw);
    ADIS16470_WriteFixed6Text(pitch, sizeof(pitch), data->pitch);
    ADIS16470_WriteFixed6Text(roll, sizeof(roll), data->roll);

    len = snprintf(tx_buffer, sizeof(tx_buffer), 
                   "%s,%s,%s\r\n",
                   yaw,
                   pitch,
                   roll);

    if ((len > 0) && (len < (int)sizeof(tx_buffer)))
    {
        ADIS16470_SendPCText(tx_buffer);
    }
}


// static uint8_t ADIS16470_CheckAsciiCRC(const char *sentence)
// {
//     const char *crc_start;
//     char *crc_end;
//     uint32_t expected_crc;
//     uint32_t calculated_crc;

//     if ((sentence == NULL) || (*sentence == '\0'))
//     {
//         return 0U;
//     }

//     crc_start = strchr(sentence, '*');
//     if (crc_start == NULL)
//     {
//         return 0U;
//     }

//     expected_crc = (uint32_t)strtoul(crc_start + 1U, &crc_end, 16);
//     if (crc_end == (crc_start + 1U))
//     {
//         return 0U;
//     }

//     calculated_crc = CalculateCRC32((UCHAR *)(sentence + 1U), (INT)(crc_start - sentence - 1));
//     if (calculated_crc == expected_crc)
//     {
//         return 1U;
//     }

//     calculated_crc = CalculateCRC32((UCHAR *)sentence, (INT)(crc_start - sentence));

//     return (calculated_crc == expected_crc) ? 1U : 0U;
// }
// IMU????????UART???????????
void ADIS16470_Init(void)
{
    ADIS16470_EnableUARTIRQ(&ADIS16470_IMU_UART);
    ADIS16470_ResetReceiveLine();
    HAL_UART_Receive_IT(&ADIS16470_IMU_UART, &adis16470_imu_rx_byte, 1U);
}

// UART??????????????????????????????????????
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &ADIS16470_IMU_UART)
    {
        ADIS16470_PushReceivedByte(adis16470_imu_rx_byte);
        HAL_UART_Receive_IT(&ADIS16470_IMU_UART, &adis16470_imu_rx_byte, 1U);
    }
}

// UART????????????????????????????????????
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &ADIS16470_IMU_UART)
    {
        HAL_UART_Receive_IT(&ADIS16470_IMU_UART, &adis16470_imu_rx_byte, 1U);
    }
}

// ??IMU???????IMU?????????????????
void ADIS16470_Start(void)
{
    uint8_t frequency = ADIS16470_IMU_FREQUENCY_HZ;
    ADIS16470_ResetReceiveLine();
    ADIS16470_ResetYPR();

    adis16470_is_running = 1U;
    ADIS16470_StartWithFrequency(&frequency);
}

// ??IMU??????????????????
void ADIS16470_Stop(void)
{
    static const char cmd[] = "unlog\r\n";
    ADIS16470_SendCommand(cmd, sizeof(cmd) - 1U);

    adis16470_is_running = 0U;
    ADIS16470_ResetReceiveLine();
    ADIS16470_ResetYPR();
}

// ??????????????IMU????????????????PC???
void ADIS16470_ProcessReceivedData(void)
{
    ADIS16470_RAWIMUSA_Data raw_data;
    ADIS16470_IMUSA_Data imu_data;
    ADIS16470_YPR_Data ypr_data;
    char line[ADIS16470_RX_LINE_BUFFER_SIZE];

    if (!adis16470_is_running)
    {
        return;
    }

    if (!adis16470_line_ready)
    {
        return;
    }

    __disable_irq();
    memcpy(line, adis16470_ready_line_buffer, sizeof(line));
    adis16470_line_ready = 0U;
    __enable_irq();

    if (ADIS16470_ParseRAWIMUSA(line, &raw_data))
    {
        ADIS16470_ProcessIMUSAData(&raw_data, &imu_data);
        // ADIS16470_SendIMUSAData(&imu_data);
        ADIS16470_ProcessYPRData(&imu_data, &ypr_data);
        ADIS16470_SendYPRData(&ypr_data);
    }
}

// const ULONG aulCrcTable[256] =
// {
//     0x00000000UL, 0x77073096UL, 0xee0e612cUL, 0x990951baUL, 0x076dc419UL, 0x706af48fUL, 0xe963a535UL, 0x9e6495a3UL,
//     0x0edb8832UL, 0x79dcb8a4UL, 0xe0d5e91eUL, 0x97d2d988UL, 0x09b64c2bUL, 0x7eb17cbdUL, 0xe7b82d07UL, 0x90bf1d91UL,
//     0x1db71064UL, 0x6ab020f2UL, 0xf3b97148UL, 0x84be41deUL, 0x1adad47dUL, 0x6ddde4ebUL, 0xf4d4b551UL, 0x83d385c7UL,
//     0x136c9856UL, 0x646ba8c0UL, 0xfd62f97aUL, 0x8a65c9ecUL, 0x14015c4fUL, 0x63066cd9UL, 0xfa0f3d63UL, 0x8d080df5UL,
//     0x3b6e20c8UL, 0x4c69105eUL, 0xd56041e4UL, 0xa2677172UL, 0x3c03e4d1UL, 0x4b04d447UL, 0xd20d85fdUL, 0xa50ab56bUL,
//     0x35b5a8faUL, 0x42b2986cUL, 0xdbbbc9d6UL, 0xacbcf940UL, 0x32d86ce3UL, 0x45df5c75UL, 0xdcd60dcfUL, 0xabd13d59UL,
//     0x26d930acUL, 0x51de003aUL, 0xc8d75180UL, 0xbfd06116UL, 0x21b4f4b5UL, 0x56b3c423UL, 0xcfba9599UL, 0xb8bda50fUL,
//     0x2802b89eUL, 0x5f058808UL, 0xc60cd9b2UL, 0xb10be924UL, 0x2f6f7c87UL, 0x58684c11UL, 0xc1611dabUL, 0xb6662d3dUL,
//     0x76dc4190UL, 0x01db7106UL, 0x98d220bcUL, 0xefd5102aUL, 0x71b18589UL, 0x06b6b51fUL, 0x9fbfe4a5UL, 0xe8b8d433UL,
//     0x7807c9a2UL, 0x0f00f934UL, 0x9609a88eUL, 0xe10e9818UL, 0x7f6a0dbbUL, 0x086d3d2dUL, 0x91646c97UL, 0xe6635c01UL,
//     0x6b6b51f4UL, 0x1c6c6162UL, 0x856530d8UL, 0xf262004eUL, 0x6c0695edUL, 0x1b01a57bUL, 0x8208f4c1UL, 0xf50fc457UL,
//     0x65b0d9c6UL, 0x12b7e950UL, 0x8bbeb8eaUL, 0xfcb9887cUL, 0x62dd1ddfUL, 0x15da2d49UL, 0x8cd37cf3UL, 0xfbd44c65UL,
//     0x4db26158UL, 0x3ab551ceUL, 0xa3bc0074UL, 0xd4bb30e2UL, 0x4adfa541UL, 0x3dd895d7UL, 0xa4d1c46dUL, 0xd3d6f4fbUL,
//     0x4369e96aUL, 0x346ed9fcUL, 0xad678846UL, 0xda60b8d0UL, 0x44042d73UL, 0x33031de5UL, 0xaa0a4c5fUL, 0xdd0d7cc9UL,
//     0x5005713cUL, 0x270241aaUL, 0xbe0b1010UL, 0xc90c2086UL, 0x5768b525UL, 0x206f85b3UL, 0xb966d409UL, 0xce61e49fUL,
//     0x5edef90eUL, 0x29d9c998UL, 0xb0d09822UL, 0xc7d7a8b4UL, 0x59b33d17UL, 0x2eb40d81UL, 0xb7bd5c3bUL, 0xc0ba6cadUL,
//     0xedb88320UL, 0x9abfb3b6UL, 0x03b6e20cUL, 0x74b1d29aUL, 0xead54739UL, 0x9dd277afUL, 0x04db2615UL, 0x73dc1683UL,
//     0xe3630b12UL, 0x94643b84UL, 0x0d6d6a3eUL, 0x7a6a5aa8UL, 0xe40ecf0bUL, 0x9309ff9dUL, 0x0a00ae27UL, 0x7d079eb1UL,
//     0xf00f9344UL, 0x8708a3d2UL, 0x1e01f268UL, 0x6906c2feUL, 0xf762575dUL, 0x806567cbUL, 0x196c3671UL, 0x6e6b06e7UL,
//     0xfed41b76UL, 0x89d32be0UL, 0x10da7a5aUL, 0x67dd4accUL, 0xf9b9df6fUL, 0x8ebeeff9UL, 0x17b7be43UL, 0x60b08ed5UL,
//     0xd6d6a3e8UL, 0xa1d1937eUL, 0x38d8c2c4UL, 0x4fdff252UL, 0xd1bb67f1UL, 0xa6bc5767UL, 0x3fb506ddUL, 0x48b2364bUL,
//     0xd80d2bdaUL, 0xaf0a1b4cUL, 0x36034af6UL, 0x41047a60UL, 0xdf60efc3UL, 0xa867df55UL, 0x316e8eefUL, 0x4669be79UL,
//     0xcb61b38cUL, 0xbc66831aUL, 0x256fd2a0UL, 0x5268e236UL, 0xcc0c7795UL, 0xbb0b4703UL, 0x220216b9UL, 0x5505262fUL,
//     0xc5ba3bbeUL, 0xb2bd0b28UL, 0x2bb45a92UL, 0x5cb36a04UL, 0xc2d7ffa7UL, 0xb5d0cf31UL, 0x2cd99e8bUL, 0x5bdeae1dUL,
//     0x9b64c2b0UL, 0xec63f226UL, 0x756aa39cUL, 0x026d930aUL, 0x9c0906a9UL, 0xeb0e363fUL, 0x72076785UL, 0x05005713UL,
//     0x95bf4a82UL, 0xe2b87a14UL, 0x7bb12baeUL, 0x0cb61b38UL, 0x92d28e9bUL, 0xe5d5be0dUL, 0x7cdcefb7UL, 0x0bdbdf21UL,
//     0x86d3d2d4UL, 0xf1d4e242UL, 0x68ddb3f8UL, 0x1fda836eUL, 0x81be16cdUL, 0xf6b9265bUL, 0x6fb077e1UL, 0x18b74777UL,
//     0x88085ae6UL, 0xff0f6a70UL, 0x66063bcaUL, 0x11010b5cUL, 0x8f659effUL, 0xf862ae69UL, 0x616bffd3UL, 0x166ccf45UL,
//     0xa00ae278UL, 0xd70dd2eeUL, 0x4e048354UL, 0x3903b3c2UL, 0xa7672661UL, 0xd06016f7UL, 0x4969474dUL, 0x3e6e77dbUL,
//     0xaed16a4aUL, 0xd9d65adcUL, 0x40df0b66UL, 0x37d83bf0UL, 0xa9bcae53UL, 0xdebb9ec5UL, 0x47b2cf7fUL, 0x30b5ffe9UL,
//     0xbdbdf21cUL, 0xcabac28aUL, 0x53b39330UL, 0x24b4a3a6UL, 0xbad03605UL, 0xcdd70693UL, 0x54de5729UL, 0x23d967bfUL,
//     0xb3667a2eUL, 0xc4614ab8UL, 0x5d681b02UL, 0x2a6f2b94UL, 0xb40bbe37UL, 0xc30c8ea1UL, 0x5a05df1bUL, 0x2d02ef8dUL
// };

// ULONG CalculateCRC32(UCHAR *szBuf, INT iSize)
// {
//     int iIndex;
//     ULONG ulCRC = 0;
//     for (iIndex = 0; iIndex < iSize; iIndex++)
//     {
//         ulCRC = aulCrcTable[(ulCRC ^ szBuf[iIndex]) & 0xff] ^ (ulCRC >> 8);
//     }
//     return ulCRC;
// }

#if defined(USART1)
void USART1_IRQHandler(void)
{
    ADIS16470_UART_IRQHandler(USART1);
}
#endif

#if defined(USART2)
void USART2_IRQHandler(void)
{
    ADIS16470_UART_IRQHandler(USART2);
}
#endif

#if defined(USART3)
void USART3_IRQHandler(void)
{
    ADIS16470_UART_IRQHandler(USART3);
}
#endif

#if defined(UART4)
void UART4_IRQHandler(void)
{
    ADIS16470_UART_IRQHandler(UART4);
}
#endif

#if defined(UART5)
void UART5_IRQHandler(void)
{
    ADIS16470_UART_IRQHandler(UART5);
}
#endif

#if defined(USART6)
void USART6_IRQHandler(void)
{
    ADIS16470_UART_IRQHandler(USART6);
}
#endif

#if defined(UART7)
void UART7_IRQHandler(void)
{
    ADIS16470_UART_IRQHandler(UART7);
}
#endif

#if defined(UART8)
void UART8_IRQHandler(void)
{
    ADIS16470_UART_IRQHandler(UART8);
}
#endif

#if defined(UART9)
void UART9_IRQHandler(void)
{
    ADIS16470_UART_IRQHandler(UART9);
}
#endif

#if defined(USART10)
void USART10_IRQHandler(void)
{
    ADIS16470_UART_IRQHandler(USART10);
}
#endif
