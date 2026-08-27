#include "as5600.h"
#include "iic.h"

uint8_t AS5600_TryReadRawAngle(uint16_t *raw_angle)
{
    uint8_t high, low;

    if (!IIC_Start()) goto error;

    if (!IIC_SendAddr(AS5600_ADDR << 1)) goto error;

    if (!IIC_SendByte(AS5600_RAW_ANGLE_HIGH)) goto error;

    if (!IIC_Start()) goto error;

    /* F1 必须在清除读地址的 ADDR 标志前打开 ACK，否则首字节可能被 NACK。 */
    IIC_Ack();

    if (!IIC_SendAddr((AS5600_ADDR << 1) | 0x01)) goto error;

    if (!IIC_ReadByte(&high)) goto error;

    IIC_Nck();

    if (!IIC_ReadByte(&low)) goto error;

    IIC_Stop();

    *raw_angle = (((uint16_t)high << 8) | low) & 0x0FFF;

    return 1;

error:
    IIC_Nck();
    IIC_Stop();
    return 0;
}

uint16_t AS5600_ReadRawAngle(void)
{
    uint16_t raw_angle = 0;

    (void)AS5600_TryReadRawAngle(&raw_angle);
    return raw_angle;
}

uint16_t AS5600_ReadAngle(void)
{
    return (uint16_t)((uint32_t)AS5600_ReadRawAngle() * 3600 / 4096);
}
