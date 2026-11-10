/**
  ******************************************************************************
  * @file    protocol_frame.c
  * @brief   帧封装 / 解析（大端 + CRC16 MODBUS）
  ******************************************************************************
  */
#include "protocol_frame.h"
#include <string.h>

/**
  * @brief  CRC16 MODBUS（反射多项式 0xA001，初值 0xFFFF）
  * @note   与参考工程 U1/U7 的 Frame_CRC16 完全一致，已用 R1 治具
  *         PING 帧（A5 5B 00 01 00 01 00 00 89 0C）核对过。
  */
uint16_t Frame_CRC16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001u) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001u);
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/**
  * @brief  组帧
  * @param  buf  输出缓冲，至少 len+10 字节
  * @retval 整帧长度
  */
uint16_t Frame_Build(uint8_t *buf, uint16_t cmd, uint16_t seq,
                     const uint8_t *data, uint16_t len)
{
    buf[0] = FRAME_HDR0;
    buf[1] = FRAME_HDR1;
    buf[2] = (uint8_t)(cmd >> 8);
    buf[3] = (uint8_t)(cmd);
    buf[4] = (uint8_t)(seq >> 8);
    buf[5] = (uint8_t)(seq);
    buf[6] = (uint8_t)(len >> 8);
    buf[7] = (uint8_t)(len);
    if (len > 0u && data != NULL) {
        memcpy(&buf[8], data, len);
    }

    uint16_t total = (uint16_t)(8u + len);
    uint16_t crc   = Frame_CRC16(buf, total);
    buf[total]      = (uint8_t)(crc >> 8);
    buf[total + 1u] = (uint8_t)(crc);
    return (uint16_t)(total + 2u);
}

/**
  * @brief  解析一整帧（raw_len 必须严格等于 10+LEN）
  * @retval 1 = 合法（帧头/长度/CRC 全部通过）
  */
uint8_t Frame_Parse(const uint8_t *raw, uint16_t raw_len,
                    uint16_t *cmd, uint16_t *seq,
                    uint8_t *data, uint16_t *len)
{
    if (raw_len < FRAME_OVERHEAD) return 0u;
    if (raw[0] != FRAME_HDR0 || raw[1] != FRAME_HDR1) return 0u;

    uint16_t dlen = (uint16_t)(((uint16_t)raw[6] << 8) | raw[7]);
    if (dlen > FRAME_MAX_DATA) return 0u;
    if (raw_len != (uint16_t)(8u + dlen + 2u)) return 0u;

    uint16_t calc = Frame_CRC16(raw, (uint16_t)(8u + dlen));
    uint16_t recv = (uint16_t)(((uint16_t)raw[8u + dlen] << 8) | raw[9u + dlen]);
    if (calc != recv) return 0u;

    if (cmd) *cmd = (uint16_t)(((uint16_t)raw[2] << 8) | raw[3]);
    if (seq) *seq = (uint16_t)(((uint16_t)raw[4] << 8) | raw[5]);
    if (len) *len = dlen;
    if (data && dlen > 0u) memcpy(data, &raw[8], dlen);
    return 1u;
}
