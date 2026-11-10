/**
  ******************************************************************************
  * @file    protocol_frame.h
  * @brief   帧格式：A5 5B | CMD(2) | SEQ(2) | LEN(2) | DATA | CRC16(2)
  *
  * 依据 IINFO/M1 测试治具PC端通讯协议.xlsx：
  *   - 所有多字节字段大端（高字节在前）
  *   - CRC16 = MODBUS（多项式 0xA001 反射，初值 0xFFFF），覆盖帧头..数据末尾
  *   - SEQ：PC 从 0x0001 递增，达 0xFFFF 回 1；ACK 原样回填；治具主动上报固定 0
  *   - 最短帧 10 字节（LEN=0）
  ******************************************************************************
  */
#ifndef __PROTOCOL_FRAME_H
#define __PROTOCOL_FRAME_H

#include "main.h"

#define FRAME_HDR0        0xA5u
#define FRAME_HDR1        0x5Bu
#define FRAME_OVERHEAD    10u          /* 2+2+2+2+2 */
#define FRAME_MAX_DATA    256u
#define FRAME_BUF_SIZE    (FRAME_OVERHEAD + FRAME_MAX_DATA)

uint16_t Frame_CRC16(const uint8_t *data, uint16_t len);

uint16_t Frame_Build(uint8_t *buf, uint16_t cmd, uint16_t seq,
                     const uint8_t *data, uint16_t len);

uint8_t  Frame_Parse(const uint8_t *raw, uint16_t raw_len,
                     uint16_t *cmd, uint16_t *seq,
                     uint8_t *data, uint16_t *len);

#endif /* __PROTOCOL_FRAME_H */
