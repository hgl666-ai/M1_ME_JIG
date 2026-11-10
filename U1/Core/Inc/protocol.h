/**
  ******************************************************************************
  * @file    protocol.h
  * @brief   M1 治具 PC 端协议（命令码 / 状态码 / 收发接口）
  *
  * 帧格式见 protocol_frame.h；命令码按 M1 功能新定（沿用 R1 治具风格），
  * 其中 0x000E / 0x000F 沿用协议表的“测量ADC / 传感器”序号。
  ******************************************************************************
  */
#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include "main.h"
#include "app_config.h"

/*===== 通用状态码（与 IINFO 协议表一致）=====*/
#define PROTO_OK           0x00u
#define PROTO_BUSY         0x01u
#define PROTO_ERR_SIZE     0x02u
#define PROTO_ERR_FLASH    0x03u
#define PROTO_ERR_PROG     0x04u
#define PROTO_ERR_TEST     0x05u
#define PROTO_ERR_NO_FW    0x06u
#define PROTO_ERR_INCOMPLETE 0x07u
#define PROTO_ERR_CRC      0x08u
#define PROTO_ERR_SEQ      0x09u
#define PROTO_ERR_COUNT    0x0Au
#define PROTO_ERR_PARAM    0x0Bu
#define PROTO_ERR_TIMEOUT  0x0Cu
#define PROTO_ERR_OVERFLOW 0x0Du
#define PROTO_NIY          0xFFu

/*===== 命令码 =====*/
#define PROTO_CMD_PING          0x0001u   /* -> 状态码                          */
#define PROTO_CMD_GET_VERSION   0x0002u   /* -> 状态码+主+次+修订                */
#define PROTO_CMD_GET_UID       0x0003u   /* -> 状态码+96bit UID(12B)           */
#define PROTO_CMD_MEASURE_ADC   0x000Eu   /* [mode] -> 状态码+ADC 快照           */
#define PROTO_CMD_SENSOR_READ   0x000Fu   /* [mode] -> 状态码+对射结果           */
#define PROTO_CMD_MOTOR_MOVE    0x0010u   /* U2: [dir][steps u32][freq u16]     */
#define PROTO_CMD_MOTOR_STOP    0x0011u   /* U2                                 */
#define PROTO_CMD_MOTOR_STATUS  0x0012u   /* U2: -> busy+done+target            */
#define PROTO_CMD_TMC_QUERY     0x0020u   /* U2: -> TMC2209 寄存器快照           */
#define PROTO_CMD_SENSOR_CONFIG 0x0021u   /* [ch][block u16][release u16]       */
#define PROTO_CMD_TMC_SET_IRUN  0x0022u   /* U2: [irun][ihold]                  */
#define PROTO_CMD_SELF_TEST     0x0030u   /* -> 各功能自检位图                   */

/*===== 快照 / 报文长度 =====*/
#define ADC_SNAPSHOT_BYTES   (11u + ADC_CH_NUM * 5u)   /* 51 */
#define SENSOR_ACK_BYTES     (2u + SENSOR_NUM * 9u)    /* 20 */
#define MOTOR_STATUS_BYTES   10u
#define TMC_QUERY_BYTES      17u
#define SELFTEST_BYTES       6u

void PROTO_Init(void);
void PROTO_Run(void);

#endif /* __PROTOCOL_H */
