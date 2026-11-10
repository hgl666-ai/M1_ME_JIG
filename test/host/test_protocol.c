/**
  ******************************************************************************
  * @file    test_protocol.c
  * @brief   协议层主机端回归测试（gcc 编译真实 protocol.c / protocol_frame.c）
  *
  * 覆盖：帧已知向量、随机往返、PING/版本/ADC/对射/自检、未知命令 NIY、
  *       CRC 错不回 ACK、粘包、垃圾前缀、超长 LEN、U2 电机/TMC 命令。
  ******************************************************************************
  */
#include "main.h"
#include "protocol.h"
#include "protocol_frame.h"
#include "bsp_uart.h"
#include "bsp_adc.h"
#include "app.h"

#include <stdio.h>

extern void test_feed(const uint8_t *buf, int len);
extern void test_reset_out(void);
extern uint8_t g_out[4096];
extern int     g_out_len;
#if defined(BOARD_U2)
extern void test_motor_set_busy(uint8_t b);
#endif

static int g_fail;
static int g_checks;

#define CHECK(cond, msg) do { \
    g_checks++; \
    if (!(cond)) { g_fail++; printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__); } \
} while (0)

/*---------------------------------------------------------------------------
 * 已知向量（来自 R1 治具文档，用于校验 CRC16/组帧字节序）
 *-------------------------------------------------------------------------*/
static void test_known_vectors(void)
{
    uint8_t buf[64];
    uint16_t n;

    n = Frame_Build(buf, 0x0001, 0x0001, NULL, 0);
    CHECK(n == 10, "PING len");
    CHECK(memcmp(buf, "\xA5\x5B\x00\x01\x00\x01\x00\x00\x89\x0C", 10) == 0, "PING bytes");

    n = Frame_Build(buf, 0x0002, 0x0001, NULL, 0);
    CHECK(memcmp(buf, "\xA5\x5B\x00\x02\x00\x01\x00\x00\x89\x48", 10) == 0, "GET_VERSION bytes");

    n = Frame_Build(buf, 0x0012, 0x0001, NULL, 0);
    CHECK(memcmp(buf, "\xA5\x5B\x00\x12\x00\x01\x00\x00\x4A\x89", 10) == 0, "MOTOR_STATUS bytes");

    n = Frame_Build(buf, 0x0020, 0x0001, NULL, 0);
    CHECK(memcmp(buf, "\xA5\x5B\x00\x20\x00\x01\x00\x00\x8E\xB0", 10) == 0, "TMC_QUERY bytes");

    uint8_t move[7] = { 0x00, 0x00, 0x00, 0x00, 0xC8, 0x03, 0xE8 };   /* dir=0 steps=200 freq=1000 */
    n = Frame_Build(buf, 0x0010, 0x0001, move, 7);
    CHECK(n == 17, "MOVE len");
    CHECK(memcmp(buf, "\xA5\x5B\x00\x10\x00\x01\x00\x07\x00\x00\x00\x00\xC8\x03\xE8\xC3\x2F", 17) == 0,
          "MOVE bytes");
}

/*---------------------------------------------------------------------------
 * 随机往返
 *-------------------------------------------------------------------------*/
static void test_roundtrip(void)
{
    uint32_t seed = 12345u;
    uint8_t  data[FRAME_MAX_DATA];
    uint8_t  buf[FRAME_BUF_SIZE + 4];

    for (int t = 0; t < 500; t++) {
        seed = seed * 1103515245u + 12345u;
        uint16_t len = (uint16_t)(seed % 40u);
        for (uint16_t i = 0; i < len; i++) {
            seed = seed * 1103515245u + 12345u;
            data[i] = (uint8_t)(seed >> 16);
        }
        uint16_t cmd = (uint16_t)(seed >> 8);
        uint16_t seq = (uint16_t)(seed & 0xFFFFu);

        uint16_t n = Frame_Build(buf, cmd, seq, data, len);

        uint16_t c2, s2, l2;
        uint8_t  d2[FRAME_MAX_DATA];
        CHECK(Frame_Parse(buf, n, &c2, &s2, d2, &l2) == 1u, "roundtrip parse");
        CHECK(c2 == cmd && s2 == seq && l2 == len, "roundtrip fields");
        CHECK(len == 0u || memcmp(d2, data, len) == 0, "roundtrip data");

        buf[n - 1] ^= 0xFFu;                      /* 破坏 CRC */
        CHECK(Frame_Parse(buf, n, &c2, &s2, d2, &l2) == 0u, "bad crc rejected");
        buf[n - 1] ^= 0xFFu;
    }
}

/*---------------------------------------------------------------------------
 * 工具：喂入一帧并取出一个响应
 *-------------------------------------------------------------------------*/
static int feed_and_expect_one(const uint8_t *frame, int len,
                               uint16_t *cmd, uint16_t *seq,
                               uint8_t *data, uint16_t *dlen)
{
    PROTO_Init();
    test_reset_out();
    test_feed(frame, len);
    PROTO_Run();
    if (g_out_len == 0) return 0;
    return (int)Frame_Parse(g_out, (uint16_t)g_out_len, cmd, seq, data, dlen);
}

static void test_commands(void)
{
    uint8_t  frame[FRAME_BUF_SIZE];
    uint8_t  rsp[FRAME_MAX_DATA];
    uint16_t cmd, seq, dlen;
    uint16_t n;

    /* PING */
    n = Frame_Build(frame, PROTO_CMD_PING, 0x0102, NULL, 0);
    CHECK(feed_and_expect_one(frame, n, &cmd, &seq, rsp, &dlen), "PING ack parsed");
    CHECK(cmd == PROTO_CMD_PING && seq == 0x0102 && dlen == 1u && rsp[0] == PROTO_OK, "PING ack fields");

    /* GET_VERSION */
    n = Frame_Build(frame, PROTO_CMD_GET_VERSION, 0x0005, NULL, 0);
    CHECK(feed_and_expect_one(frame, n, &cmd, &seq, rsp, &dlen), "VERSION ack parsed");
    CHECK(dlen == 4u && rsp[0] == PROTO_OK && rsp[1] == FW_VER_MAJOR, "VERSION ack fields");

    /* MEASURE_ADC mode 0 */
    uint8_t m0 = 0;
    n = Frame_Build(frame, PROTO_CMD_MEASURE_ADC, 0x0006, &m0, 1);
    CHECK(feed_and_expect_one(frame, n, &cmd, &seq, rsp, &dlen), "ADC ack parsed");
    CHECK(dlen == ADC_SNAPSHOT_BYTES, "ADC ack length");
    CHECK(rsp[0] == PROTO_OK && rsp[2] == ADC_CH_NUM, "ADC ack header");
    CHECK(((uint16_t)rsp[3] << 8 | rsp[4]) == 7u, "ADC ack round passthrough");

    /* SENSOR_READ */
    uint8_t m1 = 0;
    n = Frame_Build(frame, PROTO_CMD_SENSOR_READ, 0x0007, &m1, 1);
    CHECK(feed_and_expect_one(frame, n, &cmd, &seq, rsp, &dlen), "SENSOR ack parsed");
    CHECK(dlen == SENSOR_ACK_BYTES && rsp[0] == PROTO_OK && rsp[1] == 0x05u, "SENSOR ack fields");

    /* 未知命令 → NIY */
    n = Frame_Build(frame, 0x1234, 0x0008, NULL, 0);
    CHECK(feed_and_expect_one(frame, n, &cmd, &seq, rsp, &dlen), "NIY ack parsed");
    CHECK(cmd == 0x1234u && dlen == 1u && rsp[0] == PROTO_NIY, "NIY ack fields");

    /* SELF_TEST */
    n = Frame_Build(frame, PROTO_CMD_SELF_TEST, 0x0009, NULL, 0);
    CHECK(feed_and_expect_one(frame, n, &cmd, &seq, rsp, &dlen), "SELFTEST ack parsed");
    CHECK(dlen == SELFTEST_BYTES, "SELFTEST ack length");

    /* SENSOR_CONFIG 非法（release >= block）→ ERR_PARAM */
    uint8_t cfg[5] = { 0u, 0x07u, 0xD0u, 0x0Fu, 0xA0u };   /* block=2000 release=4000 */
    n = Frame_Build(frame, PROTO_CMD_SENSOR_CONFIG, 0x000A, cfg, 5);
    CHECK(feed_and_expect_one(frame, n, &cmd, &seq, rsp, &dlen), "SENSORCFG ack parsed");
    CHECK(rsp[0] == PROTO_ERR_PARAM, "SENSORCFG invalid rejected");

    /* MEASURE_ADC 非法 mode */
    uint8_t m9 = 9;
    n = Frame_Build(frame, PROTO_CMD_MEASURE_ADC, 0x000B, &m9, 1);
    CHECK(feed_and_expect_one(frame, n, &cmd, &seq, rsp, &dlen), "ADC bad mode ack");
    CHECK(rsp[0] == PROTO_ERR_PARAM, "ADC bad mode rejected");
}

/*---------------------------------------------------------------------------
 * 异常输入
 *-------------------------------------------------------------------------*/
static void test_abnormal(void)
{
    uint8_t  frame[FRAME_BUF_SIZE];
    uint8_t  rsp[FRAME_MAX_DATA];
    uint16_t cmd, seq, dlen;
    uint16_t n;

    /* CRC 错：完全不回 ACK */
    n = Frame_Build(frame, PROTO_CMD_PING, 1, NULL, 0);
    frame[n - 1] ^= 0xFFu;
    PROTO_Init(); test_reset_out(); test_feed(frame, n); PROTO_Run();
    CHECK(g_out_len == 0, "CRC error -> no ACK");

    /* 粘包：两帧连续喂入 → 两个 ACK */
    uint8_t two[64];
    uint16_t a = Frame_Build(two, PROTO_CMD_PING, 0x0011, NULL, 0);
    uint16_t b = Frame_Build(two + a, PROTO_CMD_GET_VERSION, 0x0012, NULL, 0);
    PROTO_Init(); test_reset_out(); test_feed(two, a + b); PROTO_Run();
    CHECK(g_out_len == (10 + 1) + (10 + 4), "two frames -> two ACKs");
    CHECK(Frame_Parse(g_out, 11, &cmd, &seq, rsp, &dlen) && seq == 0x0011u, "first ACK seq");
    CHECK(Frame_Parse(g_out + 11, 14, &cmd, &seq, rsp, &dlen) && seq == 0x0012u, "second ACK seq");

    /* 垃圾前缀 + 合法帧 */
    uint8_t junk[3] = { 0x00, 0xA5, 0x00 };
    uint8_t mix[64];
    memcpy(mix, junk, 3);
    n = Frame_Build(mix + 3, PROTO_CMD_PING, 0x0021, NULL, 0);
    PROTO_Init(); test_reset_out(); test_feed(mix, 3 + n); PROTO_Run();
    CHECK(g_out_len == 11, "garbage prefix recovered");   /* PING ACK = 10 + 1 */

    /* 超长 LEN（300 > 256）→ 丢帧，后续合法帧仍可解析 */
    uint8_t over[16] = { 0xA5, 0x5B, 0x00, 0x01, 0x00, 0x01, 0x01, 0x2C };
    uint8_t combo[64];
    memcpy(combo, over, 8);
    n = Frame_Build(combo + 8, PROTO_CMD_PING, 0x0031, NULL, 0);
    PROTO_Init(); test_reset_out(); test_feed(combo, 8 + n); PROTO_Run();
    CHECK(g_out_len == 11, "oversized LEN dropped, next frame ok");

    /* 半帧后复位：不产生输出，也不死循环 */
    PROTO_Init(); test_reset_out();
    test_feed((const uint8_t *)"\xA5\x5B\x00", 3);
    PROTO_Run();
    CHECK(g_out_len == 0, "partial frame no output");
}

#if defined(BOARD_U2)
static void test_u2_commands(void)
{
    uint8_t  frame[FRAME_BUF_SIZE];
    uint8_t  rsp[FRAME_MAX_DATA];
    uint16_t cmd, seq, dlen;
    uint16_t n;

    test_motor_set_busy(0u);

    /* MOTOR_STATUS */
    n = Frame_Build(frame, PROTO_CMD_MOTOR_STATUS, 1, NULL, 0);
    CHECK(feed_and_expect_one(frame, n, &cmd, &seq, rsp, &dlen), "MOTOR_STATUS ack");
    CHECK(dlen == MOTOR_STATUS_BYTES && rsp[0] == PROTO_OK, "MOTOR_STATUS fields");

    /* MOTOR_MOVE 长度不足 */
    uint8_t bad[3] = { 0, 1, 2 };
    n = Frame_Build(frame, PROTO_CMD_MOTOR_MOVE, 2, bad, 3);
    CHECK(feed_and_expect_one(frame, n, &cmd, &seq, rsp, &dlen), "MOVE short ack");
    CHECK(rsp[0] == PROTO_ERR_PARAM, "MOVE short rejected");

    /* MOTOR_MOVE 正常 */
    uint8_t mv[7] = { 0x00, 0x00, 0x00, 0x00, 0xC8, 0x03, 0xE8 };
    n = Frame_Build(frame, PROTO_CMD_MOTOR_MOVE, 3, mv, 7);
    CHECK(feed_and_expect_one(frame, n, &cmd, &seq, rsp, &dlen), "MOVE ack");
    CHECK(rsp[0] == PROTO_OK, "MOVE accepted");

    /* 运动中再 MOVE → BUSY */
    n = Frame_Build(frame, PROTO_CMD_MOTOR_MOVE, 4, mv, 7);
    CHECK(feed_and_expect_one(frame, n, &cmd, &seq, rsp, &dlen), "MOVE busy ack");
    CHECK(rsp[0] == PROTO_BUSY, "MOVE busy rejected");

    /* 运动中 TMC_QUERY → BUSY */
    n = Frame_Build(frame, PROTO_CMD_TMC_QUERY, 5, NULL, 0);
    CHECK(feed_and_expect_one(frame, n, &cmd, &seq, rsp, &dlen), "TMC busy ack");
    CHECK(rsp[0] == PROTO_BUSY, "TMC query busy");

    test_motor_set_busy(0u);

    /* 空闲 TMC_QUERY → 17 字节快照 */
    n = Frame_Build(frame, PROTO_CMD_TMC_QUERY, 6, NULL, 0);
    CHECK(feed_and_expect_one(frame, n, &cmd, &seq, rsp, &dlen), "TMC ack");
    CHECK(dlen == TMC_QUERY_BYTES && rsp[0] == PROTO_OK && rsp[1] == 1u, "TMC fields");

    /* SET_IRUN 非法：ihold > irun */
    uint8_t cur[2] = { 5u, 9u };
    n = Frame_Build(frame, PROTO_CMD_TMC_SET_IRUN, 7, cur, 2);
    CHECK(feed_and_expect_one(frame, n, &cmd, &seq, rsp, &dlen), "IRUN ack");
    CHECK(rsp[0] == PROTO_ERR_PARAM, "IRUN invalid rejected");

    /* SET_IRUN 合法 */
    uint8_t cur2[2] = { 12u, 6u };
    n = Frame_Build(frame, PROTO_CMD_TMC_SET_IRUN, 8, cur2, 2);
    CHECK(feed_and_expect_one(frame, n, &cmd, &seq, rsp, &dlen), "IRUN ok ack");
    CHECK(rsp[0] == PROTO_OK, "IRUN accepted");
}
#endif

int main(void)
{
    printf("=== M1 protocol host test (%s) ===\n",
#if defined(BOARD_U2)
           "U2"
#else
           "U1"
#endif
    );

    APP_Init();          /* 初始化快照桩数据 */
    test_known_vectors();
    test_roundtrip();
    test_commands();
    test_abnormal();
#if defined(BOARD_U2)
    test_u2_commands();
#endif

    printf("%d checks, %d failed\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
