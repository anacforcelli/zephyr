#ifndef __I2C_PUPPY_H__
#define __I2C_PUPPY_H__

#include <zephyr/drivers/i2c.h>

// Command Buffer size
#define I2C_CMD_BUF_SIZE 8

// RW bit in address/control byte
#define I2C_R_FLAG (1 << 0)
#define I2C_W_FLAG (0 << 0)

// I2C uDMA commands
#define I2C_CMD_ID_OFFSET 28

#define I2C_CMD_START_ID 0x0
#define I2C_CMD_STOP_ID 0x2
#define I2C_CMD_RD_ACK_ID 0x4
#define I2C_CMD_RD_NACK_ID 0x6
#define I2C_CMD_WR_ID 0x8
#define I2C_CMD_WAIT_ID 0xA
#define I2C_CMD_RPT_ID 0xC
#define I2C_CMD_CFG_ID 0xE
#define I2C_CMD_WAIT_EV_ID 0x1
#define I2C_CMD_EOT_ID 0x9

#define I2C_CMD_START (I2C_CMD_START_ID << I2C_CMD_ID_OFFSET)
#define I2C_CMD_STOP (I2C_CMD_STOP_ID << I2C_CMD_ID_OFFSET)
#define I2C_CMD_RD_ACK (I2C_CMD_RD_ACK_ID << I2C_CMD_ID_OFFSET)
#define I2C_CMD_RD_NACK (I2C_CMD_RD_NACK_ID << I2C_CMD_ID_OFFSET)
#define I2C_CMD_WR (I2C_CMD_WR_ID << I2C_CMD_ID_OFFSET)
#define I2C_CMD_WAIT (I2C_CMD_WAIT_ID << I2C_CMD_ID_OFFSET)
#define I2C_CMD_RPT(msglen) ((I2C_CMD_RPT_ID << I2C_CMD_ID_OFFSET) | (msglen - 1))
#define I2C_CMD_CFG(clkdiv) ((I2C_CMD_CFG_ID << I2C_CMD_ID_OFFSET) | clkdiv)
#define I2C_CMD_WAIT_EV (I2C_CMD_WAIT_EV_ID << I2C_CMD_ID_OFFSET)
#define I2C_CMD_EOT (I2C_CMD_EOT_ID << I2C_CMD_ID_OFFSET)

#endif // __I2C_PUPPY_H__
