#ifndef ROBOT_MMIO_H
#define ROBOT_MMIO_H
#include <stdint.h>
#define NAV_MMIO_BASE UINT32_C(0x10000000)
#define NAV_REG(offset) (*(volatile uint32_t *)(uintptr_t)(NAV_MMIO_BASE + (offset)))
#define NAV_BOARD_ID NAV_REG(0x00)
#define NAV_UPTIME_MS NAV_REG(0x04)
#define NAV_CYCLES NAV_REG(0x08)
#define NAV_CAPABILITIES NAV_REG(0x0c)
#define NAV_HEARTBEAT NAV_REG(0x10)
#define NAV_STATUS NAV_REG(0x14)
#define NAV_PATH_LENGTH NAV_REG(0x18)
#define NAV_ASTAR_CYCLES NAV_REG(0x1c)
#define NAV_EXPANDED NAV_REG(0x20)
#define NAV_IMU_SEQUENCE NAV_REG(0x24)
#define NAV_FW_ERROR NAV_REG(0x28)
#define NAV_SNAPSHOT NAV_REG(0x2c)
#define NAV_IMU_WORD(n) NAV_REG(0x40u + (uint32_t)(n) * 4u)
#define NAV_MOTOR_WORD(n) NAV_REG(0x80u + (uint32_t)(n) * 4u)
#define NAV_EXPECTED_ID UINT32_C(0x4e415631)
#define NAV_READY UINT32_C(1)
#define NAV_SELFTEST_OK UINT32_C(2)
#define NAV_FW_FAILED UINT32_C(4)

/* Read-only SNS1 snapshot: latch with existing MMIO_SNAPSHOT first.
 * Sixteen words cover the exact 64-byte big-endian field ordering. */
#define SENSOR_SNAPSHOT_BASE 0x100000C0u
#define SENSOR_SNAPSHOT_WORD(i) (*(volatile unsigned int *)(SENSOR_SNAPSHOT_BASE + 4u*(i)))

/* Separate navigation request/reply mailbox. Authorized local MOT4 control
 * uses the versioned 0x200..0x33c MMIO bank documented in INTERFACE.txt. */
#define NAV_RX_PENDING NAV_REG(0x100)
#define NAV_RX_RELEASE NAV_REG(0x104)
#define NAV_TX_READY NAV_REG(0x108)
#define NAV_TX_SEND NAV_REG(0x10c)
#define NAV_RX_WORD(n) NAV_REG(0x140u + (uint32_t)(n)*4u)
#define NAV_TX_WORD(n) NAV_REG(0x180u + (uint32_t)(n)*4u)
#endif
