#pragma once

// MFRC522 register addresses
// Page 0: Command and status
#define RC522_REG_COMMAND           0x01
#define RC522_REG_COM_I_EN          0x02
#define RC522_REG_DIV_I_EN          0x03
#define RC522_REG_COM_IRQ           0x04
#define RC522_REG_DIV_IRQ           0x05
#define RC522_REG_ERROR             0x06
#define RC522_REG_STATUS1           0x07
#define RC522_REG_STATUS2           0x08
#define RC522_REG_FIFO_DATA         0x09
#define RC522_REG_FIFO_LEVEL        0x0A
#define RC522_REG_WATER_LEVEL       0x0B
#define RC522_REG_CONTROL           0x0C
#define RC522_REG_BIT_FRAMING       0x0D
#define RC522_REG_COLL              0x0E

// Page 1: Communication
#define RC522_REG_MODE              0x11
#define RC522_REG_TX_MODE           0x12
#define RC522_REG_RX_MODE           0x13
#define RC522_REG_TX_CONTROL        0x14
#define RC522_REG_TX_ASK            0x15
#define RC522_REG_TX_SEL            0x16
#define RC522_REG_RX_SEL            0x17
#define RC522_REG_RX_THRESHOLD      0x18
#define RC522_REG_DEMOD             0x19
#define RC522_REG_MF_TX             0x1C
#define RC522_REG_MF_RX             0x1D
#define RC522_REG_SERIAL_SPEED      0x1F

// Page 2: Configuration
#define RC522_REG_CRC_RESULT_MSB    0x21
#define RC522_REG_CRC_RESULT_LSB    0x22
#define RC522_REG_MOD_WIDTH         0x24
#define RC522_REG_RF_CFG            0x26
#define RC522_REG_GS_N              0x27
#define RC522_REG_CW_GS_P          0x28
#define RC522_REG_MOD_GS_P         0x29
#define RC522_REG_T_MODE            0x2A
#define RC522_REG_T_PRESCALER       0x2B
#define RC522_REG_T_RELOAD_HI       0x2C
#define RC522_REG_T_RELOAD_LO       0x2D
#define RC522_REG_T_COUNTER_VAL_HI  0x2E
#define RC522_REG_T_COUNTER_VAL_LO  0x2F

// Page 3: Test
#define RC522_REG_TEST_SEL1         0x31
#define RC522_REG_TEST_SEL2         0x32
#define RC522_REG_TEST_PIN_EN       0x33
#define RC522_REG_TEST_PIN_VALUE    0x34
#define RC522_REG_TEST_BUS          0x35
#define RC522_REG_AUTO_TEST         0x36
#define RC522_REG_VERSION           0x37
#define RC522_REG_ANALOG_TEST       0x38
#define RC522_REG_TEST_DAC1         0x39
#define RC522_REG_TEST_DAC2         0x3A
#define RC522_REG_TEST_ADC          0x3B

// PCD commands (MFRC522 internal commands)
#define PCD_CMD_IDLE                0x00
#define PCD_CMD_MEM                 0x01
#define PCD_CMD_GEN_RANDOM_ID       0x02
#define PCD_CMD_CALC_CRC            0x03
#define PCD_CMD_TRANSMIT            0x04
#define PCD_CMD_NO_CMD_CHANGE       0x07
#define PCD_CMD_RECEIVE             0x08
#define PCD_CMD_TRANSCEIVE          0x0C
#define PCD_CMD_MF_AUTHENT          0x0E
#define PCD_CMD_SOFT_RESET          0x0F

// PICC commands (ISO 14443)
#define PICC_CMD_REQA               0x26
#define PICC_CMD_WUPA               0x52
#define PICC_CMD_CT                 0x88  // Cascade tag
#define PICC_CMD_SEL_CL1            0x93
#define PICC_CMD_SEL_CL2            0x95
#define PICC_CMD_SEL_CL3            0x97
#define PICC_CMD_HLTA               0x50

// MIFARE commands
#define PICC_CMD_MF_AUTH_KEY_A      0x60
#define PICC_CMD_MF_AUTH_KEY_B      0x61
#define PICC_CMD_MF_READ            0x30
#define PICC_CMD_MF_WRITE           0xA0
#define PICC_CMD_MF_DECREMENT       0xC0
#define PICC_CMD_MF_INCREMENT       0xC1
#define PICC_CMD_MF_RESTORE         0xC2
#define PICC_CMD_MF_TRANSFER        0xB0
