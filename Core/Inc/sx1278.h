/**
 * @file sx1278.h
 * @brief SX1276/77/78/79 Register Definitions
 * @version Rev. 7 - May 2020 (Semtech)
 */

#ifndef SX1278_REGS_H
#define SX1278_REGS_H

// ============================================================================
// SX1278 REGISTER MAP
// ============================================================================

// 0x00 - 0x0F
#define REG_FIFO                        0x00  // FIFO read/write access
#define REG_OP_MODE                     0x01  // Operating mode & LoRa/FSK selection
#define REG_BITRATE_MSB                 0x02  // Bit Rate setting (MSB)
#define REG_BITRATE_LSB                 0x03  // Bit Rate setting (LSB)
#define REG_FDEV_MSB                    0x04  // Frequency Deviation setting (MSB)
#define REG_FDEV_LSB                    0x05  // Frequency Deviation setting (LSB)
#define REG_FRF_MSB                     0x06  // RF Carrier Frequency (MSB)
#define REG_FRF_MID                     0x07  // RF Carrier Frequency (MID)
#define REG_FRF_LSB                     0x08  // RF Carrier Frequency (LSB)
#define REG_PA_CONFIG                   0x09  // PA selection and Output Power control
#define REG_PA_RAMP                     0x0A  // Control of PA ramp time
#define REG_OCP                         0x0B  // Over Current Protection control
#define REG_LNA                         0x0C  // LNA settings
#define REG_RX_CONFIG                   0x0D  // AFC, AGC control (FSK)
#define REG_FIFO_ADDR_PTR               0x0D  // FIFO SPI pointer (LoRa)
#define REG_RSSI_CONFIG                 0x0E  // RSSI control (FSK)
#define REG_FIFO_TX_BASE_ADDR           0x0E  // Start Tx data (LoRa)
#define REG_RSSI_COLLISION              0x0F  // RSSI Collision detector (FSK)
#define REG_FIFO_RX_BASE_ADDR           0x0F  // Start Rx data (LoRa)

// 0x10 - 0x1F
#define REG_RSSI_THRESH                 0x10  // RSSI Threshold control
#define REG_FIFO_RX_CURRENT_ADDR        0x10  // Start address of last packet received
#define REG_RSSI_VALUE                  0x11  // RSSI value (FSK)
#define REG_IRQ_FLAGS_MASK              0x11  // Optional IRQ flag mask (LoRa)
#define REG_RX_BW                       0x12  // Channel Filter BW Control (FSK)
#define REG_IRQ_FLAGS                   0x12  // IRQ flags (LoRa)
#define REG_AFC_BW                      0x13  // AFC Channel Filter BW (FSK)
#define REG_RX_NB_BYTES                 0x13  // Number of received bytes (LoRa)
#define REG_OOK_PEAK                    0x14  // OOK demodulator
#define REG_RX_HEADER_CNT_VALUE_MSB     0x14  // Number of valid headers received (MSB)
#define REG_OOK_FIX                     0x15  // OOK threshold
#define REG_RX_HEADER_CNT_VALUE_LSB     0x15  // Number of valid headers received (LSB)
#define REG_OOK_AVG                     0x16  // OOK average
#define REG_RX_PACKET_CNT_VALUE_MSB     0x16  // Number of valid packets received (MSB)
#define REG_RESERVED17                  0x17
#define REG_RX_PACKET_CNT_VALUE_LSB     0x17
#define REG_RESERVED18                  0x18
#define REG_MODEM_STAT                  0x18  // Live LoRa modem status
#define REG_RESERVED19                  0x19
#define REG_PKT_SNR_VALUE               0x19  // Estimation of last packet SNR
#define REG_AFC_FEI                     0x1A  // AFC and FEI control (FSK)
#define REG_PKT_RSSI_VALUE              0x1A  // RSSI of last packet (LoRa)
#define REG_AFC_MSB                     0x1B
#define REG_RSSI_VALUE_LORA             0x1B  // Current RSSI (LoRa)
#define REG_AFC_LSB                     0x1C
#define REG_HOP_CHANNEL                 0x1C  // FHSS start channel

// 0x1D - 0x2F
#define REG_FEI_MSB                     0x1D
#define REG_MODEM_CONFIG1               0x1D
#define REG_FEI_LSB                     0x1E
#define REG_MODEM_CONFIG2               0x1E
#define REG_PREAMBLE_DETECT             0x1F
#define REG_SYMB_TIMEOUT_LSB            0x1F

// 0x20 - 0x26
#define REG_RX_TIMEOUT1                 0x20
#define REG_PREAMBLE_MSB                0x20
#define REG_RX_TIMEOUT2                 0x21
#define REG_PREAMBLE_LSB                0x21
#define REG_RX_TIMEOUT3                 0x22
#define REG_PAYLOAD_LENGTH              0x22
#define REG_RX_DELAY                    0x23
#define REG_MAX_PAYLOAD_LENGTH          0x23
#define REG_OSC                         0x24
#define REG_HOP_PERIOD                  0x24
#define REG_PREAMBLE_MSB_FSK            0x25
#define REG_FIFO_RX_BYTE_ADDR           0x25
#define REG_PREAMBLE_LSB_FSK            0x26
#define REG_MODEM_CONFIG3               0x26

// 0x27 - 0x39
#define REG_SYNC_CONFIG                 0x27
#define REG_SYNC_VALUE1                 0x28
#define REG_SYNC_VALUE2                 0x29
#define REG_SYNC_VALUE3                 0x2A
#define REG_SYNC_VALUE4                 0x2B
#define REG_SYNC_VALUE5                 0x2C
#define REG_SYNC_VALUE6                 0x2D
#define REG_SYNC_VALUE7                 0x2E
#define REG_SYNC_VALUE8                 0x2F
#define REG_IF_FREQ1                    0x2F
#define REG_PACKET_CONFIG1              0x30
#define REG_PACKET_CONFIG2              0x31
#define REG_DETECTION_OPTIMIZE          0x31
#define REG_PAYLOAD_LENGTH_FSK          0x32
#define REG_INVERT_IQ                   0x33
#define REG_NODE_ADDR                   0x33
#define REG_BROADCAST_ADDR              0x34
#define REG_FIFO_THRESH                 0x35
#define REG_SEQ_CONFIG1                 0x36
#define REG_SEQ_CONFIG2                 0x37
#define REG_TIMER_RESOL                 0x38
#define REG_TIMER1_COEF                 0x39
#define REG_SYNC_WORD                   0x39

// 0x3A - 0x70
#define REG_TIMER2_COEF                 0x3A
#define REG_IMAGE_CAL                   0x3B
#define REG_TEMP                        0x3C
#define REG_LOW_BAT                     0x3D
#define REG_IRQ_FLAGS1                  0x3E
#define REG_IRQ_FLAGS2                  0x3F
#define REG_DIO_MAPPING1                0x40
#define REG_DIO_MAPPING2                0x41
#define REG_VERSION                     0x42
#define REG_PLL_HOP                     0x44
#define REG_TCXO                        0x4B
#define REG_PA_DAC                      0x4D
#define REG_FORMER_TEMP                 0x5B
#define REG_BITRATE_FRAC                0x5D
#define REG_AGC_REF                     0x61
#define REG_AGC_THRESH1                 0x62
#define REG_AGC_THRESH2                 0x63
#define REG_AGC_THRESH3                 0x64
#define REG_PLL                         0x70

// Internal test registers (do not overwrite)
#define REG_TEST_START                  0x71
#define REG_TEST_END                    0xFF

#define



static void         _SX_REG_WRITE(void);
static uint8_t      _SX_REG_READ(void);

#endif // SX1278_H
