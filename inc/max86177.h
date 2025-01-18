/**
 * @file max86177.h
 * @author
 *         mengyao liu <mengyao.liu@kuleuven.be>
 * @brief  max86177 library file
 * @version 0.1
 * @date 2025-01-04
 *
 * @copyright
 *         Copyright (c) 2025
 */

#ifndef MAX86177_H
#define MAX86177_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h> // for `bool`

/* ----------------------------------------------------------------------------
 *  Common Macros
 * ---------------------------------------------------------------------------- */

/* Example data headers (can be adjusted as needed) */
#define COMM_DATA_HEADER          0xAA
#define COMM_DATA_OVF_HEADER      0xBB

/* Number of bytes to read for interrupt status */
#define OS67_INTERRUPT_STATUS_CNT 5  // Adjust based on your code/log requirements

/* ----------------------------------------------------------------------------
 *  Register Addresses
 * ---------------------------------------------------------------------------- */
/* Interrupt status registers */
#define OS67_INTERRUPT_STATUS_1       0x00
#define OS67_INTERRUPT_STATUS_2       0x01
#define OS67_INTERRUPT_STATUS_1_ENABLE 0xFA

/* Other registers */
#define OS67_OVF                 0x03
#define OS67_FIFO_DATA_CNT       0x04
#define OS67_FIFO_DATA           0x08
#define OS67_FIFO_CONF_2         0x09
#define OS67_SYSTEM_CONF_1       0x0D
#define OS67_INTERRUPT_PIN_CON   0x0E

/* PPG measurement selects */
#define OS67_PPG_MEAS_SELECT_1   0x11
#define OS67_PPG_MEAS_SELECT_2   0x12
#define OS67_PPG_MEAS_SELECT_3   0x13
#define OS67_PPG_FILTER_SETUP    0x14

/* ----------------------------------------------------------------------------
 *  FIFO Configuration Bits
 * ---------------------------------------------------------------------------- */
#define OS67_FLUSH_FIFO          (1 << 4)
#define OS67_FIFO_SIZE           1

/* ----------------------------------------------------------------------------
 *  System Configuration Bits
 * ---------------------------------------------------------------------------- */
#define OS67_SHDN                (1 << 1)

/* ----------------------------------------------------------------------------
 *  Interrupt Configuration
 * ---------------------------------------------------------------------------- */
/* Interrupt enable bits (Interrupt1 Enable at register 0xFA). 
 * For example, FIFO_DATA_RDY_EN1 (bit 5).
 */
#define FIFO_DATA_RDY_EN1        (1 << 5)

/* Interrupt pin functional configuration (register 0x0E).
 * Suppose bits [2:1] = 01 => INT1 is enabled & auto-clear on status read.
 */
#define INT1_FCFG_ENABLE         0x02  

/* Example: enable FIFO_DATA_RDY_EN1 interrupt output to INT1 pin. 
 * bit5=1 => 0x20 
 */
#define INT1_STATE_ENABLE        0x20

/* Measurement bits to be enabled */
#define OS67_MEAS1_EN            (1 << 0)  // bit0
#define OS67_MEAS3_EN            (1 << 2)  // bit2

/* ----------------------------------------------------------------------------
 *  MEAS1 Setup (Registers 0x17 ~ 0x1E + PD SEL 0x1F ~ 0x21)
 * ---------------------------------------------------------------------------- */
/* 1) Register Address Macros */
#define MEAS1_SELECTS         0x17 // [7]AMB, [6..4]DRVB, [3..1]DRVA
#define MEAS1_CONFIG_1        0x18 // [7]DAC_LO_NOISE, [6]FILT2_SEL, [5..4]TINT, [3..1]AVER
#define MEAS1_CONFIG_2        0x19 // [7]SINC3_SEL, [6]FILT_SEL, [5..4]PPG4_RGE, ...
#define MEAS1_CONFIG_3        0x1A // [7..4]PPG2_DACOFF, [3..0]PPG1_DACOFF
#define MEAS1_CONFIG_4        0x1B // [7..4]PPG4_DACOFF, [3..0]PPG3_DACOFF
#define MEAS1_CONFIG_5        0x1C // [5..4]PD_SETLNG, [3..2]LED_SETLNG, [1..0]LED_RGE
#define MEAS1_LEDA_CURRENT    0x1D // [7..0] = DRVA_PA
#define MEAS1_LEDB_CURRENT    0x1E // [7..0] = DRVB_PA

/* PD selection registers */
#define MEAS1_PD_SEL_1        0x1F // PD1_SEL/PD2_SEL/PD3_SEL
#define MEAS1_PD_SEL_2        0x20 // PD4_SEL/PD5_SEL/PD6_SEL
#define MEAS1_PD_SEL_3        0x21 // PD7_SEL/PD8_SEL

/* 2) Bit Masks and Positions */
// (A) MEAS1_SELECTS (0x17)
#define MEAS1_AMB_MASK        (1 << 6)              // Bit6 => 1=ambient only, 0=normal
#define MEAS1_DRVB_POS        3
#define MEAS1_DRVB_MASK       (0x7 << MEAS1_DRVB_POS) // bits[5..3]
#define MEAS1_DRVA_POS        0
#define MEAS1_DRVA_MASK       (0x7 << MEAS1_DRVA_POS) // bits[2..0]

// (B) MEAS1_CONFIG_1 (0x18)
#define MEAS1_DAC_LO_NOISE_MASK  (1 << 7)
#define MEAS1_FILT2_SEL_MASK     (1 << 6)

/* TINT */
#define MEAS1_TINT_POS           4
#define MEAS1_TINT_MASK          (0x3 << MEAS1_TINT_POS)
#define MEAS1_TINT_14us          (0x0 << MEAS1_TINT_POS)
#define MEAS1_TINT_29us          (0x1 << MEAS1_TINT_POS)
#define MEAS1_TINT_59us          (0x2 << MEAS1_TINT_POS)
#define MEAS1_TINT_117us         (0x3 << MEAS1_TINT_POS)

/* AVER */
#define MEAS1_AVER_POS           1
#define MEAS1_AVER_MASK          (0x7 << MEAS1_AVER_POS)
#define MEAS1_AVER_1pulse        (0x0 << MEAS1_AVER_POS)
#define MEAS1_AVER_2pulse        (0x1 << MEAS1_AVER_POS)
// ...additional averaging modes as needed

// (C) MEAS1_CONFIG_2 (0x19)
#define MEAS1_SINC3_SEL_MASK     (1 << 7)
#define MEAS1_FILT_SEL_MASK      (1 << 6)

/* PPG4, PPG3, PPG2, PPG1 ADC range positions and masks */
#define MEAS1_PPG4_ADC_RGE_POS   6
#define MEAS1_PPG4_ADC_RGE_MASK  (0x3 << MEAS1_PPG4_ADC_RGE_POS)

#define MEAS1_PPG3_ADC_RGE_POS   4
#define MEAS1_PPG3_ADC_RGE_MASK  (0x3 << MEAS1_PPG3_ADC_RGE_POS)

#define MEAS1_PPG2_ADC_RGE_POS   2
#define MEAS1_PPG2_ADC_RGE_MASK  (0x3 << MEAS1_PPG2_ADC_RGE_POS)

/* Newly added for PPG1 */
#define MEAS1_PPG1_ADC_RGE_POS   0
#define MEAS1_PPG1_ADC_RGE_MASK  (0x3 << MEAS1_PPG1_ADC_RGE_POS)

// (E) MEAS1_CONFIG_5 (0x1C)
#define MEAS1_PD_SETLNG_POS      4
#define MEAS1_PD_SETLNG_MASK     (0x3 << MEAS1_PD_SETLNG_POS)

#define MEAS1_LED_SETLNG_POS     2
#define MEAS1_LED_SETLNG_MASK    (0x3 << MEAS1_LED_SETLNG_POS)

#define MEAS1_LED_RGE_POS        0
#define MEAS1_LED_RGE_MASK       (0x3 << MEAS1_LED_RGE_POS)

/* PD selection in 0x1F and 0x20 */
// PD1_SEL => 0x1F bits[2..0]
#define PD1_SEL_POS              0
#define PD1_SEL_MASK             (0x7 << PD1_SEL_POS)
// PD2_SEL => 0x1F bits[5..3]
#define PD2_SEL_POS              3
#define PD2_SEL_MASK             (0x7 << PD2_SEL_POS)

/* PD3_SEL is split between 0x1F and 0x20:
 * 0x1F bits [7..6] => PD3_SEL[2..1]
 * 0x20 bit  [0]    => PD3_SEL[0]
 */
#define PD3_SEL_HIGH_POS         6
#define PD3_SEL_HIGH_MASK        (0x3 << PD3_SEL_HIGH_POS)
#define PD3_SEL_LOW_POS          0
#define PD3_SEL_LOW_MASK         (0x1 << PD3_SEL_LOW_POS)

// PD4_SEL => 0x20 bits[3..1]
#define PD4_SEL_POS              1
#define PD4_SEL_MASK             (0x7 << PD4_SEL_POS)

/* ----------------------------------------------------------------------------
 *  MEAS3 Setup (Registers 0x2D ~ 0x34 + PD SEL 0x35 ~ 0x37)
 * ---------------------------------------------------------------------------- */
/* 1) Register Address Macros */
#define MEAS3_SELECTS          0x2D
#define MEAS3_CONFIG_1         0x2E
#define MEAS3_CONFIG_2         0x2F
#define MEAS3_CONFIG_3         0x30
#define MEAS3_CONFIG_4         0x31
#define MEAS3_CONFIG_5         0x32
#define MEAS3_LEDA_CURRENT     0x33
#define MEAS3_LEDB_CURRENT     0x34
#define MEAS3_PD_SEL_1         0x35
#define MEAS3_PD_SEL_2         0x36
#define MEAS3_PD_SEL_3         0x37

/* 2) Bit Masks and Positions */
// (A) MEAS3_SELECTS (0x2D)
#define MEAS3_AMB_MASK         (1 << 6)            // Bit6 => 1=ambient only
#define MEAS3_DRVB_POS         3
#define MEAS3_DRVB_MASK        (0x7 << MEAS3_DRVB_POS) // bits[5..3]
#define MEAS3_DRVA_POS         0
#define MEAS3_DRVA_MASK        (0x7 << MEAS3_DRVA_POS) // bits[2..0]

// (B) MEAS3_CONFIG_1 (0x2E)
#define MEAS3_DAC_LO_NOISE_MASK (1 << 6) // Bit6
#define MEAS3_FILT2_SEL_MASK    (1 << 5) // Bit5

#define MEAS3_TINT_POS          3
#define MEAS3_TINT_MASK         (0x3 << MEAS3_TINT_POS)
/* e.g., 0x3 => 117us (example assumption) */
#define MEAS3_TINT_117us        (0x3 << MEAS3_TINT_POS)

#define MEAS3_AVER_POS          0
#define MEAS3_AVER_MASK         (0x7 << MEAS3_AVER_POS)

// (C) MEAS3_CONFIG_2 (0x2F)
#define MEAS3_SINC3_SEL_MASK    (1 << 7)
#define MEAS3_FILT_SEL_MASK     (1 << 6)

#define MEAS3_PPG4_ADC_RGE_POS  4
#define MEAS3_PPG4_ADC_RGE_MASK (0x3 << MEAS3_PPG4_ADC_RGE_POS)

#define MEAS3_PPG3_ADC_RGE_POS  2
#define MEAS3_PPG3_ADC_RGE_MASK (0x3 << MEAS3_PPG3_ADC_RGE_POS)

#define MEAS3_PPG2_ADC_RGE_POS  0
#define MEAS3_PPG2_ADC_RGE_MASK (0x3 << MEAS3_PPG2_ADC_RGE_POS)

/* Also defined for PPG1 (may overlap with above usage) */
#define MEAS3_PPG1_ADC_RGE_POS  0
#define MEAS3_PPG1_ADC_RGE_MASK (0x3 << MEAS3_PPG1_ADC_RGE_POS)

/* PD and LED settings (MEAS3_CONFIG_x or other registers) */
#define MEAS3_PD_SETLNG_MASK    (0x3 << 4) // bits[5..4]
#define MEAS3_PD_SETLNG_POS     4
#define MEAS3_LED_SETLNG_MASK   (0x3 << 2) // bits[3..2]
#define MEAS3_LED_SETLNG_POS    2
#define MEAS3_LED_RGE_MASK      (0x3 << 0) // bits[1..0]
#define MEAS3_LED_RGE_POS       0

/* PD1/PD2/PD3/PD4 selection */
#define MEAS3_PD1_SEL_POS       0
#define MEAS3_PD1_SEL_MASK      (0x7 << MEAS3_PD1_SEL_POS)

#define MEAS3_PD2_SEL_POS       2
#define MEAS3_PD2_SEL_MASK      (0x7 << MEAS3_PD2_SEL_POS)

#define MEAS3_PD3_SEL_POS_HIGH  6
#define MEAS3_PD3_SEL_MASK_HIGH (0x3 << MEAS3_PD3_SEL_POS_HIGH)

#define MEAS3_PD3_SEL_POS_LOW   0
#define MEAS3_PD3_SEL_MASK_LOW  (0x1 << MEAS3_PD3_SEL_POS_LOW)

#define MEAS3_PD4_SEL_POS       1
#define MEAS3_PD4_SEL_MASK      (0x7 << MEAS3_PD4_SEL_POS)

/* ----------------------------------------------------------------------------
 *  I2C Read/Write Functions
 * ---------------------------------------------------------------------------- */
bool writeI2CRegister(uint8_t reg, uint8_t value);
bool readI2CRegister(uint8_t reg, uint8_t *values, size_t len, bool print);

/* ----------------------------------------------------------------------------
 *  Sensor Configuration Functions
 * ---------------------------------------------------------------------------- */
void OS67_enable(void);
void config_MEAS1(void);
void config_MEAS3(void);

/* ----------------------------------------------------------------------------
 *  Sensor Functional Functions
 * ---------------------------------------------------------------------------- */
int  os67_sensor_i2c_init(void);
void OS67_read_fifo(void);

#endif /* MAX86177_H */
