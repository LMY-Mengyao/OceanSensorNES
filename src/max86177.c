#include "max86177.h"
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

#define I2C0_NODE DT_NODELABEL(os67_sensor)
static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C0_NODE);

//**********************************************************************************************************************//
//**********************************************************************************************************************//

/**
 * @brief Write a single byte to a register on an I2C slave device.
 *
 * @param reg    The register address.
 * @param value  The value to write.
 * @return true  Write succeeded.
 * @return false Write failed.
 */
bool writeI2CRegister(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    int ret = i2c_write_dt(&dev_i2c, data, sizeof(data));

    if (ret < 0) {
        printk("Failed to write to I2C device (addr=0x%02X) at Reg.0x%02X. Err=%d\n",
               dev_i2c.addr, reg, ret);
        return false;
    }

    printk("Write Reg.0x%02X -> 0x%02X success.\n", reg, value);
    return true;
}

/**
 * @brief Read multiple bytes from a specified register on an I2C slave device.
 *
 * @param reg    The register address to read from.
 * @param values Pointer to the buffer where the read data will be stored.
 * @param len    Number of bytes to read.
 * @param print  Whether to print the read values (for debugging).
 * @return true  Read succeeded.
 * @return false Read failed.
 */
bool readI2CRegister(uint8_t reg, uint8_t *values, size_t len, bool print) {
    int ret = i2c_write_read_dt(&dev_i2c, &reg, 1, values, len);

    if (ret < 0) {
        printk("Failed i2c_write_read (addr=0x%02X, Reg=0x%02X). Err=%d\n",
               dev_i2c.addr, reg, ret);
        return false;
    }

    if (print) {
        printk("Read Reg.0x%02X ->", reg);
        for (size_t i = 0; i < len; i++) {
            printk(" 0x%02X", values[i]);
        }
        printk("\n");
    }

    return true;
}

/**
 * @brief Verify that the value written to a register matches the expected value.
 *
 * @param reg            The register address to verify.
 * @param expected_value The expected register value.
 * @return true          Verification passed.
 * @return false         Verification failed.
 */
bool verifyI2CRegister(uint8_t reg, uint8_t expected_value) {
    uint8_t read_value;

    if (!readI2CRegister(reg, &read_value, 1, false)) {
        printk("Verify Reg.0x%02X failed: can't read back.\n", reg);
        return false;
    }

    if (read_value != expected_value) {
        printk("Verify Reg.0x%02X mismatch! Expected=0x%02X, Got=0x%02X\n",
               reg, expected_value, read_value);
        return false;
    }

    printk("Verify Reg.0x%02X success. Value=0x%02X\n", reg, read_value);
    return true;
}

// /**
//  * @brief Write a value to a register and verify it
//  *
//  * @param reg       Register address
//  * @param value     Value to write
//  * @return true     Write and verify successful
//  * @return false    Write or verify failed
//  */
// bool writeAndVerifyI2CRegister(uint8_t reg, uint8_t value) {
//     if (!writeI2CRegister(reg, value)) {
//         printk("Failed to write Reg. 0x%02X.\n", reg);
//         return false;
//     }
//     if (!verifyI2CRegister(reg, value)) {
//         printk("Verification failed for Reg. 0x%02X.\n", reg);
//         return false;
//     }
//     return true;
// } 

int os67_sensor_i2c_init(void) {
    if (!device_is_ready(dev_i2c.bus)) {
        printk("I2C bus %s is not ready!\n", dev_i2c.bus->name);
        return -ENODEV; 
    }
    printk("I2C bus %s is ready.\n", dev_i2c.bus->name);

    k_msleep(50);
    return 0;
}

//**********************************************************************************************************************//
//**********************************************************************************************************************//
//**********************************************************************************************************************//


/**
 * @brief Configure measurement registers to initialize measurement settings.
 *
 * 1. Clear 0x11 (MEAS17..MEAS20) to disable these measurement channels.
 * 2. Clear 0x12 (MEAS9..MEAS16) to disable these measurement channels.
 * 3. Configure 0x13 (MEAS1..MEAS8) to enable the specified measurement channels.
 */
void config_meas(void)
{
    int ret = 0;
    uint8_t val = 0;

    //--- 1) Clear 0x11 (MEAS17..MEAS20)
    if (readI2CRegister(OS67_PPG_MEAS_SELECT_1, &val, 1, true)) {
        // Set all to 0 => disable MEAS17..MEAS20
        val = 0x00;
        if (!writeI2CRegister(OS67_PPG_MEAS_SELECT_1, val)) {
            printk("Failed to write MEAS_SELECT_1.\n");
            ret |= 1;
        } else if (!verifyI2CRegister(OS67_PPG_MEAS_SELECT_1, val)) {
            ret |= 1;
        }
    } else {
        ret |= 1;
    }

    //--- 2) Clear 0x12 (MEAS9..MEAS16)
    if (readI2CRegister(OS67_PPG_MEAS_SELECT_2, &val, 1, true)) {
        // Set all to 0 => disable MEAS9..MEAS16
        val = 0x00;
        if (!writeI2CRegister(OS67_PPG_MEAS_SELECT_2, val)) {
            printk("Failed to write MEAS_SELECT_2.\n");
            ret |= 1;
        } else if (!verifyI2CRegister(OS67_PPG_MEAS_SELECT_2, val)) {
            ret |= 1;
        }
    } else {
        ret |= 1;
    }

    //--- 3) Configure 0x13 (MEAS1..MEAS8)
    if (readI2CRegister(OS67_PPG_MEAS_SELECT_3, &val, 1, true)) {
        // According to requirement => bit0=1(MEAS1), bit1=0(MEAS2), bit2=1(MEAS3), others=0
        // => 0x05
        val = 0x05;
        if (!writeI2CRegister(OS67_PPG_MEAS_SELECT_3, val)) {
            printk("Failed to write MEAS_SELECT_3.\n");
            ret |= 1;
        } else if (!verifyI2CRegister(OS67_PPG_MEAS_SELECT_3, val)) {
            ret |= 1;
        }
    } else {
        ret |= 1;
    }
}


/**
 * @brief Configure MEAS1-related registers to initialize measurement parameters.
 *
 * 1. Configure MEAS1_SELECTS_REG (0x17) to disable unnecessary functions.
 * 2. Configure MEAS1_CONFIG_1 (0x18) for timing and averaging parameters.
 * 3. Configure MEAS1_CONFIG_2 (0x19) to adjust PPG ADC range.
 * 4. Configure MEAS1_CONFIG_3 (0x1A) for offset DAC (PPG1/PPG2).
 * 5. Configure MEAS1_CONFIG_4 (0x1B) for offset DAC (PPG3/PPG4).
 * 6. Configure MEAS1_CONFIG_5 (0x1C) for PD/LED stabilization time and LED range.
 * 7. Configure LED currents for LEDA/B (0x1D, 0x1E).
 * 8. Configure photodiode selections (0x1F~0x21).
 */
void config_MEAS1(void)
{
    int ret = 0;       // Used to record whether errors occur
    uint8_t val = 0;   // Used to store register read-back values

    //===========================================================
    // [A] Configure MEAS1_SELECTS_REG (0x17)
    //===========================================================
    if (readI2CRegister(MEAS1_SELECTS, &val, 1, true)) {
        val &= ~MEAS1_AMB_MASK;       // Disable AMB => bit6=0
        val &= ~MEAS1_DRVB_MASK;      // Clear DRVB => bits[5..3]=0
        val &= ~MEAS1_DRVA_MASK;      // Clear DRVA => bits[2..0]=0

        if (!writeI2CRegister(MEAS1_SELECTS, val)) {
            printk("Failed to write MEAS1_SELECTS.\n");
            ret |= 1;
        } else if (!verifyI2CRegister(MEAS1_SELECTS, val)) {
            printk("Failed to verify MEAS1_SELECTS.\n");
            ret |= 1;
        }
    } else {
        printk("Failed to read MEAS1_SELECTS.\n");
        ret |= 1;
    }

    //===========================================================
    // [B] Configure MEAS1_CONFIG_1 (0x18)
    //===========================================================
    if (readI2CRegister(MEAS1_CONFIG_1, &val, 1, true)) {
        val &= ~MEAS1_DAC_LO_NOISE_MASK;  // Disable DAC_LO_NOISE => bit7=0
        val &= ~MEAS1_FILT2_SEL_MASK;     // FILT2_SEL=0 => bit6=0

        val &= ~MEAS1_TINT_MASK;          // Clear TINT bits
        val |= MEAS1_TINT_117us;          // Set TINT=117us (0x3)

        val &= ~MEAS1_AVER_MASK;          // Clear AVER => bits[3..1]=0

        if (!writeI2CRegister(MEAS1_CONFIG_1, val)) {
            printk("Failed to write MEAS1_CONFIG_1.\n");
            ret |= 1;
        } else if (!verifyI2CRegister(MEAS1_CONFIG_1, val)) {
            ret |= 1;
        }
    } else {
        printk("Failed to read MEAS1_CONFIG_1.\n");
        ret |= 1;
    }

    //===========================================================
    // [C] Configure MEAS1_CONFIG_2 (0x19)
    //===========================================================
    if (readI2CRegister(MEAS1_CONFIG_2, &val, 1, true)) {
        val &= ~MEAS1_SINC3_SEL_MASK;     // Disable SINC3_SEL => bit7=0
        val &= ~MEAS1_FILT_SEL_MASK;      // Disable FILT_SEL => bit6=0

        val &= ~MEAS1_PPG4_ADC_RGE_MASK;  // Clear PPG4_ADC_RGE
        val |= (1 << MEAS1_PPG4_ADC_RGE_POS); // Set PPG4_ADC_RGE=1

        val &= ~MEAS1_PPG3_ADC_RGE_MASK;  // Clear PPG3_ADC_RGE
        val |= (1 << MEAS1_PPG3_ADC_RGE_POS); // Set PPG3_ADC_RGE=1

        val &= ~MEAS1_PPG2_ADC_RGE_MASK;  // Clear PPG2_ADC_RGE
        val |= (1 << MEAS1_PPG2_ADC_RGE_POS); // Set PPG2_ADC_RGE=1

        val &= ~MEAS1_PPG1_ADC_RGE_MASK;  // Clear PPG1_ADC_RGE
        val |= (1 << MEAS1_PPG1_ADC_RGE_POS); // Set PPG1_ADC_RGE=1

        if (!writeI2CRegister(MEAS1_CONFIG_2, val)) {
            printk("Failed to write MEAS1_CONFIG_2.\n");
            ret |= 1;
        } else if (!verifyI2CRegister(MEAS1_CONFIG_2, val)) {
            ret |= 1;
        }
    } else {
        printk("Failed to read MEAS1_CONFIG_2.\n");
        ret |= 1;
    }

    //===========================================================
    // [D] Configure MEAS1_CONFIG_3 (0x1A)
    //===========================================================
    if (readI2CRegister(MEAS1_CONFIG_3, &val, 1, true)) {
        val = 0x00;  // Clear offset DAC to default
        if (!writeI2CRegister(MEAS1_CONFIG_3, val)) {
            printk("Failed to write MEAS1_CONFIG_3.\n");
            ret |= 1;
        } else if (!verifyI2CRegister(MEAS1_CONFIG_3, val)) {
            ret |= 1;
        }
    } else {
        printk("Failed to read MEAS1_CONFIG_3.\n");
        ret |= 1;
    }

    //===========================================================
    // [E] Configure MEAS1_CONFIG_4 (0x1B)
    //===========================================================
    if (readI2CRegister(MEAS1_CONFIG_4, &val, 1, true)) {
        val = 0x00;  // Clear offset DAC to default
        if (!writeI2CRegister(MEAS1_CONFIG_4, val)) {
            printk("Failed to write MEAS1_CONFIG_4.\n");
            ret |= 1;
        } else if (!verifyI2CRegister(MEAS1_CONFIG_4, val)) {
            ret |= 1;
        }
    } else {
        printk("Failed to read MEAS1_CONFIG_4.\n");
        ret |= 1;
    }

    //===========================================================
    // [F] Configure MEAS1_CONFIG_5 (0x1C)
    //===========================================================
    if (readI2CRegister(MEAS1_CONFIG_5, &val, 1, true)) {
        val &= ~MEAS1_PD_SETLNG_MASK;       // Clear PD_SETLNG
        val |= (1 << MEAS1_PD_SETLNG_POS);  // Set PD_SETLNG=1

        val &= ~MEAS1_LED_SETLNG_MASK;      // Clear LED_SETLNG
        val |= (1 << MEAS1_LED_SETLNG_POS); // Set LED_SETLNG=1

        val &= ~MEAS1_LED_RGE_MASK;         // Clear LED_RGE
        val |= (3 << MEAS1_LED_RGE_POS);    // Set LED_RGE=3

        if (!writeI2CRegister(MEAS1_CONFIG_5, val)) {
            printk("Failed to write MEAS1_CONFIG_5.\n");
            ret |= 1;
        }
        // Uncomment this if verification is needed:
        /*
        else if (!verifyI2CRegister(MEAS1_CONFIG_5, val)) {
            ret |= 1;
        }
        */
    } else {
        printk("Failed to read MEAS1_CONFIG_5.\n");
        ret |= 1;
    }

    //===========================================================
    // [G] Configure LEDA/B current (0x1D, 0x1E)
    //===========================================================
    {
        uint8_t led_current = 0x0F;  // Assume DRVA_PA=0x0F, DRVB_PA=0x00

        if (readI2CRegister(MEAS1_LEDA_CURRENT, &led_current, 1, true)) {
            if (!writeI2CRegister(MEAS1_LEDA_CURRENT, led_current)) {
                printk("Failed to write LEDA_CURRENT.\n");
                ret |= 1;
            } else if (!verifyI2CRegister(MEAS1_LEDA_CURRENT, led_current)) {
                ret |= 1;
            }
        } else {
            printk("Failed to read MEAS1_LEDA_CURRENT.\n");
            ret |= 1;
        }

        led_current = 0x00;
        if (readI2CRegister(MEAS1_LEDB_CURRENT, &led_current, 1, true)) {
            if (!writeI2CRegister(MEAS1_LEDB_CURRENT, led_current)) {
                printk("Failed to write LEDB_CURRENT.\n");
                ret |= 1;
            } else if (!verifyI2CRegister(MEAS1_LEDB_CURRENT, led_current)) {
                ret |= 1;
            }
        } else {
            printk("Failed to read MEAS1_LEDB_CURRENT.\n");
            ret |= 1;
        }
    }

    //===========================================================
    // [H] Configure photodiode selections (0x1F~0x21)
    //===========================================================
    {
        uint8_t pd_val = 0x00;

        if (readI2CRegister(MEAS1_PD_SEL_1, &pd_val, 1, true)) {
            pd_val &= ~(PD1_SEL_MASK | PD2_SEL_MASK | PD3_SEL_HIGH_MASK);
            pd_val |= ((0x4 << PD1_SEL_POS) & PD1_SEL_MASK);
            pd_val |= ((0x5 << PD2_SEL_POS) & PD2_SEL_MASK);
            pd_val |= ((0x6 >> 1) << PD3_SEL_HIGH_POS);

            if (!writeI2CRegister(MEAS1_PD_SEL_1, pd_val)) {
                printk("Failed to write MEAS1_PD_SEL_1.\n");
                ret |= 1;
            } else if (!verifyI2CRegister(MEAS1_PD_SEL_1, pd_val)) {
                ret |= 1;
            }
        } else {
            printk("Failed to read MEAS1_PD_SEL_1.\n");
            ret |= 1;
        }

        if (readI2CRegister(MEAS1_PD_SEL_2, &pd_val, 1, true)) {
            pd_val &= ~(PD3_SEL_LOW_MASK | PD4_SEL_MASK);
            pd_val |= ((0x7 << PD4_SEL_POS) & PD4_SEL_MASK);

            if (!writeI2CRegister(MEAS1_PD_SEL_2, pd_val)) {
                printk("Failed to write MEAS1_PD_SEL_2.\n");
                ret |= 1;
            } else if (!verifyI2CRegister(MEAS1_PD_SEL_2, pd_val)) {
                ret |= 1;
            }
        } else {
            printk("Failed to read MEAS1_PD_SEL_2.\n");
            ret |= 1;
        }

        if (readI2CRegister(MEAS1_PD_SEL_3, &pd_val, 1, true)) {
            pd_val = 0x00;  // Clear selection
            if (!writeI2CRegister(MEAS1_PD_SEL_3, pd_val)) {
                printk("Failed to write MEAS1_PD_SEL_3.\n");
                ret |= 1;
            } else if (!verifyI2CRegister(MEAS1_PD_SEL_3, pd_val)) {
                ret |= 1;
            }
        } else {
            printk("Failed to read MEAS1_PD_SEL_3.\n");
            ret |= 1;
        }
    }

    if (ret) {
        printk("config_MEAS1() encountered error(s).\n");
    }
}



/**
 * @brief Configure MEAS3-related registers to initialize measurement parameters.
 *
 * 1. Configure MEAS3_SELECTS_REG (0x17) to disable unnecessary functions.
 * 2. Configure MEAS3_CONFIG_1 (0x18) for timing and averaging parameters.
 * 3. Configure MEAS3_CONFIG_2 (0x19) to adjust PPG ADC range.
 * 4. Configure MEAS3_CONFIG_5 (0x1C) for PD/LED stabilization time and LED range.
 * 5. Configure LED currents for LEDA/B (0x33, 0x34).
 * 6. Configure photodiode selections (0x35~0x36).
 */
void config_MEAS3(void)
{
    int ret = 0;       // Used to record whether errors occur
    uint8_t val = 0;   // Used to store register read-back values

    //===========================================================
    // [A] Configure MEAS3_SELECTS_REG (0x17)
    //===========================================================
    if (readI2CRegister(MEAS3_SELECTS, &val, 1, true)) {
        val &= ~MEAS3_AMB_MASK;       // Disable AMB => bit6=0
        val &= ~MEAS3_DRVB_MASK;      // Clear DRVB => bits[5..3]=0
        val &= ~MEAS3_DRVA_MASK;      // Clear DRVA => bits[2..0]=0

        if (!writeI2CRegister(MEAS3_SELECTS, val)) {
            printk("Failed to write MEAS3_SELECTS.\n");
            ret |= 1;
        } else if (!verifyI2CRegister(MEAS3_SELECTS, val)) {
            printk("Verification failed for MEAS3_SELECTS.\n");
            ret |= 1;
        }
    } else {
        printk("Failed to read MEAS3_SELECTS.\n");
        ret |= 1;
    }

    //===========================================================
    // [B] Configure MEAS3_CONFIG_1 (0x18)
    //===========================================================
    if (readI2CRegister(MEAS3_CONFIG_1, &val, 1, true)) {
        val &= ~MEAS3_DAC_LO_NOISE_MASK;  // Disable DAC_LO_NOISE => bit6=0
        val &= ~MEAS3_FILT2_SEL_MASK;     // FILT2_SEL=0 => bit5=0

        val &= ~MEAS3_TINT_MASK;          // Clear TINT bits
        val |= MEAS3_TINT_117us;          // Set TINT=117us (0x3)

        val &= ~MEAS3_AVER_MASK;          // Clear AVER => bits[2..0]=0

        if (!writeI2CRegister(MEAS3_CONFIG_1, val)) {
            printk("Failed to write MEAS3_CONFIG_1.\n");
            ret |= 1;
        } else if (!verifyI2CRegister(MEAS3_CONFIG_1, val)) {
            printk("Verification failed for MEAS3_CONFIG_1.\n");
            ret |= 1;
        }
    } else {
        printk("Failed to read MEAS3_CONFIG_1.\n");
        ret |= 1;
    }

    //===========================================================
    // [C] Configure MEAS3_CONFIG_2 (0x19)
    //===========================================================
    if (readI2CRegister(MEAS3_CONFIG_2, &val, 1, true)) {
        val &= ~MEAS3_SINC3_SEL_MASK;     // Disable SINC3_SEL => bit7=0
        val &= ~MEAS3_FILT_SEL_MASK;      // Disable FILT_SEL => bit6=0

        val &= ~MEAS3_PPG4_ADC_RGE_MASK;  // Clear PPG4_ADC_RGE
        val |= (1 << MEAS3_PPG4_ADC_RGE_POS); // Set PPG4_ADC_RGE=1

        val &= ~MEAS3_PPG3_ADC_RGE_MASK;  // Clear PPG3_ADC_RGE
        val |= (1 << MEAS3_PPG3_ADC_RGE_POS); // Set PPG3_ADC_RGE=1

        val &= ~MEAS3_PPG2_ADC_RGE_MASK;  // Clear PPG2_ADC_RGE
        val |= (1 << MEAS3_PPG2_ADC_RGE_POS); // Set PPG2_ADC_RGE=1

        val &= ~MEAS3_PPG1_ADC_RGE_MASK;  // Clear PPG1_ADC_RGE
        val |= (1 << MEAS3_PPG1_ADC_RGE_POS); // Set PPG1_ADC_RGE=1

        if (!writeI2CRegister(MEAS3_CONFIG_2, val)) {
            printk("Failed to write MEAS3_CONFIG_2.\n");
            ret |= 1;
        } else if (!verifyI2CRegister(MEAS3_CONFIG_2, val)) {
            printk("Verification failed for MEAS3_CONFIG_2.\n");
            ret |= 1;
        }
    } else {
        printk("Failed to read MEAS3_CONFIG_2.\n");
        ret |= 1;
    }

    //===========================================================
    // [D] Configure MEAS3_CONFIG_5 (0x1C)
    //===========================================================
    if (readI2CRegister(MEAS3_CONFIG_5, &val, 1, true)) {
        val &= ~MEAS3_PD_SETLNG_MASK;       // Clear PD_SETLNG
        val |= (1 << MEAS3_PD_SETLNG_POS);  // Set PD_SETLNG=1

        val &= ~MEAS3_LED_SETLNG_MASK;      // Clear LED_SETLNG
        val |= (1 << MEAS3_LED_SETLNG_POS); // Set LED_SETLNG=1

        val &= ~MEAS3_LED_RGE_MASK;         // Clear LED_RGE
        val |= (3 << MEAS3_LED_RGE_POS);    // Set LED_RGE=3

        if (!writeI2CRegister(MEAS3_CONFIG_5, val)) {
            printk("Failed to write MEAS3_CONFIG_5.\n");
            ret |= 1;
        }
        // Uncomment this if verification is needed:
        /*
        else if (!verifyI2CRegister(MEAS3_CONFIG_5, val)) {
            printk("Failed to verify MEAS3_CONFIG_5.\n");
            ret |= 1;
        }
        */
    } else {
        printk("Failed to read MEAS3_CONFIG_5.\n");
        ret |= 1;
    }

    //===========================================================
    // [E] Configure LEDA/B currents (0x33, 0x34)
    //===========================================================
    {
        uint8_t led_current = 0x00; // Assume initial value is 0x00

        if (readI2CRegister(MEAS3_LEDA_CURRENT, &led_current, 1, true)) {
            if (!writeI2CRegister(MEAS3_LEDA_CURRENT, led_current)) {
                printk("Failed to write MEAS3_LEDA_CURRENT.\n");
                ret |= 1;
            } else if (!verifyI2CRegister(MEAS3_LEDA_CURRENT, led_current)) {
                ret |= 1;
            }
        } else {
            printk("Failed to read MEAS3_LEDA_CURRENT.\n");
            ret |= 1;
        }

        if (readI2CRegister(MEAS3_LEDB_CURRENT, &led_current, 1, true)) {
            if (!writeI2CRegister(MEAS3_LEDB_CURRENT, led_current)) {
                printk("Failed to write MEAS3_LEDB_CURRENT.\n");
                ret |= 1;
            } else if (!verifyI2CRegister(MEAS3_LEDB_CURRENT, led_current)) {
                ret |= 1;
            }
        } else {
            printk("Failed to read MEAS3_LEDB_CURRENT.\n");
            ret |= 1;
        }
    }

    //===========================================================
    // [F] Configure photodiode selections (0x35~0x36)
    //===========================================================
    {
        uint8_t pd_val = 0x00;

        if (readI2CRegister(MEAS3_PD_SEL_1, &pd_val, 1, true)) {
            pd_val &= ~(MEAS3_PD1_SEL_MASK | MEAS3_PD2_SEL_MASK | MEAS3_PD3_SEL_MASK_HIGH);
            pd_val |= ((0x4 << MEAS3_PD1_SEL_POS) & MEAS3_PD1_SEL_MASK);
            pd_val |= ((0x5 << MEAS3_PD2_SEL_POS) & MEAS3_PD2_SEL_MASK);
            pd_val |= ((0x6 >> 1) << MEAS3_PD3_SEL_POS_HIGH);

            if (!writeI2CRegister(MEAS3_PD_SEL_1, pd_val)) {
                printk("Failed to write MEAS3_PD_SEL_1.\n");
                ret |= 1;
            } else if (!verifyI2CRegister(MEAS3_PD_SEL_1, pd_val)) {
                ret |= 1;
            }
        } else {
            printk("Failed to read MEAS3_PD_SEL_1.\n");
            ret |= 1;
        }

        if (readI2CRegister(MEAS3_PD_SEL_2, &pd_val, 1, true)) {
            pd_val &= ~(MEAS3_PD3_SEL_MASK_LOW | MEAS3_PD4_SEL_MASK);
            pd_val |= ((0x7 << MEAS3_PD4_SEL_POS) & MEAS3_PD4_SEL_MASK);

            if (!writeI2CRegister(MEAS3_PD_SEL_2, pd_val)) {
                printk("Failed to write MEAS3_PD_SEL_2.\n");
                ret |= 1;
            } else if (!verifyI2CRegister(MEAS3_PD_SEL_2, pd_val)) {
                ret |= 1;
            }
        } else {
            printk("Failed to read MEAS3_PD_SEL_2.\n");
            ret |= 1;
        }
    }

    if (ret) {
        printk("config_MEAS3() encountered error(s).\n");
    }
}


/**
 * @brief Enable the OS67 sensor and complete the initialization process.
 *
 * 1. Clear shutdown mode (System Conf1).
 * 2. Flush the FIFO queue (FIFO Conf2).
 * 3. Read and clear interrupt status (Interrupt Status 1 / 2).
 * 4. Configure interrupt pin functionality (Pin Functional Config).
 * 5. Enable FIFO data-ready interrupt (Interrupt1 Enable 1).
 * 6. Call configuration functions: config_meas(), config_MEAS1(), config_MEAS3().
 */
void OS67_enable(void)
{
    int ret = 0;  // Error accumulator
    uint8_t reg[OS67_INTERRUPT_STATUS_CNT] = {0};

    uint8_t pin_cfg = 0;   // Value of the interrupt pin configuration register (0x0E)
    uint8_t int1_en = 0;   // Value of the interrupt enable register (0xFA)

    /*----------------------------
     *  1) Clear shutdown mode (System Conf1)
     *----------------------------*/
    if (readI2CRegister(OS67_SYSTEM_CONF_1, &reg[0], 1, true)) {
        reg[0] &= ~OS67_SHDN;
        if (!writeI2CRegister(OS67_SYSTEM_CONF_1, reg[0])) {
            printk("Failed to clear shutdown.\n");
            ret |= 1;
        } else if (!verifyI2CRegister(OS67_SYSTEM_CONF_1, reg[0])) {
            ret |= 1;
        }
    } else {
        ret |= 1;
    }

    /*----------------------------
     *  2) Flush the FIFO queue (FIFO Conf2)
     *----------------------------*/
    if (readI2CRegister(OS67_FIFO_CONF_2, &reg[0], 1, true)) {
        reg[0] |= OS67_FLUSH_FIFO;
        if (!writeI2CRegister(OS67_FIFO_CONF_2, reg[0])) {
            printk("Failed to flush FIFO.\n");
            ret |= 1;
        } else if (!verifyI2CRegister(OS67_FIFO_CONF_2, reg[0])) {
            ret |= 1;
        }
    } else {
        ret |= 1;
    }

    /*----------------------------
     *  3) Read and clear interrupt status
     *----------------------------*/
    if (readI2CRegister(OS67_INTERRUPT_STATUS_1, reg, OS67_INTERRUPT_STATUS_CNT, true)) {
        printk("OS67_enable: status read = %02x %02x %02x %02x %02x\n", 
               reg[0], reg[1], reg[2], reg[3], reg[4]);
    } else {
        printk("Failed to read interrupt status.\n");
        ret |= 1;
    }

    /*----------------------------
     *  5) Enable FIFO data-ready interrupt (Interrupt1 Enable 1, 0xFA)
     *----------------------------*/
    if (readI2CRegister(OS67_INTERRUPT_STATUS_1_ENABLE, &int1_en, 1, true)) {
        int1_en |= FIFO_DATA_RDY_EN1;
        if (!writeI2CRegister(OS67_INTERRUPT_STATUS_1_ENABLE, int1_en)) {
            printk("Failed to enable FIFO_DATA_RDY_EN1.\n");
            ret |= 1;
        } else if (!verifyI2CRegister(OS67_INTERRUPT_STATUS_1_ENABLE, int1_en)) {
            ret |= 1;
        }
    } else {
        ret |= 1;
    }

    /*----------------------------
     *  6) Call configuration functions
     *----------------------------*/
    config_meas();
    config_MEAS1();
    config_MEAS3();

    if (ret != 0) {
        printk("OS67_enable encountered errors.\n");
    } else {
        printk("OS67_enable completed successfully.\n");
    }
}

/*****************************
 * OS67 Packet Structure
 * [0] - HEADER
 * [1] - MSB of Length of Data Packet
 * [2] - LSB of Length of Data Packet
 * [3] - Interrupt Status 1
 * [4] - Interrupt Status 2
 * [3+OS67_INTERRUPT_STATUS_CNT] - Sample Count MSB
 * [4+OS67_INTERRUPT_STATUS_CNT] - Sample Count LSB
 * [5+OS67_INTERRUPT_STATUS_CNT:] - FIFO Data
 *****************************/

void OS67_read_fifo(void) {
    uint8_t dataBuf[OS67_FIFO_SIZE * 3 + 6] = {0};
    int txSize = 0;
    int fifo_cnt = 0;

    printk("OS67_read_fifo GPIO\n");
    dataBuf[0] = COMM_DATA_HEADER;

    // Read Interrupt Status
    uint8_t reg_values[2] = {0};
    if (!readI2CRegister(OS67_INTERRUPT_STATUS_1, &reg_values[0], 1, true)) {
        printk("Failed to read interrupt status.\n");
        return;
    }
    if (!readI2CRegister(OS67_INTERRUPT_STATUS_2, &reg_values[1], 1, true)) {
        printk("Failed to read interrupt status2.\n");
        return;
    }
    dataBuf[3] = reg_values[0];
    dataBuf[4] = reg_values[1];
    printk("Stored in dataBuf[3]: 0x%02X\n", dataBuf[3]);
    printk("Stored in dataBuf[4]: 0x%02X\n", dataBuf[4]);
    k_msleep(50);

     // Read FIFO Overflow
    uint8_t ovf_status[2] = {0};
    if (!readI2CRegister(OS67_OVF, &ovf_status[0], 1, true)) {
        printk("Failed to read FIFO overflow status.\n");
        return;
    }
    if (!readI2CRegister(OS67_FIFO_DATA_CNT, &ovf_status[1], 1, true)) {
        printk("Failed to read FIFO overflow status.\n");
        return;
    }
    dataBuf[3 + OS67_INTERRUPT_STATUS_CNT] = ovf_status[0];
    dataBuf[4 + OS67_INTERRUPT_STATUS_CNT] = ovf_status[1];
    printk("Stored in dataBuf[3 + OS67_INTERRUPT_STATUS_CNT]: 0x%02X\n", dataBuf[3 + OS67_INTERRUPT_STATUS_CNT]);
    printk("Stored in dataBuf[4 + OS67_INTERRUPT_STATUS_CNT]: 0x%02X\n", dataBuf[4 + OS67_INTERRUPT_STATUS_CNT]); 
    if (ovf_status[0] & 0x3E) {
        dataBuf[0] = COMM_DATA_OVF_HEADER;
    }
    k_msleep(50);

    fifo_cnt = ((ovf_status[0] & 0xC0) << 2) | ovf_status[1];
    printk("Calculated FIFO Count: %d\n", fifo_cnt);
    dataBuf[3 + OS67_INTERRUPT_STATUS_CNT] = (fifo_cnt >> 8) & 0xFF;
    dataBuf[4 + OS67_INTERRUPT_STATUS_CNT] = fifo_cnt & 0xFF;
    printk("Stored in dataBuf[3 + OS67_INTERRUPT_STATUS_CNT]: 0x%02X\n", dataBuf[3 + OS67_INTERRUPT_STATUS_CNT]);
    printk("Stored in dataBuf[4 + OS67_INTERRUPT_STATUS_CNT]: 0x%02X\n", dataBuf[4 + OS67_INTERRUPT_STATUS_CNT]);
    k_msleep(500); 
    
    // Read FIFO Data
    uint8_t fifo_data[fifo_cnt * 4];
    if (!readI2CRegister(OS67_FIFO_DATA, fifo_data, fifo_cnt * 4, true)) {
        printk("Failed to read FIFO data.\n");
        return;
    }
    printk("Read FIFO Data: Reg. 0x%02X = ", OS67_FIFO_DATA);
    for (int i = 0; i < fifo_cnt * 4; i++) {
        printk("0x%02X ", fifo_data[i]);
    }
    printk("\n");

    for (int i = 0; i < fifo_cnt * 4; i++) {
        dataBuf[5 + OS67_INTERRUPT_STATUS_CNT + i] = fifo_data[i];
        printk("Stored in dataBuf[%d]: 0x%02X\n", 5 + OS67_INTERRUPT_STATUS_CNT + i, dataBuf[5 + OS67_INTERRUPT_STATUS_CNT + i]);
    }

    // Print a limited portion of dataBuf
    int printSize = 10; // Number of bytes to print from the start and end
    if (txSize <= printSize * 2) {
        printk("DataBuf: ");
        for (int i = 0; i < txSize; i++) {
            printk("0x%02X ", dataBuf[i]);
        }
        printk("\n");
    } else {
        printk("DataBuf (first %d bytes): ", printSize);
        for (int i = 0; i < printSize; i++) {
            printk("0x%02X ", dataBuf[i]);
        }
        printk("...\n");

        printk("DataBuf (last %d bytes): ", printSize);
        for (int i = txSize - printSize; i < txSize; i++) {
            printk("0x%02X ", dataBuf[i]);
        }
        printk("\n");
    }
}
