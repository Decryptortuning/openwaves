// PHY Register Definitions for ESP32 (0x3FF5Cxxx / 0x3FF5Dxxx Range)

#define DR_REG_PHY_BASE                0x3FF5C000

// AGC Control Registers
#define PHY_AGC_CTRL0                  (DR_REG_PHY_BASE + 0x004)   // Clears bit 16 in agc_reg_init().
#define PHY_AGC_CTRL1                  (DR_REG_PHY_BASE + 0x07C)   // Sets [23:16] = 0x9C in agc_reg_init().
#define PHY_AGC_WIFI_EN                (DR_REG_PHY_BASE + 0x080)   // Enables/disables Wi-Fi AGC (bit 0).
#define PHY_AGC_PARAM1                 (DR_REG_PHY_BASE + 0x088)   // Sets low byte = 5.
#define PHY_AGC_PARAM2                 (DR_REG_PHY_BASE + 0x094)   // Written as 0x01B8DD03.
#define PHY_AGC_PARAM3                 (DR_REG_PHY_BASE + 0x0A0)   // Configures gain/time window bits.
#define PHY_AGC_PARAM4                 (DR_REG_PHY_BASE + 0x0A4)   // Configures bits [13:7], [6:0].
#define PHY_AGC_PARAM5                 (DR_REG_PHY_BASE + 0x0C4)   // Sets 0x6450403F in agc_reg_init().
#define PHY_AGC_PARAM6                 (DR_REG_PHY_BASE + 0x0F8)   // Sets [17:10] = 0x37.
#define PHY_AGC_TOGGLE_PARAM           (DR_REG_PHY_BASE + 0x02C)   // AGC toggle parameter.
#define PHY_AGC_BIGPARAM               (DR_REG_PHY_BASE + 0x030)   // Large AGC parameter.

// Baseband (BB) Configuration Registers
#define PHY_BB_PARAM1                  (DR_REG_PHY_BASE + 0x0B8)   // Unclear usage; BB-related.
#define PHY_BB_PARAM2                  (DR_REG_PHY_BASE + 0x0D0)   // Another BB config register.
#define PHY_BB_PARAM3                  (DR_REG_PHY_BASE + 0x0F0)   // Loaded with large constants.
#define PHY_BB_PARAM4                  (DR_REG_PHY_BASE + 0x104)   // Clears bit 15 in agc_reg_init().
#define PHY_BB_MISC0                   (DR_REG_PHY_BASE + 0x024)   // BB miscellaneous register 0.
#define PHY_BB_MISC1                   (DR_REG_PHY_BASE + 0x028)   // BB miscellaneous register 1.
#define PHY_BB_MISC2                   (DR_REG_PHY_BASE + 0x038)   // BB miscellaneous register 2.
#define PHY_BB_MISC3                   (DR_REG_PHY_BASE + 0x044)   // BB miscellaneous register 3.
#define PHY_BB_MISC4                   (DR_REG_PHY_BASE + 0x074)   // BB miscellaneous register 4.

// Antenna Configuration Registers
#define PHY_ANT_CFG0                   (DR_REG_PHY_BASE + 0x11C)   // Sets 0xE000 in phy_ant_init().
#define PHY_ANT_CFG1                   (DR_REG_PHY_BASE + 0x120)   // Sets 0x1E101E1C.
#define PHY_ANT_CFG2                   (DR_REG_PHY_BASE + 0x124)   // Adjusted if sense_thr > 0x20.

// Noise Measurement and CCA Registers
#define PHY_CCA_PARAM0                 (DR_REG_PHY_BASE + 0x010)   // CCA parameter 0.
#define PHY_CCA_PARAM1                 (DR_REG_PHY_BASE + 0x014)   // CCA parameter 1.
#define PHY_NOISE_MEAS_CTRL            (DR_REG_PHY_BASE + 0x018)   // Noise measurement control.
#define PHY_CCA_PARAM2                 (DR_REG_PHY_BASE + 0x01C)   // CCA parameter 2.
#define PHY_NOISE_MEAS_DATA            (DR_REG_PHY_BASE + 0x050)   // Noise measurement data.

// CCA and Sense Thresholds
#define PHY_CCA_MULTI_FIELD            (DR_REG_PHY_BASE + 0x0CC)   // Manipulates thresholds.
#define PHY_RX_SENSE_EN                (DR_REG_PHY_BASE + 0x108)   // Enables RX sense.

// Calibration and Advanced Baseband (BB) Parameters
#define PHY_BB_CAL_PARAM0              (DR_REG_PHY_BASE + 0xC04)   // Calibration parameter 0.
#define PHY_BB_CAL_PARAM1              (DR_REG_PHY_BASE + 0xC08)   // Calibration parameter 1.
#define PHY_BB_CAL_PARAM2              (DR_REG_PHY_BASE + 0xC0C)   // Sets bits for channel filters.
#define PHY_BB_CAL_PARAM3              (DR_REG_PHY_BASE + 0xD04)   // Calibration constant.
#define PHY_BB_CAL_PARAM4              (DR_REG_PHY_BASE + 0xD08)   // Another calibration constant.
#define PHY_BB_CAL_PARAM5              (DR_REG_PHY_BASE + 0xD0C)   // Additional calibration param.

#define PHY_BB_ADV_PARAM0              (DR_REG_PHY_BASE + 0xCB8)   // Advanced BB param 0.
#define PHY_BB_ADV_PARAM1              (DR_REG_PHY_BASE + 0xCD8)   // Advanced BB param 1.
#define PHY_BB_ADV_PARAM2              (DR_REG_PHY_BASE + 0xCDC)   // Advanced BB param 2.
#define PHY_BB_CTRL_DEBUG              (DR_REG_PHY_BASE + 0xCE4)   // Debug control.

// Debug and Watchdog Registers (0x3FF5Dxxx Range)
#define PHY_BB_CTRL_MAIN               (DR_REG_PHY_BASE + 0x1000)  // Main BB control.
#define PHY_BB_DEBUG_1                 (DR_REG_PHY_BASE + 0x1008)  // Debug register 1.
#define PHY_BB_DEBUG_2                 (DR_REG_PHY_BASE + 0x1014)  // Debug register 2.
#define PHY_BB_DEBUG_3                 (DR_REG_PHY_BASE + 0x1018)  // Debug register 3.
#define PHY_BB_DEBUG_4                 (DR_REG_PHY_BASE + 0x101C)  // Debug register 4.
#define PHY_BB_DEBUG_5                 (DR_REG_PHY_BASE + 0x1020)  // Debug register 5.
#define PHY_BB_DEBUG_6                 (DR_REG_PHY_BASE + 0x103C)  // Debug register 6.
#define PHY_BB_WDT_CTRL                (DR_REG_PHY_BASE + 0x1040)  // Watchdog control.
#define PHY_BB_WDT_MISC                (DR_REG_PHY_BASE + 0x1044)  // Watchdog miscellaneous.
#define PHY_BB_NOISE_READ              (DR_REG_PHY_BASE + 0x1050)  // Noise read register.

// 11b/Low-Rate Parameters
#define PHY_11B_LR_PARAM0              (DR_REG_PHY_BASE + 0x804)   // Low-rate param 0.
#define PHY_11B_LR_PARAM1              (DR_REG_PHY_BASE + 0x85C)   // Low-rate param 1.
#define PHY_11B_LR_PARAM2              (DR_REG_PHY_BASE + 0x860)   // Low-rate param 2.
#define PHY_11B_LR_PARAM3              (DR_REG_PHY_BASE + 0x87C)   // Low-rate param 3.

// Filters and Calibration Parameters
#define PHY_BB_FILTER_PARAM            (DR_REG_PHY_BASE + 0xC48)   // Filter configuration.

/*
    Note: These names are proposed based on usage in phy_chip_v7.c.
    They may require refinement as more details about their operation are uncovered.
*/
