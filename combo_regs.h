// Define base addresses for the FE and FE2 regions
#define DR_REG_FE2_BASE   0x3FF45000
#define DR_REG_FE_BASE    0x3FF46000

// FE2 (0x3FF45000 block) Register Offsets
#define FE_GAINMEM_ADDR     (DR_REG_FE2_BASE + 0x0034)  // Address for gain memory operations (0x3FF45034)
#define FE_GAINMEM_DATA     (DR_REG_FE2_BASE + 0x0038)  // Data for gain memory operations (0x3FF45038)

#define FE_TXRATE_OFFSET0   (DR_REG_FE2_BASE + 0x003C)  // TX rate power offset 0 (0x3FF4503C)
#define FE_TXRATE_OFFSET1   (DR_REG_FE2_BASE + 0x0040)  // (0x3FF45040)
#define FE_TXRATE_OFFSET2   (DR_REG_FE2_BASE + 0x0044)  // (0x3FF45044)
#define FE_TXRATE_OFFSET3   (DR_REG_FE2_BASE + 0x0048)  // (0x3FF45048)
#define FE_TXRATE_OFFSET4   (DR_REG_FE2_BASE + 0x005C)  // (0x3FF4505C)
#define FE_TXRATE_OFFSET5   (DR_REG_FE2_BASE + 0x006C)  // (0x3FF4506C)
#define FE_TXRATE_OFFSET6   (DR_REG_FE2_BASE + 0x0070)  // (0x3FF45070)
#define FE_TXRATE_OFFSET7   (DR_REG_FE2_BASE + 0x0074)  // (0x3FF45074)
#define FE_TXRATE_OFFSET8   (DR_REG_FE2_BASE + 0x0078)  // (0x3FF45078)

#define FE_TONECFG          (DR_REG_FE2_BASE + 0x00A8)  // Tone configuration register (0x3FF450A8)
#define FE_TESTCTRL         (DR_REG_FE2_BASE + 0x00DC)  // Test control register (0x3FF450DC)
#define FE_PARAM3           (DR_REG_FE2_BASE + 0x00D0)  // Parameter 3 (specific purpose unknown) (0x3FF450D0)
#define FE_PARAM4           (DR_REG_FE2_BASE + 0x00D8)  // (0x3FF450D8)
#define FE_PARAM5           (DR_REG_FE2_BASE + 0x00F0)  // (0x3FF450F0)

// FE (0x3FF46000 block) Register Offsets
#define FE_CTRL0            (DR_REG_FE_BASE + 0x0000)  // Control register 0 (0x3FF46000)
#define FE_CTRL1            (DR_REG_FE_BASE + 0x0004)  // (0x3FF46004)
#define FE_CTRL2            (DR_REG_FE_BASE + 0x0008)  // (0x3FF46008)
#define FE_CTRL3            (DR_REG_FE_BASE + 0x000C)  // (0x3FF4600C)
#define FE_CTRL4            (DR_REG_FE_BASE + 0x0010)  // (0x3FF46010)
#define FE_CTRL5            (DR_REG_FE_BASE + 0x0030)  // (0x3FF46030)
#define FE_CTRL6            (DR_REG_FE_BASE + 0x0034)  // (0x3FF46034)
#define FE_CTRL7            (DR_REG_FE_BASE + 0x0038)  // (0x3FF46038)
#define FE_CTRL8            (DR_REG_FE_BASE + 0x003C)  // (0x3FF4603C)
#define FE_CTRL9            (DR_REG_FE_BASE + 0x0040)  // (0x3FF46040)
#define FE_CTRL10           (DR_REG_FE_BASE + 0x0044)  // (0x3FF46044)
#define FE_CTRL11           (DR_REG_FE_BASE + 0x0048)  // (0x3FF46048)
#define FE_CTRL12           (DR_REG_FE_BASE + 0x004C)  // (0x3FF4604C)

#define FE_AGCCTRL0         (DR_REG_FE_BASE + 0x0080)  // AGC control register 0 (0x3FF46080)
#define FE_AGCCTRL1         (DR_REG_FE_BASE + 0x008C)  // (0x3FF4608C)
#define FE_GEN_CTRL         (DR_REG_FE_BASE + 0x0090)  // (0x3FF46090)
#define FE_AGCCTRL2         (DR_REG_FE_BASE + 0x009C)  // (0x3FF4609C)

#define FE_TXCLKCFG         (DR_REG_FE_BASE + 0x00A0)  // TX clock configuration register (0x3FF460A0)
#define FE_PARAM6           (DR_REG_FE_BASE + 0x00B8)  // Parameter 6 (specific purpose unknown) (0x3FF460B8)
#define FE_BT_DIG_GAIN_FORCE (DR_REG_FE_BASE + 0x00DC) // Force BT digital gain (0x3FF460DC)
#define FE_IQ_MEAS_STATUS   (DR_REG_FE_BASE + 0x00E4)  // IQ measurement status register (0x3FF460E4)
#define FE_PARAM7           (DR_REG_FE_BASE + 0x0094)  // Parameter 7 (specific purpose unknown) (0x3FF46094)

// PHY Register Definitions for ESP32 (0x3FF5Cxxx / 0x3FF5Dxxx Range)
#define DR_REG_PHY_BASE                0x3FF5C000

// AGC Control Registers
#define PHY_AGC_CTRL0                  (DR_REG_PHY_BASE + 0x004)   // Clears bit 16 in agc_reg_init() (0x3FF5C004)
#define PHY_AGC_CTRL1                  (DR_REG_PHY_BASE + 0x07C)   // Sets [23:16] = 0x9C in agc_reg_init() (0x3FF5C07C)
#define PHY_AGC_WIFI_EN                (DR_REG_PHY_BASE + 0x080)   // Enables/disables Wi-Fi AGC (bit 0) (0x3FF5C080)
#define PHY_AGC_PARAM1                 (DR_REG_PHY_BASE + 0x088)   // Sets low byte = 5 (0x3FF5C088)
#define PHY_AGC_PARAM2                 (DR_REG_PHY_BASE + 0x094)   // Written as 0x01B8DD03 (0x3FF5C094)
#define PHY_AGC_PARAM3                 (DR_REG_PHY_BASE + 0x0A0)   // Configures gain/time window bits (0x3FF5C0A0)
#define PHY_AGC_PARAM4                 (DR_REG_PHY_BASE + 0x0A4)   // Configures bits [13:7], [6:0] (0x3FF5C0A4)
#define PHY_AGC_PARAM5                 (DR_REG_PHY_BASE + 0x0C4)   // Sets 0x6450403F in agc_reg_init() (0x3FF5C0C4)
#define PHY_AGC_PARAM6                 (DR_REG_PHY_BASE + 0x0F8)   // Sets [17:10] = 0x37 (0x3FF5C0F8)
#define PHY_AGC_TOGGLE_PARAM           (DR_REG_PHY_BASE + 0x02C)   // AGC toggle parameter (0x3FF5C02C)
#define PHY_AGC_BIGPARAM               (DR_REG_PHY_BASE + 0x030)   // Large AGC parameter (0x3FF5C030)

// Baseband (BB) Configuration Registers
#define PHY_BB_PARAM1                  (DR_REG_PHY_BASE + 0x0B8)   // Unclear usage; BB-related (0x3FF5C0B8)
#define PHY_BB_PARAM2                  (DR_REG_PHY_BASE + 0x0D0)   // Another BB config register (0x3FF5C0D0)
#define PHY_BB_PARAM3                  (DR_REG_PHY_BASE + 0x0F0)   // Loaded with large constants (0x3FF5C0F0)
#define PHY_BB_PARAM4                  (DR_REG_PHY_BASE + 0x104)   // Clears bit 15 in agc_reg_init() (0x3FF5C104)
#define PHY_BB_MISC0                   (DR_REG_PHY_BASE + 0x024)   // BB miscellaneous register 0 (0x3FF5C024)
#define PHY_BB_MISC1                   (DR_REG_PHY_BASE + 0x028)   // BB miscellaneous register 1 (0x3FF5C028)
#define PHY_BB_MISC2                   (DR_REG_PHY_BASE + 0x038)   // BB miscellaneous register 2 (0x3FF5C038)
#define PHY_BB_MISC3                   (DR_REG_PHY_BASE + 0x044)   // BB miscellaneous register 3 (0x3FF5C044)
#define PHY_BB_MISC4                   (DR_REG_PHY_BASE + 0x074)   // BB miscellaneous register 4 (0x3FF5C074)

// Antenna Configuration Registers
#define PHY_ANT_CFG0                   (DR_REG_PHY_BASE + 0x11C)   // Sets 0xE000 in phy_ant_init() (0x3FF5C11C)
#define PHY_ANT_CFG1                   (DR_REG_PHY_BASE + 0x120)   // Sets 0x1E101E1C (0x3FF5C120)
#define PHY_ANT_CFG2                   (DR_REG_PHY_BASE + 0x124)   // Adjusted if sense_thr > 0x20 (0x3FF5C124)

// Noise Measurement and CCA Registers
#define PHY_CCA_PARAM0                 (DR_REG_PHY_BASE + 0x010)   // CCA parameter 0 (0x3FF5C010)
#define PHY_CCA_PARAM1                 (DR_REG_PHY_BASE + 0x014)   // CCA parameter 1 (0x3FF5C014)
#define PHY_NOISE_MEAS_CTRL            (DR_REG_PHY_BASE + 0x018)   // Noise measurement control (0x3FF5C018)
#define PHY_CCA_PARAM2                 (DR_REG_PHY_BASE + 0x01C)   // CCA parameter 2 (0x3FF5C01C)
#define PHY_NOISE_MEAS_DATA            (DR_REG_PHY_BASE + 0x050)   // Noise measurement data (0x3FF5C050)

// CCA and Sense Thresholds
#define PHY_CCA_MULTI_FIELD            (DR_REG_PHY_BASE + 0x0CC)   // Manipulates thresholds (0x3FF5C0CC)
#define PHY_RX_SENSE_EN                (DR_REG_PHY_BASE + 0x108)   // Enables RX sense (0x3FF5C108)

// Calibration and Advanced Baseband (BB) Parameters
#define PHY_BB_CAL_PARAM0              (DR_REG_PHY_BASE + 0xC04)   // Calibration parameter 0 (0x3FF5CC04)
#define PHY_BB_CAL_PARAM1              (DR_REG_PHY_BASE + 0xC08)   // Calibration parameter 1 (0x3FF5CC08)
#define PHY_BB_CAL_PARAM2              (DR_REG_PHY_BASE + 0xC0C)   // Sets bits for channel filters (0x3FF5CC0C)
#define PHY_BB_CAL_PARAM3              (DR_REG_PHY_BASE + 0xD04)   // Calibration constant (0x3FF5CD04)
#define PHY_BB_CAL_PARAM4              (DR_REG_PHY_BASE + 0xD08)   // Another calibration constant (0x3FF5CD08)
#define PHY_BB_CAL_PARAM5              (DR_REG_PHY_BASE + 0xD0C)   // Additional calibration param (0x3FF5CD0C)

#define PHY_BB_ADV_PARAM0              (DR_REG_PHY_BASE + 0xCB8)   // Advanced BB param 0 (0x3FF5CCB8)
#define PHY_BB_ADV_PARAM1              (DR_REG_PHY_BASE + 0xCD8)   // Advanced BB param 1 (0x3FF5CD78)
#define PHY_BB_ADV_PARAM2              (DR_REG_PHY_BASE + 0xCDC)   // Advanced BB param 2 (0x3FF5CDDC)
#define PHY_BB_CTRL_DEBUG              (DR_REG_PHY_BASE + 0xCE4)   // Debug control (0x3FF5CDE4)

// Debug and Watchdog Registers (0x3FF5Dxxx Range)
#define PHY_BB_CTRL_MAIN               (DR_REG_PHY_BASE + 0x1000)  // Main BB control (0x3FF5D000)
#define PHY_BB_DEBUG_1                 (DR_REG_PHY_BASE + 0x1008)  // Debug register 1 (0x3FF5D008)
#define PHY_BB_DEBUG_2                 (DR_REG_PHY_BASE + 0x1014)  // Debug register 2 (0x3FF5D014)
#define PHY_BB_DEBUG_3                 (DR_REG_PHY_BASE + 0x1018)  // Debug register 3 (0x3FF5D018)
#define PHY_BB_DEBUG_4                 (DR_REG_PHY_BASE + 0x101C)  // Debug register 4 (0x3FF5D01C)
#define PHY_BB_DEBUG_5                 (DR_REG_PHY_BASE + 0x1020)  // Debug register 5 (0x3FF5D020)
#define PHY_BB_DEBUG_6                 (DR_REG_PHY_BASE + 0x103C)  // Debug register 6 (0x3FF5D03C)
#define PHY_BB_WDT_CTRL                (DR_REG_PHY_BASE + 0x1040)  // Watchdog control (0x3FF5D040)
#define PHY_BB_WDT_MISC                (DR_REG_PHY_BASE + 0x1044)  // Watchdog miscellaneous (0x3FF5D044)
#define PHY_BB_NOISE_READ              (DR_REG_PHY_BASE + 0x1050)  // Noise read register (0x3FF5D050)

// 11b/Low-Rate Parameters
#define PHY_11B_LR_PARAM0              (DR_REG_PHY_BASE + 0x804)   // Low-rate param 0 (0x3FF5C804)
#define PHY_11B_LR_PARAM1              (DR_REG_PHY_BASE + 0x85C)   // Low-rate param 1 (0x3FF5C85C)
#define PHY_11B_LR_PARAM2              (DR_REG_PHY_BASE + 0x860)   // Low-rate param 2 (0x3FF5C860)
#define PHY_11B_LR_PARAM3              (DR_REG_PHY_BASE + 0x87C)   // Low-rate param 3 (0x3FF5C87C)

// Filters and Calibration Parameters
#define PHY_BB_FILTER_PARAM            (DR_REG_PHY_BASE + 0xC48)   // Filter configuration (0x3FF5CC48)

/*
    Note: These names are proposed based on usage in phy_chip_v7.c.
    They may require refinement as more details about their operation are uncovered.
*/
