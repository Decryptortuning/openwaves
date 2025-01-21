// Define base addresses for the FE and FE2 regions
#define DR_REG_FE2_BASE   0x3FF45000
#define DR_REG_FE_BASE    0x3FF46000

// FE2 (0x3FF45000 block) Register Offsets
#define FE_GAINMEM_ADDR     (DR_REG_FE2_BASE + 0x0034)  // Address for gain memory operations
#define FE_GAINMEM_DATA     (DR_REG_FE2_BASE + 0x0038)  // Data for gain memory operations

#define FE_TXRATE_OFFSET0   (DR_REG_FE2_BASE + 0x003C)  // TX rate power offset 0
#define FE_TXRATE_OFFSET1   (DR_REG_FE2_BASE + 0x0040)
#define FE_TXRATE_OFFSET2   (DR_REG_FE2_BASE + 0x0044)
#define FE_TXRATE_OFFSET3   (DR_REG_FE2_BASE + 0x0048)
#define FE_TXRATE_OFFSET4   (DR_REG_FE2_BASE + 0x005C)
#define FE_TXRATE_OFFSET5   (DR_REG_FE2_BASE + 0x006C)
#define FE_TXRATE_OFFSET6   (DR_REG_FE2_BASE + 0x0070)
#define FE_TXRATE_OFFSET7   (DR_REG_FE2_BASE + 0x0074)
#define FE_TXRATE_OFFSET8   (DR_REG_FE2_BASE + 0x0078)

#define FE_TONECFG          (DR_REG_FE2_BASE + 0x00A8)  // Tone configuration register
#define FE_TESTCTRL         (DR_REG_FE2_BASE + 0x00DC)  // Test control register
#define FE_PARAM3           (DR_REG_FE2_BASE + 0x00D0)  // Parameter 3 (specific purpose unknown)
#define FE_PARAM4           (DR_REG_FE2_BASE + 0x00D8)
#define FE_PARAM5           (DR_REG_FE2_BASE + 0x00F0)

// FE (0x3FF46000 block) Register Offsets
#define FE_CTRL0            (DR_REG_FE_BASE + 0x0000)  // Control register 0
#define FE_CTRL1            (DR_REG_FE_BASE + 0x0004)
#define FE_CTRL2            (DR_REG_FE_BASE + 0x0008)
#define FE_CTRL3            (DR_REG_FE_BASE + 0x000C)
#define FE_CTRL4            (DR_REG_FE_BASE + 0x0010)
#define FE_CTRL5            (DR_REG_FE_BASE + 0x0030)
#define FE_CTRL6            (DR_REG_FE_BASE + 0x0034)
#define FE_CTRL7            (DR_REG_FE_BASE + 0x0038)
#define FE_CTRL8            (DR_REG_FE_BASE + 0x003C)
#define FE_CTRL9            (DR_REG_FE_BASE + 0x0040)
#define FE_CTRL10           (DR_REG_FE_BASE + 0x0044)
#define FE_CTRL11           (DR_REG_FE_BASE + 0x0048)
#define FE_CTRL12           (DR_REG_FE_BASE + 0x004C)

#define FE_AGCCTRL0         (DR_REG_FE_BASE + 0x0080)  // AGC control register 0
#define FE_AGCCTRL1         (DR_REG_FE_BASE + 0x008C)
#define FE_AGCCTRL2         (DR_REG_FE_BASE + 0x009C)

#define FE_TXCLKCFG         (DR_REG_FE_BASE + 0x00A0)  // TX clock configuration register
#define FE_PARAM6           (DR_REG_FE_BASE + 0x00B8)  // Parameter 6 (specific purpose unknown)
#define FE_BT_DIG_GAIN_FORCE (DR_REG_FE_BASE + 0x00DC) // Force BT digital gain
#define FE_IQ_MEAS_STATUS   (DR_REG_FE_BASE + 0x00E4)  // IQ measurement status register
#define FE_PARAM7           (DR_REG_FE_BASE + 0x0094)  // Parameter 7 (specific purpose unknown)
