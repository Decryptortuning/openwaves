typedef unsigned char   undefined;

typedef unsigned char    bool;
typedef unsigned char    byte;
typedef unsigned int    dword;
typedef long double    longdouble;
typedef long long    longlong;
typedef unsigned long long    qword;
typedef int    sdword;
typedef long long    sqword;
typedef short    sword;
typedef unsigned char    uchar;
typedef unsigned int    uint;
typedef unsigned long    ulong;
typedef unsigned long long    ulonglong;
typedef unsigned char    undefined1;
typedef unsigned short    undefined2;
typedef unsigned int    undefined3;
typedef unsigned int    undefined4;
typedef unsigned long long    undefined6;
typedef unsigned long long    undefined8;
typedef unsigned short    ushort;
typedef unsigned short    word;
#define unkbyte9   unsigned long long
#define unkbyte10   unsigned long long
#define unkbyte11   unsigned long long
#define unkbyte12   unsigned long long
#define unkbyte13   unsigned long long
#define unkbyte14   unsigned long long
#define unkbyte15   unsigned long long
#define unkbyte16   unsigned long long

#define unkuint9   unsigned long long
#define unkuint10   unsigned long long
#define unkuint11   unsigned long long
#define unkuint12   unsigned long long
#define unkuint13   unsigned long long
#define unkuint14   unsigned long long
#define unkuint15   unsigned long long
#define unkuint16   unsigned long long

#define unkint9   long long
#define unkint10   long long
#define unkint11   long long
#define unkint12   long long
#define unkint13   long long
#define unkint14   long long
#define unkint15   long long
#define unkint16   long long

#define unkfloat1   float
#define unkfloat2   float
#define unkfloat3   float
#define unkfloat5   double
#define unkfloat6   double
#define unkfloat7   double
#define unkfloat9   long double
#define unkfloat11   long double
#define unkfloat12   long double
#define unkfloat13   long double
#define unkfloat14   long double
#define unkfloat15   long double
#define unkfloat16   long double

#define BADSPACEBASE   void
#define code   void

typedef ulong size_t;

typedef uchar uint8;

typedef sword sint16;

typedef uchar u8;

typedef dword uint32;

typedef char sint8;

typedef char S8;

typedef dword u32;

typedef sdword s32;

typedef sdword int32;

typedef char int8;

typedef char s8;

typedef qword uint64;

typedef uint sizetype;

typedef uchar uint8_t;

typedef sword s16;

typedef word uint16;

typedef char int8_t;

typedef word u16;

typedef sqword sint64;

typedef dword uint32_t;

typedef uchar SDL_Octet;

typedef sdword sint32;

typedef uint UnsignedLongInt;

typedef short ShortInt;

typedef word uint16_t;

typedef uint CriticalType;

typedef ushort UnsignedShortInt;

typedef qword u64;

typedef sdword S32;

typedef struct RTC_TX_INFO RTC_TX_INFO, *PRTC_TX_INFO;

struct RTC_TX_INFO {
    u32 rate:6;
    u32 len:12;
    u32 bw40:1;
    u32 lr:1;
    u32 ldpc:1;
    u32 auto_rts_enable:1;
    u32 auto_cts_enable:1;
    u32 wait_ack_enable:1;
    u32 backoff:10;
    u32 aifs:4;
    undefined field10_0x6;
    undefined field11_0x7;
    u32 txframe_ptr;
    u32 key_type:3;
    undefined field14_0xd;
    undefined field15_0xe;
    undefined field16_0xf;
    u32 key_ptr;
};

typedef struct RTC_clock RTC_clock, *PRTC_clock;

struct RTC_clock {
    uint32_t freq;
    uint32_t period;
};

typedef enum SW_REJECT_TYPE {
    SW_NOREJECT=0,
    SW_REJECT=1
} SW_REJECT_TYPE;

typedef enum PM_SLEEP_MODE {
    PM_STALL_MACBB=1,
    PM_MODEM_SLEEP=2,
    PM_LIGHT_SLEEP=4,
    PM_LIGHT_SLEEP_NO_WIFIPD=8,
    PM_LIGHT_SLEEP_NO_WIFIPD_NO_LSP=16,
    PM_LIGHT_SLEEP_ALLON=32,
    PM_LIGHT_SLEEP_ALLON_LV=64,
    PM_MODEM_SLEEP_WIFI_PD=128,
    PM_LIGHT_SLEEP_ALLON_HI=256,
    PM_LIGHT_SLEEP_ALLON_pdMEM=512,
    PM_LIGHT_SLEEP_ALLON_HI_pdMEM=1024
} PM_SLEEP_MODE;

typedef enum WIFI_RATE {
    RATE_2M_SHORT=1,
    RATE_5_5M_SHORT=2,
    RATE_11M_SHORT=3,
    RATE_1M=4,
    RATE_2M_LONG=5,
    RATE_5_5M_LONG=6,
    RATE_11M_LONG=7,
    RATE_48M=8,
    RATE_24M=9,
    RATE_12M=10,
    RATE_6M=11,
    RATE_54M=12,
    RATE_36M=13,
    RATE_18M=14,
    RATE_9M=15
} WIFI_RATE;

typedef enum WIFI_CHANNEL {
    CHN_1=1,
    CHN_2=2,
    CHN_3=3,
    CHN_4=4,
    CHN_5=5,
    CHN_6=6,
    CHN_7=7,
    CHN_8=8,
    CHN_9=9,
    CHN_10=10,
    CHN_11=11,
    CHN_12=12,
    CHN_13=13,
    CHN_14=14
} WIFI_CHANNEL;

typedef struct rwip_rf_api rwip_rf_api, *Prwip_rf_api;

struct rwip_rf_api {
    void (*reset)(void);
    void (*force_agc_enable)(bool);
    bool (*txpwr_dec)(uint8_t);
    bool (*txpwr_inc)(uint8_t);
    void (*txpwr_max_set)(uint8_t);
    uint8_t (*txpwr_dbm_get)(uint8_t, uint8_t);
    uint8_t (*txpwr_cs_get)(int8_t);
    int8_t (*rssi_convert)(uint8_t);
    uint32_t (*reg_rd)(uint16_t);
    void (*reg_wr)(uint16_t, uint32_t);
    void (*sleep)(void);
    uint8_t txpwr_max;
    int8_t rssi_high_thr;
    int8_t rssi_low_thr;
    int8_t rssi_interf_thr;
    uint8_t wakeup_delay;
};

typedef struct phy_init_ctrl_s phy_init_ctrl_s, *Pphy_init_ctrl_s;

struct phy_init_ctrl_s {
    uint8 param_ver_id;
    uint8 crystal_select;
    sint8 gain_cmp[3];
    sint8 gain_cmp_ext2[3];
    sint8 gain_cmp_ext3[3];
    sint8 gain_cmp_bt_ofs[3];
    uint8 target_power_init[6];
    uint8 ratepwr_offset[8];
    uint8 pwr_ind_11b_en;
    uint8 pwr_ind_11b[2];
    uint8 fcc_enable;
    uint8 chan_pwr_limit_cbw20[14];
    uint8 fc_chan_pwr_limit_cbw40[5];
    uint8 pwrdet_new_en;
    uint8 fc_chan_pwr_limit_cbw40_h[4];
    uint8 init_remain_1[4];
    uint16 spur_freq_cfg_1;
    uint8 spur_freq_cfg_div_1;
    uint16 spur_freq_en_1;
    uint16 spur_freq_cfg_2;
    uint8 spur_freq_cfg_div_2;
    uint16 spur_freq_en_2;
    uint16 spur_freq_cfg_3;
    uint8 spur_freq_cfg_div_3;
    uint16 spur_freq_en_3;
    uint8 force_freq_offset_enbale;
    sint8 force_freq_offset_num;
    uint8 chan_pwr_limit_cbw20_gn[14];
};

typedef struct sleep_param_s sleep_param_s, *Psleep_param_s;

typedef struct sleep_param_s sleep_param_t;

typedef SDL_Octet U8;

typedef ShortInt S16;

typedef UnsignedShortInt U16;

struct sleep_param_s {
    uint32 param_flag;
    sint16 target_power_chan_backoff[4];
    uint16 txdc_table[20];
    uint16 wifi_txiq[2];
    uint16 wifi_rxiq[4];
    uint8 para_txcap[12];
    uint16 lb_txiq;
    sint16 rx_noise;
    uint8 rc_dout;
    int8 phy_channel_num;
    uint8 ht40_mode_cfg;
    int8 phy_sub_chan_cfg;
    uint8 filter_dcap_wifi[4];
    uint8 filter_dcap_bt[2];
    U8 phy_pwctrl_target_power[6];
    sint8 tx_pwctrl_atten[24];
    uint8 bt_pa_gain[9];
    sint8 bt_dig_atten[9];
    uint8 bt_txpwr_atten;
    sint8 delta_bt_atten[5];
    uint16 bt_bb_gain;
    uint16 bt_txiq[3];
    uint16 bt_txdc_table[12];
    uint8 wifi_rx_gain_max;
    uint8 bt_rx_gain_max;
    sint16 loop_noisefloor;
    S16 loop_pwctrl_pwdet_error_accum;
    S16 loop_pwctrl_correct_atten;
    S8 loop_pwctrl_correct_power_qdb;
    S8 loop_pwctrl_power_qdb_docal;
    U8 remain0[2];
    U16 loop_tx_rf_ana_gain[2];
    uint32 vdd33_code;
};

typedef struct phy_init_ctrl_s phy_init_ctrl_t;

typedef struct phy_api_ctrl_s phy_api_ctrl_s, *Pphy_api_ctrl_s;

struct phy_api_ctrl_s {
    uint8 low_power_en;
    sint8 vdd33_const;
    uint8 app_test_flg;
    uint8 app_tx_en;
    uint8 app_fcc_en;
    uint8 app_tx_chn;
    uint8 app_tx_rateOffset;
    uint8 rd_reg_addr0;
    uint8 rd_reg_addr1;
    uint8 rd_reg_addr2;
    uint8 rd_reg_addr3;
    uint8 rd_reg_test_en;
    uint8 app_tx_dut;
    uint8 app_tx_tone;
    sint8 app_tone_offset_khz;
};

typedef struct phy_api_ctrl_s phy_api_ctrl_t;

typedef struct cmd_desc cmd_desc, *Pcmd_desc;

typedef struct cmd_desc cmddesc;

typedef void (*_xtos_handler)(...);

struct cmd_desc {
    bool valid;
    bool duplicate;
    u32 dup_id;
    _xtos_handler cmd_handler;
    void *args;
};

typedef enum pm_sw_reject_t {
    PM_SW_NOREJECT=0,
    PM_SW_REJECT=1
} pm_sw_reject_t;

typedef struct LE_DTM_state LE_DTM_state, *PLE_DTM_state;

struct LE_DTM_state {
    u32 state;
    u32 cmd;
    u32 on;
    u32 chan;
    u32 len;
    u32 pac_type;
    u32 lst_cmd;
};

typedef struct LE_rx_adv_header LE_rx_adv_header, *PLE_rx_adv_header;

struct LE_rx_adv_header {
    u32 type;
    u32 TxAdd;
    u32 RxAdd;
    u32 len;
};

typedef struct polling_sample polling_sample, *Ppolling_sample;

struct polling_sample {
    u32 phase;
    u32 power;
};

typedef struct esp_phy_rx_result esp_phy_rx_result, *Pesp_phy_rx_result;

struct esp_phy_rx_result {
    u32 phy_rx_correct_count;
    int phy_rx_rssi;
    u32 phy_rx_total_count;
    u32 phy_rx_result_flag;
};

typedef enum esp_phy_ble_type_t {
    PHY_BLE_TYPE_1010=0,
    PHY_BLE_TYPE_00001111=1,
    PHY_BLE_TYPE_prbs9=2,
    PHY_BLE_TYPE_00111100=4,
    PHY_BLE_TYPE_MAX=5
} esp_phy_ble_type_t;

typedef enum esp_phy_wifi_rate_t {
    PHY_RATE_1M=0,
    PHY_RATE_2M=1,
    PHY_RATE_5M5=2,
    PHY_RATE_11M=3,
    PHY_RATE_48M=8,
    PHY_RATE_24M=9,
    PHY_RATE_12M=10,
    PHY_RATE_6M=11,
    PHY_RATE_54M=12,
    PHY_RATE_36M=13,
    PHY_RATE_18M=14,
    PHY_RATE_9M=15,
    PHY_RATE_MCS0=16,
    PHY_RATE_MCS1=17,
    PHY_RATE_MCS2=18,
    PHY_RATE_MCS3=19,
    PHY_RATE_MCS4=20,
    PHY_RATE_MCS5=21,
    PHY_RATE_MCS6=22,
    PHY_RATE_MCS7=23,
    PHY_WIFI_RATE_MAX=24
} esp_phy_wifi_rate_t;

typedef enum esp_phy_ble_rate_t {
    PHY_BLE_RATE_1M=0,
    PHY_BLE_RATE_2M=1,
    PHY_BLE_RATE_125K=2,
    PHY_BLE_RATE_500k=3,
    PHY_BLE_RATE_MAX=4
} esp_phy_ble_rate_t;

typedef enum RESET_REASON {
    NO_MEAN=0,
    POWERON_RESET=1,
    SW_RESET=3,
    OWDT_RESET=4,
    DEEPSLEEP_RESET=5,
    SDIO_RESET=6,
    TG0WDT_SYS_RESET=7,
    TG1WDT_SYS_RESET=8,
    RTCWDT_SYS_RESET=9,
    INTRUSION_RESET=10,
    TGWDT_CPU_RESET=11,
    SW_CPU_RESET=12,
    RTCWDT_CPU_RESET=13,
    EXT_CPU_RESET=14,
    RTCWDT_BROWN_OUT_RESET=15,
    RTCWDT_RTC_RESET=16
} RESET_REASON;

typedef struct link_supervision_struct link_supervision_struct, *Plink_supervision_struct;

struct link_supervision_struct {
    u32 init;
    u32 last_anchor;
    u32 time_out;
    u32 detach;
};

typedef struct packet_struct packet_struct, *Ppacket_struct;

struct packet_struct {
    uint type_code;
    uint link_type;
    uint payload_header_length;
    uint payload_length;
    uint payload1_header_length;
};

typedef enum rtc_cpu_freq_t {
    RTC_CPU_FREQ_XTAL=0,
    RTC_CPU_FREQ_80M=1,
    RTC_CPU_FREQ_160M=2,
    RTC_CPU_FREQ_240M=3,
    RTC_CPU_FREQ_2M=4
} rtc_cpu_freq_t;

typedef struct rtc_sleep_config_t rtc_sleep_config_t, *Prtc_sleep_config_t;

struct rtc_sleep_config_t {
    uint32_t soc_clk_sel:2;
    uint32_t lslp_mem_inf_fpu:1;
    uint32_t rtc_mem_inf_fpu:1;
    uint32_t rtc_mem_inf_follow_cpu:1;
    uint32_t rtc_fastmem_pd_en:1;
    uint32_t rtc_slowmem_pd_en:1;
    uint32_t rtc_peri_pd_en:1;
    uint32_t wifi_pd_en:1;
    uint32_t rom_mem_pd_en:1;
    uint32_t deep_slp:1;
    uint32_t wdt_flashboot_mod_en:1;
    uint32_t dig_dbias_wak:3;
    uint32_t dig_dbias_slp:3;
    uint32_t rtc_dbias_wak:3;
    uint32_t rtc_dbias_slp:3;
    uint32_t lslp_meminf_pd:1;
    uint32_t vddsdio_pd_en:1;
};

typedef enum rtc_xtal_freq_t {
    RTC_XTAL_FREQ_AUTO=0,
    RTC_XTAL_FREQ_24M=24,
    RTC_XTAL_FREQ_26M=26,
    RTC_XTAL_FREQ_40M=40
} rtc_xtal_freq_t;

typedef UnsignedLongInt U32;

typedef struct TX_LINK TX_LINK, *PTX_LINK;

struct TX_LINK {
    u32 link_dscr;
    u32 buf_addr;
    u32 next_addr;
};

typedef enum anon_enum_32 {
    CALI_RTC_MUX=0,
    CMD_OK=0,
    CPU_XTAL=0,
    DISEN_WAKEUP=0,
    NO_INT=0,
    NO_SLEEP=0,
    OK=0,
    XTAL_AUTO=0,
    CALI_8MD256=1,
    CMD_FAIL=1,
    CPU_80M=1,
    EXT_EVENT0_TRIG=1,
    EXT_EVENT0_TRIG_EN=1,
    FAIL=1,
    PM_STALL_MACBB=1,
    RATE_2M_SHORT=1,
    WAKEUP_INT=1,
    CALI_32K_XTAL=2,
    CPU_160M=2,
    EXT_EVENT1_TRIG=2,
    EXT_EVENT1_TRIG_EN=2,
    PENDING=2,
    PM_MODEM_SLEEP=2,
    RATE_5_5M_SHORT=2,
    REJECT_INT=2,
    BUSY=3,
    CPU_240M=3,
    RATE_11M_SHORT=3,
    CANCEL=4,
    CPU_2M=4,
    GPIO_TRIG=4,
    GPIO_TRIG_EN=4,
    PM_LIGHT_SLEEP=4,
    RATE_1M=4,
    SDIO_IDLE_INT=4,
    RATE_2M_LONG=5,
    RATE_5_5M_LONG=6,
    RATE_11M_LONG=7,
    PM_LIGHT_SLEEP_NO_WIFIPD=8,
    RATE_48M=8,
    RTC_WDT_INT=8,
    TIMER_EXPIRE=8,
    TIMER_EXPIRE_EN=8,
    RATE_24M=9,
    RATE_12M=10,
    RATE_6M=11,
    RATE_54M=12,
    RATE_36M=13,
    RATE_18M=14,
    RATE_9M=15,
    PM_LIGHT_SLEEP_NO_WIFIPD_NO_LSP=16,
    RTC_TIME_VALID_INT=16,
    SDIO_TRIG=16,
    SDIO_TRIG_EN=16,
    XTAL_24M=24,
    XTAL_26M=26,
    MAC_TRIG=32,
    MAC_TRIG_EN=32,
    PM_LIGHT_SLEEP_ALLON=32,
    XTAL_40M=40,
    PM_LIGHT_SLEEP_ALLON_LV=64,
    UART0_TRIG=64,
    UART0_TRIG_EN=64,
    PM_MODEM_SLEEP_WIFI_PD=128,
    UART1_TRIG=128,
    UART1_TRIG_EN=128,
    PM_LIGHT_SLEEP_ALLON_HI=256,
    TOUCH_TRIG=256,
    TOUCH_TRIG_EN=256,
    PM_LIGHT_SLEEP_ALLON_pdMEM=512,
    SAR_TRIG=512,
    SAR_TRIG_EN=512,
    BT_TRIG=1024,
    BT_TRIG_EN=1024,
    PM_LIGHT_SLEEP_ALLON_HI_pdMEM=1024,
    BIT_RATE_9600=9600,
    BIT_RATE_19200=19200,
    BIT_RATE_38400=38400,
    BIT_RATE_57600=57600,
    BIT_RATE_74880=74880,
    BIT_RATE_115200=115200,
    BIT_RATE_230400=230400,
    BIT_RATE_460800=460800,
    BIT_RATE_576000=576000,
    BIT_RATE_921600=921600
} anon_enum_32;

typedef enum STATUS {
    OK=0,
    FAIL=1,
    PENDING=2,
    BUSY=3,
    CANCEL=4
} STATUS;

typedef struct UartDevice.conflict UartDevice.conflict, *PUartDevice.conflict;

typedef enum UartBautRate {
    BIT_RATE_9600=9600,
    BIT_RATE_19200=19200,
    BIT_RATE_38400=38400,
    BIT_RATE_57600=57600,
    BIT_RATE_74880=74880,
    BIT_RATE_115200=115200,
    BIT_RATE_230400=230400,
    BIT_RATE_460800=460800,
    BIT_RATE_576000=576000,
    BIT_RATE_921600=921600
} UartBautRate;

typedef enum UartBitsNum4Char {
    FIVE_BITS=0,
    SIX_BITS=1,
    SEVEN_BITS=2,
    EIGHT_BITS=3
} UartBitsNum4Char;

typedef enum UartExistParity {
    STICK_PARITY_DIS=0,
    STICK_PARITY_EN=40
} UartExistParity;

typedef enum UartParityMode {
    NONE_BITS=0,
    ODD_BITS=0,
    EVEN_BITS=16
} UartParityMode;

typedef enum UartStopBitsNum {
    ONE_STOP_BIT=0,
    ONE_HALF_STOP_BIT=4,
    TWO_STOP_BIT=4
} UartStopBitsNum;

typedef enum UartFlowCtrl {
    NONE_CTRL=0,
    HARDWARE_CTRL=1,
    XON_XOFF_CTRL=2
} UartFlowCtrl;

typedef struct RcvMsgBuff.conflict RcvMsgBuff.conflict, *PRcvMsgBuff.conflict;

typedef enum RcvMsgState {
    BAUD_RATE_DET=0,
    WAIT_SYNC_FRM=1,
    SRCH_MSG_HEAD=2,
    RCV_MSG_BODY=3,
    RCV_ESC_CHAR=4
} RcvMsgState;

typedef enum RcvMsgBuffState {
    EMPTY=0,
    UNDER_WRITE=1,
    WRITE_OVER=2
} RcvMsgBuffState;

struct RcvMsgBuff.conflict {
    uint8 *pRcvMsgBuff;
    uint8 *pWritePos;
    uint8 *pReadPos;
    uint8 TrigLvl;
    enum RcvMsgBuffState BuffState;
};

struct UartDevice.conflict {
    enum UartBautRate baut_rate;
    enum UartBitsNum4Char data_bits;
    enum UartExistParity exist_parity;
    enum UartParityMode parity;
    enum UartStopBitsNum stop_bits;
    enum UartFlowCtrl flow_ctrl;
    uint8 buff_uart_no;
    uint8 tx_uart_no;
    struct RcvMsgBuff.conflict rcv_buff;
    enum RcvMsgState rcv_state;
    int received;
};

typedef struct TrxMsgBuff TrxMsgBuff, *PTrxMsgBuff;

struct TrxMsgBuff {
    uint32 TrxBuffSize;
    uint8 *pTrxBuff;
};

typedef struct UartDevice UartDevice, *PUartDevice;

typedef struct RcvMsgBuff RcvMsgBuff, *PRcvMsgBuff;

struct RcvMsgBuff {
    uint32 RcvBuffSize;
    uint8 *pRcvMsgBuff;
    uint8 *pWritePos;
    uint8 *pReadPos;
    uint8 TrigLvl;
    enum RcvMsgBuffState BuffState;
};

struct UartDevice {
    enum UartBautRate baut_rate;
    enum UartBitsNum4Char data_bits;
    enum UartExistParity exist_parity;
    enum UartParityMode parity;
    enum UartStopBitsNum stop_bits;
    enum UartFlowCtrl flow_ctrl;
    struct RcvMsgBuff rcv_buff;
    struct TrxMsgBuff trx_buff;
    enum RcvMsgState rcv_state;
    int received;
    int buff_uart_no;
};

typedef struct phy_romfuncs phy_romfuncs, *Pphy_romfuncs;

struct phy_romfuncs {
    void (*phy_disable_agc_)(void);
    void (*phy_enable_agc_)(void);
    void (*disable_agc_)(...);
    void (*enable_agc_)(...);
    void (*phy_disable_cca_)(void);
    void (*phy_enable_cca_)(void);
    uint (*pow_usr_)(uint, uint);
    uint8 (*gen_rx_gain_table_)(uint32 *, uint8, uint16 *, sint8 *, uint8, bool);
    void (*set_loopback_gain_)(uint16, uint16, uint16);
    void (*set_cal_rxdc_)(uint8, uint8, uint32 *);
    void (*loopback_mode_en_)(bool);
    s32 (*get_data_sat_)(s32, s32, s32);
    void (*set_pbus_mem_)(...);
    void (*write_gain_mem_)(u32, u32, u8);
    void (*rx_gain_force_)(int, int);
    void (*set_txclk_en_)(bool);
    void (*set_rxclk_en_)(bool);
    void (*start_tx_tone_step_)(bool, sint16, uint8, bool, sint16, uint8);
    void (*start_tx_tone_)(int, int, int, int, int, int);
    void (*stop_tx_tone_)(int);
    void (*bb_bss_bw_40_en_)(int);
    void (*clk_force_on_vit_)(int);
    void (*bb_rx_ht20_cen_bcov_en_)(int);
    void (*bb_tx_ht20_cen_)(int);
    void (*spur_reg_write_one_tone_)(U8, int);
    int (*spur_coef_cfg_)(int8, uint16, int8);
    void (*bb_wdg_test_en_)(int, int, int, int, int, int);
    int8 (*mhz2ieee_)(uint16, uint32);
    void (*bb_bss_cbw40_dig_)(int);
    void (*cbw2040_cfg_)(bool);
    void (*noise_floor_auto_set_)(...);
    sint16 (*phy_get_noisefloor_)(...);
    sint16 (*check_noise_floor_)(...);
    void (*set_noise_floor_)(sint16);
    void (*chip_v7_rx_rifs_en_)(uint8);
    U32 (*rtc_mem_backup_)(int, int, int);
    U32 (*rtc_mem_recovery_)(int, int, int);
    uint8 (*chip_i2c_readReg_)(uint8, uint8, uint8);
    uint8 (*i2c_readReg_)(uint8, uint8, uint8);
    void (*chip_i2c_writeReg_)(uint8, uint8, uint8, uint8);
    void (*i2c_writeReg_)(uint8, uint8, uint8, uint8);
    uint8 (*i2c_readReg_Mask_)(uint8, uint8, uint8, uint8, uint8);
    void (*i2c_writeReg_Mask_)(uint8, uint8, uint8, uint8, uint8, uint8);
    void (*pbus_force_mode_)(bool);
    u32 (*pbus_rd_addr_)(u8, u8);
    u32 (*pbus_rd_shift_)(u8, u8);
    void (*pbus_force_test_)(u8, u8, u16);
    u16 (*pbus_rd_)(u8, u8);
    void (*pbus_debugmode_)(void);
    void (*pbus_workmode_)(void);
    void (*pbus_set_rxgain_)(uint32);
    void (*pbus_xpd_rx_off_)(...);
    void (*pbus_xpd_rx_on_)(U16);
    void (*pbus_xpd_tx_off_)(...);
    void (*pbus_xpd_tx_on_)(U16, U16);
    void (*pbus_set_dco_)(U16 *);
    void (*rfpll_reset_)(void);
    void (*restart_cal_)(void);
    void (*write_rfpll_sdm_)(uint8 *);
    void (*wait_rfpll_cal_end_)(void);
    void (*rfpll_set_freq_)(uint32, uint8, sint16, uint8 *);
    uint16 (*set_channel_freq_)(int8, uint8, sint16);
    void (*phy_freq_correct_)(bool, S16);
    void (*set_rf_freq_offset_)(uint8, U16, sint16);
    void (*chip_v7_rx_init_)(void);
    void (*chip_v7_tx_init_)(void);
    void (*chip_v7_bt_init_)(void);
    uint8 (*txbbgain_to_index_)(uint16);
    uint16 (*index_to_txbbgain_)(uint8);
    void (*txdc_cal_init_)(U16 *, U16, U16, bool);
    void (*txdc_cal_v70_)(sint16 *);
    void (*en_pwdet_)(...);
    void (*txcal_debuge_mode_)(...);
    void (*txcal_work_mode_)(...);
    int8 (*txiq_set_reg_)(int8, bool);
    void (*read_sar_dout_)(U16 *);
    S16 (*get_fm_sar_dout_)(S16 *, S16 *);
    S16 (*txtone_linear_pwr_)(bool);
    void (*txiq_get_mis_pwr_)(bool, uint8, S16, S16 *, S16 *, bool);
    void (*txiq_cover_)(U8, S16, int8 *, bool);
    sint32 (*abs_temp_)(sint32);
    void (*iq_est_enable_)(bool, uint16);
    void (*iq_est_disable_)(...);
    void (*dc_iq_est_)(bool, uint16, sint32 *);
    void (*pbus_rx_dco_cal_)(uint16, sint16 *, uint16, bool, bool);
    void (*rxiq_get_mis_)(U8, int8 *, bool);
    int8 (*rxiq_set_reg_)(int8, bool);
    void (*rxiq_cover_mg_mp_)(uint8, int8 *, int8 *, bool);
    void (*rfcal_rxiq_)(uint8, S16, U8, sint8 *, bool);
    uint16 (*get_rfcal_rxiq_data_)(S16, U8, bool);
    sint8 (*set_chan_cal_interp_)(sint8 *, uint8);
    void (*set_txcap_reg_)(U8 *, uint8);
    void (*rfcal_txcap_)(uint8, uint8, bool, U8 *);
    S16 (*linear_to_db_)(int, uint8);
    S16 (*get_power_db_)(uint16);
    S16 (*meas_tone_pwr_db_)(S8);
    S16 (*tx_pwr_backoff_)(U8 *);
    void (*rfcal_pwrctrl_)(U8, U8 *, U8, U8, S8 *, uint16, S8, bool);
    void (*tx_atten_set_interp_)(S8 *, u8, S8);
    void (*target_power_add_backoff_)(U8 *, U8 *, S16);
    U8 (*get_rf_gain_qdb_)(U8);
    void (*correct_rf_ana_gain_)(S8 *, U16 *, U16 *);
    U16 (*phy_get_vdd33_)(...);
    S16 (*get_sar_dout_)(uint16);
    S8 (*get_pwctrl_correct_)(S16, S16 *, U8, U8);
    void (*tx_pwctrl_bg_init_)(...);
};

typedef struct RxControl RxControl, *PRxControl;

struct RxControl {
    int rssi:8;
    uint rate:5;
    uint cur_service:1;
    uint sig_mode:2;
    uint legacy_length:12;
    uint rxmatch0:1;
    uint rxmatch1:1;
    uint rxmatch2:1;
    uint rxmatch3:1;
    uint MCS:7;
    uint CWB:1;
    uint HT_length:16;
    uint Smoothing:1;
    uint Not_Sounding:1;
    uint Aggregation:1;
    uint STBC:2;
    uint FEC_CODING:1;
    uint SGI:1;
    int noise_floor:8;
    uint ampdu_cnt:8;
    uint channel:4;
    uint rxstart_time_cycle:7;
    uint is_group:1;
    uint timestamp:32;
    uint bb_info:32;
    undefined field25_0x14;
    uint rx_channel_estimate_len:10;
    uint rxstart_time_cycle_dec:11;
    uint ant_status:1;
    uint sig_len:12;
    uint dump_len:12;
    uint rx_state:8;
};

typedef struct Elf32_Rela Elf32_Rela, *PElf32_Rela;

struct Elf32_Rela {
    dword r_offset; // location to apply the relocation action
    dword r_info; // the symbol table index and the type of relocation
    dword r_addend; // a constant addend used to compute the relocatable field value
};

typedef struct Elf32_Sym Elf32_Sym, *PElf32_Sym;

struct Elf32_Sym {
    dword st_name;
    dword st_value;
    dword st_size;
    byte st_info;
    byte st_other;
    word st_shndx;
};

typedef enum Elf_SectionHeaderType {
    SHT_NULL=0,
    SHT_PROGBITS=1,
    SHT_SYMTAB=2,
    SHT_STRTAB=3,
    SHT_RELA=4,
    SHT_HASH=5,
    SHT_DYNAMIC=6,
    SHT_NOTE=7,
    SHT_NOBITS=8,
    SHT_REL=9,
    SHT_SHLIB=10,
    SHT_DYNSYM=11,
    SHT_INIT_ARRAY=14,
    SHT_FINI_ARRAY=15,
    SHT_PREINIT_ARRAY=16,
    SHT_GROUP=17,
    SHT_SYMTAB_SHNDX=18,
    SHT_ANDROID_REL=1610612737,
    SHT_ANDROID_RELA=1610612738,
    SHT_GNU_ATTRIBUTES=1879048181,
    SHT_GNU_HASH=1879048182,
    SHT_GNU_LIBLIST=1879048183,
    SHT_CHECKSUM=1879048184,
    SHT_SUNW_move=1879048186,
    SHT_SUNW_COMDAT=1879048187,
    SHT_SUNW_syminfo=1879048188,
    SHT_GNU_verdef=1879048189,
    SHT_GNU_verneed=1879048190,
    SHT_GNU_versym=1879048191
} Elf_SectionHeaderType;

typedef struct Elf32_Shdr Elf32_Shdr, *PElf32_Shdr;

struct Elf32_Shdr {
    dword sh_name;
    enum Elf_SectionHeaderType sh_type;
    dword sh_flags;
    dword sh_addr;
    dword sh_offset;
    dword sh_size;
    dword sh_link;
    dword sh_info;
    dword sh_addralign;
    dword sh_entsize;
};

typedef struct Elf32_Ehdr Elf32_Ehdr, *PElf32_Ehdr;

struct Elf32_Ehdr {
    byte e_ident_magic_num;
    char e_ident_magic_str[3];
    byte e_ident_class;
    byte e_ident_data;
    byte e_ident_version;
    byte e_ident_osabi;
    byte e_ident_abiversion;
    byte e_ident_pad[7];
    word e_type;
    word e_machine;
    dword e_version;
    dword e_entry;
    dword e_phoff;
    dword e_shoff;
    dword e_flags;
    word e_ehsize;
    word e_phentsize;
    word e_phnum;
    word e_shentsize;
    word e_shnum;
    word e_shstrndx;
};




void BT_tx_LE_en(u32 enable);
int hoppe_tx_tone(uint16 freq,uint16 delay_num,S16 reg_freq_1,U8 tone1_atten,int bt,int *alltime);
void BT_tx_8m_disable(void);
void BT_fill_tx_buffer_1m_1010(void);
void BT_fill_tx_buffer_1m_0011(void);
void BT_fill_tx_buffer_1m_prbs9(void);
void BT_fill_tx_buffer_2m_1010(void);
void BT_fill_tx_buffer_2m_prbs9(void);
void BT_fill_tx_buffer_3m_1010(void);
void BT_fill_tx_buffer_3m_prbs9(void);
void BT_start_tx_1m(uint freq_point);
void BT_start_tx_1m_new(void);
void BT_start_tx_2m_3m(uint freq_point);
void BT_start_tx_2m_3m_new(void);
void BT_wait4tx_end(uint data_rate);
void BT_wait4tx_end_new(void);
void BT_start_tx(uint32 data_rate,uint freq_point);
void BT_start_tx_8m(uint32 data_rate);
void BT_set_tx_on_guard_time(void);
void BT_set_tx_on_guard_time_new(void);
void BT_fill_tx_buffer(uint32 data_rate,uint32 data_type);
void BT_tx_forever(uint16 tx_freq_offset_500k);
void BT_start_tx_packet(uint16 tx_freq_offset_500k,uint32 data_rate,uint32 delay);
void BT_start_tx_packet_8m(uint16 tx_freq_offset_500k,uint32 data_rate,uint32 delay);
void BT_tx_packet(uint16 freq_odd,uint16 freq_even,uint16 tx_freq_offset_500k,uint32 slot_time,uint32 delay,uint32 data_rate,uint32 data_type,uint32 tx_slot,uint32 tx_off_delay);
void BT_tx_packet_8m(uint16 freq_odd,uint16 freq_even,uint16 tx_freq_offset_500k,uint32 slot_time,uint32 delay,uint32 data_rate,uint32 data_type,uint32 tx_slot,uint32 tx_off_delay);
void BT_mac_iqview_debug(u32 nowhite);
void BT_rx_start(uint16 data_rate,uint16 freq_offset_500k,uint16 start,uint16 bit_len,uint16 freq);
void bt_bb_tx_end_int_init(void);
void bt_bb_tx_end_service(void);
void bt_bb_rx_start_int_init(void);
void bt_bb_rx_start_service(void);
void bt_rw_start_int_init(void);
void bt_rw_start_service(void);
void BT_tx_LE_en(u32 enable);
void LE_fill_tx_access_addr(u32 ac);
void LE_fill_tx_PDU_byte(u32 nbyte,u32 data);
void LE_fill_tx_header_adv(u32 type,u32 length);
void LE_fill_tx_header_data(u32 length);
void LE_fill_tx_payload_byte(u32 nbyte,u32 data);
void LE_fill_tx_buffer_1010(void);
void LE_start_tx(u32 channel,u32 data_type,u32 loop);
void BT_get_pwr(u32 n,u32 *power_min,u32 *power_average,u32 *power_filted);
void BT_hoppe(void);
void BTAutotest(void);
void le_uart_intr(void *para);
u32 le_dtm_tx_timer(u32 init);
void le_bb_init(void);
void set_le_rx_base(u32 addr);
void set_le_tx_attributes(u32 type,u32 crypt,u32 len,u32 link_addr,u32 device_sel);
void set_le_tx_header(u32 header);
void set_le_rx_attributes(u32 type,u32 crypt,u32 device_sel);
void set_ac_timer(u32 en,u32 delay);
void set_ifs(u32 ifs,u32 rxtx_delay,u32 txrx_delay);
void le_tx_en(u32 device_sel);
u32 le_wait4tx_end(void);
void le_rx_en(u32 device_sel);
u32 le_wait4rx_end(void);
u32 get_le_rx_header(u32 rx_base);
u32 get_le_rx_len(u32 rx_base,u32 type);
u32 get_le_payload_4byte(u32 rx_base,u32 i);
void le_init_loopback(u32 chan_id,u32 type,u32 crypt,u32 device_sel);
void le_rx_compare(u32 len,u32 *total_bits,u32 *err_bits,u32 *ref);
void le_loopback_slave(u32 type,u32 crypt,u32 device_sel,u32 loop_time,u32 *total_bits,u32 *err_bits,u32 *ref,u32 *cnpac);
void le_run_master(u32 device_sel);
void le_test_master(u32 chan_id,u32 type,u32 crypt,u32 device_sel,u32 len);
void le_test_slave(u32 chan_id,u32 type,u32 crypt,u32 device_sel);
u32 get_ADV_PDU_type(u32 rx_base);
u32 get_ADV_PDU_header(u32 rx_base,LE_rx_adv_header *header);
void fill_ADV_PAC(u32 PDU_type,u32 TxAdd,u32 RxAdd,u32 len,u32 payload_link);
void fill_DATA_PAC(u32 LLID,u32 SN,u32 NESN,u32 MD,u32 len,u32 payload_link);
sint16 noise_init_le(void);
void LE_rx_prbs9_status(u32 *total_bits,u32 *err_bits,u32 type);
void LE_rx_per(u32 chan_id,u32 type);
void LE_rx_per_debug(void);
void LE_rx_ber(u32 bits,u32 type,u32 chan_id);
void fill_TEST_PAC(LE_DTM_state *state);
u32 le_dtm_tx_timer(u32 init);
void le_dtm_state_refresh(LE_DTM_state *state,u32 cmd);
void le_dtm_rsp(LE_DTM_state *state);
void le_dtm(void);
uint32_t rf_rw_reg_rd(uint16_t addr);
void rf_rw_reg_wr(uint16_t addr,uint32_t value);
uint8_t rf_rw_txpwr_dbm_get(uint8_t txpwr_idx,uint8_t modulation);
void rf_rw_reset(void);
int8_t rf_rw_rssi_convert(uint8_t rssi_reg);
bool rf_rw_txpwr_inc(uint8_t link_id);
void rf_rw_txpwr_max_set(uint8_t link_id);
uint8_t rf_rw_txpwr_cs_get(int8_t txpwr_dbm);
undefined4 rf_rw_txpwr_dec(void);
void rf_rw_force_agc_enable(void);
void rf_rw_sleep(void);
void rw_init_em_radio_table(void);
void rf_rw_bt_init(void);
void rf_rw_le_init(void);
void rw_bt_core_on(void);
void rw_le_core_on(void);
void rw_bt_core_off(void);
void rw_le_core_off(void);
void rf_rw_init(rwip_rf_api *api);
void rw_le_scan_refresh(u32 txdesc_refresh,u32 winsz);
void rw_le_et_on_mask(u32 mask);
void rw_le_et_init(void);
void rw_le_set_dbaddr(u32 bdaddru,u32 bdaddrl);
void rw_le_init_cs(u32 format,u32 sync_word,u32 crc_init,u32 ral_en,u32 ral_mode,u32 local_rpa_sel,u32 filter_policy,u32 ch_idx,u32 hopint,u32 fh_en,u32 txpwr,u32 rxwide,u32 rxwinsz,u32 txdesc,u32 maxevtime);
void rw_le_cs_set_txpwr(u32 txpwr);
void rw_le_txpwr_toggle_test(void);
void rw_le_cs_set_freq(u32 addr,u32 freq);
void rw_le_cs_set_txdesc(u32 addr,u32 desc);
void rw_le_init_tx_descriptor(u32 is_connevt,u32 ptr,u32 nxt_ptr,u32 llid,u32 nesn,u32 sn,u32 md,u32 len,u32 type,u32 txadd,u32 rxadd,u32 advlen,u32 bufptr);
void rw_le_init_rx_descryptor(u32 ptr,u32 nxt_ptr,u32 bufptr,u32 ralptr);
void rw_le_set_current_rxdecriptor(void);
void rw_le_rxstat_unpack(u32 desc_addr,u32 *rxlinklbl,u32 *rxpriverr,u32 *rxtimeerr,u32 *rxbdaddrmatch,u32 *rxnesnerr,u32 *rxsnerr,u32 *rxmicerr,u32 *rxcrcerr,u32 *rxlenerr,u32 *rxtyperr,u32 *rxsyncerr,s8 *rxrssi);
u32 rw_le_rxdone(u32 desc_addr);
void rw_le_rxundone(u32 desc_addr);
void rw_le_rx_status_print(u32 desc_addr);
void rw_evt_refresh(u32 txdesc_refresh);
u32 rw_le_scan_print(u32 desc_addr,u32 filter,u32 rx_adva);
void rw_evt_refresh(u32 txdesc_refresh);
void rw_le_scan_refresh(u32 txdesc_refresh,u32 winsz);
void rw_evt_refresh_check_rx(u32 txdesc_refresh);
void rw_pktcntl_fsm_print(void);
void rw_evtschdl_fsm_print(void);
void rw_evtcntl_fsm_print(void);
void rw_evtcntladv_fsm_print(void);
void rw_le_error_print(void);
void rw_le_rx_per_syncw_service_polling(void);
void rw_fill_adv_buf(u32 ptr);
void rw_fill_scanrsp_buf(u32 ptr);
void rw_le_adv_init(u32 tx_desc0,u32 tx_desc1,u32 format,u32 bdaddrl,u32 bdaddrh,u32 interval);
void rw_le_advscan_init(u32 tx_desc0,u32 format,u32 bdaddrl,u32 bdaddrh,u32 interval);
void rw_le_test_init(u32 tx_desc0,u32 tx_desc1,u32 format,u32 bdaddrl,u32 bdaddrh,u32 interval,u32 freq,u32 txpwr,u32 rxwide,u32 rxwinsz);
void rw_le_adv(void);
void rw_le_advscan(void);
void rw_le_advscan_test_init(void);
void rw_le_adv_test_init(void);
void freq_print(void);
void hoppe_print(void);
void rw_le_tx(u32 freqa,u32 freqb,u32 freqc,u32 txpwr,u32 mask);
void rw_le_tx_nohoppe(u32 freq,u32 txpwr,u32 mask,u32 len,u32 data_type,u32 tx_num_in);
void rw_le_tx_nohoppe_syncw(u32 freq,u32 txpwr,u32 mask,u32 len,u32 data_type,u32 syncw,u32 tx_num_in);
void rw_le_tx_hoppe_syncw(u32 txpwr,u32 mask,u32 len,u32 data_type,u32 syncw);
void rw_le_tx_nohoppe_init(u32 freq,u32 txpwr,u32 mask,u32 len);
void fcc_le_tx(u32 txpwr,u32 chan,u32 len,u32 data_type,u32 tx_num_in);
void fcc_le_tx_syncw(u32 txpwr,u32 chan,u32 len,u32 data_type,u32 syncw,u32 tx_num_in);
void rw_le_rx_per(u32 chan);
bool esp_get_lerx_result(int32 *rx_result);
void get_bt_rx_rssi(u32 *pwr_in_band,u32 *gain,u32 *pwr_full_band,u32 total_p);
void rw_le_rx_per_syncw(u32 chan,u32 syncw);
void rw_le_rx_per_syncw_rssi(u32 chan,u32 syncw);
void rw_le_rx_per_init(u32 chan);
void rw_le_rx_per_syncw_polling(u32 chan);
void rw_le_rx_per_syncw_polling_stop(void);
void rw_le_adv_cmd(u32 txpwr);
void cmd_polling_test0(u32 a);
void cmd_polling_test1(u32 b);
void cmd_polling_test2(polling_sample *b);
void cmd_polling_test3(void);
void rw_bb_rx_refesh(void);
void rw_bb_tx_refesh(void);
void rw_txpwr_regulate(void);
void rw_frm_refresh(u32 txdesc_refresh);
void rw_frm_refresh_pscan(u32 txdesc_refresh);
void bt_tx_pwr_check(void);
void bt_tx_pwr_force(void);
void rw_bch_gen(u32 lap,u32 *bchl,u32 *bchh);
void rw_bch_gen_test(void);
void rw_dm_activate(void);
void rw_et_init(void);
void rw_et_csb_init(void);
void rw_et_off(void);
void rw_et_one(u32 one);
void rw_et_on(void);
void rw_et_on_mask(u32 mask);
void rw_et_fill(u32 mask,u32 cs_addr);
void rw_init_encryt_space(void);
void rw_init_cs_inqscan(void);
void rw_init_cs_tx_test(void);
void rw_init_cs_rx_test(void);
void rw_init_cs(u32 format,u32 ltaddr,u32 edr,u32 whdsb,u32 bdaddrl,u32 bdaddrh,u32 bchl,u32 bchh,u32 txpwr,u32 hoppe_en,u32 freq,u32 tx_eir,u32 rxwide,u32 rxwinsz,u32 txdesc,u32 maxfrmtime,u32 rxthr);
void rw_cs_set_freq(u32 addr,u32 freq);
void rw_cs_set_txdesc(u32 addr,u32 desc);
u32 rw_cs_get_rxclkn(u32 ptr);
u32 rw_cs_get_rxbit(u32 ptr);
void rw_init_tx_descriptor(u32 ptr,u32 nxt_ptr,u32 seqn,u32 arqn,u32 flow,u32 type,u32 ltaddr,u32 length,u32 pflow,u32 llid,u32 aclbufptr,u32 lmpbufptr);
void rw_init_tx_descriptor_dh1or2dh1_test(u32 ptr,u32 nxt_ptr,u32 aclbufptr,u32 lmpbufptr);
void rw_init_tx_descriptor_dh3or3dh3_test(u32 ptr,u32 nxt_ptr,u32 aclbufptr,u32 lmpbufptr);
void rw_init_tx_descriptor_dm1_test(u32 ptr,u32 nxt_ptr,u32 aclbufptr,u32 lmpbufptr);
void rw_init_tx_descriptor0(void);
void rw_init_tx_descriptor1(void);
void rw_init_rx_descryptor(u32 desc_addr,u32 nxt_desc_addr,u32 aclbuf_addr,u32 lmpbuf_addr);
void rw_init_rx_descryptor0(void);
void rw_init_rx_descryptor1(void);
void rw_init_tx_aclbuffer0(void);
void rw_init_tx_aclbuffer1(void);
void rw_init_tx_lmpbuffer0(void);
void rw_init_tx_lmpbuffer1(void);
void rw_test_init(void);
void rw_set_current_rxdecriptor(void);
void rw_fill_FHS_buf(u32 ptr,u32 parity_l,u32 parity_h,u32 lap,u32 eir,u32 sr,u32 uap,u32 nap,u32 cod,u32 ltaddr);
u32 rw_get_FHS_clkn(u32 ptr);
void rw_fill_eir_buf(u32 ptr);
void rw_bt_rxstat_unpack(u32 desc_addr,u32 *rxlinklbl,u32 *rxeirstat,u32 *rxguarderr,u32 *rxbadlt,u32 *rxfecerr,u32 *rxseqerr,u32 *rxmicerr,u32 *rxcrcerr,u32 *rxhecerr,u32 *rxsyncerr);
u32 rw_bt_rxdone(u32 desc_addr);
void rw_bt_rxundone(u32 desc_addr);
void rw_rx_status_print(u32 desc_addr);
u32 rw_rx_status_print_(u32 desc_addr);
void rw_rx_head_get(u32 desc_addr,u32 *ptype,u32 *len,u32 *llid,s8 *rssi);
s8 rw_rx_rssi_get(u32 desc_addr);
void rw_rx_refresh(void);
void rw_bb_ac_refesh(void);
void rw_bb_rx_refesh(void);
void rw_bb_tx_refesh(void);
void rw_txpwr_regulate(void);
void rw_frm_refresh(u32 txdesc_refresh);
void rw_frm_refresh_pscan(u32 txdesc_refresh);
void rw_frm_refresh_no_cmd(u32 txdesc_refresh);
void rw_frm_refresh_check_rx(void);
void rw_pcntl_fsm_print(void);
void rw_pcntl_fsm_tx_print(void);
void rw_pcntl_fsm_tx_pld_print(void);
void rw_fsm_print(void);
void rw_error_print(void);
void rw_inq_tx_freq_print(u32 all);
void em_test(void);
void rom_test(u32 base,u32 len,u32 repeat);
void em_clean(void);
void rw_em_print(void);
void rw_em_diag_print(void);
void rw_reg_diag_print(u32 addr,u32 *diag_last,u32 *diag_print_start,u32 label);
void rw_le_em_diag_print(void);
void rw_debug_freq(u32 tx_freq,u32 rx_freq);
void rw_inq_scan(void);
void rw_inq_scan_cmd(u32 tx_freq,u32 rx_freq);
void rw_page_scan_cmd(u32 debug_freq,u32 tx_freq,u32 rx_freq);
void rw_mst_cmd(u32 debug_freq,u32 tx_freq,u32 rx_freq);
void rw_set_inq_abtrain(void);
void rw_inq(void);
void rw_inq_cmd(u32 tx_freq,u32 rx_freq);
void rw_page_cmd(u32 debug_freq,u32 tx_freq,u32 rx_freq);
void rw_tx_test(u32 txpwr,u32 hoppe,u32 freq,u32 edr,u32 type,u32 length,u32 et_mask,u32 data_type,u32 tx_num_in);
void rw_tx_test_init(u32 txpwr,u32 hoppe,u32 freq,u32 edr,u32 type,u32 length,u32 et_mask);
void rw_inq_test_init(u32 txpwr,u32 hoppe,u32 freq,u32 edr,u32 type,u32 length,u32 et_mask);
void rw_tx_test_wifi_tx(u32 txpwr,u32 hoppe,u32 freq,u32 edr,u32 type,u32 length,u32 et_mask);
void fe_force_tx_tone(u32 wifi_chan);
void bt_force_tx_tone(u32 start,u32 bt_chan,u32 power);
void wifi_tx_coex_init(void);
void force_wifi_toggle(u32 n);
void RW_stop_radiocntl(void);
void RW_resume_radiocntl(void);
void t0_toggle_force_wifi(void);
void t0_toggle_scan_check(void);
void t0_toggle_adv_check(void);
void t1_toggle_refresh_rw(void);
void t1_toggle_check_rx(void);
void t1_toggle_refresh_rw_le(void);
void t1_toggle_refresh_rw_le_check_rx(void);
void t1_toggle_refresh_rw_le_advscan(void);
void fcc_bt_en(void);
void fcc_tx(u32 txpwr,u32 hoppe,u32 chan,u32 rate,u32 DH_type,u32 data_type,u32 tx_num_in);
void fcc_tx_len(u32 txpwr,u32 hoppe,u32 chan,u32 rate,u32 length,u32 data_type,u32 tx_num_in);
void rw_rx_per(u32 edr,u32 chan);
void rw_rx_prbs9_status(u32 desc_addr,u32 *total_bits,u32 *err_bits);
bool esp_get_btrx_result(int32 *rx_result);
void rw_rx_per_ulap(u32 edr,u32 chan,u32 ulap,u32 ltaddr);
void rw_rx_per_ulap_rssi(u32 edr,u32 chan,u32 ulap,u32 ltaddr);
void rw_rx_per_init(u32 edr,u32 chan);
void bt_bb_dump(u32 data_sel,u32 dump_len);
u32 rw_rx_dump(u32 edr,u32 chan,u32 ulap,u32 ltaddr,u32 data_sel,u32 dump_len,u32 status_sel);
u32 rw_clkn(void);
u32 rw_next_et_entry(u32 clkn);
void rw_tx_buf_init(void);
void rw_dut_timing_refresh(u32 *clknos,u32 *bitos,u32 rxbit);
void rw_dut_timing_refresh_cs(u32 ptr,u32 *clknos,u32 *bitos,u32 rxbit,u32 refresh);
u32 rw_dut_try_con(u32 ulap,u32 nap,u32 bchl,u32 bchh,u32 freq,u32 freq1,u32 t,u32 *clknos,u32 *bitos,u32 nmin,u32 nmax,u32 *mulap,u32 *mnap);
void rw_dut_con(u32 ulap,u32 nap,u32 bchl,u32 bchh,u32 hoppe,u32 freq,u32 whdsb,u32 edr,u32 type,u32 len,u32 et_mask,u32 *nfrm,u32 *nduttx,u32 *ndutrx,u32 *ntesttx,u32 *ntestrx,u32 n,u32 nfrmmax,u32 *clknos,u32 *bitos,u32 txpwr);
u32 rw_tester_try_con(u32 ulap,u32 nap,u32 mulap,u32 mnap,u32 bchl,u32 bchh,u32 freq,u32 n,u32 nmin,s8 *rssi);
void rw_tester_con(u32 ulap,u32 nap,u32 bchl,u32 bchh,u32 hoppe,u32 freq,u32 n,u32 whdsb,u32 edr,u32 type,u32 len,u32 et_mask,u32 *nduttx,u32 *ndutrx,u32 *ntesttx,u32 *ntestrx,u32 *testrxac,u32 *testrxa,u32 nfrmmax,s8 *rssi_max,s8 *rssi_min,s8 *rssi_avg,s8 *rssi_max_tester,s8 *rssi_min_tester,s8 *rssi_avg_tester,u32 txpwr);
void rw_bt_func_tester(void);
u32 rw_bt_func_test_dut(u32 ulap,u32 nap,u32 freq,u32 n,u32 *nslvtx,u32 *nslvrx,u32 *ntx,u32 *nrx,u32 *nrxac,u32 *nrxall,s8 *con_rssi_max,s8 *con_rssi_min,s8 *con_rssi_avg,s8 *tester_rssi_max,s8 *tester_rssi_min,s8 *tester_rssi_avg);
void rw_bt_func_dut(void);
void rw_bt_func_test_init(void);
void rw_bt_func_test_tester(u32 freq0,u32 freq1,u32 n);
void rw_slp_test(void);
void cmd_polling_test(void);
void rw_autotest(void);
u32 BT_wait4tx_end_(void);
void BT_test_mode_fill_tx_payload_(u32 lpayload,u32 data_type,u32 loopback,u32 device_sel,u32 tx_addr_base);
void init_NALL_packet_struct(void);
void init_POLL_packet_struct(void);
void init_FHS_packet_struct(void);
void init_DM1_packet_struct(void);
void init_DH1_packet_struct(void);
void init_2DH1_packet_struct(void);
void init_HV1_packet_struct(void);
void init_HV2_packet_struct(void);
void init_HV3_packet_struct(void);
void init_DV_packet_struct(void);
void init_EV3_packet_struct(void);
void init_EV4_packet_struct(void);
void init_EV5_packet_struct(void);
void init_2EV3_packet_struct(void);
void init_3EV3_packet_struct(void);
void init_2EV5_packet_struct(void);
void init_3EV5_packet_struct(void);
void init_AUX1_packet_struct(void);
void init_DM3_packet_struct(void);
void init_DH3_packet_struct(void);
void init_DM5_packet_struct(void);
void init_DH5_packet_struct(void);
void init_3DH1_packet_struct(void);
void init_2DH3_packet_struct(void);
void init_3DH3_packet_struct(void);
void init_2DH5_packet_struct(void);
void init_3DH5_packet_struct(void);
void BT_init_packet_struct(void);
u32 TX_CONF0(u32 sel);
u32 TX_CONF1(u32 sel);
u32 TX_HEAD(u32 sel);
u32 RX_CONF(u32 sel);
void prbs9_gen(u32 *pattern);
void prbs9_test(void);
u32 BT_test_mode_get_rx_payload_byte(u32 byte_position);
u32 BT_test_mode_get_rx_payload_4byte(u32 byte_position);
u32 BT_test_mode_get_tx_payload_byte(u32 data_type,u32 loopback,u32 byte_offset,u32 word_offset,u32 byte_cnt,u32 rx_addr_base);
void BT_test_mode_fill_tx_payload(u32 lheader,u32 lpayload,u32 data_type,u32 loopback,u32 device_sel);
int BT_tx_sim_debug(void);
int BT_tx_sim(u32 next_valid,u32 slave_sel);
int BT_tx_sim_rx_prepare(u32 next_valid,u32 slave_sel);
int BT_rx_sim_debug(void);
int BT_rx_sim(u32 slave_sel);
int BT_rx_sim_tx_prepare(u32 slave_sel);
void BT_rx_en(u32 slave_sel);
void BT_tx_en(u32 slave_sel);
void BT_wait4tx_start(u32 slave_sel);
void BT_wait4rx_start(u32 slave_sel);
u32 BT_wait4rx_end(void);
u32 BT_wait4tx_end_(void);
void BT_set_cmd_timer(u32 clksel,u32 time,u32 time_delay_us);
void BT_mac_set_tx_bytelength(uint tx_buf_addr,int pheader_length,int pheader1_length,int payloadlength,u32 device_sel);
void BT_mac_set_tx_type_code(uint tx_buf_addr,uint type_code,u32 device_sel);
void BT_mac_set_tx_link_type(uint link_type,u32 device_sel);
void BT_mac_set_rx_link_type(uint link_type,u32 device_sel);
void BT_eir_set_tx_bytelength(int pheader_length,int payloadlength);
void BT_eir_set_tx_type_code(uint type_code);
void set_eir_packetheader(u32 LT_ADDR,u32 TYPE,u32 FLOW,u32 SEQN,u32 ARQN);
void set_eir_LLID(u32 LLID);
void BT_fill_eir_sample_pac(u32 type,u32 pheader_len,u32 len,u32 payload_addr);
int BT_mac_loopback_df(u32 device_sel);
void BT_mac_loopback_master_debug(packet_struct packet,u32 loopback);
void BT_mac_loopback_master(packet_struct packet,u32 loopback);
void BT_mac_loopback_slave(u32 loopback,u32 link_type,u32 slave_sel);
void BT_testmode_init(uint16 freq_offset_500k,uint16 freq,u32 nowhite);
void BT_set_packet_attributes(u32 type_code,u32 link_type,u32 payload_header_len,u32 payload1_header_len,u32 payload_len,u32 device_sel);
void BT_s_set_nulap(u32 m_ulap,u32 m_nap,u32 s_ulap,u32 s_nap);
void BT_m_set_nulap(u32 m_ulap,u32 m_nap,u32 s_ulap,u32 s_nap);
u32 BT_get_fhs_lap(void);
u32 BT_get_fhs_uap(void);
u32 BT_get_fhs_nap(void);
u32 BT_get_fhs_lt_addr(void);
u32 BT_get_fhs_clk(void);
u32 BT_get_fhs_eir(void);
u32 BT_get_fhs_cod(void);
u32 BT_get_LLID(void);
void BT_s_page_set_master_nulap(u32 slave_sel);
u32 BT_s_page(u32 hoppe_en,u32 clk_debug,u32 slave_sel,u32 fast);
u32 BT_m_page(u32 hoppe_en,u32 clk_debug,u32 lt_addr);
void BT_rx_prbs9_status(u32 *total_bits,u32 *err_bits);
void BT_rx_prbs9_err_bits(void);
void BT_rx_content_status(u32 *total_bits,u32 *err_bits,u32 data_type);
sint16 noise_init_bt(void);
void BT_loopback_master(u32 loopback,uint16 freq_offset_500k,uint16 freq,u32 link_type,u32 nowhite);
void BT_loopback_slave(u32 link_type,u32 loopback,u32 rx_payload_length,uint16 freq_offset_500k,uint16 freq,u32 nowhite);
void BT_inquiry_test(u32 hoppe_en,u32 chan);
void BT_inquiry_scan_test(u32 hoppe_en,u32 chan);
void BT_page_test(u32 link_type,u32 hoppe_en,u32 loopback);
void BT_page_scan_test(u32 link_type,u32 hoppe_en,u32 loopback);
void BT_mac_tx_packet(packet_struct packet,u32 data_type);
void BT_con_tx_test(u32 chan_a,u32 chan_b,u32 pa_type,u32 pb_type,u32 pa_data,u32 pb_data);
void BT_mac_rx_start_debug(u32 link_type,u32 chan);
void BT_gen_prbs9(void);
void freq_offset_cfg(u32 tx_offset_bb,u32 rx_offset_bb,u32 tx_offset_hoppe,u32 rx_offset_hoppe,u32 cmpx_on,u32 cmpx_offset,u32 bt_mode_force_en,u32 bt_mode_force,u32 bt_rx_force_on);
void BT_rx_ber(u32 bits,u32 link_type,u32 chan,u32 time_out,u32 be_thresh);
void BT_rx_per(u32 link_type,u32 chan);
void BT_rx_dump(u32 link_type,u32 chan);
void BT_get_inq_info(void);
void BT_inq(u32 hoppe_en);
void BT_inq_scan(u32 hoppe_en);
void BT_page(u32 hoppe_en,u32 mulap,u32 mnap,u32 sulap,u32 snap,u32 clk_offset);
void BT_page_scan(u32 hoppe_en,u32 slave_sel);
int BT_con_loopback_df(u32 device_sel);
void BT_con_loopback_master(u32 loopback);
void BT_con_loopback_slave(u32 link_type,u32 loopback,u32 payload_length,u32 slave_sel);
void BT_con(u32 device_sel);
void BT_m_discon(void);
void BT_s_discon(u32 device_sel);
u32 GetCmd_BT_testmode(void);
void bt_bblc_init(void);
u32 is_ACLC(void);
void get_packetheader(u32 *LT_ADDR,u32 *TYPE,u32 *FLOW,u32 *SEQN,u32 *ARQN);
void set_packetheader(u32 LT_ADDR,u32 TYPE,u32 FLOW,u32 SEQN,u32 ARQN,u32 device_sel);
void set_ARQN(u32 ARQN,u32 device_sel);
void set_SEQN(u32 SEQN,u32 device_sel);
void set_LLID(u32 LLID,u32 device_sel);
void BT_test_mode_fill_tx_payload_(u32 lpayload,u32 data_type,u32 loopback,u32 device_sel,u32 tx_addr_base);
void print_type4(u32 type,u32 link_type);
void type_check(u32 gtype,u32 link_type,u32 etype);
u32 BT_testmode_inq(u32 *nap,u32 *uap,u32 *lap,u32 *clk27_2,u32 *clkn,u32 *cod);
void BT_testmode_inq_scan(u32 fast);
void ac_lost_print(u32 status,u32 init);
void al_get_power(u32 rx_status);
void print_opcode_DUT(u32 opcode);
u32 LM_get_opcode(void);
void LM_get_test_control(u32 *test_scenario,u32 *hopping_mode,u32 *TX_frequency,u32 *RX_frequency,u32 *power_control_mode,u32 *poll_period,u32 *packet_type,u32 *length_of_test_data);
void LM_hopping_mode(u32 freq);
void BT_testmode_fill_test_packet(void);
void LM_accepted(u32 op_code,u32 tx_buffer_addr);
void LM_accepted_ext(u32 op_code,u32 tx_buffer_addr,u32 TID);
void LM_not_accepted(u32 op_code,u32 err_code,u32 tx_buffer_addr);
void LM_fill_payload(u32 len,u32 tx_buffer_addr);
void link_supervision(link_supervision_struct *param,u32 rx_status);
void BT_testmode_lc_ARQN_refresh(void);
void BT_testmode_lc_tx_model(u32 tx_payload,u32 device_sel,u32 lt_addr,u32 packet_type,u32 LLID,u32 packetheader_len,u32 arq);
void BT_testmode_lc_tx_loopback(u32 device_sel);
void print_opcode_tester(u32 opcode);
void LM_hopping_mode_tester(u32 freq);
void LM_test_activate(u32 tx_buffer_addr);
void LM_test_control(u32 test_scenario,u32 hopping_mode,u32 tx_freq,u32 rx_freq,u32 power_ctrl_mode,u32 poll_period,u32 packet_type,u32 len,u32 tx_buffer_addr);
void LM_detatch(u32 error_code,u32 tx_buffer_addr);
void BT_testmode_lm_state(u32 tx_buffer_addr,u32 get_ACLC,u32 get_opcode,u32 get_accepted_opcode,u32 get_ext_opcode);
void BT_testmode_senario_print(u32 senario);
void BT_testmode_lm_model_tester(u32 get_pac);
void BT_testmode_lc_model_master_response(u32 lt_addr);
void BT_testmode_lc_model_master(void);
void BT_testmode_tester(u32 ulap,u32 nap,u32 dut_lap_est);
void cmd_polling_handler(void);
bool cmd_polling_attach(bool duplicate,u32 dup_id,_xtos_handler cmd_handler,void *args);
bool cmd_polling_dettach(bool duplicate,u32 dup_id,_xtos_handler cmd_handler);
void cmd_polling_start(u32 period);
RESET_REASON rtc_get_reset_reason(void);
void wdt_nrst_inc(void);
uint32_t wdt_nrst(void);
void wdt_init(u32 stage,u32 mode,u32 timeout);
void wdt_feed(void);
void t0_start_toggle(u32 alarm);
void t0_toggle_service(void);
void t1_start_toggle(u32 alarm);
void t1_toggle_service(void);
void esp_phy_rftest_config(uint8 conf);
void esp_phy_rftest_init(void);
void esp_phy_tx_contin_en(bool contin_en);
void esp_phy_cbw40m_en(bool en);
void esp_phy_wifi_tx(u32 chan,esp_phy_wifi_rate_t rate,s8 backoff,u32 length_byte,u32 packet_delay,u32 packet_num);
void esp_phy_wifi_rx(u32 chan,esp_phy_wifi_rate_t rate);
void esp_phy_get_rx_result(esp_phy_rx_result *rx_result);
void esp_phy_wifi_tx_tone(u32 start,u32 chan,u32 backoff);
void esp_phy_short_gi_en(bool en);
void esp_phy_ble_tx(u32 txpwr,u32 chan,u32 len,esp_phy_ble_type_t data_type,u32 syncw,esp_phy_ble_rate_t rate,u32 tx_num_in);
void esp_phy_ble_rx(u32 chan,u32 syncw,esp_phy_ble_rate_t rate);
void esp_phy_bt_tx_tone(u32 start,u32 chan,u32 backoff);
void test_digital_pads_slpsel(u32 maskl,u32 maskh,u32 value);
void test_digital_pads_slppu(u32 maskl,u32 maskh,u32 value);
void test_digital_pads_pd(u32 maskl,u32 maskh,u32 value);
void test_digital_pads_drv(u32 maskl,u32 maskh,u32 value);
void test_digital_pads_pu(u32 maskl,u32 maskh,u32 value);
void test_digital_pads_slppd(u32 maskl,u32 maskh,u32 value);
void test_digital_pads_slpoe(u32 maskl,u32 maskh,u32 value);
void test_digital_pads_GPIO(u32 maskl,u32 maskh);
void test_digital_pads_ie(u32 maskl,u32 maskh,u32 value);
void test_rtc_pads_muxsel(u32 mask,u32 value);
void test_slp_pad_ctrl(u32 rtc_excpt_mask);
void test_pad_mode_sel(u8 mode,u32 maskl,u32 maskh);
void ESP_TEST_GPIO(u32 *test_gpio,u32 *Input_result);
void ESP_TEST_GPIO_DRV(u32 *test_gpio,u32 *Input_result,u8 drv);
void CMDSTOP_GPIO(void);
uint16 pvt(u16 dig_dbias,bool pvt_res_en,u16 pvt_delay);
uint16 pvt_test(u16 dig_dbias,bool pvt_res_en,u16 pvt_delay);
void pvt_pwr_ctrl(u8 pvt_pu_en);
uint32 clk8M(u32 req,u32 delay);
uint32 GPIO_cali(u32 num);
u8 slow_clk_write_efuse(u8 set_temp);
u8 slow_clk_read_temp(void);
bool esp_crc8(bool *p,uint len);
u8 check_usermac_from_efuse(uint8 *macaddr);
u8 wr_adc_blk0(void);
u8 wr_adc_blk0_data3_14b(void);
u8 write_coding_to_efuse(void);
bool count_ones(bool in);
void make_efuse(u8 *all_bytes,u8 *efuse_result);
u8 write_usermac_to_efuse(u32 mac_upper,u32 mac_lower);
void Vsar2_pwc_mea_ate(u32 bias,u8 pad);
void DIG_pwc_mea_ate(u32 bias,u8 pad);
void RTC_pwc_mea_ate(u32 bias,u8 pad);
void clr_pwc_mea(void);
void delay_ns(uint n_ns);
void tx_add_pocketnum(void);
sint16 get_temp_cal_wifi(void);
void tx_a_frame(uint queue);
void test_tx_frame(uint queue,uint tx_rate,uint tx_num,uint tx_ext_idle_us,uint tx_cbw40,uint ht_dup);
void set_mac_filter(u32 mac_lower,u32 mac_upper);
sint16 get_rx_freq_local(void);
bool esp_get_rx_result(int32 *rx_result);
void do_rx_poll(uint rx_rate_in);
int rx_data_check(uint RX_buff_start,uint rxrate);
void read_macaddr_from_otp(uint8 *macaddr);
void fill_txaddr(void);
uint32_t fill_txdataframe(int cmd,int data_length,uint32_t addr0lo,uint32_t addr0hi,uint32_t key_word,uint32_t *rate,uint32_t *gi_bit,u32 tx_rate_in);
void fill_txbeacon(u8 *byte0,u8 *byte1,u8 *byte2,u32 *byte128,u8 *byte_m);
void setmacaddr(uint32_t da_lo,uint32_t da_hi);
void settxframe_chipid(uint32_t txdata_addr,uint32_t chip_id_lo,uint32_t chip_id_hi);
void settxframe_rate(uint32_t plcp1_addr,uint32_t htsig_addr,uint32_t len,uint32_t rate);
void get_macaddr(uint32_t *mac_addr);
void set_macrxfilter(bool mac_filter_en);
void ate_rxframe_fb(uint8 chan_num,u32 fb_rate,u32 fb_len);
void ap_ack_test(uint8 chan_num,u32 fb_rate,u32 fb_len);
uint32_t tx_data_frame(uint32_t timelim);
void tx_ack_start(uint16 tx_num,u32 backoff,u32 aifs,u32 delay_ms,u32 *result);
void tx_ack_init(u32 ap_addr0,u32 ap_addr1,uint8 tx_rate,u32 tx_length,bool tx_cbw40);
void tx_ack_test(u32 ap_addr0,u32 ap_addr1,uint8 tx_rate,uint16 tx_num,u32 tx_length,u32 backoff,u32 aifs,u32 delay_ms,bool tx_cbw40);
pm_sw_reject_t pm_set_sleep_mode_local(PM_SLEEP_MODE sleep_mode,_func_void_varargs *pmac_save_params);
void light_sleep(u8 sleep_mode,u32 sleep_time);
void sleep_proc_test(u8 sleep_mode,u8 wakeup,u32 sleep_time);
void wifi_ack_test(u32 ap_addr0,u32 ap_addr1,uint8 tx_rate,u32 tx_length,u32 chan_org,u32 chan_cfg_org,u16 tx_num,u8 test_num,u8 backoff,u8 test_mode,u32 delay_ms);
S16 ram_get_power_db(uint16 pw_offset);
sint32 test_txtone_pwr(int tone_atten,int loop_num,int mode,S16 step);
void get_rx_buffer(uint8 chan,uint8 mac_sel,u32 mac_id);
void test_noise_floor(u8 channel,u32 delay);
void oneblock_buf_init(void);
void clear_mem(U32 mem_start_addr,uint depth);
void clear_txdumpmem(void);
int link_dscr_check(int buf_start,int buf_end,int size,int dscr_start,int dscr_end,int dscr_cnt,int last_len,int checklastlen);
void rxlink_count(void);
void get_encapaddr(void);
void get_txdumpaddr(uint32 *txdump_addr,int32 *plcp_11g_len);
void crypto_disable(void);
void crypto_encap(uint que_no,uint32 location,uint32 keyentry_mask,uint32 amsdu_valid);
void do_decryto_poll(int32 match_mask);
void do_ampdu_rx_poll(int32 match_mask);
void crypto_decap(int valid_entry_mask,int location,int addr_lo,int addr_hi,int bssid_lo,int bssid_hi,int force_keysrch,int amsdu_valid);
void get_micaddr(void);
void crypto_enmic(int32 data_len,int32 start_offset);
u32 rtc_cmd_wr_reg(u32 addr,u32 high_bit,u32 low_bit,u32 data);
u32 rtc_cmd_rd_reg(u32 addr,u32 high_bit,u32 low_bit);
u32 rtc_cmd_wr_i2c(u32 wr_en,u32 i2c_sel,u32 addr,u32 high_bit,u32 low_bit,u32 data);
u32 rtc_cmd_wait_delay(u32 delay);
u32 rtc_cmd_meas_tsens(u32 meas_cyc,u32 delay);
u32 rtc_cmd_meas_saradc(u32 meas_cyc,u32 sar_sel,u32 sar_mux,u32 dreg);
u32 rtc_cmd_write_mem(u32 write_way,u32 result_mux,u32 dreg);
u32 rtc_cmd_reg0_alu(u32 mask,u32 alu,u32 dreg);
u32 rtc_cmd_stage_alu(u32 flag,u32 alu);
u32 rtc_cmd_force_branch(u32 branch);
u32 rtc_cmd_reg0_branch(u32 branch,u32 judge,u32 thres);
u32 rtc_cmd_stage_branch(u32 branch,u32 judge,u32 thres);
u32 rtc_cmd_cpu_wakeup(u32 wake);
u32 rtc_cmd_sleep_cyc_sel(u32 cyc_sel);
u32 rtc_cmd_meas_end(void);
void read_rtc_mem(u32 start,u32 num);
u32 test_rtc_cmd(u32 sar1_en_pad,u32 sar2_en_pad);
void mac_write(uint addr,uint value);
uint mac_read(uint addr);
void mac_buffer_get(void);
uint32 DurAddrGet(uint queue);
uint32 RealQGet(uint queue);
uint32 Plcp0AddrGet(uint queue);
uint32 Plcp1AddrGet(uint queue);
uint32 HTsigAddrGet(uint queue);
uint32 ConfAddrGet(uint queue);
uint32 BackOffCountGet(uint queue);
void fill_tx_frame(uint tx_queue,uint tx_rate,uint length,uint psdu0_len,uint psdu1_len,uint psdu2_len,uint psdu3_len,uint32 key_entry_no,uint32 bssid_no,uint32 link_start_addr,uint32 gi_bit,uint32 ap_mac_5,uint32 ap_mac_4,uint32 ap_mac_3,uint32 ap_mac_2,uint32 ap_mac_1,uint32 ap_mac_0);
void rx_buffer_init(uint rx_buff_start,uint rx_buff_size);
void rx_link_des(void);
void rx_buffer_ena(void);
void rx_ampdu_buffer_fresh(uint rx_ampdu_buff_start,uint rx_ampdu_buff_size);
void rx_ampdu_buffer_init(uint rx_ampdu_buff_start,uint rx_ampdu_buff_size);
void rx_ampdu_entry_fresh(uint rx_ampdu_entry_start,uint rx_ampdu_entry_size);
void rx_ampdu_entry_init(uint rx_ampdu_entry_start,uint rx_ampdu_entry_size);
void rx_ampdu_entrysd_fresh(uint rx_ampdu_entrysd_start,uint rx_ampdu_entrysd_size);
void rx_ampdu_entrysd_init(uint rx_ampdu_entrysd_start,uint rx_ampdu_entrysd_size);
void mac_init(void);
void adctrig(int32 smp_num_aft_trig,int32 trigmode,int32 trigcase,int32 sample_88m,int32 dump_trig,int32 rx_gain_mode,int32 rx_gain,int32 rx_gain0,int32 rx_gain0_wait_us);
void module_test_cal_print(bool print_en);
S16 linear_to_db_64bits(uint64 power_sig,uint8 frac_bits);
S16 get_iq_est_snr(uint64 *get_power,uint8 shift_bits,bool print_en);
void freq_offset_get_pwr(U8 loop_bits,S16 tone_step,U8 corr_cycle_bits,uint64 *pwr_sum,S16 *rxiq_remain,bool rxiq_sign,bool print_en);
void get_corr_power(int *power,U8 shift_bits,bool print_en);
void test_set_rf_freq_offset(uint8 crystal_select,uint16 freq,sint16 freq_offset);
void ate_rxdc_remain_check(void);
void txdc_delta_max(sint16 *delta,sint16 *delta_max);
void txdc_stable_sub(bool bt_mode,sint16 *txdc);
void txdc_stable_test(bool print_en);
void rxspur_test(bool print_en);
void ate_rfpll_cap_sign(bool print_en_in);
void rftest_init(void);
void esp_en_reboot(void);
void cbw40m_en(bool en);
void wifitxout_func(uint32 *para_array,uint32 para_num);
void esp_tx_func_org(uint32 *para_array,uint32 para_num);
void esp_tx_func_notxon(u32 chan,u32 rate,u32 backoff,u32 pocket_len,u32 delay_time);
void esp_tx_func_duty(u32 chan,u32 rate,u32 backoff,u8 duty);
void tx_contin_en(u8 contin_en);
void wifitxout(uint32 *para_array,uint32 para_num);
void esp_tx_func(int *para_array,int para_num);
void esp_tx_func_5p(u32 chan,u32 rate,u32 backoff,u32 pocket_len,u32 delay_time);
void wifiscwout(uint32 en,uint32 chan,uint32 backoff);
void esp_rx_func(uint32 chan,uint32 rate);
void fcc_bt_tx_func(u32 txpwr,u32 hoppe,u32 chan,u32 rate,u32 DH_type,u32 data_type);
void fcc_le_tx_func(u32 txpwr,u32 chan,u32 len,u32 data_type);
void fcc_le_tx_syncw_func(u32 txpwr,u32 chan,u32 len,u32 data_type,u32 syncw,u32 tx_num_in);
void rw_rx_per_func(u32 edr,u32 chan,u32 ulap,u32 ltaddr);
void rw_le_rx_per_func(u32 chan,u32 syncw);
void bt_tx_tone(u32 start,u32 bt_chan,u32 backoff);
void esp_ble_tx_func(u32 txpwr,u32 chan,u32 len,u32 data_type,u32 syncw,u32 rate,u32 tx_num_in);
void esp_ble_rx_func(u32 chan,u32 syncw,u32 rate);
void RF_init_sel(bool init_bin_sel,u32 flash_addr);
void esp_chg_freqoff(s8 ppm,u32 init_params_addr);
void esp_set_target_txp(uint32 *para_array);
void esp_get_target_txp(uint32 *para_array);
void rfpll_1p2_test(void);
void esp_origin_mac(void);
void esp_chk_mac(void);
void esp_set_mac(u32 mac_upper,u32 mac_lower);
void ESP_TEST_GPIO_func(uint32 *para_array);
void pvt_test_func(void);
bool remove_head_newline(char *strln,char **pnext);
STATUS esp_phy_getstopcmd(void);
STATUS GetStopCmd(void);
uint32_t spi_qe0(uint32_t spi_no);
uint32_t spi_qe1(uint32_t spi_no);
uint32_t spi_qe2(uint32_t spi_no);
uint32 SPI_get_id(void);
void SPI_SET_QE(void);
void gpio_set_flash_drv(u8 clk_drv,u8 cmd_drv,u8 data0_drv,u8 data1_drv,u8 data2_drv,u8 data3_drv);
void spi_clk_local(uint8 freqdiv);
void spi_test_init(uint32 flash_mode,uint32 clk_div,uint32 drv);
void SPI_read_local(uint32 spi_addr,sint32 length);
void SPI_page_program_local(uint32 spi_addr,sint32 length);
void flash_test_init(uint32 clk_div,uint32 drv,u32 flash_mode);
void gpio_test_init(u32 gpio_en);
void gpio_test_run(void);
void gpio_set_psram_drv(uint32_t spiconfig0,uint8_t spiconfig1,uint8_t drv);
void SPIPsramExitQIO(uint8_t div);
uint32_t SPIPsramReadID(uint8_t div);
void SPIPsramEnterQIO(uint8_t div);
void SPIPsramCLK(uint8_t div);
int SPIPsramEnable(uint8_t PSRAM_CS_IO,uint8_t PSRAM_CLK_IO,uint8_t div);
uint32_t SPIPsramInitCheck(uint8_t PSRAM_CS_IO,uint8_t PSRAM_CLK_IO,uint8_t div);
uint32_t PsramQIOConf(uint8_t PSRAM_CS_IO,uint8_t PSRAM_CLK_IO,uint32_t clk_div,uint32_t drv);
void PsramQIORead(uint32_t spi_addr,uint32_t length);
void flash_test_run(u32 length);
void rfpll_cal_time(u8 chan_freq,u8 mode);
void flash_init_param_print(bool flash_valid);
void FillTxPacket(uint32 PacketLen,uint32 PDU0Len,uint32 PDU1Len,uint32 Rate,uint32 key_entry_no,uint32 bssid_no,uint32 ap_mac_5,uint32 ap_mac_4,uint32 ap_mac_3,uint32 ap_mac_2,uint32 ap_mac_1,uint32 ap_mac_0);
void dig_gain_rse(u8 rate,u8 clip_val,u8 clip_cmp,s8 backoff,bool cbw40);
void dig_gain_rse_stop(void);
void txpwr_track_disable(void);
void WifiTxStart(uint32 tx_att,uint32 TxPacketNum,uint frame_delay,uint tx_cbw40,uint ht_dup,int dis_agc);
uint8 my_temprature_sens_read(u16 delay,u8 clk_div);
void write_efuse_dig_reserve(void);
void read_efuse_dig_reserve(void);
int32 sampledeal(uint32 sample);
void accumiq(uint32 start_addr,uint32 burst_len);
void dc_iq_est_test(bool dc_est_en,uint16 smp_num,sint32 *dc_est,sint32 *power);
void ram_get_corr_power(int *power,U8 shift_bits);
void get_rx_tone_pwr(float tone_freq_set);
void noise_init_check(int chan_en,int upd_en);
void test_rf_cal_level(u8 rf_cal_level,uint32 flash_addr);
void improve_dig_rtc_vol(void);
void burn_in_test(void);
void rx_per_init(void);
void clk_pull_out(u32 clk_sel);
void clk50m_pull_out(u32 clk_sel);
void print_efuse_mac(void);
void rtc_brown_out_set(void);
void pad_gpio_set(u32 pad_addr,bool high_en);
void rtc_brown_out_test(u8 thres,u8 rst);
void test_tx_vdd33(uint32 delay_us);
void coex_bt_wifi_test(void);
void rftest_optimize(void);
bool init_param_read(void);
void flash_init_param_print(bool flash_valid);
void tx_lo_for_5g(void);
void phy_temp_track(uint8 rate);
void phy_ofdm_bandage_opt(uint8 rate,sint8 backoff,bool tx_cbw40m_en);
void write_rtc_mem_test(void);
u32 change_data_rate(u32 rate);
void get_length_delay(u32 *len_delay,u8 rate);
void remove_11b_4p8G_spur(u8 remove_en,u8 num,u8 thres);
void txpwr_track_correct(s8 pwcorrect_v,bool print_en);
void rx_pbus_set(uint16 rxon_rfrx,uint16 rxon_bb1,uint16 rxoff_rfrx,uint16 rxoff_bb1);
void tx_pbus_set(uint16 txon_rfrx,uint16 txon_bb1,uint16 txoff_rfrx,uint16 txoff_bb1);
void pa_pbus_set(uint16 paon_txrf1,uint16 paoff_txrf1);
void force_txon_en(bool en);
void gpio_conf_for_certific(void);
void read_flash_init_para(u32 addr,u8 *data);
void write_flash_init_para(u32 addr,u8 *data);
void fcc_bt_force_tx_tone(u32 start,u32 bt_chan,u32 power);
U16 get_sar2_vol(uint8 atten);
void coex_test(u8 mode);
void set_rtc_dig_dbias(uint8 dig_dbias);
void rtc_clk_bbpll_set_new(uint8 ref);
void rtc_clk_cpu_freq_set_new(uint8 ref);
void bbpll_80m_cal(uint8 ref);
void bt_rx_spur_cal(uint32 chan,bool print_en);
void bt_2480_opt_enter(uint32 chan);
void bt_2480_opt_exit(void);
void bt_3200_opt(bool en);
void test_2480_power(uint32 shift);
u32 ble_chan_trans(u32 chan);
uint16 bt_chan_change(bool bt_en,uint8 chan_num,bool tx_en);
void bt_set_freq_cal(bool bt_en,uint8 chan_num,bool tx_en);
void set_freq_test(uint32 *para_array,uint32 para_num);
void pbus_test_init(bool bt_sel);
void pbus_wr_que(bool bt_sel,bool rfrx_en,bool filt_en,bool pa_en,uint16 delay_us,uint16 *txdc);
void pbus_debug_test(bool bt_sel,uint8 rfrx_num,uint8 txbb_num,uint8 pa_num,uint16 delay1,uint16 delay2,uint16 delay3,uint16 rx_bb,uint16 rx_pa,uint16 rfrx,uint32 *txdc_in);
void force_power_level(bool force_en,uint8 level);
void pvt_ate(void);
void get_path_loss_offset(s32 *pwr_list);
void mac_filter_enable(bool enable);
void force_rx_gain(bool force_en,uint8 gain,bool bt_mode);
void rx_gain_comp_cal(bool print_en,bool bt_mode);
void WifiRxStart(WIFI_RATE rx_rate);
void wifi_init(void);
void idf_enable(uint8 conf);
void BT_tx_8m_enable(uint hoppe_n500k);
void BT_tx_if_init(uint guard,uint tx_on_ahead,uint tx_on_behind,uint init_tx_DC);
void BT_init_rx_filters(uint16 freq_offset_500k);
void bt_dgmixer_fstep_250k(void);
void bt_rfoffset_en(void);
void bt_bb_init_cmplx(void);
void bt_bb_init_cmplx_reg(void);
void rw_coex_on(void);
void force_bt_mode(void);
void force_wifi_mode(u32 bt_mux_off);
void unforce_wifi_mode(void);
void coex_bt_high_prio(void);
void bt_rxfilt(void);
void bt_txfilt(void);
void bt_cmplx_hq_wr(u32 value);
void bt_cmplx_lq_wr(u32 value);
u32 bt_cmplx_hq_re(void);
u32 bt_cmplx_lq_re(void);
void pm_wakeup_opt(u32 wakeup_opt,u32 reject_opt);
u32 get_chip_version(void);
void pm_set_sleep_cycles(u32 sleep_cycles);
void pm_rtc_clock_cali(RTC_clock *RTC);
void pm_prepare_to_sleep(void);
undefined4 pm_sdio_nidle(void);
u8 pm_goto_sleep(u32 sleep_mode);
void pm_sleep_set_mac(void);
void pm_mac_deinit(void);
void pm_set_wakeup_mac(void);
bool pm_check_mac_idle(void);
void pm_set_sleep_btco(void);
void pm_set_wakeup_btco(void);
void pm_unmask_bt(u8 tw);
void pm_mac_init(void);
SW_REJECT_TYPE pm_set_sleep_mode_full(u32 sleep_mode,_func_void_varargs *pmac_save_params);
uint8 temprature_sens_read(void);
void dac_out(uint8 dac_en,u8 tone_en,uint16 dc_value,uint16 tone_scale,uint16 tone_step);
void touch_init(uint16 pad_en);
void touch_read(uint16 *pad_out,uint16 sample_num);
void vdd33_init(void);
uint16 get_vdd33(void);
uint16 adc1_read_test(uint8 pad,uint8 atten,uint16 pad_en_wait);
uint16 adc1_amp_read_full(uint8 pad,uint8 atten,uint16 stg1,uint16 stg2,uint16 stg3);
uint16 hall_sens_read_full(void);
uint16 hall_sens_amp_read_full(uint16 stg1,uint16 stg2,uint16 stg3);
uint16 adc2_read_test(uint8 pad,uint8 atten);
void adc1_pad_init(u8 pad_en);
uint16 adc1_read(uint8 pad,uint8 atten);
uint16 adc1_amp_read(void);
uint16 hall_sens_read(void);
uint16 hall_sens_amp_read(void);
void adc2_pad_init(u8 pad_en);
uint16 adc2_read(uint8 pad,uint8 atten);
void adc_pad_int(u8 adc_num,u32 adc1_channel,u32 adc2_channel);
void adc_pad_init(u32 pad);
void dac_pad_init(u8 dac_pad,uint8 dac_val);
void rtc_cmd_wakeup_conf(void);
void rtc_pads_muxsel(u32 mask,u32 value);
void rtc_pads_funsel(u32 mask,u32 value);
void rtc_pads_slpsel(u32 mask,u32 value);
void rtc_pads_slpoe(u32 mask,u32 value);
void rtc_pads_slpie(u32 mask,u32 value);
void rtc_pads_funie(u32 mask,u32 value);
void rtc_pads_pu(u32 mask,u32 value);
void rtc_pads_pd(u32 mask,u32 value);
void rtc_pads_hold(u32 mask,u32 value);
void rtc_apbbridge_sel(u32 sel);
void rtc_powerup_rf(void);
void rtc_powerdown_rf(void);
u32 rtc_get_st(void);
u32 rtc_is_st_idle(void);
void rtc_soc_clk_ck12m(void);
void rtc_init_full(u32 fast_clk_sel,u32 ck12m_div,u32 ck12m_wait,u32 xtal_wait,u32 pll_wait,u32 ana_clk_sel,u32 slw_ck_dcap,u32 ck12m_dfreq,u32 pwrctl_init,u32 ck12m_fpu,u32 xtl_fpu,u32 bias_core_fpu,u32 bias_i2c_fpu,u32 bias_fnslp,u32 bbpll_fpu,u32 bbplli2c_fpu,u32 rtcreg_fpu,u32 rtcdboost_fpu,u32 dec_hbw,u32 inc_hbp,u32 rtc_dbias,u32 dig_dbias,u32 x32k_dac,u32 lslp_meminf_pd);
void rtc_pad_gpio_wakeup(u32 mask,u32 value);
void rtc_pad_ext_wakeup(u32 mask,u32 wakeup_sel,u32 wakeup_lv);
void rtc_cmd_ext_wakeup(u32 mask,u32 wakeup_sel);
void rtc_wifi_force_pd(void);
void rtc_sdreg_off(void);
void rtc_slp_prep_lite_12M(u32 deep_slp,u32 cpu_lp_mode);
void cfg_sdio_volt(uint8 drefh,uint8 drefm,uint8 drefl);
void rtc_wifi_force_pd_off(void);
void rtc_digital_lp_mode_off_stg1(void);
void rtc_digital_lp_mode_off_stg2(void);
void rtc_sar_sleep_timer_start(u32 cycles);
void rtc_mac_tx_init(RTC_TX_INFO *rtc_tx_infor);
u32 rtc_mac_tx(void);
void ram_disable_agc(void);
void ram_enable_agc(void);
void ram_write_gain_mem(u32 data_h,u32 data_l,u8 addr);
void ram_set_txclk_en(bool txclk_en);
void ram_set_rxclk_en(bool rxclk_en);
void disable_wifi_agc(void);
void enable_wifi_agc(void);
void wr_bt_tx_atten(sint8 *bt_dig_atten);
void wr_bt_tx_gain_mem(uint8 pa_in,uint8 bb_in);
void set_tx_gain_table(U16 pa_gain,U16 bbgain);
u8 set_most_pwr_reg(void);
void set_xpd_sar(bool on);
void bb_wdt_rst_enable(bool enable);
void bb_wdt_int_enable(bool enable);
void bb_wdt_timeout_clear(void);
uint32 bb_wdt_get_status(void);
void bt_tx_gain_cal(void);
void bt_tx_gain_cal_set(void);
void bt_tx_gain_cal_set(void);
void phy_wifi_enable_set(uint8 enable);
void ram_set_noise_floor(sint16 noise);
void phy_close_rf(void);
U8 get_target_power_offset(U8 rate,U8 *ratepw_offset);
uint8 ram_txbbgain_to_index(uint16 txbb_gain);
sint8 ram_set_chan_cal_interp(sint8 *chan_data,uint8 chan);
void write_txrate_power_offset(bool chg_offset);
void get_phy_target_power(U8 *target_power,S16 vdd33_offset);
void force_txrx_off(bool off);
void phy_pwdet_onetime_en(void);
void ram_read_sar_dout(U16 *sar_data);
void get_rate_fcc_index(uint8 chan,uint8 *fcc_index_all);
uint8 get_rate_target_power(uint8 rate);
void get_chan_pwr_index(void);
void write_wifi_dig_gain(S8 *gain_table);
void correct_rf_ana_gain_new(sint16 *correct_qdb,U16 *pa_gain,U16 *bb_gain,bool bt_mode);
void tx_gain_table_set(void);
uint8 set_chan_dig_gain(uint8 chan);
void tx_pwctrl_cal(U8 *target_power,U8 *ratepw_offset,S16 *pwdet_error_accum,bool correct_en,bool print_en);
void tx_pwctrl_background(bool correct_en,bool print_en);
U8 ram_get_rf_gain_qdb(U8 rf_gain_ind);
uint8 wifi_11g_rate_chg(uint8 rate_11g);
void ram_set_txcap_reg(U8 *para_txcap,uint8 chan);
uint32 get_i2c_read_mask(uint8 block);
sint8 pll_correct_dcap(uint8 freq,u8 *pll_cap_ext,bool wifi_chan_en);
void bb_rst_en_set(bool en);
void phy_dis_hw_set_freq(void);
void phy_force_wifi_chan(void);
void phy_en_hw_set_freq(void);
uint8 ram_chip_i2c_readReg(uint8 block,uint8 host_id,uint8 reg_add);
void ram_chip_i2c_writeReg(uint8 block,uint8 host_id,uint8 reg_add,uint8 pData);
void phy_unforce_wifi_chan(void);
void wifi_track_pll_cap(void);
uint16 phy_get_fetx_delay(void);
sint16 get_temp_cal(u8 tsen_init,u8 tsen_meas);
sint8 btpwr_pll_track(uint8 freq,sint8 pll_cap_delta);
void phy_bt_ifs_set(void);
uint8 tsens_code_read(void);
sint16 btpwr_tsens_track(void);
void bt_track_tx_power(uint8 freq,sint8 cap_delta);
void bt_track_pll_cap(void);
uint16 chan_to_freq(int8 channel);
void get_i2c_write_data(uint8 index,uint8 block,uint8 host_id,uint8 reg_add,uint8 pData,uint8 *block_m,uint8 *host_id_m,uint8 *reg_add_m,uint8 *pData_m);
void i2c_write_master(uint8 *block,uint8 *host_id,uint8 *reg_add,uint8 *pData,uint8 num);
void ram_pbus_debugmode(void);
void ram_pbus_force_test(u8 pbus_no,u8 bus_en,u16 config_data);
void force_txrxoff(bool off_en);
void ram_bb_bss_cbw40_dig(int cbw40);
void ram_set_pbus_mem(void);
void ram_start_tx_tone(int tone1_en,int freq_1_mhz,int tone1_atten,int tone2_en,int freq_2_mhz,int tone2_atten);
void ram_bb_tx_ht20_cen(int tx_ht20_cen_en);
sint16 ram_phy_get_noisefloor(void);
sint16 ram_check_noise_floor(void);
void ram_cbw2040_cfg(bool cbw20_sel);
void ram_bb_bss_bw_40_en(int bb_ht_2040);
void bt_txdc_cal(void);
void bt_txiq_cal(void);
int spur_cal(uint16 freq,int8 BW_h,uint16 spur_freq_cfg,uint8 spur_freq_cfg_div);
int ram_spur_coef_cfg(int8 channel,uint16 freq,int8 sub_chan_cfg);
void set_chan_rxcomp(sint8 offset);
void phy_ant_init(void);
void tx_delay_cfg(bool ht40_en);
void bb_bss_cbw40(int8 sub_chan_cfg);
void tx_paon_set(void);
void agc_reg_init(void);
void bb_reg_init(void);
void mac_enable_bb(void);
void bb_wdg_cfg(void);
void rx_11b_opt(bool rx_11b_opt_en);
void opt_11b_resart(void);
void phy_reg_init(void);
void set_chan_reg(void);
void i2c_master_reset(void);
uint8 ram_gen_rx_gain_table(uint32 *rx_gain_table,uint8 gain_lpf_max,uint8 *rx_gain_swp,sint8 *rx_gain_swp_step,uint8 rx_gain_swp_num,uint8 max_bb_gain,bool print_en);
void set_rx_gain_cal_iq(bool bt_mode,S16 tone_freq,uint16 *rxbb_iq,bool rxiq_print_en);
void rx_chan_dc_sort(uint32 *chan_dc,uint8 *chan_flag);
void set_rx_gain_cal_dc(bool bt_mode,uint8 mode_start,uint8 mode_end,uint8 *rx_gain_swp,uint32 *rxrf_dc,uint32 *rxbb_dc,uint32 *chan_dc,uint8 rf_dc_num,uint8 bb_dc_num,uint8 chan_dc_num);
void wr_rx_gain_mem(bool bt_mode,bool rfbb_dc_upd_only,uint8 *rx_gain_swp,uint32 *rxrf_dc,uint32 *rxbb_dc,uint32 *chan_dc,uint8 table_num,uint32 *rx_gain_table);
void set_rx_gain_testchip_70(bool bt_mode,uint32 *param_flag,uint8 *rx_gain_swp,uint8 table_num,uint32 *rx_gain_table,uint8 rf_dc_num);
void bt_correct_bbgain(uint16 *bbgain,sint8 *dig_atten_in);
uint16 bt_index_to_bb(uint16 index);
uint16 bt_bb_to_index(uint16 bbgain);
void bt_txdc_cal(void);
uint8 get_bbgain_db(uint16 bbgain);
void bt_txiq_cal(void);
void force_bttx_gain(bool force_en,uint8 pa_in,uint8 bb_in,uint8 dig_in);
void phy_bttx_low_power(bool force_en,uint8 level);
void phy_wifitx_low_power(bool force_en,uint8 level,sint8 gain_atten);
void set_tx_gain_table_bt(void);
void set_tx_dig_gain(int force_en,int force_value);
int spur_cal(uint16 freq,int8 BW_h,uint16 spur_freq_cfg,uint8 spur_freq_cfg_div);
int set_chanfreq(uint16 chanFreq,int8 sub_chan_cfg);
int set_chanfreq_nomac(uint16 chanFreq,int8 sub_chan_cfg);
void chip_sleep_prot_en(void);
void chip_sleep_prot_dis(void);
void chip_v7_rxmax_ext_dig(void);
void chip_v7_rxmax_ext(uint8 ext_level);
void set_cca(bool en,s8 cca_thr);
void set_rx_sense(sint8 sense_thr);
sint16 read_hw_noisefloor(void);
void phy_get_txpwr_param(S8 *txpwr_backoff,S8 *txpwr_diff_flash,U16 *txpwr_ana_gain,sint8 *txpwr_dig_atten,S8 *txpwr_correct_pwr,S16 *txpwr_meas_error);
void noise_check_loop(uint8 check_level,bool set_noise_en);
void noise_init(void);
void target_power_backoff(S8 backoff_qdb);
void phy_set_rfrx_dcap(bool enable);
void chip_v7_set_chan_misc(int8 channel);
void set_rx_gain_table(u16 freq,uint8 wifi_gain_offset);
void rx_blocking_set(uint8 level);
void rx_gain_level(bool level);
void txiq_cal_init(void);
void phy_rx11blr_cfg(uint8 en);
void analog_gain_init(void);
void phy_param_set(uint8 mode_set);
void bb_init(void);
void wifi_rifs_mode_en(bool enable);
void phy_chan_filt_set(bool filt_en,bool merge_en);
uint8 phy_get_tx_rate(void);
int register_chipv7_phy_init_param(s8 *init_param);
void uart_wait_idle(U8 uart_no);
void phy_get_romfunc_addr(void);
uint32 phy_byte_to_word(uint8 *data);
void rf_cal_data_recovery(uint8 *rf_cal_data);
void rf_cal_data_backup(uint8 *rf_cal_data);
uint32 phy_get_rf_cal_version(void);
bool phy_rfcal_data_check(bool check_flag,uint8 *rf_cal_data,uint8 *init_param);
void i2cmst_reg_init(void);
void fe_reg_init(void);
void reg_init_begin(bool wakeup);
void phy_wakeup_init(void);
char * get_phy_version_str(void);
void phy_version_print(void);
void get_iq_value(sint8 *iq_out,uint16 iq_in);
uint16 phy_rfcal_data_check_value(uint8 *rf_cal_data,sleep_param_t *cal_param,bool check_en);
void tx_cont_en(void);
void tx_cont_dis(void);
void tx_cont_cfg(uint8 set_en);
S16 phy_get_tx_pwr(void);
void phy_init_pwr_print(void);
sint16 phy_get_rx_freq(u8 rate,u32 bb_info);
void reset_rf_dig(void);
int register_chipv7_phy(u8 *init_param,uint8 *rf_cal_data,uint8 rf_cal_level);
void phy_set_most_tpw(s8 most_target_power);
s8 phy_get_most_tpw(void);
void phy_rx_sense_set(uint8 sense_thr);
void ant_dft_cfg(bool default_ant);
void ant_wifitx_cfg(uint8 ant0,uint8 ant1);
void ant_wifirx_cfg(bool auto_en,uint8 ant0,uint8 ant1);
void ant_bttx_cfg(uint8 ant0);
void ant_btrx_cfg(bool auto_en,uint8 ant0,uint8 ant1);
void ant_tx_cfg(uint8 ant0);
void ant_rx_cfg(bool auto_en,uint8 ant0,uint8 ant1);
void esp_tx_state_out(uint8 wifi_gpio_num,uint8 bt_gpio_num);
void phy_chan_dump_cfg(bool shift_force_en,uint8 shift_force,bool lltf_dump_en,bool htltf_dump_en,bool stbcltf2_dump_en);
void chan14_mic_cfg(u8 set_en);
void chan14_mic_enable(bool en,s8 most_pwr);
void phy_get_adc_rand(bool start_flag);
void phy_enable_low_rate(void);
void phy_disable_low_rate(void);
void phy_close_pa(bool force_close);
void btpwr_backoff(s8 backoff);
uint8 phy_dig_reg_backup(bool backup_en,uint32 *mem_addr);
void get_iq_est_snr_1(uint64 *get_power,uint8 shift_bits,bool print_en);
uint64 freq_offset_get_pwr_1(U8 loop_bits,S16 tone_step,U8 corr_cycle_bits,uint64 *tot_pwr_o,bool rxiq_sign,bool print_en);
void get_spur4m_pwr(uint32 *out_pwr);
uint8 rx_spur_cal(bool flag,bool print_en,uint32 *rx_power_min);
void bt_opt_write_mem(uint8 chan_freq,uint8 xtal_dac,uint8 oc_bw);
void bt_rx_spur_opt(bool enable);
void phy_init_param_set(uint8 param);
void pbus_print(void);
void phy_reg_check(void);
void phy_i2c_check(void);
void phy_cal_print(void);
void RFChannelSel(int8 channel,int8 sub_chan_cfg);
STATUS phy_change_channel(uint16 chanFreq,bool bHomeChan,bool pBReset,int8 sub_chan_cfg);
STATUS phy_change_channel_nomac(uint16 chanFreq,bool bHomeChan,bool pBReset,int8 sub_chan_cfg);
S16 get_rate_pwctrl_offset(uint8 rate);
void phy_set_bbfreq_init(bool en_11b);
void ram_tx_pwctrl_bg_init(void);
void pwdet_sar2_init(void);
void ram_en_pwdet(void);
uint16 ram_index_to_txbbgain(uint8 index);
void ram_txdc_cal_v70(sint16 *dc_comp);
void txcal_debuge_mode(bool bt_mode);
void ram_txcal_work_mode(void);
S16 ram_get_fm_sar_dout(S16 *vsig_dc,S16 *vref_dc);
void ram_txiq_get_mis_pwr(bool gain_en,uint8 bb_att,S16 tone_freq,S16 *pwr1,S16 *pwr2,bool pkdet_en);
void ram_txiq_cover(U8 tone_atten,S16 tone_freq,int8 *mis_data,bool pkdet_en);
void rfcal_txiq(uint16 bbgain,uint16 *txdc,uint16 *para_txiq,uint8 tone_freq,int8 tone_atten,uint8 func_sel);
void ram_iq_est_enable(bool iq_est_en,uint16 est_length);
void ram_iq_est_disable(void);
void ram_dc_iq_est(bool dc_est_en,uint16 smp_num,sint32 *dc_est);
void ram_pbus_rx_dco_cal(uint16 smp_num,sint16 *dc_comp,uint16 ext_delay_us,bool rxdc_print_en,bool print_debug);
void rxdc_est_min(uint16 smp_num,bool cal_en,int32 *dc_est);
void pbus_rx_dco_cal_1step(bool bt_mode,uint8 mode,uint16 smp_num,sint16 *dc_comp,int32 *dc_est,uint8 *dc_flag);
void rc_cal(void);
void ram_rfcal_txcap(uint8 tone_freq,uint8 tone_atten,bool print_debug,U8 *indata);
void tx_cap_init(void);
S16 ram_meas_tone_pwr_db(S8 tone_atten);
void ram_rfcal_pwrctrl(U8 tone_freq,U8 *rate_power,U8 pwctrl_num,U8 pwctrl_tone_offset,S8 *rate_atten,uint16 pwdetect_offset,S8 atten_init,bool print_debug);
S16 ram_tx_pwr_backoff(U8 *target_pwr,bool *linear_flag);
void cal_rf_ana_gain(void);
S16 tx_pwctrl_init_cal(U8 chan_num,u8 backoff_en);
void tx_pwctrl_init(u8 backoff_en);
void bt_tx_pwctrl_init(void);
S8 set_bt_chan_cal_interp(uint8 chan);
S8 phy_set_bt_dig_gain(uint8 chan);
U16 ram_phy_get_vdd33(void);
void txpwr_offset(S16 m_offset);
sint16 phy_get_bb_freqoffset(void);
void phy_pwdet_always_en(bool always_en);
void dpd_scale_set(void);
void i2c_bbpll_init(void);
void i2c_bbpll_init(void);
void bb_bss_cbw40_ana(int cbw40);
void i2c_bt_filter_set(void);
void phy_i2c_init(void);
void ram_pbus_force_mode(bool pbus_force_en);
void ram_pbus_xpd_tx_on(U16 pa_gain,U16 bbgain);
void i2c_xtal_init(void);
void i2c_rfpll_init(void);
void ram_rfpll_reset(void);
void ram_restart_cal(void);
void ram_wait_rfpll_cal_end(void);
void ram_rfpll_set_freq(uint32 freq,uint8 crystal_select,sint16 freq_offset,uint8 *x_reg);
void get_lna_vga_dcap_val(uint16 pll_cap,sint16 *rfrx_cap);
void chip_v7_rxmax_ext_ana(uint8 ext_level);
void phy_freq_correct_opt(void);
void chip_v7_adc_wr_dly(uint8 dly1,uint8 dly2,uint8 dly3,uint8 dly4,uint8 dly5,uint8 dly6);
void i2c_bbtop_init(void);
void i2c_rftx_init(void);
void i2c_bias_init(void);
void rfpll_1p2_opt(void);
void get_rf_freq_cap(uint16 freq,sint16 freq_offset,uint8 *x_reg,uint8 *cap_array);
void correct_rfpll_offset(sint16 freq_offset,uint8 crystal_select);
void wr_rf_freq_mem(uint8 chan_freq,uint32 *mem_data);
void write_freq_mem_all(uint8 *rf_cal_data);
uint8 get_rfrx_dcap_bt(uint8 reg_addr);
void get_rf_freq_init(void);
void get_rf_freq_init(void);
void bt_i2c_read_set(u8 i2c_mst,u8 i2c_block,u8 i2c_addr,u8 i2c_data_mask,bool i2c_rd_en,bool i2c_comp_en);
void bt_i2c_read_mem(void);
void bt_i2c_write_set(U8 *i2c_mst,U8 *i2c_block,U8 *i2c_addr,U8 *i2c_addr_tx,U8 *i2c_data_tx,U8 *i2c_addr_rx,U8 *i2c_data_rx,U8 i2c_num,U8 *i2c_rd_en);
void bt_i2c_set_wifi_data(U8 *data,U8 i2c_num);
void phy_wifi_pll_track(bool enable);
void phy_bt_pll_track(bool enable);
void tsens_read_init(void);
void phy_bt_power_track(bool enable);
void bt_get_i2c_data(U8 *i2c_mst,U8 *i2c_block,U8 *i2c_addr,U8 *i2c_addr_tx,U8 *i2c_data_tx,U8 *i2c_addr_rx,U8 *i2c_data_rx,uint8 i2c_num,uint8 *i2c_rd_en);
void write_wifi_chan_data(uint8 chan_freq);
void set_chan_freq_hw_init(S8 tx_freq_offset,S8 rx_freq_offset);
void rf_init(void);
void check_rfpll_write_i2c(void);
void phy_hw_set_freq_enable(bool enable);
void set_chan_freq_sw_start(U8 chan_freq,sint16 freq_offset,uint8 crystal_select);
uint16 set_channel_rfpll_freq(int8 channel,uint8 crystal_select,sint16 freq_offset);
void chip_v7_set_chan_nomac(int8 channel,int8 sub_chan_cfg);
void chip_v7_set_chan(int8 channel,int8 sub_chan_cfg);
void chip_v7_set_chan_offset(sint16 freq_offset);
void chip_v7_set_chan_ana(int8 channel);
void freq_write_wifi_chan(uint8 chan_freq);
void phy_unforce_chan(void);
void phy_force_chan(uint8 chan_freq);
void phy_set_wifi_mode_only(bool wifi_only);
undefined rtc_set_dig_gpio_out();
undefined gpio_matrix_in();
undefined __divdi3();
undefined gpio_pad_select_gpio();
undefined gpio_matrix_out();
undefined __divsf3();
undefined rtc_clk_wait_for_slow_cycle();
undefined rtc_slp_prep_lite();
undefined inquiry_scan();
undefined le_advscan();
undefined adv_scan_test();
void * memcpy(void * __dest, void * __src, size_t __n);
undefined cmd_parse();
undefined uart_tx_switch();
undefined rtc_time_get();
undefined spi_flash_attach();
undefined inquiry();
undefined wifi_init_bt();
undefined intr_matrix_set();
undefined __assert_func();
undefined SPIEraseSector();
undefined spi_clk_config();
undefined UartGetCmdLn();
undefined le_adv_test();
undefined rtc_uart_div_modify();
undefined rtc_init_clk();
undefined rtc_set_cpu_freq();
undefined spi_usr_cmd();
undefined gpio_pad_set_drv();
undefined pm_wakeup_init();
undefined burn_efuse();
undefined __floatunsidf();
undefined rtc_dbias_cfg();
undefined rtc_get_xtal();
undefined SPIWrite();
undefined READ_SPI_INIT_PARAM();
undefined SPIRead();
undefined rtc_slp_prep();
undefined le_advscan_test();
undefined rtc_get_gpio_dig_conf();
undefined rtc_slowck_cali();
undefined ets_delay_us();
undefined pm_close_rf();
undefined _xtos_ints_on();
undefined ets_efuse_get_spiconfig();
undefined phy_printf();
undefined rtc_clk_apll_enable();
undefined esp_dport_access_reg_read();
undefined pm_open_rf();
undefined PsramEnableVspiClk();
undefined page_scan();
void encrypt(char * __block, int __edflag);
undefined __divdf3();
undefined ets_unpack_flash_code();
undefined _xtos_set_interrupt_handler_arg();
undefined set_rtc_memory_crc();
void * memset(void * __s, int __c, size_t __n);
undefined rtc_xtal_32k_test();
undefined ets_update_cpu_frequency();
undefined __truncdfsf2();
undefined uart_tx_one_char();
undefined phy_exit_critical();
undefined cmdstop_callback();
undefined FlashBoardTest();
undefined le_adv();
undefined SPIReadModeCnfig();
undefined rtc_8m_ena();
int strcmp(char * __s1, char * __s2);
undefined rtc_printf();
undefined rtc_clk_xtal_freq_get();
int sprintf(char * __s, char * __format, ...);
undefined PsramBoardTest();
undefined rtc_init_lite();
undefined GetUartDevice();
undefined rtc_clk_apb_freq_update();
undefined rtc_sleep_set_wakeup_time();
undefined page();
undefined _xtos_ints_off();
undefined ets_isr_attach();
undefined gpio_input_get();
undefined uart_div_modify();
undefined rtc_restore_dig_gpio();
undefined gpio_input_get_high();
undefined ets_isr_unmask();
undefined gpio_output_set_high();
undefined rtc_sleep();
undefined phy_get_romfuncs();
undefined WaitFlashIdle();
undefined rtc_usec2rtc();
undefined rtc_sleep_init();
undefined phy_enter_critical();
undefined spi_flash_wren();
undefined rtc_get_cpu_clk_grade();
undefined PsramDisableVspiClk();
undefined gpio_output_set();

