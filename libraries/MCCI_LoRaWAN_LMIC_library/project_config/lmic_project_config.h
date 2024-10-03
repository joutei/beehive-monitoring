// project-specific definitions
#define CFG_eu868 1
//#define CFG_us915 1
//#define CFG_au915 1
//#define CFG_as923 1
//#define LMIC_COUNTRY_CODE LMIC_COUNTRY_CODE_JP      /* for as923-JP; also define CFG_as923 */
//#define CFG_kr920 1
//#define CFG_in866 1
//#define CFG_sx1272_radio 1
#define CFG_sx1276_radio 1
//#define LMIC_USE_INTERRUPTS
#define DISABLE_PING // Used in class B, useless for A
#define DISABLE_BEACONS //same 
//#define LMIC_ENABLE_arbitrary_clock_error 1
#define DISABLE_JOIN
#define DISABLE_LMIC_FAILURE_TO
#define DISABLE_MCMD_DutyCycleReq
#define DISABLE_MCMD_RXParamSetupReq
#define DISABLE_MCMD_NewChannelReq
#define DISABLE_MCMD_DlChannelReq
#define DISABLE_MCMD_RXTimingSetupReq
#define LMIC_MAX_FRAME_LENGTH 64
