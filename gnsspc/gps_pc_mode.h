#ifndef GPS_PC_MODE_H
#define GPS_PC_MODE_H

#include "sprd_fts_type.h"
/*just a part of gps_pc_mode from gps_pc_mode in gps.c\
  this value need sync with it*/
//temp modification
#define DIAG_CMD_GPS_AUTO_TEST 0x3A

enum gps_pc_mode{
	GPS_WARM_START	  = 1,
	GPS_PS_START	  = 100,

	GPS_COLD_START	  = 125,

	GPS_LOG_ENABLE	  = 136,
	GPS_LOG_DISABLE   = 520,
	GPS_HOT_START	  = 1024,
	GPS_LTE_DIS = 4128,
	GPS_TCXO	  = 50001,  /*TCXO stability test*/
	GPS_CW_CN0        = 50002,  /*CW CN0 test*/
	GPS_RF_GPS	  = 50003,  /*RF data tool GPS mode*/
	GPS_TSX_TEMP	  = 50004,  /*TSX TEMP test*/
	GPS_NEW_TCXO	  = 50005,  /*TCXO new stability test*/
	GPS_RF_GL    	  = 50006,  /*RF data tool GLONASS mode*/
	GPS_RF_BD         = 50007,  /*RF data tool BEIDOU mode*/

	GPS_FAC_START	  = 65535,  /*factory start*/
};

enum diag_gnss_subtype{
	DIAG_GNSS_HOT_START = 0,
	DIAG_GNSS_WARM_START = 1,
	DIAG_GNSS_COLD_START = 2,
	DIAG_GNSS_STOP = 3,
	DIAG_GNSS_RECEIVE_NMEA = 5,
	DIAG_GNSS_MODE_SET = 7,
	DIAG_GNSS_PS_START = 8,
	DIAG_GNSS_TCXO = 10,
	DIAG_GNSS_RF_GPS = 11,
	DIAG_GNSS_RF_GLONASS = 12,
	DIAG_GNSS_RF_BEIDOU = 13,
	DIAG_GNSS_FAC_START = 20,
	DIAG_GNSS_NEW_TCXO = 21,
	DIAG_GNSS_ORBIT_SWITCH = 23,
	GNSS_SUBTYPE_GET_SIGNALMODE = 24,
	GNSS_SUBTYPE_SET_SIGNALMODE = 25,
	DIAG_GNSS_MAX,
};
#define EUT_GPS_OK                  "+SPGPSTEST:OK"
#define EUT_GPS_ERROR               "+SPGPSTEST:ERR="
#define EUT_GPS_REQ                 "+SPGPSTEST:EUT="
#define EUT_GPS_PRN_REQ             "+SPGPSTEST:PRN="
#define EUT_GPS_SNR_REQ             "+SPGPSTEST:SNR="
#define EUT_GPS_RSSI_REQ            "+SPGPSTEST:RSSI="
#define EUT_GPS_TSXTEMP_REQ         "+SPGPSTEST:TSXTEMP="
#define EUT_GPS_TCXO_REQ            "+SPGPSTEST:TCXO="
#define EUT_GPS_READ_REQ            "+SPGPSTEST:READ="
#define EUT_GPS_SEARCH_REQ          "+SPGPSTEST:SEARCH="
#define EUT_GPS_SNR_NO_EXIST        "NO_EXIST"
#define EUT_GPS_NO_FOUND_STAELITE   "NO_FOUND_SATELLITE"
#define EUT_GPS_SV_ID               "SV_ID="
#define EUT_GPS_SV_NUMS             "SV_NUM="

#define EUT_GPSERR_SEARCH					(153)
#define EUT_GPSERR_PRNSTATE 				(154)
#define EUT_GPSERR_PRNSEARCH				(155)
#define EUT_GPSERR_INIT 					(156)

// GPS PC MODE
#define INIT_MODE  0x7

#define DIAG_TXDATA_FILE "/productinfo/txdata.txt"

/* sync with gps_common.h */
typedef enum DevicepowerState
{
	DEVICE_DOWNLOADING = 1,
	DEVICE_DOWNLOADED,
	DEVICE_POWERONING,
	DEVICE_POWERONED,
	DEVICE_WORKING,
	DEVICE_WORKED,
	DEVICE_IDLING,
	DEVICE_IDLED,
	DEVICE_SLEEPING,
	DEVICE_SLEEP,
	DEVICE_WAKEUPING,
	DEVICE_RESET,
	DEVICE_POWEROFFING,
	DEVICE_POWEROFFED,
	DEVICE_ABNORMAL
}TDevicepowerState;
/* Begin: cp-mode record */
typedef struct{
    unsigned char length;
    unsigned char* list;
}GNSS_WORKMODE_LIST;
typedef enum{
	GNSS_MODELIST_GET,
	GNSS_MODELIST_SET,
	GNSS_MODELIST_MAX
}GNSS_ListOp_e;
/* End */

/* begin: mainly for test log print and diag cp-mode config */
typedef enum{
	GNSS_DEF_INIT, /* INIT by APP normal init */
	GNSS_DIAGTEST_INIT = 0xFFE0, /* INIT by diag test, gnsstool now */
	GNSS_ATTEST_INIT, /* INIT by at test, cali now  */
	GNSS_MMITEST_INIT, /* INIT by nativemmi test */
}GNSS_INIT_TYPE;
/* end: mainly for test log print  and diag cp-mode config */
typedef enum {
	GNSSPC_SET_NONE = 0,
	GNSSPC_SET_REG,
	GNSSPC_SET_INIT,
	GNSSPC_SET_WORKMODE,

	GNSSPC_GET_REG,
	GNSSPC_GET_TSX,
	GNSSPC_GET_GPSSTATE,
	GNSSPC_GET_MODELIST,
	GNSSPC_TEST_INTERFACE,
}gnsspc_hal_opcode_e;

typedef struct{
    unsigned int addr;
    unsigned int value;
}WR_reg;

typedef enum {
	GNSSPC_AUTOTEST_OPEN = 1,
	GNSSPC_AUTOTEST_SEARCH,
	GNSSPC_AUTOTEST_SVN,
	GNSSPC_AUTOTEST_CLOSE,
	GNSSPC_AUTOTEST_SNR,
}gnsspc_autotest_cmd;

typedef void (*report_ptr)(const char* nmea, int length);
extern int set_report_ptr(DYMIC_WRITETOPC_FUNC * write_interface_ptr);
extern int gps_export_start(void);
extern int gps_export_stop(void);
extern int get_nmea_data(char *nbuff);
extern int set_gps_mode(unsigned int mode);
extern int gnss_hal_init(int type);
extern int get_stop_mode(void);
extern void gps_at_parse(char *buf,char *rsp);
void cw_data_capture(const char* nmea, int length);
int gnss_nativeMMI_test(char *buf, char *rsp);
int (*hal_gnsspc_requestData)(int flag, char* pData);
int gnss_auto_test(char *buf, int len, char *rsp, int rsplen);
#endif
