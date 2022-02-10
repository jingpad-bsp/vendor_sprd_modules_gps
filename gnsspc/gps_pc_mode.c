///============================================================================
/// Copyright 2012-2014  spreadtrum  --
/// This program be used, duplicated, modified or distributed
/// pursuant to the terms and conditions of the Apache 2 License.
/// ---------------------------------------------------------------------------
/// file gps_lib.c
/// for converting NMEA to Android like APIs
/// ---------------------------------------------------------------------------
/// zhouxw mod 20130920,version 1.00,include test,need add supl so on...
///============================================================================
#include <errno.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/epoll.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <signal.h>
#include <cutils/sockets.h>
#include <cutils/properties.h>
#include "../gnsshal/common/inc/gps.h"
#include <pthread.h>
#include <stdio.h>
#include <cutils/log.h>
#include <stdlib.h>
#include <unistd.h>    
#include <sys/stat.h>   
#include <string.h>  
#include <semaphore.h>
#include <dlfcn.h>

#include "gps_pc_mode.h"

#ifdef LOG_TAG
#undef LOG_TAG
#endif
#define  LOG_TAG  "LIBGPS_ENGPC"

#define  GPS_DEBUG
#ifdef GPS_DEBUG
#define  E(...)   ALOGE(__VA_ARGS__)
#define  D(...)   ALOGD(__VA_ARGS__)
#else
#define  E(...)   ((void)0)
#define  D(...)   ((void)0)
#endif

#define  GNSS_WR_REG

static int eutmode = 0;
static int eut_gps_state;
static int gps_search_state;
static char buf[512];
static sem_t sem_a;
static pthread_mutex_t mutex_a = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mutex_b = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mutex_tsx = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mutex_cwcn = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mutex_tcxo = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t mutex_mmitest = PTHREAD_MUTEX_INITIALIZER;
static GpsSvStatus SvState;
static GpsLocation GPSloc;
static int fix_status;
#define  GREENEYE_I    1
#define  GREENEYE_II   2
static int chip_id = 0;
static char *version = "VERSION: 2017-09-15\r\n";
static DYMIC_WRITETOPC_FUNC g_func[WRITE_TO_MAX] = {NULL};

#define TOKEN_LEN(tok)  (tok.end>tok.p?tok.end-tok.p:0)
#define  MAX_NMEA_TOKENS  32
int cwcn_value = 0;
double tsx_value = 0;
double tcxo_value = 0;
extern int CN0; // CW_CN0
#define LIBGPS_PATH "/vendor/lib/hw/gps.default.so"
typedef struct {
	const char*  p;
	const char*  end;
} Token;

typedef struct {
	int     count;
	Token   tokens[ MAX_NMEA_TOKENS ];
} NmeaTokenizer;

static int str2int( const char*  p, const char*  end )
{
	int   result = 0;
	int   len    = end - p;

	for ( ; len > 0; len--, p++ )
	{
		int  c;

		if (p >= end)
			goto Fail;

		c = *p - '0';
		if ((unsigned)c >= 10)
			goto Fail;

		result = result*10 + c;
	}
	return  result;

Fail:
	return -1;
}

static double str2float( const char*  p, const char*  end )
{
	int   result = 0;
	int   len    = end - p;
	char  temp[16];

	if (len >= (int)sizeof(temp))
		return 0.;
	if(len == 0)
		return 0.;
	memcpy( temp, p, len );
	temp[len] = 0;
	return strtod( temp, NULL );
}

static int strhex2int( const char*  p, int len )
{
	int   result = 0;
	D("strhex2int: (%s) len=%d \n", p, len);
	//len = len -1;

	if (len <= 0)
		goto Fail;
	for ( ; len > 0; len--, p++ )
	{
		int  c;

		if(*p == '\0')
			break;
		if ((*p >= 'a') && (*p <= 'f'))
			c = *p - 'a' + 10;
		else if ((*p >= 'A') && (*p <= 'F'))
			c = *p - 'A' + 10;
		else if ((*p >= '0') && (*p <= '9'))
			c = *p - '0';
		else
		{
			D("strhex2int: fail \n");
			//goto Fail;
			break;
		}
		result = result*0x10 + c;
	}

	D("strhex2int: (%d) \n", result);

	return  result;

Fail:
	return -1;
}

static int nmea_tokenizer_init( NmeaTokenizer*  t, const char*  p, const char*  end )
{
	int    count = 0;
	char*  q;

	// the initial '$' is optional
	if (p < end && p[0] == '$')
		p += 1;

	// remove trailing newline
	if (end > p && end[-1] == '\n') {
		end -= 1;
		if (end > p && end[-1] == '\r')
			end -= 1;
	}

	// get rid of checksum at the end of the sentecne
	if (end >= p+3 && end[-3] == '*') {
		end -= 3;
	}

	while (p < end) {
		const char*  q = p;

		q = memchr(p, ',', end-p);
		if (q == NULL)
			q = end;

		if (q >= p) {
			if (count < MAX_NMEA_TOKENS) {
				t->tokens[count].p   = p;
				t->tokens[count].end = q;
				count += 1;
			}
		}
		if (q < end)
			q += 1;

		p = q;
	}

	t->count = count;
	return count;
}

static Token  nmea_tokenizer_get( NmeaTokenizer*  t, int  index )
{
	Token  tok;
	static const char*  dummy = "";

	if (index < 0 || index >= t->count) 
	{
		tok.p = tok.end = dummy;
	} 
	else
	{
		tok = t->tokens[index];
	}

	return tok;
}

static int nmea_update_cwcn(Token cwcn)
{
	Token   tok = cwcn;

	if (tok.p >= tok.end)
		return -1;
	pthread_mutex_lock(&mutex_cwcn);
	cwcn_value = str2int(tok.p, tok.end);
	D("nmea_update_cwcn: value=%d \n", cwcn_value);	
	pthread_mutex_unlock(&mutex_cwcn);
	return 0;
}

static int nmea_update_tsx(Token tsx)
{
	Token   tok = tsx;

	if (tok.p >= tok.end)
		return -1;
	pthread_mutex_lock(&mutex_tsx);
	tsx_value = str2float(tok.p, tok.end);
	D("nmea_update_tsx: value=%f \n", tsx_value);
	pthread_mutex_unlock(&mutex_tsx);
	return 0;
}

static int nmea_update_tcxo(Token tcxo)
{
	Token   tok = tcxo;

	if (tok.p >= tok.end)
		return -1;
	pthread_mutex_lock(&mutex_tcxo);
	tcxo_value = str2float(tok.p, tok.end);
	D("nmea_update_tcxo: value=%f \n", tcxo_value);
	pthread_mutex_unlock(&mutex_tcxo);
	return 0;
}

static void  nmea_parse(const char* nmea, int length)
{
	NmeaTokenizer  tzer[1];
	Token          tok;

	if((chip_id == GREENEYE_II) && (eut_gps_state == 2))
	{
		cw_data_capture(nmea, length); // CW_CN0
	}
	else
	{
		if((length < 9) && (eut_gps_state != 5))
		{
			//E("nmea_parse: sentence too short.");
			return;
		}
	}

	nmea_tokenizer_init(tzer, nmea, nmea+length);

	tok = nmea_tokenizer_get(tzer, 0);
	if (tok.p + 5 > tok.end) 
	{
		//E("sentence id '%.*s' too short, ignored.", tok.end-tok.p, tok.p);
		return;
	}

	//if ( !memcmp(tok.p,"PCGDS",5))
	if ( !memcmp(tok.p,"PCGDS",5))
	{
		D("PCGDS enter now\n");
		Token  name = nmea_tokenizer_get(tzer,1);
		if( !memcmp(name.p,"CWCN0",5))
		{
			D("PCGDS ==>> CWCN0\n");
			Token  value = nmea_tokenizer_get(tzer,2);
			nmea_update_cwcn(value);
		}
		else if( !memcmp(name.p,"TSXTEMP",7))
		{
			D("PCGDS ==>> TSXTEMP\n");
			Token  value = nmea_tokenizer_get(tzer,2);
			nmea_update_tsx(value);
		}
		else if( !memcmp(name.p,"TCXO",4))
		{
			D("PCGDS ==>> TCXO\n");
			Token  value = nmea_tokenizer_get(tzer,2);
			nmea_update_tcxo(value);
		}
	}
}

static void location_callback(GpsLocation* location)
{
	fix_status = 1;
	memcpy(&GPSloc,location,sizeof(GpsLocation));
	D("%s called\n",__FUNCTION__);
}

static void status_callback(GpsStatus* status)
{
	D("%s called\n",__FUNCTION__);
}

static void sv_status_callback(GpsSvStatus* sv_status)
{
	D("%s called\n",__FUNCTION__);
	pthread_mutex_lock(&mutex_b);
	memcpy(&SvState,sv_status,sizeof(GpsSvStatus));
	pthread_mutex_unlock(&mutex_b);
}
/* static report_ptr g_func; */
int set_report_ptr(DYMIC_WRITETOPC_FUNC * write_interface_ptr)
{
	static bool flag = false;
	D("%s", __FUNCTION__);

	if (flag == false) {
		for(int i = 0; i < WRITE_TO_MAX; i++) {
			g_func[i] = write_interface_ptr[i];
			if(g_func[i] != NULL)
				D("%s ad 0x%x, i %d", __FUNCTION__, g_func[i], i);
		}
		flag = true;
	}
	return 0;
}
static void nmea_callback(GpsUtcTime timestamp, const char* nmea, int length)
{
	int ret = 0;

	if(g_func[WRITE_TO_HOST_DIAG] != NULL)
	{
		char tmpbuf[2048] = {0};
		int r_cnt = 0, ser_fd;
		MSG_HEAD_T* msg_head = (MSG_HEAD_T*)(tmpbuf + 1);

		D("%s enter", __FUNCTION__);

		tmpbuf[0] = 0x7e;  /* ·â×°Í· 0x7e */
		msg_head->type = DIAG_CMD_GPS_AUTO_TEST;
		msg_head->subtype = DIAG_GNSS_RECEIVE_NMEA;

		if (length > 0) {
			memcpy(tmpbuf + 1 + sizeof(MSG_HEAD_T), nmea, length);
			msg_head->len = length + sizeof(MSG_HEAD_T);
		}
		tmpbuf[msg_head->len + 2 - 1]= 0x7e;  /* ·â×°Î² 0x7e */
		g_func[WRITE_TO_HOST_DIAG](tmpbuf,(msg_head->len +2));
	}
	nmea_parse(nmea,length);
}
static void set_capabilities_callback(uint32_t capabilities)
{
	D("%s called, capabilities: %d\n",__FUNCTION__,capabilities);
}

static void acquire_wakelock_callback()
{
	D("%s called\n",__FUNCTION__);
}

static void release_wakelock_callback()
{
	D("%s called\n",__FUNCTION__);
}

static void request_utc_time_callback()
{
	D("%s called\n",__FUNCTION__);
}

static void gnss_set_system_info_test(const GnssSystemInfo* info)
{
	D("%s called\n",__FUNCTION__);
}

static void gnss_sv_status_callback_test(GnssSvStatus* sv_info)
{
	int i = 0;

	D("%s called\n",__FUNCTION__);
	if(NULL == sv_info )
	{
		return;
	}
	pthread_mutex_lock(&mutex_b);
	SvState.num_svs = sv_info->num_svs;
	for(i = 0; i < SvState.num_svs; i ++ )
	{
		SvState.sv_list[i].prn = sv_info->gnss_sv_list[i].svid;
		SvState.sv_list[i].snr = sv_info->gnss_sv_list[i].c_n0_dbhz;
		SvState.sv_list[i].azimuth = sv_info->gnss_sv_list[i].azimuth;
		SvState.sv_list[i].elevation = sv_info->gnss_sv_list[i].elevation;
	}
	pthread_mutex_unlock(&mutex_b);
}

static pthread_t create_thread_callback(const char* name, void (*start)(void *), void* arg)
{
	pthread_t pid;
	D("%s called\n",__FUNCTION__); 
	pthread_create(&pid, NULL,(void *)start, arg);
	return pid;
}

GpsCallbacks sGpsCallbacks = {
	sizeof(GpsCallbacks),
	location_callback,
	status_callback,
	sv_status_callback,
	nmea_callback,
	set_capabilities_callback,
	acquire_wakelock_callback,
	release_wakelock_callback,
	create_thread_callback,
	request_utc_time_callback,
	gnss_set_system_info_test,
	gnss_sv_status_callback_test
};

GpsInterface *pGpsface;
AGpsInterface *pAGpsface;

#ifdef GNSS_WR_REG
int write_register(unsigned int addr,unsigned int value)
{
	WR_reg *w_regData;

	D("%s :addr=0x%x, value=0x%x\n", __FUNCTION__,addr, value);
	w_regData->addr = addr;
	w_regData->value = value;
	hal_gnsspc_requestData(GNSSPC_SET_REG,(char *)w_regData);
	return 0;
}

unsigned int read_register(unsigned int addr)
{
	WR_reg *r_regData;
	int ret = -1;
	unsigned int value = 0;

	D("%s :addr=0x%x\n", __FUNCTION__,addr);
	r_regData = malloc(sizeof(WR_reg));
	if(r_regData == NULL)
	{
		E("%s :malloc failed", __FUNCTION__);
	}
	else
	{
		memset(r_regData,0,sizeof(WR_reg));
		r_regData->addr = addr;
		ret = hal_gnsspc_requestData(GNSSPC_GET_REG,(char *)r_regData);
		if(ret == 0)
		{
			value = ((WR_reg*)r_regData)->value;
			D("%s : value=0x%x\n", __FUNCTION__,value);
		}
		else
		{
			E("%s :read reg failed.", __FUNCTION__);
		}
		free(r_regData);
	}
	return value;
}

int config_register(int data,char *rsp)
{
	unsigned int value = 0;
	unsigned int addr = 0x00F4;

	if((gps_search_state == 0) && (eut_gps_state == 0))
	{
		sprintf(rsp,"%s%d",EUT_GPS_ERROR,EUT_GPSERR_PRNSEARCH);
		return 0;
	}

	D("config_register\n");
	value = read_register(addr);
	D("config_register: addr=0x%x, value=0x%x \n",addr,value);
	sprintf(rsp,"addr=0x%x, value=0x%x",addr,value);

	return 0;
}

#endif

int start_engine(void)
{
	pid_t pid; 
	pid=fork();
	if(pid==0)
	{
		D("start engine");
		system("GPSenseEngine");
	}
	return 0;
}

int search_for_engine(void)
{
	char rsp[256];
	FILE *stream;
	int ret = 0;

	memset(rsp,0,sizeof(rsp));

        stream = popen("ps | grep GPSenseEngine","r");
        if(stream == NULL)
        {
                E("GPSenseEngine is start fail\n");
                ret = 0;
        }
        else
        {
        	fgets(rsp,sizeof(rsp),stream);
        	if(strstr(rsp,"GPSenseEngine") != NULL)
        	{
	        	D("GPSenseEngine is start success\n");
	        	ret = 1;
                }
        	else
         	{
	     		E("GPSenseEngine is start fail\n");
			ret = 0;
		}
	}
	pclose(stream);
	return ret;
}

int gps_export_start(void)
{
	int ret = 0;
	int i =0;
	unsigned int state = 0;
	char * p_data;
	int state_ret =  -1;

	E("gps_export_start is enter\n");

	memset(&SvState,0,sizeof(GpsSvStatus));

	if(pGpsface == NULL)
	{
		E("start fail,for not init");
		return -1;
	}

	pGpsface->start();
	p_data = malloc(sizeof(int));
	if(p_data == NULL)
	{
		E("%s :malloc failed", __FUNCTION__);
	}
	else
	{
		memset(p_data ,0,sizeof(int));
		for (i=0; i<40; i++)
		{/* Wait that gps_start finish*/
			state_ret  = hal_gnsspc_requestData(GNSSPC_GET_GPSSTATE,p_data);
			if(state_ret  == 0)
			{
				ret = 1;
				state = (unsigned int)*p_data;
				if (state == DEVICE_WORKED)
					break;
				usleep(100000);
			}
			else
				E("%s, get state count is %d", __FUNCTION__, i);
		}
		E("%s, wait start count is %d", __FUNCTION__, i);
		free(p_data);
	}

	return ret;
}

int gps_export_stop(void)
{
	int ret =0;
	D("gps_export_stop ret is %d\n",ret);

	if(pGpsface == NULL)
	{
		E("stop fail,for not init");
		return -1;
	}
	pGpsface->stop();
	return ret;
}
int gps_export_clean(void)
{
	if(pGpsface == NULL)
	{
		E("cleanup fail,for not init");
		return -1;
	}
	pGpsface->cleanup();

	return 0;
}
int set_gps_mode(unsigned int mode)
{
	int ret = 0;

	D("%s %d\n", __FUNCTION__, mode);
	if(pGpsface == NULL)
	{
		E("set mode fail,for not init");
		return -1;
	}
	switch(mode)
	{
		case DIAG_GNSS_HOT_START: // Hot start
			pGpsface->delete_aiding_data(GPS_HOT_START);   //trigger cs/hs/ws
			ret = 1;
			break;
		case DIAG_GNSS_WARM_START: // Warm start
			pGpsface->delete_aiding_data(GPS_WARM_START);   //trigger cs/hs/ws
			ret = 1;
			break;
		case DIAG_GNSS_COLD_START: // Cold start
			pGpsface->delete_aiding_data(GPS_COLD_START);   //trigger cs/hs/ws
			ret = 1;
			break;
		case DIAG_GNSS_PS_START: // ps start
			E("post debug is open");
			pGpsface->delete_aiding_data(GPS_PS_START);   //trigger cs/hs/ws
			ret = 0;
			break;
		case DIAG_GNSS_TCXO: // TCXO mode
			D("set TCXO mode");
			pGpsface->delete_aiding_data(GPS_TCXO);	// TCXO stability test
			ret = 1;
			break;
		case DIAG_GNSS_RF_GPS: // GPS rf data test
		case DIAG_GNSS_RF_GLONASS: // GLONASS rf data test
		case DIAG_GNSS_RF_BEIDOU: // Beidou rf dta test
			D("set rf data tool mode");
			{
				int temp = 0;
				eut_gps_state = 5;
				if(mode == DIAG_GNSS_RF_GPS){
					temp = GPS_RF_GPS;
				}
				else if(mode == DIAG_GNSS_RF_GLONASS){
					temp = GPS_RF_GL;
				}
				else{
					temp = GPS_RF_BD;
				}
				pGpsface->delete_aiding_data(temp);
				ret = 1;
			}
			break;
		case DIAG_GNSS_FAC_START: // Fac start
			pGpsface->delete_aiding_data(GPS_FAC_START);   //trigger cs/hs/ws
			ret = 1;
			break;
		case DIAG_GNSS_NEW_TCXO:
			D("set TSX-TCXO  new mode");
			pGpsface->delete_aiding_data(GPS_NEW_TCXO);    // TCXO	stability test
			ret = 1;
			break;
		case DIAG_GNSS_ORBIT_SWITCH:
			D("%s, set ORBIT switch", __FUNCTION__);
			pGpsface->delete_aiding_data(GPS_LTE_DIS);    // TCXO  stability test
			ret = 1;
			break;
		default:
			break;
	}

	return ret;
}

int get_nmea_data(char *nbuff)
{
	int len = 0;
	D("get nmea data enter");
#if 0
	sem_wait(&sem_a);
	pthread_mutex_lock(&mutex_a);
	len = strlen(buf);  //don't add 1 for '\0'
	if(len > 9)
	{
		memcpy(nbuff,buf,len);
	}
	pthread_mutex_unlock(&mutex_a);
#endif
	return len;
}


void astatus_cb(AGpsStatus* status)
{
	D("astatus  enter");
	return;
}

pthread_t acreate_thread_cb(const char* name, void (*start)(void *), void* arg)
{
	pthread_t apid = 0;
	return apid;
}

AGpsCallbacks acallbacks = {
	astatus_cb,
	acreate_thread_cb,
};

int gnss_hal_init(int type)
{
	static int first_open = 0;
	int ret = 0;
	if(first_open == 0)
	{
		void *handle;
		GpsInterface* (*get_interface)(struct gps_device_t* dev);
		void* (*get_extension)(const char* name);
		char *error;
		int i = 0;

		D("begin gps init\n");
		if(access("/system/etc/GPSenseEngine.xml",0) == -1)
		{
			//GreenEye2
			chip_id = GREENEYE_II;
		}
		else
		{
			//GreenEye
			chip_id = GREENEYE_I; // no used, reserved for a while
		}

		D("before dlopen");
		handle = dlopen(LIBGPS_PATH, RTLD_LAZY);
		if (!handle) {
			E("%s\n", dlerror());
			return 0;
		}
		D("after dlopen\n");
		dlerror();    /* Clear any existing error */
		get_interface = dlsym(handle, "gps_get_hardware_interface");
		D("obtain GPS HAL interface \n");
		pGpsface = get_interface(NULL);
		pGpsface->init(&sGpsCallbacks);

		get_extension= dlsym(handle, "gps_getExtension");
		pAGpsface = get_extension("agps");
		pAGpsface->init(&acallbacks);
		hal_gnsspc_requestData = dlsym(handle, "gps_gnsspcRequestData");
		if (hal_gnsspc_requestData)
			hal_gnsspc_requestData(GNSSPC_SET_INIT,&type);
		else
			E("Don't get  hal request date interface \n");
		first_open = 1;
		sem_init(&sem_a,0,0);
	}
	return INIT_MODE;
}

int get_stop_mode(void)
{
	return DIAG_GNSS_STOP;
}

int eut_parse(int data,int sub_data, char *rsp)
{
	eutmode = 1;

	D("%s %d\n", __FUNCTION__, data);
	if((data == 1)||(data == 2)||(data == 3)||(data == 4))
	{
		eut_gps_state = 1;

		if(data == 2)
		{
			eut_gps_state = 2;
			pGpsface->delete_aiding_data(GPS_CW_CN0);    // CW CN0 test
		}
		else if(data == 3)
		{
			eut_gps_state = 3;
			pGpsface->delete_aiding_data(GPS_TSX_TEMP);    // TSX TEMP test
		}
		else if(data == 4)
		{
			eut_gps_state = 4;
			pGpsface->delete_aiding_data(GPS_TCXO);    // TCXO stability  test
		}
		else if(data == 1)
		{
			pGpsface->delete_aiding_data(GPS_FAC_START);	// factory start
		}

		gps_export_start();
		strcpy(rsp,EUT_GPS_OK);
	}
	else if(data == 0)
	{
		eut_gps_state = 0;
		gps_export_stop();   //block,for test
		strcpy(rsp,EUT_GPS_OK);
	}

	return 0;
}

int eut_eq_parse(int data,int sub_data,char *rsp)
{
	sprintf(rsp,"%s%d",EUT_GPS_REQ,eut_gps_state);
	return 0;
}

int search_eq_parse(int data,int sub_data,char *rsp)
{
	sprintf(rsp,"%s%d",EUT_GPS_SEARCH_REQ,gps_search_state);
	return 0;
}

int search_parse(int data,int sub_data,char *rsp)
{	
	if(data != eut_gps_state)
	{
		eut_parse(data,sub_data,rsp);
	}
	gps_search_state = eut_gps_state;
	strcpy(rsp,EUT_GPS_OK);
	return 0;
}

int get_prn_list(char *rsp)
{
	int i = 0,lenth = 0;
	memcpy(rsp,EUT_GPS_PRN_REQ,strlen(EUT_GPS_PRN_REQ));
	lenth = strlen(EUT_GPS_PRN_REQ);
	pthread_mutex_lock(&mutex_b);
	for(i = 0; i < SvState.num_svs; i++)
	{
		lenth = lenth + sprintf(rsp + lenth,"%d,",SvState.sv_list[i].prn);
	}
	pthread_mutex_unlock(&mutex_b);
	return SvState.num_svs;
}

int prnstate_parse(int data,int sub_data,char *rsp)
{
	if((gps_search_state == 0) && (eut_gps_state == 0))
	{
		sprintf(rsp,"%s%d",EUT_GPS_ERROR,EUT_GPSERR_PRNSEARCH);
		return 0;
	}

	if(get_prn_list(rsp) == 0)
	{
		sprintf(rsp,"%s%s",EUT_GPS_PRN_REQ,EUT_GPS_NO_FOUND_STAELITE);
	}
	return 0;
}

int snr_parse(int data,int sub_data,char *rsp)
{
	int max_id = 0,i = 0;
	D("snr parse enter");
	if((gps_search_state == 0) && (eut_gps_state == 0))
	{
		E("snr_parse: gps has not search");
		sprintf(rsp,"%s%d",EUT_GPS_ERROR,EUT_GPSERR_PRNSEARCH);
		return 0;
	}
	if(SvState.num_svs == 0)
	{
		E("snr_parse: num_svs is 0, return \n");
		sprintf(rsp,"%s no sv_num is found",EUT_GPS_SNR_REQ);
		return 0;
	}
	pthread_mutex_lock(&mutex_b);
	for(i = 0; i < SvState.num_svs; i++)
	{
		if(SvState.sv_list[i].snr > SvState.sv_list[max_id].snr)
		{
			max_id = i;
		}
	}
	sprintf(rsp,"%s%f %s%d %s%d",EUT_GPS_SNR_REQ,SvState.sv_list[max_id].snr,
			EUT_GPS_SV_ID,SvState.sv_list[max_id].prn,
			EUT_GPS_SV_NUMS,SvState.num_svs);
	pthread_mutex_unlock(&mutex_b);
	D("snr_parse: %s",rsp);
	return 0;
}

int prn_parse(int data,int sub_data,char *rsp)
{
	int i = 0,found = 0;
	if((gps_search_state == 0) && (eut_gps_state == 0))
	{
		sprintf(rsp,"%s%d",EUT_GPS_ERROR,EUT_GPSERR_PRNSEARCH);
		return 0;
	}

	pthread_mutex_lock(&mutex_b);
	for(i = 0; i < SvState.num_svs; i++)
	{
		if(SvState.sv_list[i].prn == data)
		{
			sprintf(rsp,"%s%d,CN0=%f",EUT_GPS_PRN_REQ,data,SvState.sv_list[i].snr);
			found = 1;
			break;
		}
	}
	pthread_mutex_unlock(&mutex_b);

	if(found == 0)
	{
		E("prn_parse: can not find prn and it's snr \n");
		sprintf(rsp,"%s%s",EUT_GPS_PRN_REQ,EUT_GPS_NO_FOUND_STAELITE);
	}
	return 0;
}

int fix_parse(int data,int sub_data,char *rsp)
{
	int i = 0,found = 0;
	if((gps_search_state == 0) && (eut_gps_state == 0))
	{
		sprintf(rsp,"%s%d",EUT_GPS_ERROR,EUT_GPSERR_PRNSEARCH);
		return 0;
	}

	if(fix_status == 0)
	{
		E("fix_parse: cannot fix location \n");
		sprintf(rsp,"+SPGPSTEST:LOCATION=FAIL");
	}	
	else
	{
		E("fix_parse: fixed \n");
		sprintf(rsp,"+SPGPSTEST:LOCATION=SUCC,%f,%f",GPSloc.latitude,GPSloc.longitude);
	}
	return 0;
}

int cwcn_parse(int data,int sub_data,char *rsp)
{	
	D("cwcn_parse  enter");
	if((gps_search_state == 0) && (eut_gps_state == 0))
	{
		E("cwcn_parse: gps has not search");
		sprintf(rsp,"%s%d",EUT_GPS_ERROR,EUT_GPSERR_PRNSEARCH);
		return 0;
	}

	if(chip_id == GREENEYE_I)
	{
		pthread_mutex_lock(&mutex_cwcn);
		D("cwcn_parse: cwcn_value=%d \n", cwcn_value);
		sprintf(rsp,"%s%d",EUT_GPS_RSSI_REQ, cwcn_value);
		pthread_mutex_unlock(&mutex_cwcn);
		D("cwcn_parse: %s",rsp);
	}
	else
	{
		D("cwcn_parse: CN0=%d \n", CN0);
		sprintf(rsp,"%s%d",EUT_GPS_RSSI_REQ, CN0);
		D("cwcn_parse: %s",rsp);
	}

	return 0;
}

int tsx_temp_parse(int data,int sub_data,char *rsp)
{
	int i=0;
	char tsx_temp[40];
	int ret = -1;
	D("tsx_temp_parse  enter");
	if((gps_search_state == 0) && (eut_gps_state == 0))
	{
		E("tsx_temp_parse: gps has not search");
		sprintf(rsp,"%s%d",EUT_GPS_ERROR,EUT_GPSERR_PRNSEARCH);
		return 0;
	}

	ret = hal_gnsspc_requestData(GNSSPC_GET_TSX,tsx_temp);
	if (ret == 0)
		sprintf(rsp,"%s%s",EUT_GPS_TSXTEMP_REQ,tsx_temp);
	else
		sprintf(rsp,"%sERROR",EUT_GPS_TSXTEMP_REQ);
	E("tsx_temp_parse: %s",rsp);

	return 0;
}

int tcxo_stability_parse(int data,int sub_data,char *rsp)
{
	D("tcxo_stability_parse  enter");
	if((gps_search_state == 0) && (eut_gps_state == 0))
	{
		E("tcxo_stability_parse: gps has not search");
		sprintf(rsp,"%s%d",EUT_GPS_ERROR,EUT_GPSERR_PRNSEARCH);
		return 0;
	}

	pthread_mutex_lock(&mutex_tcxo);
	D("tcxo_stability_parse: tcxo_value=%f \n",tcxo_value);
	sprintf(rsp,"%s%f",EUT_GPS_TCXO_REQ,tcxo_value);
	pthread_mutex_unlock(&mutex_tcxo);
	D("tcxo_stability_parse: %s",rsp);
	return 0;
}
/*--------------------------------------------------------------------------
Function:  gnss_nativeMMI_test

Description:
get the gps test state, for Native MMI gps test

Parameters:

Return: fail: -1; success: num_svs
--------------------------------------------------------------------------*/
int gnss_nativeMMI_test(char *buf, char *rsp)
{
	int i =0, count =0, ret =0, strlen = 0;

	E("gnss_nativeMMI_test enter");
	pthread_mutex_lock(&mutex_mmitest);

	/* Sure gnss_hal_init in the beginning */
	if (gnss_hal_init(GNSS_MMITEST_INIT) != INIT_MODE) {
		sprintf(rsp,"%d", EUT_GPSERR_INIT);
		return 0;
	}

	pGpsface->delete_aiding_data(GPS_FAC_START);	// factory start
	gps_export_start();

	/* mmitest request num >=4 && snr one of svs > 35*/
	do {
		pthread_mutex_lock(&mutex_b);
		if (SvState.num_svs >= 4) {
			for (i = 0; i < SvState.num_svs; i++)
			{
				if (rsp != NULL) {
					strlen += sprintf(rsp + strlen, "%f,",
						        SvState.sv_list[i].snr);
				}
				if ((SvState.sv_list[i].snr > 35) && (ret == 0))
				{
					ret = SvState.num_svs;
				}
			}
		}
		pthread_mutex_unlock(&mutex_b);
		if (ret > 0) {
			*(rsp + strlen + 1) ='\0';
			E("%s: %s", __FUNCTION__, rsp);
			break;
		}
		else if (strlen > 0) {
			memset(rsp, '\0', strlen);
			strlen = 0;
		}
		usleep(200000);
		count ++;
	} while(count < 150);

	gps_export_stop();
	if (ret <= 0) {
		ret = -1;
	}
	E("%s: %d, wait cnt %d", __FUNCTION__, ret, count*200);
	pthread_mutex_unlock(&mutex_mmitest);
	return ret;
}
int gnss_auto_test(char *buf, int len, char *rsp, int rsplen) {
	int ret = -1;
	int length = 0;
	int cmd  = 0;
	char data[1014] = {0 };
	MSG_HEAD_T *msg_head = NULL;

	if ((rsp == NULL) || (buf == NULL)) {
		E("%s: NULL\n", __FUNCTION__);
		return 0;
	}
	D("pc->engpc:%d number--> %x %x %x %x %x %x %x %x %x %x %x ",
         len,buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7],buf[8],buf[9],buf[10]);

	cmd = *(buf + 1 + sizeof(MSG_HEAD_T));
	switch(cmd){
	case GNSSPC_AUTOTEST_OPEN:
		if (gnss_hal_init(GNSS_DIAGTEST_INIT) == INIT_MODE) {
			ret = 0;
		}
		break;
	case GNSSPC_AUTOTEST_SEARCH:
		if (pGpsface != NULL){
			E(" init success");
			pGpsface->delete_aiding_data(GPS_FAC_START);
			if(gps_export_start() > 0)
                        {
        			ret = 0;
			}
                }
		break;
        case GNSSPC_AUTOTEST_SVN:
		{
			int svn = SvState.num_svs;
			data[0] = (uchar)svn;
			length = 1;
			ret = 0;
		}
		break;
        case GNSSPC_AUTOTEST_CLOSE:
		gps_export_stop();
		gps_export_clean();
		ret = 0;
		break;
        case GNSSPC_AUTOTEST_SNR:
		{
			int svn = SvState.num_svs;
			data[0] = (uchar)svn;
			D("%s: svn = %d\n", __FUNCTION__, svn);
			for(int i = 0; i < svn; i++)
			{
				data[i+1] = (uchar)SvState.sv_list[i].snr;
				D("%s: i=%d, SvState.sv_list[i].snr=%d data[i+1]=%x\n",
                                  __FUNCTION__, i, SvState.sv_list[i].snr, data[i+1]);
			}
			length = svn + 1;
			ret = 0;
		}
		break;
	default:
		break;
	}

	memcpy(rsp, buf, 1 + sizeof(MSG_HEAD_T) - 1);
	msg_head = (MSG_HEAD_T *)(rsp + 1);
	msg_head->len = 8;

	D("msg_head,ret=%d",ret);
	if(ret < 0)
	{
		rsp[sizeof(MSG_HEAD_T)] = 1;    //38 01 表示测试失败。
	}
	else if (ret == 0)
	{
		rsp[sizeof(MSG_HEAD_T)] = 0;    //38 00 表示测试ok
		if(length > 0)
		{
			memcpy(rsp + 1 + sizeof(MSG_HEAD_T), data, length); //将获取到的length个数据，复制到38 00 后面
			msg_head->len += length; //返回长度：基本数据长度8+获取到的length个字节数据长度。
		}
	}
	D("rsp[1 + sizeof(MSG_HEAD_T):%d]:%d",sizeof(MSG_HEAD_T),rsp[sizeof(MSG_HEAD_T)]);

	//填充协议尾部的7E
	rsp[msg_head->len + 2 - 1] = 0x7E;  //加上数据尾标志
	D("%s :return len:%d", __FUNCTION__, msg_head->len + 2);
	D("engpc->pc:%x %x %x %x %x %x %x %x %x %x",rsp[0],rsp[1],rsp[2],
          rsp[3],rsp[4],rsp[5],rsp[6],rsp[7],rsp[8],rsp[9]); //78 xx xx xx xx _ _38 _ _ 打印返回的10个数据

	return msg_head->len + 2;

}

int eut_read_register(int addr, int sub_data, char *rsp)
{	
	D("eut_read_register  enter");	
	unsigned int value = 0;

	if((gps_search_state == 0) && (eut_gps_state == 0))
	{
		E("eut_read_register: gps has not search");
		sprintf(rsp,"%s%d",EUT_GPS_ERROR,EUT_GPSERR_PRNSEARCH);
		return 0;
	}
	D("eut_read_register: addr=0x%x \n",addr);
	value = read_register(addr);

	sprintf(rsp,"%s0x%x",EUT_GPS_READ_REQ,value);
	D("eut_read_register: %s",rsp);
	return 0;
}

int eut_write_register(int addr, int value, char *rsp)
{	
	D("eut_write_register  enter");
	int ret = 0;

	if((gps_search_state == 0) && (eut_gps_state == 0))
	{
		E("eut_write_register: gps has not search");
		sprintf(rsp,"%s%d",EUT_GPS_ERROR,EUT_GPSERR_PRNSEARCH);
		return 0;
	}

	D("eut_write_register: addr=0x%x, value=0x%x \n",addr, value);
	ret = write_register(addr,value);
	strcpy(rsp,EUT_GPS_OK);
	D("eut_write_register: %s",rsp);
	return 0;
}
int gnss_cmd_logic_parse(int data,int sub_data, char *rsp)
{
	char tempcmd[2] ={0};

	D("%s eut %d type:%d subtype:%d\n", __FUNCTION__, eut_gps_state,
			data, sub_data);
	tempcmd[0] = data;
	tempcmd[1] = sub_data;
		
	if (eut_gps_state == 1) {
		hal_gnsspc_requestData(GNSSPC_TEST_INTERFACE, &tempcmd[0]);
	}
	strcpy(rsp,EUT_GPS_OK);
	return 0;
}

//typedef int (*eut_function)(int data,char *rsp);
typedef int (*eut_function)(int data,int sub_data, char *rsp);
typedef struct
{
	char *name;
	eut_function func;
}eut_data; 

eut_data eut_gps_at_table[] = {
	{"EUT?",eut_eq_parse},
	{"EUT",eut_parse},
	{"SEARCH?",search_eq_parse},
	{"SEARCH",search_parse},
	{"PRN?",prnstate_parse},
	{"SNR?",snr_parse},
	{"PRN",prn_parse},
	{"LOCATION?",fix_parse},
	{"RSSI?",cwcn_parse},            // CW CN0
	{"TSXTEMP?",tsx_temp_parse},     // TSX temp read
	{"TCXO?",tcxo_stability_parse},  // TCXO stability verify
	{"READ",eut_read_register},
	{"WRITE",eut_write_register},
	{"CMD",gnss_cmd_logic_parse},
};

#if 0
int (*state[ENG_GPS_NUM])(int data,char *rsp) = {
	eut_parse,
	eut_eq_parse,
	search_eq_parse,
	search_parse,
	prnstate_parse,
	snr_parse,
	prn_parse,
};

static char *eut_gps_name[ENG_GPS_NUM] = {
	"EUT",
	"EUT?",
	"SEARCH?",
	"SEARCH",
	"PRNSTATE?",
	"SNR?",
	"PRN",	
};

//EUT
int get_cmd_index(char *buf)
{
	int i;
	for(i = 0;i < ENG_GPS_NUM;i++)
	{
		if(strstr(buf,eut_gps_name[i]) != NULL)
		{
			break;
		}
	}
	return i;
}
#endif

int get_sub_str(char *buf, char **revdata, char a, char *delim, unsigned char count, unsigned char substr_max_len)
{
	int len, len1, len2;
	char *start = NULL;
	char *substr = NULL;
	char *end = buf;
	int str_len = strlen(buf);

	start = strchr(buf, a);
	substr = strstr(buf, delim);

	if(!substr)
	{
		/* if current1 not exist, return this function.*/
		return 0;
	}

	while (end && *end != '\0')
	{
		end++;
	}

	if((NULL != start) && (NULL != end))
	{
		char *tokenPtr = NULL;
		unsigned int index = 1; /*must be inited by 1, because data[0] is command name */

		start++;
		substr++;
		len = substr - start - 1;

		/* get cmd name */
		memcpy(revdata[0], start, len);

		/* get sub str by delimeter */
		tokenPtr = strtok(substr, delim);
		while(NULL != tokenPtr && index < count) 
		{
			strncpy(revdata[index++], tokenPtr, substr_max_len);

			/* next */
			tokenPtr = strtok(NULL, delim);
		}

	}

	return 0;
}

int get_sub_str_colon(char *buf, char **revdata, char a, char *delim, unsigned char count, unsigned char substr_max_len)
{
	int len, len1, len2;
	char *start = NULL;
	char *substr = NULL;
	char *end = buf;
	int str_len = strlen(buf);

	start = strchr(buf, a);
	substr = strstr(buf, delim);

	if(!substr)
	{
		/* if current1 not exist, return this function.*/
		return 0;
	}

	while (end && *end != '\0')
	{
		end++;
	}

	if((NULL != start) && (NULL != end))
	{
		char *tokenPtr = NULL;
		unsigned int index = 1; /*must be inited by 1, because data[0] is command name */

		start++;
		substr++;
		len = substr - start - 1;

		/* get cmd name */
		memcpy(revdata[0], start, len);

		/* get sub str by delimeter */
		tokenPtr = strtok(substr, ":");
		while(NULL != tokenPtr && index < count) 
		{
			strncpy(revdata[index++], tokenPtr, substr_max_len);

			/* next */
			tokenPtr = strtok(NULL, ":");
		}

	}

	return 0;
}

void gps_at_parse(char *buf,char *rsp)
{
	int i = 0;   //data is get from buf,used arg1,arg2.
	int arg1_data,arg2_data;
	static char args0[32+1];
	static char args1[32+1];
	static char args2[32+1];
	static char args3[32+1];
	//should init to 0
	memset(args0,0,sizeof(args0));
	memset(args1,0,sizeof(args1));
	memset(args2,0,sizeof(args2));
	memset(args3,0,sizeof(args3));

	char *data[4] = {args0, args1, args2, args3};
	//get_sub_str(buf, data, '=', ",", 4, 32);
	get_sub_str_colon(buf, data, '=', ",", 4, 32);

	D("gps_at_parse enter");
#if 0
	index = get_cmd_index(buf);
	if((index > ENG_GPS_NUM - 1) || (index < 0))
	{
		E("get index error!!\n");
		return;
	}
	state[index](data,rsp);
#endif
	for(i = 0; i < (sizeof(eut_gps_at_table) / sizeof(eut_data)); i++)
	{
		if(strstr(buf,eut_gps_at_table[i].name) != NULL)
		{			
			if(!memcmp(eut_gps_at_table[i].name,"READ",strlen("READ")))
			{
				arg1_data = strhex2int(data[1],strlen(data[1]));
				D("command arg1=%d, arg2=%d,i=%d \n",atoi(data[1]),atoi(data[2]),i);
				eut_gps_at_table[i].func(arg1_data,atoi(data[2]),rsp);
				break;
			}
			else if(!memcmp(eut_gps_at_table[i].name,"WRITE",strlen("WRITE")))
			{
				arg1_data = strhex2int(data[1],strlen(data[1]));
				arg2_data = strhex2int(data[2],strlen(data[2]));
				D("command arg1=%d, arg2=%d,i=%d \n",arg1_data,arg2_data,i);
				eut_gps_at_table[i].func(arg1_data,arg2_data,rsp);
				break;
			}
			else
			{
				D("command arg1=%d, arg2=%d,i=%d \n",atoi(data[1]),atoi(data[2]),i);
				eut_gps_at_table[i].func(atoi(data[1]),atoi(data[2]),rsp);
				break;
			}
		}
	}
	return;
}

