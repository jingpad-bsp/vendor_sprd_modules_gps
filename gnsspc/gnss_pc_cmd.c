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
#define  GNSS_E(...)   ALOGE(__VA_ARGS__)
#define  GNSS_D(...)   ALOGD(__VA_ARGS__)
#else
#define  GNSS_E(...)   ((void)0)
#define  GNSS_D(...)   ((void)0)
#endif

static int gnss_diag_test(char *buf, int len, char *rsp,
		int rsplen) {
	int ret = 0, start_ret = 0, stop_mode = 0, chip_type = 0, data_len = 0, mode_ret = -1;
	static int init = 0;
	MSG_HEAD_T *msg_head = NULL;
	GNSS_WORKMODE_LIST *tempbuf;

	if ((rsp == NULL) || (buf == NULL)) {
		GNSS_E("%s: NULL\n", __FUNCTION__);
		return 0;
	}
	memcpy(rsp, buf, 2 + sizeof(MSG_HEAD_T));
	msg_head = (MSG_HEAD_T *)(rsp + 1);
	GNSS_D("%s: type %d, len %d\n", __FUNCTION__, msg_head->subtype, len);

	/* Sure gnss_hal_init in the beginning */
	if (gnss_hal_init(GNSS_DIAGTEST_INIT) != INIT_MODE) {
		ret = -1;
		goto diagtestend;
	}

	switch(msg_head->subtype){
		case GNSS_SUBTYPE_SET_SIGNALMODE:
			{
				unsigned char workmode = rsp[sizeof(MSG_HEAD_T) + 1];
				hal_gnsspc_requestData(GNSSPC_SET_WORKMODE,(char *)&workmode);
			}
			break;

		case GNSS_SUBTYPE_GET_SIGNALMODE:
			tempbuf = malloc(sizeof(GNSS_WORKMODE_LIST));
			if(tempbuf == NULL)
			{
				GNSS_E("%s :malloc failed", __FUNCTION__);
			}
			else
			{
				memset(tempbuf,0,sizeof(GNSS_WORKMODE_LIST));
				mode_ret = hal_gnsspc_requestData(GNSSPC_GET_MODELIST, (char *)tempbuf);
				if (mode_ret == 0) {
					rsp[sizeof(MSG_HEAD_T) + 2] = tempbuf->length;
					memcpy(&rsp[sizeof(MSG_HEAD_T) + 3], tempbuf->list, tempbuf->length);
					data_len = tempbuf->length + 1;
					GNSS_D("%s: type %d, dlen %d list:%d \n", __FUNCTION__, msg_head->subtype, data_len, *tempbuf->list);
				}
				free(tempbuf);
			}
			break;

		default:
			if (DIAG_GNSS_STOP == msg_head->subtype) {
				ret = gps_export_stop();
				if (ret < 0)
					goto diagtestend;
			}
			ret = set_gps_mode(msg_head->subtype);
			if (ret == 1) {
				GNSS_D("%s: gps_export_start\n", __FUNCTION__);
				start_ret = gps_export_start();
			}
			break;
	}
diagtestend:
	if ((!start_ret) && (ret >= 0)) {
		rsp[sizeof(MSG_HEAD_T) + 1] = 0;
	} else {
		rsp[sizeof(MSG_HEAD_T) + 1] = 1;
	}
	/* gnsstool ack to pc:MSG_HEAD_T + DAT(ONEBYTE 0: success;1: fail)*/
	msg_head->len = sizeof(MSG_HEAD_T) + 1 + data_len;

	rsp[msg_head->len + 2 - 1] = 0x7E;
	GNSS_D("%s: ack %d\n", __FUNCTION__, rsp[sizeof(MSG_HEAD_T) + 1]);
	return msg_head->len + 2;
}

static int gnss_at_test(char *buf, char *rsp) {
	int ret = 0;
	int i = 0;
	char *ptr = NULL;

	if ((buf == NULL) || (rsp == NULL)) {
		GNSS_E("%s: NULL", __FUNCTION__);
		return 0;
	}

	/* Sure gnss_hal_init in the beginning */
	if (gnss_hal_init(GNSS_ATTEST_INIT) != INIT_MODE) {
		sprintf(rsp,"%d", EUT_GPSERR_INIT);
		return 0;
	}

	ptr = buf + 1 + sizeof(MSG_HEAD_T);
	while (*(ptr + i) != 0x7e) {
		i++;
	}
	*(ptr + i - 1) = '\0';

	gps_at_parse(ptr, rsp);
	GNSS_D("%s %s", __FUNCTION__, rsp);
	return ret;
}
#if 0
static int gnss_read_tsx_data(TOOLS_DIAG_AP_TSX_DATA_T * res)
{
	int rcount;
	int ret = 0;
	FILE * fp = NULL;

	if(NULL == res)
	{
		GNSS_D("%s: res is NULL!!!",__FUNCTION__);
		ret = -1;
		return ret;
	}
	if(access(DIAG_TXDATA_FILE, F_OK) == 0) {
		GNSS_D("%s: %s exists",__FUNCTION__, DIAG_TXDATA_FILE);
		fp = fopen(DIAG_TXDATA_FILE, "r");
		if(NULL == fp)
		{
			GNSS_D("%s: fopen fail errno=%d, strerror(errno)=%s",
					__FUNCTION__, errno, strerror(errno));
			ret = -1;
			return ret;
		}
		rcount = fread(&res->value[0], sizeof(TSX_DATA_T), 2, fp);
		if(rcount <= 0)
		{
			ret = -1;
		}
		GNSS_D("%s: fread count %d",__FUNCTION__, rcount);
		fclose(fp);
	}else{
		ret = -1;
		GNSS_D("%s: %s not exists",__FUNCTION__, DIAG_TXDATA_FILE);
	}
	return ret;
}

static int gnss_write_tsx_data(TOOLS_DIAG_AP_TSX_DATA_T * req)
{
	int rcount;
	int ret = 0, fd = -1;
	FILE * fp = NULL;
	mode_t old_mask;
	static char first_flag = 1;

	if(NULL == req)
	{
		GNSS_D("%s: req is NULL!!!",__FUNCTION__);
		ret = -1;
		return ret;
	}
	GNSS_D("%s: %s exists",__FUNCTION__, DIAG_TXDATA_FILE);
	if(first_flag)
		old_mask = umask(0);
	fp = fopen(DIAG_TXDATA_FILE, "w+");
	if(first_flag)
		umask(old_mask);
	if(NULL == fp)
	{
		GNSS_D("%s: fopen fail errno=%d, strerror(errno)=%s",__FUNCTION__, errno, strerror(errno));
		first_flag = 1;
		ret = -1;
		return ret;
	}
	else
	{
		first_flag = 0;
	}
	rcount = fwrite(&req->value[0], sizeof(TSX_DATA_T), 2, fp);
	GNSS_D("%s: fread count %d",__FUNCTION__, rcount);
	if(2 != rcount)
	{
		ret = -1;
	}
	else
	{
		fflush(fp);
		fd = fileno(fp);
		if(fd > 0) {
			fsync(fd);
		} else {
			GNSS_D("%s: fileno() error, strerror(errno)=%s", __FUNCTION__, strerror(errno));
			ret = -1;
		}
	}
	fclose(fp);
	return ret;
}
static int gnss_diag_txdata(char *buf, int len, char *rsp, int rsplen)
{
	int ret = 0;
	char *rsp_ptr;
	MSG_HEAD_T* msg_head_ptr;
	TOOLS_DIAG_AP_CNF_T* aprsp;
	TOOLS_DIAG_AP_TSX_DATA_T* src_tsxdata;
	TOOLS_DIAG_AP_TSX_DATA_T* tsxdata;

	if(NULL == buf){
		GNSS_D("%s,null pointer",__FUNCTION__);
		return 0;
	}
	rsp_ptr = rsp;
	memset(rsp_ptr, 0x00, rsplen);

	msg_head_ptr = (MSG_HEAD_T*)(buf + 1);
	rsplen = sizeof(TOOLS_DIAG_AP_TSX_DATA_T)+ sizeof(TOOLS_DIAG_AP_CNF_T) +
		sizeof(MSG_HEAD_T);
	aprsp = (TOOLS_DIAG_AP_CNF_T*)(rsp_ptr + sizeof(MSG_HEAD_T));
	tsxdata = (TOOLS_DIAG_AP_TSX_DATA_T*)(rsp_ptr + sizeof(MSG_HEAD_T) +
			sizeof(TOOLS_DIAG_AP_CNF_T));
	memcpy(rsp_ptr,msg_head_ptr,sizeof(MSG_HEAD_T));
	((MSG_HEAD_T*)rsp_ptr)->len = rsplen;
	aprsp->status = DIAG_AP_CMD_TSX_DATA;
	aprsp->length= sizeof(TOOLS_DIAG_AP_TSX_DATA_T);
	src_tsxdata = (TOOLS_DIAG_AP_TSX_DATA_T*)(buf + 1 +
			sizeof(TOOLS_DIAG_AP_CNF_T) + sizeof(MSG_HEAD_T));
	if(0 == src_tsxdata->cmd)
	{
		/*memcpy api defect, trigger crash in androidN*/
		/*memcpy(tsxdata, src_tsxdata, sizeof(TOOLS_DIAG_AP_TSX_DATA_T));*/
		for(int y=0; y<sizeof(TOOLS_DIAG_AP_TSX_DATA_T); y++) {
			*((char *)tsxdata + y) = *((char *)src_tsxdata + y);
		}
		ret = gnss_write_tsx_data(src_tsxdata);
		if(0 == ret)
		{
			tsxdata->res_status = 0;
		}else{
			tsxdata->res_status = 1;
		}
	}else if(1 == src_tsxdata->cmd){
		ret = gnss_read_tsx_data(tsxdata);
		tsxdata->cmd = 1;
		if(0 == ret)
		{
			tsxdata->res_status = 0;
		}else{
			tsxdata->res_status = 1;
		}
	}else{
		GNSS_D("%s: tsx_data cmd not read and write !!!\n", __FUNCTION__);
	}
out:
	return rsplen;
}

static int gnss_read_tsx_data_ext(TOOLS_DIAG_AP_TSX_DATA_EXT_T * res)
{
	int rcount;
	int ret = 0;
	FILE * fp = NULL;

	if(NULL == res)
	{
		GNSS_D("%s: res is NULL!!!",__FUNCTION__);
		ret = -1;
		return ret;
	}
	if(access(DIAG_TXDATA_FILE, F_OK) == 0) {
		GNSS_D("%s: %s exists",__FUNCTION__, DIAG_TXDATA_FILE);
		fp = fopen(DIAG_TXDATA_FILE, "r");
		if(NULL == fp)
		{
			GNSS_D("%s: fopen fail errno=%d, strerror(errno)=%s",
					__FUNCTION__, errno, strerror(errno));
			ret = -1;
			return ret;
		}
		rcount = fread(&res->value[0], sizeof(TSX_DATA_EXT_T), 4, fp);
		if(rcount <= 0)
		{
			ret = -1;
		}
		GNSS_D("%s: fread count %d",__FUNCTION__, rcount);
		fclose(fp);
	}else{
		ret = -1;
		GNSS_D("%s: %s not exists",__FUNCTION__, DIAG_TXDATA_FILE);
	}
	return ret;
}
static int gnss_write_tsx_data_ext(TOOLS_DIAG_AP_TSX_DATA_EXT_T * req)
{
	int rcount;
	int ret = 0, fd = -1;
	FILE * fp = NULL;
	mode_t old_mask;
	static char first_flag = 1;

	if(NULL == req)
	{
		GNSS_D("%s: req is NULL!!!",__FUNCTION__);
		ret = -1;
		return ret;
	}
	GNSS_D("%s: %s exists",__FUNCTION__, DIAG_TXDATA_FILE);
	if(first_flag)
		old_mask = umask(0);
	fp = fopen(DIAG_TXDATA_FILE, "w+");
	if(first_flag)
		umask(old_mask);
	if(NULL == fp)
	{
		GNSS_D("%s: fopen fail errno=%d, strerror(errno)=%s",__FUNCTION__, errno, strerror(errno));
		first_flag = 1;
		ret = -1;
		return ret;
	}
	else
	{
		first_flag = 0;
	}
	rcount = fwrite(&req->value[0], sizeof(TSX_DATA_EXT_T), 4, fp);
	GNSS_D("%s: fread count %d",__FUNCTION__, rcount);
	if(4 != rcount)
	{
		GNSS_D("%s: rcount is not matched!",__FUNCTION__);
		ret = -1;
	}
	else
	{
		fflush(fp);
		fd = fileno(fp);
		if(fd > 0) {
			fsync(fd);
		} else {
			GNSS_D("%s: fileno() error, strerror(errno)=%s", __FUNCTION__, strerror(errno));
			ret = -1;
		}
	}
	fclose(fp);
	return ret;
}

static int gnss_diag_txdata_ext(char *buf, int len, char *rsp, int rsplen)
{
	int ret = 0;
	char *rsp_ptr;
	MSG_HEAD_T* msg_head_ptr;
	TOOLS_DIAG_AP_CNF_T* aprsp;
	TOOLS_DIAG_AP_TSX_DATA_EXT_T* src_tsxdata;
	TOOLS_DIAG_AP_TSX_DATA_EXT_T* tsxdata;

	if(NULL == buf){
		GNSS_D("%s,null pointer",__FUNCTION__);
		return 0;
	}
	msg_head_ptr = (MSG_HEAD_T*)(buf + 1);
	rsplen = sizeof(TOOLS_DIAG_AP_TSX_DATA_EXT_T) +
		sizeof(TOOLS_DIAG_AP_CNF_T) + sizeof(MSG_HEAD_T);
	rsp_ptr = rsp;
	memset(rsp_ptr, 0x00, rsplen);
	aprsp = (TOOLS_DIAG_AP_CNF_T*)(rsp_ptr + sizeof(MSG_HEAD_T));
	tsxdata = (TOOLS_DIAG_AP_TSX_DATA_EXT_T*)(rsp_ptr + sizeof(MSG_HEAD_T) +
			sizeof(TOOLS_DIAG_AP_CNF_T));
	memcpy(rsp_ptr,msg_head_ptr,sizeof(MSG_HEAD_T));
	((MSG_HEAD_T*)rsp_ptr)->len = rsplen;
	aprsp->status = DIAG_AP_CMD_TSX_DATA_EXT;
	aprsp->length= sizeof(TOOLS_DIAG_AP_TSX_DATA_EXT_T);
	src_tsxdata = (TOOLS_DIAG_AP_TSX_DATA_EXT_T*)(buf + 1 +
			sizeof(TOOLS_DIAG_AP_CNF_T) + sizeof(MSG_HEAD_T));
	if(0 == src_tsxdata->cmd)
	{
		/*memcpy api defect, trigger crash in androidN*/
		/* memcpy(tsxdata, src_tsxdata, sizeof(TOOLS_DIAG_AP_TSX_DATA_T)); */
		for(int y=0; y<sizeof(TOOLS_DIAG_AP_TSX_DATA_EXT_T); y++) {
			*((char *)tsxdata + y) = *((char *)src_tsxdata + y);
		}
		ret = gnss_write_tsx_data_ext(src_tsxdata);
		GNSS_D("res_status=%d ret=%d",tsxdata->res_status, ret);
		if(0 == ret)
		{
			tsxdata->res_status = 0;
		}else{
			tsxdata->res_status = 1;
		}
		GNSS_D("res_status=%d",tsxdata->res_status);
	}else if(1 == src_tsxdata->cmd){
		ret = gnss_read_tsx_data_ext(tsxdata);
		tsxdata->cmd = 1;
		if(0 == ret)
		{
			tsxdata->res_status = 0;
		}else{
			tsxdata->res_status = 1;
		}
	}else{
		GNSS_D("%s: tsx_data cmd not read and write !!!\n", __FUNCTION__);
	}
out:
	return rsplen;
}
#endif
void register_this_module_ext(struct eng_callback *reg, int *num)
{
	int moudles_num = 0;
	int i = 0;

	GNSS_D("%s", __FUNCTION__);

	for (i= 0; i < DIAG_GNSS_MAX; i++) {
		(reg + i)->type = DIAG_CMD_GPS_AUTO_TEST; //main cmd
		(reg + i)->subtype = i; //sub cmd
		(reg + i)->diag_ap_cmd = i + 1; //whether data cmd is exist?
		(reg + i)->eng_diag_func = gnss_diag_test; // rsp function ptr
		(reg + i)->eng_set_writeinterface_func = set_report_ptr;
		moudles_num++;
	}
	(reg + i)->type = 0x38; //main cmd
	(reg + i)->subtype = 0x16; //sub cmd
	(reg + i)->eng_diag_func = gnss_auto_test; // rsp function ptr
	moudles_num++;
	i++;
#if 0
	/* old tsx structure cmd */
	(reg + i)->type = DIAG_CMD_APCALI; //main cmd
	(reg + i)->subtype = DIAG_AP_CMD_TSX_DATA; //sub cmd
	(reg + i)->diag_ap_cmd = i + 1; //whether data cmd is exist?
	(reg + i)->eng_diag_func = gnss_diag_txdata; // rsp function ptr
	moudles_num++;
	i++;

	/* new tsx structure cmd */
	(reg + i)->type = DIAG_CMD_APCALI; //main cmd
	(reg + i)->subtype = DIAG_AP_CMD_TSX_DATA_EXT; //sub cmd
	(reg + i)->diag_ap_cmd = i + 1; //whether data cmd is exist?
	(reg + i)->eng_diag_func = gnss_diag_txdata_ext; // rsp function ptr
	moudles_num++;
	i++;
#endif
	sprintf((reg + i)->at_cmd, "%s", "AT+SPGPSTEST");
	(reg + i)->eng_linuxcmd_func = gnss_at_test;
	moudles_num++;
	*num = moudles_num;
	GNSS_D("%s module cmd:%s\n", __FUNCTION__, (reg + i)->at_cmd);
	i++;
	sprintf((reg + i)->at_cmd, "%s", "AT+GPSTEST"); /* for nativeMMI(factorytest) */
	(reg + i)->eng_linuxcmd_func = gnss_nativeMMI_test;
	moudles_num++;
	*num = moudles_num;
	GNSS_D("%s navitive module cmd:%s\n", __FUNCTION__, (reg + i)->at_cmd);

}


