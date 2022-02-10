///============================================================================
/// Copyright 2012-2017  spreadtrum  --
/// This program be used, duplicated, modified or distributed
/// pursuant to the terms and conditions of the Apache 2 License.
/// ---------------------------------------------------------------------------
/// file gps_hal.c
/// for  gps hardware APIs
/// ---------------------------------------------------------------------------
/// jinlei.dou mod 20190325,version 1.00,
///============================================================================
#include <dlfcn.h>
#include <cutils/log.h>
#include <stdlib.h>
#include <string.h>
#include "gps.h"

#ifdef LOG_TAG
#undef LOG_TAG
#endif
#define  LOG_TAG  "GPS_HAL"
#define  E(...)   ALOGE(__VA_ARGS__)
#define  D(...)   ALOGD(__VA_ARGS__)

#ifndef NULL
#define NULL 0
#endif

#define GNSSMGT64BIT_PATH "/vendor/lib64/libgnssmgt.so"
#define GNSSMGT32BIT_PATH "/vendor/lib/libgnssmgt.so"

int   (*gps_initMgt)( GpsCallbacks* callbacks );
int   (*gps_startMgt)( void );
int   (*gps_stopMgt)( void );
void  (*gps_cleanupMgt)( void );
int   (*gps_injectTimeMgt)(GpsUtcTime time, int64_t timeReference,
			int uncertainty);
int  (*gps_injectBestLocationMgt)(GnssLocation *pLoc);
int  (*gps_injectLocationMgt)(double latitude, double longitude, float accuracy);
void  (*gps_deleteAidingDataMgt)(GpsAidingData flags);
int   (*gps_setPositionModeMgt)(GpsPositionMode mode, GpsPositionRecurrence recurrence,
			uint32_t min_interval, uint32_t preferred_accuracy, uint32_t preferred_time);

int  (*gps_xtraInitMgt)( GpsXtraCallbacks* callbacks );
int  (*gps_injectXtraDataMgt)( char* data, int length );

size_t (*gps_getInternalStateMgt)(char* buffer, size_t bufferSize);

void (*assist_gpsInitMgt)(AGpsCallbacks* callbacks);
int (*assist_gpsDataConnOpenMgt)(const char* apn);
int (*assist_gpsDataConnClosedMgt)();
int (*assist_gpsDataConnFailedMgt)();
int (*assist_gpsSetServerMgt)(AGpsType type, const char* hostname, int port);
int (*assist_gpsDataConnOpenApnMgt)(const char* apn,ApnIpType apnIpType);

int  (*install_certificatesMgt) ( const DerEncodedCertificate* certificates, size_t length );
int  (*revoke_certificatesMgt) ( const Sha1CertificateFingerprint* fingerprints, size_t length );

void (*gps_niInitMgt) (GpsNiCallbacks *callbacks);
void (*gps_niRespondMgt) (int notif_id, GpsUserResponseType user_response);

void (*assist_rilInitMgt)( AGpsRilCallbacks* callbacks );
void (*assist_rilRefLocationMgt) (const AGpsRefLocation *agps_reflocation, size_t sz_struct);
void (*assist_rilSetIdMgt) (AGpsSetIDType type, const char* setid);
void (*assist_rilSendNiMessageMgt) (uint8_t *msg, size_t len);
void (*assist_rilUpdateNetworkStateMgt) (int connected, int type, int roaming, const char* extra_info);
void (*assist_rilUpdateNetworkAvailabilityMgt) (int avaiable, const char* apn);

void  (*gps_geofenceInitMgt)( GpsGeofenceCallbacks* callbacks );
void (*gps_geofenceAddAreaMgt) (int32_t geofence_id, double latitude, double longitude,
			double radius_meters, int last_transition, int monitor_transitions,
			int notification_responsiveness_ms, int unknown_timer_ms);
void (*gps_geofencePauseMgt) (int32_t geofence_id);
void (*gps_geofenceResumeMgt) (int32_t geofence_id, int monitor_transitions);
void (*gps_geofenceRemoveAreaMgt) (int32_t geofence_id);

int (*gps_measurementInitMgt) (GpsMeasurementCallbacks* callbacks);
void (*gps_measurementCloseMgt) ();

int (*gps_navigationMessageInitMgt) (GpsNavigationMessageCallbacks* callbacks);
void (*gps_navigationMessageCloseMgt) ();

void (*gnss_configurationUpdateMgt) (const char* config_data, int32_t length);

void (*gnss_unisocNetworkHandleBindMgt) (long int handle_val);

int (*gnsspc_requestDataMgt)(int flag, char* pData);

int (*gps_visibilityCtrl_initMgt)(GnssVisibilityControlCallbacks *callbacks);
int (*gps_nfwLocationAcessEnMgt)(const char *proxyApps);
bool (*gps_esExtensionSecSetMgt)(unsigned int seconds);
int (*gps_measurementCorrectionsInitMgt) (GnssMeasurementCorrectionCallbacks *callbacks);
int (*gps_measurementSetCorrectionsMgt) (GnssMeasurementCorrections_t *pMeasCorr, int iSatCount);


/********the funcitons of sGpsXtraInterface list  start********/
static int gps_xtra_init(GpsXtraCallbacks* callbacks)
{
	int ret = 0;

	D("%s entry  callbacks: %p", __func__, callbacks);
	ret = gps_xtraInitMgt(callbacks);
	return ret;
}

static int gps_xtra_injectData( char* data, int length )
{
	int ret = 0;

	D("%s entry, length:%d\n",__func__, length);
	if(data != NULL)
		D("%s entry, data:%s",__func__, data);
	ret = gps_injectXtraDataMgt(data,length);

	return ret;
}

static const GpsXtraInterface	 sGpsXtraInterface =
{
	.size				  =  sizeof(GpsXtraInterface),
	.init				  =  gps_xtra_init,
	.inject_xtra_data	  =  gps_xtra_injectData,
};
/********the funcitons of sGpsXtraInterface list  end********/

/********the funcitons of sGpsDebugInterface list  start********/
static size_t gps_getInternalState(char* buffer, size_t bufferSize)
{
	size_t ret = 0;

	D("%s entry buffer %s, len =%zu\n",__func__,buffer,bufferSize);
	ret = gps_getInternalStateMgt(buffer,bufferSize);

	return ret;
}

static const GpsDebugInterface	 sGpsDebugInterface =
{
	.size                 =  sizeof(GpsDebugInterface),
	.get_internal_state   =  gps_getInternalState,
};
/********the funcitons of sGpsDebugInterface list  end********/

/********the funcitons of sAssistGpsInterface list  start********/
static void assist_gps_init( AGpsCallbacks* callbacks )
{
	D("%s entry,callbacks:%p",__func__, callbacks);
	assist_gpsInitMgt(callbacks);
}


static int assist_gps_dataConnOpen( const char* apn )
{
	int ret = 0;

	D("%s entry,apn:%s",__func__, apn);
	ret = assist_gpsDataConnOpenMgt(apn);

	return ret;
}



static int assist_gps_dataConnClosed(void)
{
	int ret = 0;

	D("%s entry",__func__);
	ret = assist_gpsDataConnClosedMgt();

	return ret;
}

static int assist_gps_dataConnFailed(void)
{
	int ret = 0;

	D("%s entry",__func__);
	ret = assist_gpsDataConnFailedMgt();

	return ret;
}

static int assist_gps_SetServer( AGpsType type, const char* hostname, int port )
{
	int ret = 0;

	D("%s entry type:%d port:%d",__func__,type, port);
	if(hostname != NULL)
		D("%s entry hostname:%s",__func__, hostname);
	ret = assist_gpsSetServerMgt(type, hostname, port);

	return ret;
}

static int assist_gps_dataConnCpenApn(const char* apn, ApnIpType apnIpType)
{
	int ret = 0;

	D("%s entry apnIpType:%d",__func__, apnIpType);
	if(apn != NULL)
		D("%s entry apn:%s",__func__, apn);
	assist_gpsDataConnOpenApnMgt(apn,apnIpType);

	return ret;
}

static const AGpsInterface sAssistGpsInterface =
{
	.size =              sizeof(AGpsInterface),
	.init =              assist_gps_init,
	.data_conn_open =    assist_gps_dataConnOpen,
	.data_conn_closed =  assist_gps_dataConnClosed,
	.data_conn_failed =  assist_gps_dataConnFailed,
	.set_server =        assist_gps_SetServer,
	.data_conn_open_with_apn_ip_type =  assist_gps_dataConnCpenApn,
};
/********the funcitons of sAssistGpsInterface list  end********/

/********the funcitons of sSuplCertificateInterface list  start********/
int  supl_install_certificates ( const DerEncodedCertificate* certificates, size_t length)
{
	int ret = 0;

	D("%s entry length:%d",__func__, length);
	ret = install_certificatesMgt(certificates, length);

	return ret;
}

int  supl_revoke_certificates ( const Sha1CertificateFingerprint* fingerprints, size_t length)
{
	int ret = 0;

	D("%s entry length:%d",__func__, length);
	ret = revoke_certificatesMgt(fingerprints, length);

	return ret;
}

static const SuplCertificateInterface sSuplCertificateInterface =
{
	.size =              sizeof(SuplCertificateInterface),
	.install_certificates =   supl_install_certificates,
	.revoke_certificates =    supl_revoke_certificates,
};
/********the funcitons of sSuplCertificateInterface list  end********/

/********the funcitons of sGpsNiInterface list  start********/
static void gps_ni_init(GpsNiCallbacks *callbacks)
{
	D("%s entry callbacks:%p",__func__, callbacks);
	gps_niInitMgt(callbacks);
}

static void gps_ni_respond(int notif_id, GpsUserResponseType user_response)
{
	D("%s entry notif_id:%d user_response:%d",__func__, notif_id, user_response);
	gps_niRespondMgt(notif_id, user_response);
}

static const  GpsNiInterface  sGpsNiInterface = {
	.size =    sizeof(GpsNiInterface),
	.init =    gps_ni_init,
	.respond = gps_ni_respond,
};
/********the funcitons of sGpsNiInterface list  end********/

/********the funcitons of sAssistGpsRilInterface list  start********/
static void assist_ril_init(AGpsRilCallbacks *callbacks)
{
	D("%s entry callbacks:%p",__func__, callbacks);
	assist_rilInitMgt(callbacks);
}

static void assist_ril_refLocation(const AGpsRefLocation *agps_reflocation, size_t sz)
{
	D("%s entry sz:%d",__func__, sz);
	assist_rilRefLocationMgt(agps_reflocation,sz);
}

static void assist_ril_setId(AGpsSetIDType type, const char *setid)
{

	D("%s entry type:%d",__func__, type);
	if(setid != NULL)
		D("%s entry setid:%s",__func__, setid);
	assist_rilSetIdMgt(type,setid);

}

static void assist_ril_sendNiMessage(uint8_t *msg, size_t sz)
{
	D("%s entry sz:%d",__func__, sz);
	assist_rilSendNiMessageMgt(msg, sz);
}

static void assist_ril_updateNetworkState(int connected, int type, int roaming, const char* extra_info)
{
	D("%s: connected:%d,type:%d,roaming:%d,extra:%s",__func__,\
		connected,type,roaming,extra_info);
	assist_rilUpdateNetworkStateMgt(connected, type, roaming, extra_info);
}

static void assist_ril_updateNetworkAvail(int avaiable, const char* apn)
{
	D("%s entry avaiable:%d ",__func__, avaiable);
	if(apn != NULL)
		D("%s entry avaiable:%s ",__func__, apn);
	assist_rilUpdateNetworkAvailabilityMgt(avaiable,apn);
}

static const AGpsRilInterface sAssistGpsRilInterface =
{
	.size =                        sizeof(AGpsRilInterface),
	.init =                        assist_ril_init,
	.set_ref_location =            assist_ril_refLocation,
	.set_set_id =                  assist_ril_setId,
	.ni_message =                  assist_ril_sendNiMessage,
	.update_network_state =        assist_ril_updateNetworkState,
	.update_network_availability = assist_ril_updateNetworkAvail,
};
/********the funcitons of sAssistGpsRilInterface list  end********/

/********the funcitons of sGpsGeoInterface list  start********/
void  gps_geofence_init(GpsGeofenceCallbacks* callbacks )
{
	D("%s entry callbacks:%p",__func__, callbacks);
	gps_geofenceInitMgt(callbacks);
}
void gps_geofence_addArea(int32_t geofence_id, double latitude,
		double longitude, double radius_meters,
		int last_transition, int monitor_transitions,
		int notification_responsiveness_ms,
		int unknown_timer_ms)
{
	D("%s:%d,%f,%f,%f,%d,%d,%d,%d\n",__func__,\
			geofence_id,latitude,longitude,radius_meters,last_transition,\
			monitor_transitions,notification_responsiveness_ms,unknown_timer_ms);
	gps_geofenceAddAreaMgt(geofence_id, latitude,
		longitude,  radius_meters,last_transition, monitor_transitions,
		notification_responsiveness_ms,unknown_timer_ms);
}
void gps_geofence_pause(int32_t geofence_id)
{
	D("%s entry geofence_id:%d",__func__, geofence_id);
	gps_geofencePauseMgt(geofence_id);
}

void gps_geofence_resume(int32_t geofence_id, int monitor_transitions)
{
	D("%s entry geofence id  %d transitions= %d\n",__func__,geofence_id,monitor_transitions);
	gps_geofenceResumeMgt(geofence_id,monitor_transitions);
}

void gps_geofence_removeArea(int32_t geofence_id)
{

	D("%s entry geofence_id  %d",__func__,geofence_id);
	gps_geofenceRemoveAreaMgt(geofence_id);

}

static const GpsGeofencingInterface sGpsGeoInterface =
{
	.size                 =     sizeof(GpsGeofencingInterface),
	.init                 =     gps_geofence_init,
	.add_geofence_area	 =     gps_geofence_addArea,
	.pause_geofence       =     gps_geofence_pause,
	.resume_geofence      =     gps_geofence_resume,
	.remove_geofence_area =     gps_geofence_removeArea,
};
/********the funcitons of sGpsGeoInterface list  end********/

/********the funcitons of sGpsMeasurementInterface list  start********/
int gps_measurement_init(GpsMeasurementCallbacks* callbacks)
{
	int ret = 0;

	D("%s entry callbacks:%p",__func__, callbacks);
	ret = gps_measurementInitMgt(callbacks);

	return ret;
}

void gps_measurement_close(void)
{
	D("%s entry",__func__);
	gps_measurementCloseMgt();
}

int gps_measurement_corrections_init(GnssMeasurementCorrectionCallbacks* callbacks)
{
	int ret = 0;

	D("%s entry callbacks:%p",__func__, callbacks);
	//call GNSS_MGT functions
	ret  =  gps_measurementCorrectionsInitMgt(callbacks);
	return ret;
}

int gps_measurement_set_corrections(GnssMeasurementCorrections_t *corrections, int single_count)
{
	int ret = 0;
	D("%s entry ",__func__);
	ret  =  gps_measurementSetCorrectionsMgt(corrections, single_count);

	return ret;
}

int gps_visibility_control_init(GnssVisibilityControlCallbacks* callbacks)
{
	int ret = 0;

	D("%s entry callbacks:%p",__func__, callbacks);
	ret  =  gps_visibilityCtrl_initMgt(callbacks);
	D("%s entry ret=%d",__func__, ret);
	return ret;
}

int gps_visibility_control_enable_nfw_location_access(char* proxy_apps)
{
	int ret = 0;

	ret  =  gps_nfwLocationAcessEnMgt(proxy_apps);
	D("%s entry ret=%d",__func__, ret);
	return ret;
}

bool gps_es_extension_sec_set(unsigned int seconds)
{
	int ret = 0;

	ret = gps_esExtensionSecSetMgt(seconds);
	D("%s entry ret=%d",__func__, ret);
	return ret;
}

static const GpsMeasurementInterface sGpsMeasurementInterface =
{
	.size                 = sizeof(GpsMeasurementInterface),
	.init                 = gps_measurement_init,
	.close                = gps_measurement_close,
};

static const GpsMeasureCorrectionsInterface sGpsMeasureCorrectionsInterface =
{
	.size			= sizeof(GpsMeasureCorrectionsInterface),
	.init				= gps_measurement_corrections_init,
	.setCorrections	= gps_measurement_set_corrections,
};

static const GpsVisibilityControlInterface sGpsVisibilityControlInterface =
{
	.size			= sizeof(GpsVisibilityControlInterface),
	.init				= gps_visibility_control_init,
	.enableNfwLocationAccess	= gps_visibility_control_enable_nfw_location_access,
};

/********the funcitons of sGpsMeasurementInterface list  end********/

/********the funcitons of sGpsNavigationMessageInterface list  start********/
int gps_navigation_messageInit(GpsNavigationMessageCallbacks* callbacks)
{
	int ret = 0;

	D("%s entry callbacks:%p", __func__, callbacks);
	gps_navigationMessageInitMgt(callbacks);

	return ret;
}

void gps_navigation_messageClose(void)
{
	D("%s entry", __func__);
	gps_navigationMessageCloseMgt();
}

static const GpsNavigationMessageInterface sGpsNavigationMessageInterface =
{
	.size                 = sizeof(GpsNavigationMessageInterface),
	.init                 = gps_navigation_messageInit,
	.close                = gps_navigation_messageClose,
};
/********the funcitons of sGpsNavigationMessageInterface list  end********/

/********the funcitons of sGnssConfigurationInterface list  start********/
void gnss_configuration_update (const char* config_data, int32_t length)
{
	D("%s entry length:%d", __func__, length);
	gnss_configurationUpdateMgt(config_data,length);
}


static const GnssConfigurationInterface sGnssConfigurationInterface =
{
	.size                 = sizeof(GnssConfigurationInterface),
	.configuration_update = gnss_configuration_update,
};
/********the funcitons of sGnssConfigurationInterface list  end********/

/********the funcitons of sGnssUnisocExtInterface list  start********/
/*--------------------------------------------------------------------------
Function:  gnss_network_handle_bind
Description:
	network handle bind interface for hidl
Parameters: long int handle_val:  net bind handle value
Return: none
--------------------------------------------------------------------------*/
static void gnss_unisoc_networkHandleBind(long int handle_val)
{
	int ret =0;

	D("%s enter", __func__);
	gnss_unisocNetworkHandleBindMgt(handle_val);

}

static const GnssUnisocExtInterface sGnssUnisocExtInterface =
{
	.size			 = sizeof(GnssUnisocExtInterface),
	.network_handle_bind  = gnss_unisoc_networkHandleBind,
};
/********the funcitons of sGnssUnisocExtInterfacelist  end********/
/*--------------------------------------------------------------------------
Function:  gps_gnsspcRequestData
Description:
	gnsspc tranform data  interface with gps hal
Parameters: int flag:  gnsspc mode flag
              char* pData: gnsspc mode set/get data
Return: 0 success, other's failed.
--------------------------------------------------------------------------*/
int gps_gnsspcRequestData(int flag, char* pData)
{
	int ret = -1;

	D("%s enter", __func__);

	ret = gnsspc_requestDataMgt(flag, pData);
	E("%s:flag = %d   ret = %d.", __func__,flag, ret);

	return ret;
}
/********the funcitons of _GpsInterface list  start********/
static int gps_init(GpsCallbacks* callbacks)
{
	int ret = 0;
	char *error;
	static void *handle;

	D("%s entry  callbacks: %p", __func__, callbacks);
	if(handle == NULL){
		D("%s:  init gnss manager first.", __func__);
#if __LP64__
		handle = dlopen(GNSSMGT64BIT_PATH, RTLD_LAZY);
#else
		handle = dlopen(GNSSMGT32BIT_PATH, RTLD_LAZY);
#endif
		if (!handle)
		{
			E("%s error:%s\n",__func__, dlerror());
			return -1;
		}
		dlerror();    /* Clear any existing error */
		D("%s: dlopen gnssmgt.so success\n", __func__);

		gps_initMgt = dlsym(handle, "gnssmgt_init");
		if((error = dlerror()) != NULL){
			E("%s gps_initMgt error:%s\n",__FUNCTION__,error);
			dlclose(handle);
			return -1;
		}

		gps_startMgt = dlsym(handle, "gnssmgt_start");
		if((error = dlerror()) != NULL){
			E("%s gps_startMgt error:%s\n",__FUNCTION__,error);
			dlclose(handle);
			return -1;
		}

		gps_stopMgt = dlsym(handle, "gnssmgt_stop");
		if((error = dlerror()) != NULL){
			E("%s gps_stopMgt error:%s\n",__FUNCTION__,error);
			dlclose(handle);
			return -1;
		}

		gps_cleanupMgt = dlsym(handle, "gnssmgt_cleanup");
		if((error = dlerror()) != NULL){
			E("%s gps_cleanupMgt error:%s\n",__FUNCTION__,error);
			dlclose(handle);
			return -1;
		}

		gps_injectTimeMgt = dlsym(handle, "gnssmgt_injectTime");
		if((error = dlerror()) != NULL){
			E("%s gps_injectTimeMgt error:%s\n",__FUNCTION__,error);
			dlclose(handle);
			return -1;
		}

		gps_injectLocationMgt = dlsym(handle, "gnssmgt_injectLocation");
		if((error = dlerror()) != NULL){
			E("%s gps_injectLocationMgt error:%s\n",__FUNCTION__,error);
			dlclose(handle);
			return -1;
		}

		gps_injectBestLocationMgt = dlsym(handle, "gnssmgt_injectBestLocation");
		if((error = dlerror()) != NULL){
			E("%s gps_injectLocationMgt error:%s\n",__FUNCTION__,error);
			dlclose(handle);
			return -1;
		}

		gps_deleteAidingDataMgt = dlsym(handle, "gnssmgt_delAidingData");
		if((error = dlerror()) != NULL){
			E("%s gps_deleteAidingDataMgt error:%s\n",__FUNCTION__,error);
			dlclose(handle);
			return -1;
		}

		gps_setPositionModeMgt = dlsym(handle, "gnssmgt_setPosMode");
		if((error = dlerror()) != NULL){
			E("%s gps_setPositionModeMgt error:%s\n",__FUNCTION__,error);
			dlclose(handle);
			return -1;
		}

		gps_xtraInitMgt = dlsym(handle, "gnssmgt_xtra_init");
		if((error = dlerror()) != NULL){
			E("%s gps_xtraInitMgt error:%s\n",__FUNCTION__,error);
			dlclose(handle);
			return -1;
		}

		gps_injectXtraDataMgt = dlsym(handle, "gnssmgt_xtra_injectData");
		if((error = dlerror()) != NULL){
			E("%s gps_injectXtraDataMgt error:%s\n",__FUNCTION__,error);
			dlclose(handle);
			return -1;
		}

		gps_getInternalStateMgt = dlsym(handle, "gnssmgt_dbg_getInternalState");
		if((error = dlerror()) != NULL){
			E("%s gps_getInternalStateMgt error:%s\n",__FUNCTION__,error);
			dlclose(handle);
			return -1;
		}

		assist_gpsInitMgt = dlsym(handle, "gnssmgt_agps_init");
		if((error = dlerror()) != NULL){
			E("%s assist_gpsInitMgt error:%s\n",__FUNCTION__,error);
			dlclose(handle);
			return -1;
		}

		assist_gpsDataConnOpenMgt = dlsym(handle, "gnssmgt_agps_openConn");
		if((error = dlerror()) != NULL){
			E("%s assist_gpsDataConnOpenMgt error:%s\n",__FUNCTION__,error);
			dlclose(handle);
			return -1;
		}

		assist_gpsDataConnClosedMgt = dlsym(handle, "gnssmgt_agps_closeConn");
		if((error = dlerror()) != NULL){
			E("%s assist_gpsDataConnClosedMgt error:%s\n",__FUNCTION__,error);
			dlclose(handle);
			return -1;
		}

		assist_gpsDataConnFailedMgt = dlsym(handle, "gnssmgt_agps_openFailed");
		if((error = dlerror()) != NULL){
			E("%s assist_gpsDataConnFailedMgt error:%s\n",__FUNCTION__,error);
			dlclose(handle);
			return -1;
		}

		assist_gpsSetServerMgt = dlsym(handle, "gnssmgt_agps_setServer");
		if((error = dlerror()) != NULL){
			E("%s assist_gpsSetServerMgt error:%s\n",__FUNCTION__,error);
			dlclose(handle);
			return -1;
		}

		assist_gpsDataConnOpenApnMgt = dlsym(handle, "gnssmgt_agps_openWithApnIpType");
		if((error = dlerror()) != NULL){
			E("%s assist_gpsDataConnOpenApnMgt error:%s\n",__FUNCTION__,error);
			dlclose(handle);
			return -1;
		}

		install_certificatesMgt = dlsym(handle, "gnssmgt_cert_install");
		if((error = dlerror()) != NULL){
			E("%s install_certificatesMgt error:%s\n",__FUNCTION__,error);
			dlclose(handle);
			return -1;
		}

		revoke_certificatesMgt = dlsym(handle, "gnssmgt_cert_revoke");
		if((error = dlerror()) != NULL){
			E("%s revoke_certificatesMgt error:%s\n",__FUNCTION__,error);
			dlclose(handle);
			return -1;
		}

		gps_niInitMgt = dlsym(handle, "gnssmgt_ni_init");
		if((error = dlerror()) != NULL){
			E("%s gps_niInitMgt error:%s\n",__FUNCTION__,error);
			dlclose(handle);
			return -1;
		}

		gps_niRespondMgt = dlsym(handle, "gnssmgt_ni_respond");
		if((error = dlerror()) != NULL){
			E("%s gps_niRespondMgt error:%s\n",__FUNCTION__,error);
			dlclose(handle);
			return -1;
		}

		assist_rilInitMgt = dlsym(handle, "gnssmgt_agps_ril_init");
		if((error = dlerror()) != NULL){
			E("%s assist_rilInitMgt error:%s\n",__FUNCTION__,error);
			dlclose(handle);
			return -1;
		}

		assist_rilRefLocationMgt = dlsym(handle, "gnssmgt_agps_ril_setRefLoc");
		if((error = dlerror()) != NULL){
			E("%s assist_rilRefLocationMgt error:%s\n",__FUNCTION__,error);
			dlclose(handle);
			return -1;
		}

		assist_rilSetIdMgt = dlsym(handle, "gnssmgt_agps_ril_setSetID");
		if((error = dlerror()) != NULL){
			E("%s assist_rilSetIdMgt error:%s\n",__FUNCTION__,error);
			dlclose(handle);
			return -1;
		}

		assist_rilSendNiMessageMgt = dlsym(handle, "gnssmgt_agps_ril_recvNiMsg");
		if((error = dlerror()) != NULL){
			E("%s assist_rilSendNiMessageMgt error:%s\n",__FUNCTION__,error);
			dlclose(handle);
			return -1;
		}

		assist_rilUpdateNetworkStateMgt = dlsym(handle, "gnssmgt_agps_ril_updateNwState");
		if((error = dlerror()) != NULL){
			E("%s assist_rilUpdateNetworkStateMgt error:%s\n",__FUNCTION__,error);
			dlclose(handle);
			return -1;
		}

		assist_rilUpdateNetworkAvailabilityMgt = dlsym(handle, "gnssmgt_agps_ril_updateNwAvailability");
		if((error = dlerror()) != NULL){
			E("%s assist_rilUpdateNetworkAvailabilityMgt error:%s\n",__FUNCTION__,error);
			dlclose(handle);
			return -1;
		}

		gps_geofenceInitMgt = dlsym(handle, "gnssmgt_geo_init");
		if((error = dlerror()) != NULL){
			E("%s gps_geofenceInitMgt error:%s\n",__FUNCTION__,error);
			dlclose(handle);
			return -1;
		}

		gps_geofenceAddAreaMgt = dlsym(handle, "gnssmgt_geo_addArea");
		if((error = dlerror()) != NULL){
			E("%s gps_geofenceAddAreaMgt error:%s\n",__FUNCTION__,error);
			dlclose(handle);
			return -1;
		}

		gps_geofencePauseMgt = dlsym(handle, "gnssmgt_geo_pause");
		if((error = dlerror()) != NULL){
			E("%s gps_geofencePauseMgt error:%s\n",__FUNCTION__,error);
			dlclose(handle);
			return -1;
		}

		gps_geofenceResumeMgt = dlsym(handle, "gnssmgt_geo_resume");
		if((error = dlerror()) != NULL){
			E("%s gps_geofenceResumeMgt error:%s\n",__FUNCTION__,error);
			dlclose(handle);
			return -1;
		}

		gps_geofenceRemoveAreaMgt = dlsym(handle, "gnssmgt_geo_removeArea");
		if((error = dlerror()) != NULL){
			E("%s gps_geofenceRemoveAreaMgt error:%s\n",__FUNCTION__,error);
			dlclose(handle);
			return -1;
		}

		gps_measurementInitMgt = dlsym(handle, "gnssmgt_meas_init");
		if((error = dlerror()) != NULL){
			E("%s gps_measurementInitMgt error:%s\n",__FUNCTION__,error);
			dlclose(handle);
			return -1;
		}

		gps_measurementCloseMgt = dlsym(handle, "gnssmgt_meas_close");
		if((error = dlerror()) != NULL){
			E("%s gps_measurementCloseMgt error:%s\n",__FUNCTION__,error);
			dlclose(handle);
			return -1;
		}

		gps_navigationMessageInitMgt = dlsym(handle, "gnssmgt_navi_msg_init");
		if((error = dlerror()) != NULL){
			E("%s gps_navigationMessageInitMgt error:%s\n",__FUNCTION__,error);
			dlclose(handle);
			return -1;
		}

		gps_navigationMessageCloseMgt = dlsym(handle, "gnssmgt_navi_msg_close");
		if((error = dlerror()) != NULL){
			E("%s gps_navigationMessageCloseMgt error:%s\n",__FUNCTION__,error);
			dlclose(handle);
			return -1;
		}

		gnss_configurationUpdateMgt = dlsym(handle, "gnssmgt_cfg_update");
		if((error = dlerror()) != NULL){
			E("%s gnss_configurationUpdateMgt error:%s\n",__FUNCTION__,error);
			dlclose(handle);
			return -1;
		}

		gnss_unisocNetworkHandleBindMgt = dlsym(handle, "gnssmgt_unisoc_bindNwHandle");
		if((error = dlerror()) != NULL){
			E("%s gnss_unisocNetworkHandleBindMgt error:%s\n",__FUNCTION__,error);
			dlclose(handle);
			return -1;
		}

          	gnsspc_requestDataMgt = dlsym(handle, "gnssmgt_handlePcRequest");
		if((error = dlerror()) != NULL){
			E("%s gnsspc_requestDataMgt error:%s\n",__FUNCTION__,error);
			dlclose(handle);
			return -1;
		}

		gps_visibilityCtrl_initMgt = dlsym(handle, "gnssmgt_gnssVisibilityCtrl_init");
		if((error = dlerror()) != NULL){
			E("%s gps_visibilityCtrl_initMgt error:%s\n",__FUNCTION__,error);
			dlclose(handle);
			return -1;
		}

		gps_nfwLocationAcessEnMgt = dlsym(handle, "gnssmgt_nfwLocationAccess_enable");
		if((error = dlerror()) != NULL){
			E("%s gps_nfwLocationAcessEnMgt error:%s\n",__FUNCTION__,error);
			dlclose(handle);
			return -1;
		}

		gps_esExtensionSecSetMgt = dlsym(handle, "gnssmgt_esExtensionSec_set");
		if((error = dlerror()) != NULL){
			E("%s gps_esExtensionSecSetMgt error:%s\n",__FUNCTION__,error);
			dlclose(handle);
			return -1;
		}

		gps_measurementCorrectionsInitMgt = dlsym(handle, "gnssmgt_meas_corr_init");
		if((error = dlerror()) != NULL){
			E("%s gps_measurementCorrectionsInitMgt error:%s\n",__FUNCTION__,error);
			dlclose(handle);
			return -1;
			}
		gps_measurementSetCorrectionsMgt = dlsym(handle,"gnssmgt_injectMeasurementCorrections");
		if((error = dlerror()) != NULL){
			E("%s gps_measurementSetCorrectionsMgt error:%s\n",__FUNCTION__,error);
			dlclose(handle);
			return -1;
			}

	}
	ret = gps_initMgt(callbacks);

	return ret;
}

static int gps_start(void)
{

	int ret = 0;

	D("%s entry", __func__);
	ret = gps_startMgt();

	return ret;
}

static int gps_stop(void)
{
	int ret = 0;

	D("%s entry", __func__);
	ret = gps_stopMgt();

	return ret;
}

static void  gps_cleanup(void)
{
	D("%s entry", __func__);
	gps_cleanupMgt();
}

static int gps_injectTime(GpsUtcTime time, int64_t timeReference, int uncertainty)
{
	int ret = 0;

	D("%s entry  time:%lld,timeref= %lld, uncert =%d", __func__,\
		(long long int)time,(long long int)timeReference,uncertainty);
	ret = gps_injectTimeMgt(time,timeReference,uncertainty);

	return ret;
}

static int gps_injectLocation(double latitude, double longitude, float accuracy)
{
	int ret = 0;

	D("%s entry  latitude = %g,longitude= %g accuracy =%f\n",__func__,\
			latitude,longitude,accuracy);
	ret = gps_injectLocationMgt(latitude,longitude,accuracy);

	return ret;
}
static int gps_injectBestLocation(GnssLocation* GnssLocation)
{
	int ret = 0;
	D("%s enter", __func__);
	ret = gps_injectBestLocationMgt(GnssLocation);
	D("latitude=%f, longitude=%f, timestampNs=%ld",
		    GnssLocation->latitude,
		    GnssLocation->longitude,
		    GnssLocation->elapsedRealtime.timestampNs);

	return ret;
}

static void gps_deleteAidingData(GpsAidingData flags)
{
	D("%s entry  flags: %d", __func__, flags);
	gps_deleteAidingDataMgt(flags);
}

static int gps_setPositionMode(GpsPositionMode mode, GpsPositionRecurrence recurrence,
		uint32_t min_interval, uint32_t preferred_accuracy, uint32_t preferred_time)
{
	int ret = 0;

	D("%s entry  mode: %d recurrence: %d interval: %d accuracy: %d time: %d ",\
		__func__, mode, recurrence, min_interval, preferred_accuracy, preferred_time);
	ret = gps_setPositionModeMgt(mode,recurrence,min_interval,preferred_accuracy,preferred_time);

	return ret;
}
const void* gps_getExtension(const char* name)
{
	void* ret = NULL;

	D("%s entry", __func__);
	if(name == NULL){
		D("%s entry, name is NULL", __func__);
		return ret;
	}

	if (strcmp(name, AGPS_INTERFACE) == 0) {
		D("Set agps callback\n");
		return &sAssistGpsInterface;
	}

	if (strcmp(name, SUPL_CERTIFICATE_INTERFACE) == 0) {
		D("Set supl certificate callback\n");
		return &sSuplCertificateInterface;
	}

	if (strcmp(name, GPS_NI_INTERFACE) == 0) {
		D("Set NI callback\n");
		return &sGpsNiInterface;
	}

	if (strcmp(name,AGPS_RIL_INTERFACE) == 0) {
		D("Set ril callback\n");
		return &sAssistGpsRilInterface;
	}

	if (strcmp(name,GPS_XTRA_INTERFACE) == 0) {
		D("Set xtra callback\n");
		return &sGpsXtraInterface;
	}

	if (strcmp(name,GPS_GEOFENCING_INTERFACE) == 0) {
		D("Set geofence callback\n");
		return &sGpsGeoInterface;
	}

	if (strcmp(name,GPS_DEBUG_INTERFACE) == 0) {
		D("Set debug callback\n");
		return &sGpsDebugInterface;
	}

	if (strcmp(name,GNSS_CONFIGURATION_INTERFACE) == 0) {
		D("set configuration callback\n");
		return &sGnssConfigurationInterface;
	}

	if (strcmp(name,GPS_MEASUREMENT_INTERFACE) == 0) {
		D("set measurement callback\n");
		return &sGpsMeasurementInterface;
	}

	if (strcmp(name,GPS_NAVIGATION_MESSAGE_INTERFACE) == 0) {
		D("set navigation message callback\n");
		return &sGpsNavigationMessageInterface;
	}

	if (strcmp(name,GNSS_UNISOC_EXT_INTERFACE) == 0) {
		D("set unisoc ext interface\n");
		return &sGnssUnisocExtInterface;
	}

	if (strcmp(name,GNSS_MEASUREMENT_CORRECTIONS_INTERFACE) == 0) {
		D("set measurement correction interface\n");
		return &sGpsMeasureCorrectionsInterface;
	}

	if (strcmp(name,GNSS_VISIBILITY_CONTROL_INTERFACE) == 0) {
		D("set visibility control interface\n");
		return &sGpsVisibilityControlInterface;
	}

	return ret;
}

static const GpsInterface  _GpsInterface = {
                size:           sizeof(GpsInterface),
			    init: 			gps_init,
			    start:			gps_start,
			    stop:			gps_stop,
			    cleanup:			gps_cleanup,
			    inject_time:		gps_injectTime,
			    inject_location:		gps_injectLocation,
			    inject_best_location:		gps_injectBestLocation,
			    delete_aiding_data:	gps_deleteAidingData,
			    set_position_mode:	gps_setPositionMode,
			    get_extension: 		gps_getExtension,
};
/********the funcitons of _GpsInterface list  end********/

const GpsInterface* gps_get_hardware_interface(struct gps_device_t* dev)
{
	D("Call gps_get_hardware_interface %p",dev);
	return &_GpsInterface;
}

static int open_gps(const struct hw_module_t* module, char const* name,
		struct hw_device_t** device)
{
	D("Call open_gps name = %s",name);
	struct gps_device_t *gps_device = malloc(sizeof(struct gps_device_t));
	if (gps_device)
	{
		memset(gps_device, 0, sizeof(struct gps_device_t));
		gps_device->common.tag		  = HARDWARE_DEVICE_TAG;
		gps_device->common.version	  = 0;
		gps_device->common.module	  = (struct hw_module_t*)module;
		gps_device->get_gps_interface = gps_get_hardware_interface;

		*device = (struct hw_device_t*)gps_device;
		return 0;
	}
	return 1;
}

static struct hw_module_methods_t hw_module_methods = {
	.open = open_gps
};

struct hw_module_t HAL_MODULE_INFO_SYM = {
	.tag = HARDWARE_MODULE_TAG,
	.version_major = 1,
	.version_minor = 0,
	.id = GPS_HARDWARE_MODULE_ID,
	.name = "UNISOC Module",
	.author = "UNISOC",
	.methods = &hw_module_methods,
};

