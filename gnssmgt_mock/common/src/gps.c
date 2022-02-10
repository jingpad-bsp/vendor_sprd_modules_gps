#include <stdio.h>
#include <inttypes.h>
#include <dlfcn.h>
#include <fcntl.h>
#include <sys/epoll.h>
#include <sys/wait.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <math.h>
#include <cutils/sockets.h>
#include <cutils/properties.h>
#include <termios.h>
#include <hardware_legacy/power.h>
#include <inttypes.h>
#include <sys/un.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <log/log.h>
#include <errno.h>

#include "gnssmgt_api.h"

GpsState  _gps_state[1];
GpsStatus Mockgstate;
GpsLocation MockLocation;
GnssSvStatus MockSvStatus;
GpsSvStatus MockgpsSvStatus;
GpsUtcTime Mocktimestamp;
const char* MockNmea;
int MockLength;
GnssSystemInfo Mockinfo;
GpsNiNotification Mocknotification;
uint32_t Mockflags;
uint32_t Mockcapabilities;
int Mockcapability;
AGpsStatus MockStatus;
int32_t MockGeofenceId;
int32_t MockTransition;
int32_t MockGeostatus;
GpsData MockGpsData;
GnssData MockGnssData;
GnssNavigationMessage MockMessage;
GnssNfwNotification MockVisnotification;

#ifdef LOG_TAG
#undef LOG_TAG
#endif

#define  LOG_TAG  "GNSSMGT"

#define DBG(...)    ALOGD(__VA_ARGS__)
#define DBGE(...)   ALOGE(__VA_ARGS__)

/*****************************GNSS接口*************************************/

int gnssmgt_init(GpsCallbacks *callbacks)
{
	GpsState*  s = _gps_state;

	DBG("%s: enter, callbacks[%p]",__FUNCTION__, callbacks);


	if (NULL == callbacks)
	{
		DBGE("%s: callbacks is NULL", __FUNCTION__);
		return -1;
	}else
	{
		if (callbacks->size != sizeof(GpsCallbacks))
		{
			DBGE("%s: data is mismatched! input size(%zu), local size(%zu)",
	                __FUNCTION__, callbacks->size, sizeof(GpsCallbacks));
			return -1;
		}
	}
	memset(s,0,sizeof(GpsState));
	s->callbacks = *callbacks;
	//能力值上报
	Mockcapabilities = 215;
	s->callbacks.set_capabilities_cb(Mockcapabilities);
	DBG("%s: set_capabilities_cb end",__FUNCTION__);
	s->init = 1;

	return 0;
}

int gnssmgt_start(void)
{
	GpsState* s = _gps_state;

	DBG("%s: enter", __FUNCTION__);

	if (!s->init)
	{
		DBGE("%s: called with uninitialized state", __FUNCTION__);
	}else
	{
		//位置仿真数据
		MockLocation.size = sizeof(GpsLocation);
		MockLocation.flags = 1;
		MockLocation.latitude = 407838351;
		MockLocation.longitude = -746143763;
		MockLocation.altitude = 1;
		MockLocation.speed = 1;
		MockLocation.bearing = 1;
		MockLocation.accuracy = 1;
		MockLocation.timestamp = 1;
		MockLocation.veraccuracy = 1;
		MockLocation.speedaccuracy = 1;
		MockLocation.bearaccuracy = 1;
		s->callbacks.location_cb(&MockLocation);
		DBG("%s: location_cb end",__FUNCTION__);

		//状态仿真数据
		Mockgstate.size = sizeof(GpsStatus);
		Mockgstate.status = GPS_STATUS_SESSION_BEGIN;
		s->callbacks.status_cb(&Mockgstate);
		DBG("%s: status_cb end",__FUNCTION__);

		//卫星状态仿真
		MockSvStatus.size = sizeof(GnssSvStatus);
		MockSvStatus.num_svs = 1;
		MockSvStatus.gnss_sv_list[0].svid = 22;
		MockSvStatus.gnss_sv_list[0].constellation = 3;
		MockSvStatus.gnss_sv_list[0].c_n0_dbhz = 26.000000;
		MockSvStatus.gnss_sv_list[0].elevation = 0.000000;
		MockSvStatus.gnss_sv_list[0].azimuth = 0.000000;
		MockSvStatus.gnss_sv_list[0].flags = 11;
		MockSvStatus.gnss_sv_list[0].carrier_freq = 1600312448.000000;
		s->callbacks.gnss_sv_status_cb(&MockSvStatus);
		DBG("%s: gnss_sv_status_cb end",__FUNCTION__);
		//gps卫星状态仿真
		MockgpsSvStatus.size = sizeof(GpsSvStatus);
		MockgpsSvStatus.num_svs = 1;
		MockgpsSvStatus.sv_list[0].prn = 1;
		MockgpsSvStatus.sv_list[0].snr = 1.000000;
		MockgpsSvStatus.sv_list[0].elevation = 1.000000;
		MockgpsSvStatus.sv_list[0].azimuth = 1.000000;
		MockgpsSvStatus.ephemeris_mask = 1;
		MockgpsSvStatus.almanac_mask = 1;
		MockgpsSvStatus.used_in_fix_mask = 1;
		s->callbacks.sv_status_cb(&MockgpsSvStatus);
		DBG("%s: sv_status_cb end",__FUNCTION__);
		//NMEA数据仿真
		Mocktimestamp = 25095664;
		MockNmea = "test";
		MockLength = 51;
		s->callbacks.nmea_cb(Mocktimestamp,MockNmea,MockLength);
		DBG("%s: nmea_cb end",__FUNCTION__);
		//UTC时间上报
		s->callbacks.request_utc_time_cb();
		DBG("%s: request_utc_time_cb end",__FUNCTION__);
		/*
		//系统信息
		Mockinfo.year_of_hw = 2019;
		s->callbacks.set_system_info_cb(&Mockinfo);
		*/
	}
	return 0;
}

int gnssmgt_stop(void)
{
	GpsState*  s = _gps_state;

	 if (!s->init)
	{
		DBGE("%s: gps is not init yet!", __FUNCTION__);
		return -1;
	}else
	{
		//状态信息上报
		Mockgstate.size = sizeof(GpsStatus);
		Mockgstate.status = GPS_STATUS_SESSION_END;
		s->callbacks.status_cb(&Mockgstate);
		DBG("%s: status_cb end Mockgstate.status = %d", __func__,Mockgstate.status);
	}
	return 0;
}


void gnssmgt_cleanup(void)
{
	GpsState*  s = _gps_state;

	DBG("%s: enter", __FUNCTION__);

	if (!s->init)
	{
		DBGE("%s: gps is not init yet!", __FUNCTION__);
		return;
	}else
	{
		//状态信息上报
		Mockgstate.size = sizeof(GpsStatus);
		Mockgstate.status = GPS_STATUS_ENGINE_OFF;
		s->callbacks.status_cb(&Mockgstate);
		DBG("%s: status_cb end Mockgstate.status = %d", __func__,Mockgstate.status);
	}
	s->init = 0;
}

int gnssmgt_injectTime(
      GpsUtcTime gpsTime, int64_t refTime, int uncertainty)
{
	DBG("%s: enter, gpsTime[%lld], refTime[%lld], uncertainty[%d]",
	__FUNCTION__, (long long int)gpsTime, (long long int)refTime, uncertainty);

	if((gpsTime == 0) && (refTime == 0) && (uncertainty == 0))
	{
		DBGE("%s: Incorrect value of incoming parameter", __FUNCTION__);
		return -1;
	}

	return 0;
}

int gnssmgt_injectLocation(
        double latitude, double longitude, float accuracy)
{
	DBG("%s: enter, latitude[%f], longitude[%f], accuracy[%f]",
		__FUNCTION__, latitude,longitude,accuracy);
	return 0;
}

int gnssmgt_injectBestLocation(GnssLocation* pLoc)
{
	DBG("%s: enter, pLoc[%p]", __FUNCTION__, pLoc);
	if(pLoc == NULL)
	{
		DBGE("%s: input NULL param", __FUNCTION__);
		return -1;
	}
	return 0;
}

void gnssmgt_delAidingData(GpsAidingData aidingData)
{
	DBG("%s: enter, aidingData[%d]", __FUNCTION__, aidingData);
	return ;
}

int gnssmgt_setPosMode(
        GpsPositionMode         mode,
        GpsPositionRecurrence   recurrence,
        uint32_t                minInterval,
        uint32_t                prefAccuracy,
        uint32_t                prefTime)
{
	DBG("%s: enter, mode[%u], recurrence[%u], minInterval[%u], prefAccuracy[%u],prefTime[%u]",
               __FUNCTION__, mode, recurrence, minInterval, prefAccuracy, prefTime);

	GpsState*  s = _gps_state;

	if (!s->init)
	{
		DBGE("%s: gps is not init yet!", __FUNCTION__);
		return -1;
	}else
	{
		if ((mode != 0) && (mode != 1))
		{
			DBGE("%s: Mode value is an illegal value", __FUNCTION__);
			return -1;
		}
		if ((recurrence != 0) && (recurrence != 1))
		{
			DBGE("%s: recurrence value is an illegal value", __FUNCTION__);
			return -1;
		}
	}
	return 0;
}

/*****************************AGNSS接口*************************************/

void gnssmgt_agps_init(AGpsCallbacks* callbacks)
{
	GpsState* s = _gps_state;

	DBG("%s: enter, callbacks[%p]",__FUNCTION__, callbacks);

	if (callbacks == NULL)
	{
		DBGE("%s: callbacks is NULL", __FUNCTION__);
		return ;
	}

	//回调函数注册
	s->agps_callbacks = *callbacks;
	//agps状态上报
	MockStatus.size = sizeof(AGpsStatus);
	MockStatus.type = AGPS_TYPE_SUPL;
	MockStatus.status = GPS_REQUEST_AGPS_DATA_CONN;
	MockStatus.ipaddr = 0;
	s->agps_callbacks.status_cb(&MockStatus);
	DBG("%s: status_cb end",__FUNCTION__);
	return ;
}

int gnssmgt_agps_openConn(const char* apn)
{
	DBG("%s: enter, apn[%s]",__FUNCTION__, apn);

	if (apn == NULL)
	{
		DBG("%s: apn is NULL", __FUNCTION__);
		return -1;
	}
	return 0;
}

int gnssmgt_agps_closeConn(void)
{
	DBG("%s: enter", __FUNCTION__);
	return 0;
}

int gnssmgt_agps_openFailed(void)
{
	DBG("%s: enter",__FUNCTION__);
	return 0;
}

int gnssmgt_agps_setServer(
    AGpsType type, const char* hostname, int port)
{
	DBG("%s: enter, type[%d], hostname[%s], port[%d]",__FUNCTION__, type, hostname, port);

	if (hostname == NULL)
	{
		DBGE("%s: hostname is null", __FUNCTION__);
		return -1;
	}

	return 0;
}

int gnssmgt_agps_openWithApnIpType(
	const char *apn, ApnIpType apnIpType)
{
	DBG("%s: enter, apn[%s], apnIpType[%d]", __FUNCTION__,apn, apnIpType);

	if (apn == NULL)
	{
		DBGE("%s: apn is a null value ", __FUNCTION__);
		return -1;
	}
	return 0;
}

int gnssmgt_agps_openWithApnIpTypeEx(
	long long int networkHandle,
	const char *apn,
	ApnIpType apnIpType)
{
	DBG("%s: enter, networkHandle[%lld], apn[%s], apnIpType[%d]", __FUNCTION__, networkHandle, apn, apnIpType);

	if (apn == NULL)
	{
		DBGE("%s: apn is a null value ", __FUNCTION__);
		return -1;
	}

	return 0;
}

/*****************************ni接口*************************************/
void gnssmgt_ni_init(GpsNiCallbacks *callbacks)
{
	GpsState*  s = _gps_state;

	DBG("%s: enter, callbacks[%p]",__FUNCTION__, callbacks);

	if (callbacks == NULL)
	{
		DBGE("%s: callbacks is NULL", __FUNCTION__);
		return ;
	}

	//回调函数注册
	s->ni_callbacks = *callbacks;
	//ni通知上报
	Mocknotification.size = sizeof(GpsNiNotification);
	Mocknotification.notification_id = 1;
	Mocknotification.ni_type = 2;
	Mocknotification.notify_flags = 3;
	Mocknotification.timeout = 0;
	Mocknotification.default_response = 4;
	Mocknotification.requestor_id[0] = 'A';
	Mocknotification.text[0] = 'B';
	Mocknotification.requestor_id_encoding = 5;
	Mocknotification.extras[0] = 'C';
	s->ni_callbacks.notify_cb(&Mocknotification);
	DBG("%s: notify_cb end",__FUNCTION__);
}

void gnssmgt_ni_respond(int notifID, GpsUserResponseType resp)
{
	DBG("%s: enter, notifID[%d], resp[%d]", __FUNCTION__, notifID, resp);
	return ;
}

/*****************************AGnssRil接口*************************************/

void gnssmgt_agps_ril_init(AGpsRilCallbacks *callbacks)
{
	GpsState *s = _gps_state;

	DBG("%s: enter, callbacks[%p]",__FUNCTION__, callbacks);

	if (callbacks == NULL)
	{
		DBGE("%s: callbacks is NULL", __FUNCTION__);
		return ;
	}

	//回调函数注册
	s->agps_ril_callbacks = *callbacks;
	//请求SETid
	Mockflags = AGPS_SETID_TYPE_MSISDN;
	s->agps_ril_callbacks.request_setid(Mockflags);
	DBG("%s: request_setid end",__FUNCTION__);
	//请求参考位置
	s->agps_ril_callbacks.request_refloc(0);
	DBG("%s: request_refloc end",__FUNCTION__);
}

void gnssmgt_agps_ril_setRefLoc(
        const AGpsRefLocation *refLocation, size_t szStruct)
{
	DBG("%s: enter, refLocation[%p], szStruct[%d]",__FUNCTION__, refLocation, szStruct);

	if (!refLocation)
	{
		DBGE("%s: refLocation is NULL", __FUNCTION__);
		return ;
	}
	if (szStruct != sizeof(AGpsRefLocation))
	{
		DBGE("%s: size of AGpsRefLocation is not correct", __FUNCTION__);
		return ;
	}
}

void gnssmgt_agps_ril_setSetID(AGpsSetIDType type, const char *setid)
{
	DBG("%s: enter, type[%d], setid[%s]",__FUNCTION__, type, setid);

	if (setid == NULL)
	{
		DBGE("%s: setid  is NULL", __FUNCTION__);
		return ;
	}
	if (type > AGPS_SETID_TYPE_MSISDN)
	{
		DBGE("%s: Incorrect value of incoming parameter", __FUNCTION__);
		return ;
	}

}

void gnssmgt_agps_ril_recvNiMsg(uint8_t *msg, size_t len)
{
	DBG("%s: enter, msg[%p], len[%d]", __FUNCTION__, msg, len);

	if (!msg)
	{
		DBGE("%s: msg is NULL", __FUNCTION__);
	}
	return ;
}

void gnssmgt_agps_ril_updateNwState(
        int connected, int type, int roaming, const char* extraInfo)
{
	DBG("%s: enter, type[%d],roaming[%d], extra[%s]",
	    __FUNCTION__,type,roaming,extraInfo);

	if (extraInfo == NULL)
	{
		DBGE("%s: extraInfo is NULL", __FUNCTION__);
		return ;
	}

	return ;
}

void gnssmgt_agps_ril_updateNwAvailability(
        int avaiable, const char *apn)
{
	DBGE("%s: enter, network avaiable[%d],apn[%s]",__FUNCTION__,avaiable,apn);

	if (apn == NULL)
	{
		DBGE("%s: apn is NULL ", __FUNCTION__);
		return ;
	}
	return ;
}

/*****************************GnssXtra接口*************************************/

int gnssmgt_xtra_init(GpsXtraCallbacks *callbacks)
{
	GpsState*  s = _gps_state;

	DBG("%s: enter, callbacks[%p]",__FUNCTION__, callbacks);

	if (callbacks == NULL)
	{
		DBGE("%s: callbacks is NULL", __FUNCTION__);
		return -1;
	}

	//回调函数注册
	s->xtra_callbacks = *callbacks;
	s->xtra_callbacks.download_request_cb();
	DBG("%s: download_request_cb end",__FUNCTION__);
	return 0;
}

int gnssmgt_xtra_injectData(char *data, int length)
{
	DBG("%s: enter, data[%s], length[%d]",__FUNCTION__, data, length);

	if (data == NULL)
	{
		DBGE("%s: data is NULL", __FUNCTION__);
		return -1;
	}
	return 0;
}

/*****************************GnssGeofencing接口*************************************/

void gnssmgt_geo_init(GpsGeofenceCallbacks *callbacks)
{
	GpsState*  s = _gps_state;

	DBG("%s: enter, callbacks[%p]",__FUNCTION__, callbacks);

	if (callbacks == NULL)
	{
		DBGE("%s: callbacks is NULL", __FUNCTION__);
		return ;
	}

	//回调函数注册
	s->geofence_callbacks= *callbacks;

	//上报GEOFEN状态及ID相关信息
	MockGeofenceId = 1;
	MockTransition = 1;
	MockGeostatus = 1;
	s->geofence_callbacks.geofence_transition_callback(MockGeofenceId, &MockLocation, MockTransition, Mocktimestamp);
	DBG("%s: geofence_transition_callback end",__FUNCTION__);
	s->geofence_callbacks.geofence_status_callback(MockGeostatus, &MockLocation);
	DBG("%s: geofence_status_callback end",__FUNCTION__);
	s->geofence_callbacks.geofence_add_callback(MockGeofenceId, MockGeostatus);
	DBG("%s: geofence_add_callback end",__FUNCTION__);
	s->geofence_callbacks.geofence_remove_callback(MockGeofenceId, MockGeostatus);
	DBG("%s: geofence_remove_callback end",__FUNCTION__);
	s->geofence_callbacks.geofence_pause_callback(MockGeofenceId, MockGeostatus);
	DBG("%s: geofence_pause_callback end",__FUNCTION__);
	s->geofence_callbacks.geofence_resume_callback(MockGeofenceId, MockGeostatus);
	DBG("%s: geofence_resume_callback end",__FUNCTION__);

	return ;
}

void gnssmgt_geo_addArea(
     int32_t geofence_id,
     double  latitude,
     double  longitude,
     double  radius_meters,
     int     last_transition,
     int     monitor_transitions,
     int     notification_responsiveness_ms,
     int     unknown_timer_ms)
{
	DBG("%s: enter, geofence_id[%d], latitude[%f], last_transition[%d]",
            __FUNCTION__, geofence_id, latitude,last_transition);
	return ;
}

void gnssmgt_geo_pause(int32_t geofence_id)
{
	DBG("%s: enter, geofence_id[%d]",__FUNCTION__, geofence_id);
	return ;
}

void gnssmgt_geo_resume(int32_t geofence_id, int monitor_transitions)
{
	DBG("%s: enter, geofence_id[%d], monitor_transitions[%d]",__FUNCTION__, geofence_id, monitor_transitions);
	return ;
}

void gnssmgt_geo_removeArea(int32_t geofence_id)
{
	DBG("%s: enter, geofence_id[%d]",__FUNCTION__, geofence_id);
	return ;
}

/*****************************GnssDebug接口*************************************/
size_t gnssmgt_dbg_getInternalState(char *buffer, size_t bufferSize)
{
	DBG("%s: enter, buffer[%s], bufferSize[%zu]",__FUNCTION__, buffer,bufferSize);
	if (buffer == NULL)
	{
		DBGE("%s: buffer is NULL", __FUNCTION__);
		return -1;
	}
	return 0;
}

/*****************************GnssConfiguration接口*************************************/

void gnssmgt_cfg_update(const char *config_data, int32_t length)
{
	DBG("%s: enter, config_data[%s], length[%d]", __FUNCTION__, config_data, length);
	if (config_data == NULL)
	{
		DBGE("%s: config_data is NULL", __FUNCTION__);
		return ;
	}
	return ;
}

/*****************************GnssMeasurement接口*************************************/

int gnssmgt_meas_init(GpsMeasurementCallbacks *callbacks)
{
	GpsState*  s = _gps_state;

	DBG("%s: enter, callbacks[%p]",__FUNCTION__, callbacks);

	if(callbacks == NULL)
	{
		DBGE("%s: callbacks is NULL", __FUNCTION__);
		return GPS_MEASUREMENT_ERROR_GENERIC;
	}else
	{
		if (callbacks->size != sizeof(GpsMeasurementCallbacks))
		{
			DBGE("%s: data is mismatched! input size(%zu), local size(%zu)",
	                __FUNCTION__, callbacks->size, sizeof(GpsMeasurementCallbacks));
			return GPS_MEASUREMENT_ERROR_GENERIC;
		}
	}
	if(s->measurement_callbacks.measurement_callback ||
	s->measurement_callbacks.gnss_measurement_callback)
	{
		DBGE("%s: measurement_callback[%p] or gnss_measurement_callback[%p] has already init",
		__FUNCTION__,
		s->measurement_callbacks.measurement_callback,
		s->measurement_callbacks.gnss_measurement_callback);

		return GPS_MEASUREMENT_ERROR_ALREADY_INIT;
	}
	//注册回调函数
	s->measurement_callbacks = *callbacks;
	DBG("%s: callback OK",__FUNCTION__);
	//上报观测量信息
	MockGpsData.size = sizeof(GpsData);
	MockGpsData.measurement_count = 1;
	MockGpsData.measurements[0].size = sizeof(GpsMeasurement);
	MockGpsData.measurements[0].flags = 41473;
	MockGpsData.measurements[0].prn = 98;
	MockGpsData.measurements[0].time_offset_ns = 0.000000;
	MockGpsData.measurements[0].state = 3;
	MockGpsData.measurements[0].received_gps_tow_ns = 0;
	MockGpsData.measurements[0].received_gps_tow_uncertainty_ns = 0;
	MockGpsData.measurements[0].c_n0_dbhz = 15.000000;
	MockGpsData.measurements[0].pseudorange_rate_mps = -75.083662;
	MockGpsData.measurements[0].pseudorange_rate_uncertainty_mps = 99999999.000000;
	MockGpsData.measurements[0].accumulated_delta_range_state = 0;
	MockGpsData.measurements[0].accumulated_delta_range_m = 0.000000;
	MockGpsData.measurements[0].accumulated_delta_range_uncertainty_m = 0.000000;
	MockGpsData.measurements[0].pseudorange_m = 0.000000;
	MockGpsData.measurements[0].pseudorange_uncertainty_m = 0.000000;
	MockGpsData.measurements[0].code_phase_chips = 0.000000;
	MockGpsData.measurements[0].code_phase_uncertainty_chips = 0.000000;
	MockGpsData.measurements[0].carrier_frequency_hz = 1600875008.000000;
	MockGpsData.measurements[0].carrier_cycles = 37191817;
	MockGpsData.measurements[0].carrier_phase = 0.153730;
	MockGpsData.measurements[0].carrier_phase_uncertainty = 0.000000;
	MockGpsData.measurements[0].loss_of_lock = 0;
	MockGpsData.measurements[0].bit_number = 0;
	MockGpsData.measurements[0].time_from_last_bit_ms = 0;
	MockGpsData.measurements[0].doppler_shift_hz = 0.000000;
	MockGpsData.measurements[0].doppler_shift_uncertainty_hz = 0.000000;
	MockGpsData.measurements[0].multipath_indicator = 0;
	MockGpsData.measurements[0].snr_db = 0.000000;
	MockGpsData.measurements[0].elevation_deg = 0.000000;
	MockGpsData.measurements[0].elevation_uncertainty_deg = 0.000000;
	MockGpsData.measurements[0].azimuth_deg = 0.000000;
	MockGpsData.measurements[0].azimuth_uncertainty_deg = 0.000000;
	MockGpsData.measurements[0].used_in_fix = true;
	MockGpsData.clock.size = sizeof(GpsClock);
	MockGpsData.clock.flags = 0;
	MockGpsData.clock.leap_second = 18;
	MockGpsData.clock.type = 2;
	MockGpsData.clock.time_ns = 365163457929441;
	MockGpsData.clock.time_uncertainty_ns = 0;
	MockGpsData.clock.full_bias_ns = 0;
	MockGpsData.clock.bias_ns = 0.000000;
	MockGpsData.clock.bias_uncertainty_ns = 0.000000;
	MockGpsData.clock.drift_nsps = 0.000000;
	MockGpsData.clock.drift_uncertainty_nsps = 0.000000;
	s->measurement_callbacks.measurement_callback(&MockGpsData);
	DBG("%s: measurement_callback end",__FUNCTION__);
	//上报观测量信息
	MockGnssData.size = sizeof(GnssData);
	MockGnssData.measurement_count = 1;
	MockGnssData.measurements[0].size = sizeof(GnssMeasurement);
	MockGnssData.measurements[0].flags = 41473;
	MockGnssData.measurements[0].svid = 98;
	MockGnssData.measurements[0].constellation = GNSS_CONSTELLATION_GLONASS;
	MockGnssData.measurements[0].time_offset_ns = 0.000000;
	MockGnssData.measurements[0].state = 3;
	MockGnssData.measurements[0].received_sv_time_in_ns = 0;
	MockGnssData.measurements[0].received_sv_time_uncertainty_in_ns = 0;
	MockGnssData.measurements[0].c_n0_dbhz = 15.000000;
	MockGnssData.measurements[0].pseudorange_rate_mps = -75.083662;
	MockGnssData.measurements[0].pseudorange_rate_uncertainty_mps = 99999999.000000;
	MockGnssData.measurements[0].accumulated_delta_range_state = 0;
	MockGnssData.measurements[0].accumulated_delta_range_m = 0.000000;
	MockGnssData.measurements[0].accumulated_delta_range_uncertainty_m = 0.000000;
	MockGnssData.measurements[0].carrier_frequency_hz = 1600875008.000000;
	MockGnssData.measurements[0].carrier_cycles = 37191817;
	MockGnssData.measurements[0].carrier_phase = 0.153730;
	MockGnssData.measurements[0].carrier_phase_uncertainty = 0.000000;
	MockGnssData.measurements[0].multipath_indicator = 0;
	MockGnssData.measurements[0].snr_db = 0.000000;
	MockGnssData.measurements[0].agcLevelDb = 0.000000;
	MockGnssData.clock.size = sizeof(GnssClock);
	MockGnssData.clock.flags = 0;
	MockGnssData.clock.leap_second = 18;
	MockGnssData.clock.time_ns = 365163457929441;
	MockGnssData.clock.time_uncertainty_ns = 0;
	MockGnssData.clock.full_bias_ns = 0;
	MockGnssData.clock.bias_ns = 0.000000;
	MockGnssData.clock.bias_uncertainty_ns = 0.000000;
	MockGnssData.clock.drift_nsps = 0.000000;
	MockGnssData.clock.drift_uncertainty_nsps = 0.000000;
	MockGnssData.clock.hw_clock_discontinuity_count = 0;
	s->measurement_callbacks.gnss_measurement_callback(&MockGnssData);
	DBG("%s: gnss_measurement_callback end",__FUNCTION__);

	return 0;
}

void gnssmgt_meas_close(void)
{
	GpsState*  s = _gps_state;

	DBG("%s: enter", __FUNCTION__);
	//清除回调函数地址
	s->measurement_callbacks.measurement_callback = NULL;
	s->measurement_callbacks.gnss_measurement_callback = NULL;
}

/*****************************GnssNavigationMessage接口*************************************/

int gnssmgt_navi_msg_init(GpsNavigationMessageCallbacks *callbacks)
{
	GpsState*  s = _gps_state;

	DBG("%s: enter, callbacks[%p]",__FUNCTION__, callbacks);

	if(callbacks == NULL)
	{
		DBGE("%s: callbacks is NULL", __FUNCTION__);
		return GPS_NAVIGATION_MESSAGE_ERROR_GENERIC;
	}else
	{
		if (callbacks->size != sizeof(GpsNavigationMessageCallbacks))
		{
			DBGE("%s: data is mismatched! input size(%zu), local size(%zu)",
	                __FUNCTION__, callbacks->size, sizeof(GpsNavigationMessageCallbacks));
			return GPS_NAVIGATION_MESSAGE_ERROR_GENERIC;
		}
	}
	if(s->navigation_message_callbacks.gnss_navigation_message_callback != NULL)

	{
		DBGE("%s: gnss_navigation_message_callback has already init", __FUNCTION__);
		return GPS_NAVIGATION_MESSAGE_ERROR_ALREADY_INIT;
	}
	//注册回调函数
	s->navigation_message_callbacks = *callbacks;
	//上报观测量信息
	MockMessage.size = sizeof(GnssNavigationMessage);
	MockMessage.svid = 0;
	MockMessage.type = 0;
	MockMessage.status = 0;
	MockMessage.message_id = 0;
	MockMessage.submessage_id = 0;
	MockMessage.data_length = 0;
	MockMessage.data = NULL;
	s->navigation_message_callbacks.gnss_navigation_message_callback(&MockMessage);
	DBG("%s: gnss_navigation_message_callback end",__FUNCTION__);
	//s->navigation_message_callbacks.navigation_message_callback(&MockMessage);

	return 0;
}

void gnssmgt_navi_msg_close(void)
{
	GpsState*  s = _gps_state;

	DBG("%s enter", __FUNCTION__);
	//清除回调函数地址
	s->navigation_message_callbacks.navigation_message_callback = NULL;
	s->navigation_message_callbacks.gnss_navigation_message_callback = NULL;
}

/*****************************GnssMeasurementCorrections接口*************************************/

int gnssmgt_meas_corr_init(GnssMeasurementCorrectionCallbacks *callbacks)
{
	GpsState*	s = _gps_state;

	DBG("%s: enter, callbacks[%p]",__FUNCTION__, callbacks);

	if(callbacks == NULL)
	{
		DBG("%s: callbacks NULL",__FUNCTION__);
		return -1;
	}else
	{
		if (callbacks->size != sizeof(GnssMeasurementCorrectionCallbacks))
		{
			DBGE("%s: data is mismatched! input size(%zu), local size(%zu)",
	          __FUNCTION__, callbacks->size, sizeof(GnssMeasurementCorrectionCallbacks));
			return -1;
		}
	}
	if(s->measurement_corrections_callbacks.size == callbacks->size &&
       s->measurement_corrections_callbacks.mea_correction_capability_callback == \
		callbacks->mea_correction_capability_callback)
	{
		DBG("%s: measurement_corrections_callbacks has already init",__FUNCTION__);
	}
	else
	{
		//注册回调函数
		s->measurement_corrections_callbacks = *callbacks;
		//上报修正能力
		Mockcapability = 1;
		s->measurement_corrections_callbacks.mea_correction_capability_callback(Mockcapability);
		DBG("%s: mea_correction_capability_callback end",__FUNCTION__);
	}
	return 0;
}

int gnssmgt_injectMeasurementCorrections(GnssMeasurementCorrections_t *pMeasCorr, int iSatCount)
{
	DBG("%s: enter", __FUNCTION__);

	if((pMeasCorr == NULL) || (iSatCount < 0))
	{
		DBGE("%s: input param(s) error, pMeasCorr[%p], iSatCount[%d]",
		__FUNCTION__, pMeasCorr, iSatCount);
		return -1;
	}
	return 0;
}

/*****************************GnssUnisocExt接口*************************************/

void gnssmgt_unisoc_bindNwHandle(long long int handle_val)
{
	DBG("%s: enter, handle_val[%lld]", __FUNCTION__, handle_val);
	return ;
}

/*****************************GnssVisibilityControl接口*************************************/

int gnssmgt_gnssVisibilityCtrl_init(GnssVisibilityControlCallbacks *callbacks)
{
	GpsState* s = _gps_state;

	DBG("%s: enter", __FUNCTION__);

	if (NULL == callbacks)
	{
		DBGE("%s: invalid input param", __FUNCTION__);
		return -1;
	}else
	{
		if (callbacks->size != sizeof(GnssVisibilityControlCallbacks))
		{
			DBGE("%s: data is mismatched! input size(%zu), local size(%zu)",
	                __FUNCTION__, callbacks->size, sizeof(GnssVisibilityControlCallbacks));
			return -1;
		}
	}
	//回调函数注册
	s->visCtrl_callbacks = *callbacks;
	//上报通知消息
	MockVisnotification.proxyAppPackageName = "test";
	MockVisnotification.protocolStack = CTRL_PLANE;
	MockVisnotification.otherProtocolStackName = "test2";
	MockVisnotification.requestor = CARRIER;
	MockVisnotification.requestorId = "abc";
	MockVisnotification.responseType = REJECTED;
	MockVisnotification.inEmergencyMode = true;
	MockVisnotification.isCachedLocation = true;
	s->visCtrl_callbacks.nfw_notify_callback(&MockVisnotification);
	DBG("%s: nfw_notify_callback end",__FUNCTION__);
	s->visCtrl_callbacks.is_in_emergency_session_calback();
	DBG("%s: is_in_emergency_session_calback end",__FUNCTION__);

	return 0;
}

int gnssmgt_nfwLocationAccess_enable(const char *proxyApps)
{

	DBG("%s: enter, proxyApps[%p]", __FUNCTION__, proxyApps);

	if (NULL == proxyApps)
	{
		DBGE("%s: invalid input param.", __FUNCTION__);
		return -1;
	}
	return 0;
}

/*****************************supl接口*************************************/
int gnssmgt_cert_install(
    const DerEncodedCertificate* certificates, size_t length)
{
	DBG("%s: enter, certificates[%p], length[%d]", __FUNCTION__, certificates, length);
	if(!certificates)
	{
		DBGE("%s: input NULL param", __FUNCTION__);
		return -1;
	}
	if(length != sizeof(DerEncodedCertificate))
	{
		DBGE("%s: size should be %zu,but now is %zu",__FUNCTION__,sizeof(DerEncodedCertificate), length);
		return -1;
	}
	return 0;
}

int gnssmgt_cert_revoke(
			const Sha1CertificateFingerprint* fingerprints, size_t length)
{
	DBG("%s: enter, fingerprints[%p], length[%d]", __FUNCTION__, fingerprints, length);

	if(!fingerprints)
	{
		DBGE("%s: input NULL param", __FUNCTION__);
		return -1;
	}
	if(length != sizeof(Sha1CertificateFingerprint))
	{
		DBGE("%s: size should be %zu,but now is %zu",__FUNCTION__,sizeof(Sha1CertificateFingerprint), length);
		return -1;
	}

	return 0;
}

/*****************************gnsspc相关接口*************************************/
int gnssmgt_handlePcRequest(GNSSMGT_PC_OPCODE_e opCode, void *pData)
{
	DBG("%s: enter, opcode is [%d]", __FUNCTION__, opCode);

	if (NULL == pData)
	{
		DBGE("%s: invalid input param,pData is null.", __FUNCTION__);
		return -1;
	}
	if ((opCode < 0) || (opCode > 8))
	{
		DBGE("%s: Incorrect value of incoming parameter", __FUNCTION__);
		return -1;
	}
	return 0;
}

/******************************************************************/
BOOL gnssmgt_esExtensionSec_set(unsigned int seconds)
{
	DBG("%s: enter, duration is %d seconds", __FUNCTION__, seconds);
	return TRUE;
}

