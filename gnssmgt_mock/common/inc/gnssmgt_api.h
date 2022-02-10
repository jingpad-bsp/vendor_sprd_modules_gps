#ifndef _GNSSMGT_API_H_
#define _GNSSMGT_API_H_

#include "gps.h"




typedef enum {
    PC_OPCODE_NONE = 0,

    PC_OPCODE_SET_REG,
    PC_OPCODE_SET_INIT_TYPE,
    PC_OPCODE_SET_WORK_MODE,

    PC_OPCODE_GET_REG,
    PC_OPCODE_GET_TSX,
    PC_OPCODE_GET_ENG_PWR_STATE,
    PC_OPCODE_GET_WORK_MODE_LIST,

    PC_OPCODE_TEST_INF,
}GNSSMGT_PC_OPCODE_e;

typedef struct {
    unsigned int addr;
    unsigned int val;
}GNSSMGT_PC_REG_OPT_t;

/******************************************************************************/
// Description: initialize the location engine.
// Global resource dependence:
// Author:
// Note:
// Return value:  0 -SUCC, others - FAIL
/******************************************************************************/
int gnssmgt_init(GpsCallbacks *callbacks);

/******************************************************************************/
// Description: start the tracking session.
// Global resource dependence:
// Author:
// Note:
// Return value: 0 -SUCC, others - FAIL
/******************************************************************************/
int gnssmgt_start(void);

/******************************************************************************/
// Description: stop the tracking session.
// Global resource dependence:
// Author:
// Note:
// Return value: 0 -SUCC, others - FAIL
/******************************************************************************/
int gnssmgt_stop(void);

/******************************************************************************/
// Description: clean location engine.
// Global resource dependence:
// Author:
// Note:
// Return value: 0 -SUCC, others - FAIL
/******************************************************************************/
void gnssmgt_cleanup(void);

/******************************************************************************/
// Description: clean location engine.
// Global resource dependence:
// Author:
// Note:
// Return value:
/******************************************************************************/
int gnssmgt_injectTime(
        GpsUtcTime gpsTime, int64_t refTime, int uncertainty);

/******************************************************************************/
// Description: location injection.
// Global resource dependence:
// Author:
// Note:
// Return value: 0 -SUCC, others - FAIL
/******************************************************************************/
int gnssmgt_injectLocation(
        double latitude, double longitude, float accuracy);

/******************************************************************************/
// Description: best inject best location.
// Global resource dependence:
// Author:
// Note:
// Return value: 0 -SUCC, others - FAIL
/******************************************************************************/
int gnssmgt_injectBestLocation(GnssLocation* pLoc);

/******************************************************************************/
// Description: delete the aiding data.
// Global resource dependence:
// Author:
// Note:
// Return value:
/******************************************************************************/
void gnssmgt_delAidingData(GpsAidingData aidingData);

/******************************************************************************/
// Description: sets the mode and fix frequency.
// Global resource dependence:
// Author:
// Note:
// Return value: 0 -SUCC, others - FAIL
/******************************************************************************/
int gnssmgt_setPosMode(
        GpsPositionMode         mode,
        GpsPositionRecurrence   recurrence,
        uint32_t                minInterval,
        uint32_t                prefAccuracy,
        uint32_t                prefTime);

/******************************************************************************/
// Description: initialize agps interface.
// Global resource dependence:
// Author:
// Note:
// Return value:
/******************************************************************************/
void gnssmgt_agps_init(AGpsCallbacks* callbacks);

/******************************************************************************/
// Description: on-demand data connection opening is successfu.
// Global resource dependence:
// Author:
// Note:
// Return value: 0 -SUCC, others - FAIL
/******************************************************************************/
int gnssmgt_agps_openConn(const char* apn);

/******************************************************************************/
// Description: on-demand data connection closing is done.
// Global resource dependence:
// Author:
// Note:
// Return value: 0 -SUCC, others - FAIL
/******************************************************************************/
int gnssmgt_agps_closeConn(void);

/******************************************************************************/
// Description: on-demand data connection opening has failed.
// Global resource dependence:
// Author:
// Note:
// Return value: 0 -SUCC, others - FAIL
/******************************************************************************/
int gnssmgt_agps_openFailed(void);

/******************************************************************************/
// Description: set agps server cfg
// Global resource dependence:
// Author:
// Note:
// Return value: 0 -SUCC, others - FAIL
/******************************************************************************/
int gnssmgt_agps_setServer(
    AGpsType type, const char* hostname, int port);

/******************************************************************************/
// Description: on-demand data connection opening is successful.
// Global resource dependence:
// Author:
// Note:
// Return value: 0 -SUCC, others - FAIL
/******************************************************************************/
int gnssmgt_agps_openWithApnIpType(
    const char *apn, ApnIpType apnIpType);

/******************************************************************************/
// Description: update network connection status with networkHandle.
// Global resource dependence:
// Author:
// Note:
// Return value: 0 -SUCC, others - FAIL
/******************************************************************************/
int gnssmgt_agps_openWithApnIpTypeEx(
    long long int networkHandle,
    const char *apn,
    ApnIpType apnIpType);

/******************************************************************************/
// Description: initializes the NI interface.
// Global resource dependence:
// Author:
// Note:
// Return value:
/******************************************************************************/
void gnssmgt_ni_init(GpsNiCallbacks *callbacks);

/******************************************************************************/
// Description: sends an NI respond.
// Global resource dependence:
// Author:
// Note:
// Return value:
/******************************************************************************/
void gnssmgt_ni_respond(int notifID, GpsUserResponseType resp);

/******************************************************************************/
// Description:
// Global resource dependence:
// Author:
// Note:
// Return value:
/******************************************************************************/
void gnssmgt_agps_ril_init(AGpsRilCallbacks *callbacks);

/******************************************************************************/
// Description:
// Global resource dependence:
// Author:
// Note:
// Return value:
/******************************************************************************/
void gnssmgt_agps_ril_setRefLoc(
        const AGpsRefLocation *refLocation, size_t szStruct);

/******************************************************************************/
// Description:
// Global resource dependence:
// Author:
// Note:
// Return value:
/******************************************************************************/
void gnssmgt_agps_ril_setSetID(AGpsSetIDType type, const char *setid);

/******************************************************************************/
// Description:
// Global resource dependence:
// Author:
// Note:
// Return value:
/******************************************************************************/
void gnssmgt_agps_ril_recvNiMsg(uint8_t *msg, size_t len);

/******************************************************************************/
// Description:
// Global resource dependence:
// Author:
// Note:
// Return value:
/******************************************************************************/
void gnssmgt_agps_ril_updateNwState(
        int connected, int type, int roaming, const char* extraInfo);

/******************************************************************************/
// Description:
// Global resource dependence:
// Author:
// Note:
// Return value:
/******************************************************************************/
void gnssmgt_agps_ril_updateNwAvailability(
        int avaiable, const char *apn);

/******************************************************************************/
// Description: initialize XTRA module.
// Global resource dependence:
// Author:
// Note:
// Return value:  0 -SUCC, others - FAIL
/******************************************************************************/
int gnssmgt_xtra_init(GpsXtraCallbacks *callbacks);

/******************************************************************************/
// Description: inject XTRA data.
// Global resource dependence:
// Author:
// Note:
// Return value:  0 -SUCC, others - FAIL
/******************************************************************************/
int gnssmgt_xtra_injectData(char *data, int length);

/******************************************************************************/
// Description: initialize geofence module.
// Global resource dependence:
// Author:
// Note:
// Return value:
/******************************************************************************/
void gnssmgt_geo_init(GpsGeofenceCallbacks *callbacks);

/******************************************************************************/
// Description: add geofence area.
// Global resource dependence:
// Author:
// Note:
// Return value:
/******************************************************************************/
void gnssmgt_geo_addArea(
        int32_t geofence_id,
        double  latitude,
        double  longitude,
        double  radius_meters,
        int     last_transition,
        int     monitor_transitions,
        int     notification_responsiveness_ms,
        int     unknown_timer_ms);

/******************************************************************************/
// Description: pause monitoring a particular geofence.
// Global resource dependence:
// Author:
// Note:
// Return value:
/******************************************************************************/
void gnssmgt_geo_pause(int32_t geofence_id);

/******************************************************************************/
// Description: resume monitoring a particular geofence.
// Global resource dependence:
// Author:
// Note:
// Return value:
/******************************************************************************/
void gnssmgt_geo_resume(int32_t geofence_id, int monitor_transitions);

/******************************************************************************/
// Description: remove a geofence area.
// Global resource dependence:
// Author:
// Note:
// Return value:
/******************************************************************************/
void gnssmgt_geo_removeArea(int32_t geofence_id);

/******************************************************************************/
// Description: return bugreport.
// Global resource dependence:
// Author:
// Note:
// Return value: size
/******************************************************************************/
size_t gnssmgt_dbg_getInternalState(char *buffer, size_t bufferSize);

/******************************************************************************/
// Description: deliver GNSS configuration contents.
// Global resource dependence:
// Author:
// Note:
// Return value:
/******************************************************************************/
void gnssmgt_cfg_update(const char *config_data, int32_t length);

/******************************************************************************/
// Description: initialize measurements module.
// Global resource dependence:
// Author:
// Note:
// Return value:  0 -SUCC, others - FAIL
/******************************************************************************/
int gnssmgt_meas_init(GpsMeasurementCallbacks *callbacks);

/******************************************************************************/
// Description: stops updates measurements.
// Global resource dependence:
// Author:
// Note:
// Return value:
/******************************************************************************/
void gnssmgt_meas_close(void);

/******************************************************************************/
// Description: initialize navigation message module.
// Global resource dependence:
// Author:
// Note:
// Return value:  0 -SUCC, others - FAIL
/******************************************************************************/
int gnssmgt_navi_msg_init(GpsNavigationMessageCallbacks *callbacks);

/******************************************************************************/
// Description: stops updates navigation message.
// Global resource dependence:
// Author:
// Note:
// Return value:
/******************************************************************************/
void gnssmgt_navi_msg_close(void);

/******************************************************************************/
// Description: initialize measurement corrections module.
// Global resource dependence:
// Author:
// Note:
// Return value:  0 -SUCC, others - FAIL
/******************************************************************************/
int gnssmgt_meas_corr_init(GnssMeasurementCorrectionCallbacks *callbacks);

/******************************************************************************/
// Description: set measurement corrections module.
// Global resource dependence:
// Author:
// Note:
// Return value:  0 -SUCC, others - FAIL
/******************************************************************************/
int gnssmgt_injectMeasurementCorrections(GnssMeasurementCorrections_t *pMeasCorr, int iSatCount);

/******************************************************************************/
// Description: installs a set of Certificates used for SUPL connections
//              to the AGPS server.
// Global resource dependence:
// Author:
// Note:
// Return value:  0 -SUCC, others - FAIL
/******************************************************************************/
int gnssmgt_cert_install(
    const DerEncodedCertificate* certificates, size_t length);

/******************************************************************************/
// Description: a list of certificates used for SUPL connections are revoked.
// Global resource dependence:
// Author:
// Note:
// Return value:  0 -SUCC, others - FAIL
/******************************************************************************/
int gnssmgt_cert_revoke(
                const Sha1CertificateFingerprint* fingerprints, size_t length);

/******************************************************************************/
// Description: bind network handle
// Global resource dependence:
// Author:
// Note:
// Return value:
/******************************************************************************/
void gnssmgt_unisoc_bindNwHandle(long long int handle_val);

/******************************************************************************/
// Description: handle GNSS PC request from GPS HAL
// Global resource dependence:
// Author:
// Note:
// Return value:  0 -SUCC, others - FAIL
/******************************************************************************/
int gnssmgt_handlePcRequest(GNSSMGT_PC_OPCODE_e opCode, void *pData);

/******************************************************************************/
// Description: registers the callback for gnss visibility control
// Global resource dependence:
// Author:
// Note:
// Return value:  0 -SUCC, others - FAIL
/******************************************************************************/
int  gnssmgt_gnssVisibilityCtrl_init(GnssVisibilityControlCallbacks *callbacks);

/******************************************************************************/
// Description: enables/disables non-framework entity location access permission.
// Global resource dependence:
// Author:
// Note:
// Return value:  0 -SUCC, others - FAIL
/******************************************************************************/
int  gnssmgt_nfwLocationAccess_enable(const char *proxyApps);

/******************************************************************************/
// Description: sets the emergency session extension duration.
// Global resource dependence:
// Param:Number of seconds to extend the emergency session duration post emergency call.
// Author:
// Note:
// Return value:  0 -SUCC, others - FAIL
/******************************************************************************/
BOOL gnssmgt_esExtensionSec_set(unsigned int seconds);

#endif

