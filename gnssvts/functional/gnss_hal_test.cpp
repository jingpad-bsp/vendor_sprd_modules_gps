/*
 * Copyright (C) 2018 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define LOG_TAG "GnssHalTest"
#include <gnss_hal_test.h>
#include <chrono>
#include "Utils.h"
#include <inttypes.h>

using ::android::hardware::gnss::common::Utils;

// Implementations for the main test class for GNSS HAL
void GnssHalTest::SetUp() {
	gnss_hal_ = ::testing::VtsHalHidlTargetTestBase::getService<IGnss>(
		GnssHidlEnvironment::Instance()->getServiceName<IGnss>());
	ASSERT_NE(gnss_hal_, nullptr);
}
void GnssHalTest::TearDown() {
	if (gnss_hal_ != nullptr) {
		gnss_hal_->cleanup();
		gnss_hal_ = nullptr;
	}
}
Return<void> GnssHalTest::GnssCallback::gnssSetSystemInfoCb(
		const IGnssCallback_1_0::GnssSystemInfo& info) {
	ALOGI("Info received, year %d", info.yearOfHw);
	info_cbq_.store(info);
	return Void();
}
Return<void> GnssHalTest::GnssCallback::gnssRequestTimeCb() {
	ALOGI("Time request received");
	requesttime_count++;
	return Void();
}
Return<void> GnssHalTest::GnssCallback::gnssSetCapabilitesCb(uint32_t capabilities) {
	ALOGI("Capabilities received %d", capabilities);
	capabilities_cbq_.store(capabilities);
	return Void();
}
Return<void> GnssHalTest::GnssCallback::gnssNameCb(const android::hardware::hidl_string& name) {
	ALOGI("Name received: %s", name.c_str());
	name_cbq_.store(name);
	return Void();
}
Return<void> GnssHalTest::GnssCallback::gnssNmeaCb(int64_t timestamp, const android::hardware::hidl_string& nmea) {
	ALOGI("Nmea received timestamp = %" PRId64", nmea[%p]",timestamp, &nmea);
	nmea_count++;
	return Void();
}
Return<void> GnssHalTest::GnssCallback::gnssLocationCb_2_0(const GnssLocation_2_0& location) {
	ALOGI("Location (v2.0) received");
	location_cbq_2_0.store(location);
	return Void();
}
Return<void> GnssHalTest::GnssCallback::gnssLocationCb(const GnssLocation_1_0& location) {
	ALOGI("Location (v1.0) received");
	location_cbq_1_0.store(location);
	return Void();
}
Return<void> GnssHalTest::GnssCallback::gnssStatusCb(const IGnssCallback_1_0::GnssStatusValue status) {
	ALOGI("GnssStatus received %hhu", status);
	status_cbq_.store(status);
	return Void();
 }
Return<void> GnssHalTest::GnssCallback::gnssSvStatusCb(const IGnssCallback_1_0::GnssSvStatus& Gnsssvstatus) {
	ALOGI("gnssSvStatus received");
	gnss_sv_status_cbq_.store(Gnsssvstatus);
	return Void();
}
Return<void> GnssHalTest::GnssCallback::gnssSetCapabilitiesCb_2_0(uint32_t capabilities) {
	ALOGI("Capabilities (v2.0) received %d", capabilities);
	capabilities_cbq_2_0.store(capabilities);
	return Void();
}
Return<void> GnssHalTest::GnssCallback::gnssRequestLocationCb(bool request) {
	ALOGI("location request received ");
	request_location_cbq_.store(request);
	return Void();
}
Return<void> GnssHalTest::AGnssCallback::agnssStatusCb(IAGnssCallback_2_0::AGnssType type, IAGnssCallback_2_0::AGnssStatusValue status) {
	ALOGI("agnssStatus received type = %hhu status = %hhu", type, status);
	agnss_status_cbq_.store(status);
	return Void();
}
Return<void> GnssHalTest::GnssMeasurementCallback::GnssMeasurementCb(
	const IGnssMeasurementCallback_1_0::GnssData& data) {
	ALOGD("GnssMeasurement 1.0 received. Size = %d", (int)data.measurements.size());
	gnssmeasurement_cbq_1_0.store(data);
	return Void();
}
Return<void> GnssHalTest::GnssMeasurementCallback::gnssMeasurementCb(
	const IGnssMeasurementCallback_1_1::GnssData& data) {
	ALOGD("GnssMeasurement 1.1 received. Size = %d", (int)data.measurements.size());
	gnssmeasurement_cbq_1_1.store(data);
	return Void();
}
Return<void> GnssHalTest::GnssMeasurementCallback::gnssMeasurementCb_2_0(
	const IGnssMeasurementCallback_2_0::GnssData& data) {
	ALOGD("GnssMeasurement 2.0 received. Size = %d", (int)data.measurements.size());
	gnssmeasurement_cbq_2_0.store(data);
	return Void();
}
Return<void> GnssHalTest::GnssMeasurementCorrectionsCallback::setCapabilitiesCb(
		uint32_t capabilities) {
	ALOGI("GnssMeasurementCorrectionsCallback capabilities received %d", capabilities);
	capabilities_cbq_.store(capabilities);
	return Void();
}
Return<void> GnssHalTest::GnssNavigationMessageCallback::gnssNavigationMessageCb(const IGnssNavigationMessageCallback_1_0::GnssNavigationMessage& message) {
	ALOGI("GnssNavigationmessage received");
	gnssNavigationMessage_cbq_.store(message);
	return Void();
}
Return<void> GnssHalTest::GnssVisibilityControlCallback::nfwNotifyCb(const IGnssVisibilityControlCallback_1_0::NfwNotification& notification){
	ALOGI("nfwnotify received");
	nfwnotify_cbq_.store(notification);
	return Void();
}
Return<bool> GnssHalTest::GnssVisibilityControlCallback::isInEmergencySession(){
	ALOGI("isInEmergencySession received");
	isinemergency_count++;
	return true;
}

Return<void> GnssHalTest::GnssGeofenceCallback::gnssGeofenceTransitionCb(int32_t geofenceId, const GnssLocation_1_0& location, IGnssGeofenceCallback_1_0::GeofenceTransition transition, int64_t timestamp){
	ALOGI("geofentransition received, id = %d, transition = %d, location[%p], timestamp = %" PRId64" ", geofenceId, transition, &location, timestamp);
	geofentransition_count++;
	return Void();
}
Return<void> GnssHalTest::GnssGeofenceCallback::gnssGeofenceStatusCb(IGnssGeofenceCallback_1_0::GeofenceAvailability status, const GnssLocation_1_0& lastLocation){
	ALOGI("geofenstatus received, status = %d location[%p]", status,&lastLocation);
	geofenstatus_count++;
	return Void();
}
Return<void> GnssHalTest::GnssGeofenceCallback::gnssGeofenceAddCb(int32_t geofenceId, IGnssGeofenceCallback_1_0::GeofenceStatus status){
	ALOGI("geofenadd received, id = %32d, status = %d", geofenceId, status);
	geofenadd_count++;
	return Void();
}
Return<void> GnssHalTest::GnssGeofenceCallback::gnssGeofenceRemoveCb(int32_t geofenceId, IGnssGeofenceCallback_1_0::GeofenceStatus status){
	ALOGI("geofenremove received, id = %32d, status = %d", geofenceId, status);
	geofenremove_count++;
	return Void();
}
Return<void> GnssHalTest::GnssGeofenceCallback::gnssGeofencePauseCb(int32_t geofenceId, IGnssGeofenceCallback_1_0::GeofenceStatus status){
	ALOGI("geofenpause received, id = %32d, status = %d", geofenceId, status);
	geofenpause_count++;
	return Void();
}
 Return<void> GnssHalTest::GnssGeofenceCallback::gnssGeofenceResumeCb(int32_t geofenceId, IGnssGeofenceCallback_1_0::GeofenceStatus status){
	ALOGI("geofenresume received, id = %32d, status = %d", geofenceId, status);
	geofenresume_count++;
	return Void();
}
Return<void> GnssHalTest::AGnssRilCallback::requestRefLocCb(){
	ALOGI("reflocation request received");
	refloc_count++;
	return Void();
}
Return<void> GnssHalTest::AGnssRilCallback::requestSetIdCb(uint32_t flags){
	ALOGI("setid request received %d",flags);
	lastrequestsetid_count++;
	return Void();
}
Return<void> GnssHalTest::GnssNiCallback::niNotifyCb(const IGnssNiCallback_1_0::GnssNiNotification& notification){
	ALOGI("ni notification received [%p]", &notification);
	gnssninotify_cbq_.store(notification);
	return Void();
}
Return<void> GnssHalTest::GnssXtraCallback::downloadRequestCb(){
	ALOGI("downloadrequest received");
	downloadrequest_count++;
	return Void();
}

