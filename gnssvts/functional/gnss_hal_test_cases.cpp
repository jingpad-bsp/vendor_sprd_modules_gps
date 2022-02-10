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

#define LOG_TAG "GnssHidl&HalTestCases"

#include <VtsHalHidlTargetTestBase.h>
#include <gnss_hal_test.h>
#include "Utils.h"

using android::hardware::hidl_string;
using android::hardware::hidl_vec;

using IGnssConfiguration_1_0 = android::hardware::gnss::V1_0::IGnssConfiguration;
using IGnssConfiguration_1_1 = android::hardware::gnss::V1_1::IGnssConfiguration;
using IGnssConfiguration_2_0 = android::hardware::gnss::V2_0::IGnssConfiguration;
using IGnssMeasurement_2_0 = android::hardware::gnss::V2_0::IGnssMeasurement;
using IGnssMeasurement_1_1 = android::hardware::gnss::V1_1::IGnssMeasurement;
using IGnssMeasurement_1_0 = android::hardware::gnss::V1_0::IGnssMeasurement;
using IAGnssRil_2_0 = android::hardware::gnss::V2_0::IAGnssRil;
using IAGnssRil_1_0 = android::hardware::gnss::V1_0::IAGnssRil;
using IAGnss_2_0 = android::hardware::gnss::V2_0::IAGnss;
using IAGnss_1_0 = android::hardware::gnss::V1_0::IAGnss;
using IGnss_2_0 = android::hardware::gnss::V2_0::IGnss;
using IGnssBatching_V1_0 = android::hardware::gnss::V1_0::IGnssBatching;
using IGnssBatching_V2_0 = android::hardware::gnss::V2_0::IGnssBatching;
using IGnssNavigationMessage_1_0 = android::hardware::gnss::V1_0::IGnssNavigationMessage;
using IGnssNi_1_0 = android::hardware::gnss::V1_0::IGnssNi;
using IGnssXtra_1_0 = android::hardware::gnss::V1_0::IGnssXtra;
using IGnssDebug_2_0 = android::hardware::gnss::V2_0::IGnssDebug;
using IGnssDebug_1_0 = android::hardware::gnss::V1_0::IGnssDebug;
using IGnssXtra_1_0 = android::hardware::gnss::V1_0::IGnssXtra;
using IGnssGeofencing_1_0 = android::hardware::gnss::V1_0::IGnssGeofencing;
using android::hardware::gnss::common::Utils;
using android::hardware::gnss::measurement_corrections::V1_0::IMeasurementCorrections;
using android::hardware::gnss::measurement_corrections::V1_0::MeasurementCorrections;
using android::hardware::gnss::V1_0::IGnssNi;
using android::hardware::gnss::V2_0::ElapsedRealtimeFlags;
using android::hardware::gnss::V2_0::GnssConstellationType;
using android::hardware::gnss::V2_0::IGnssCallback;
using IGnssVisibilityControl = android::hardware::gnss::visibility_control::V1_0::IGnssVisibilityControl;
//using IGnssUnisocExt_1_0 = vendor::sprd::hardware::gnss::V1_0::IGnssUnisocExt;
//using vendor_IGnss = vendor::sprd::hardware::gnss::V2_0::IGnss;

/*test for gnss module*/
TEST_F(GnssHalTest, TestGnssFields) {
	sp<GnssCallback> gnss_cb_ = new GnssHalTest::GnssCallback();
	ASSERT_NE(gnss_cb_, nullptr);
	/*Gnss 1.0接口*/
	auto result = gnss_hal_->setCallback(gnss_cb_);
	if (!result.isOk()) {
		ALOGE("result of failed setCallback %s", result.description().c_str());
	}
	ASSERT_TRUE(result.isOk());
	ASSERT_TRUE(result);
	EXPECT_TRUE(gnss_cb_->capabilities_cbq_.retrieve(gnss_cb_->last_capabilities_, TIMEOUT_SEC));
	EXPECT_TRUE(gnss_cb_->capabilities_cbq_.retrieve(gnss_cb_->last_capabilities_, TIMEOUT_SEC));
	EXPECT_EQ(gnss_cb_->capabilities_cbq_.calledCount(), 2);

	//开启gps、设置位置模式并检测位置上报。
	auto result_start = gnss_hal_->start();
	EXPECT_TRUE(result_start.isOk());
	EXPECT_TRUE(result_start);
	auto result_setposi = gnss_hal_->setPositionMode(
		android::hardware::gnss::V1_0::IGnss::GnssPositionMode::MS_BASED, android::hardware::gnss::V1_0::IGnss::GnssPositionRecurrence::RECURRENCE_PERIODIC, 500, 0, 0);
	ASSERT_TRUE(result_setposi.isOk());
	EXPECT_TRUE(result_setposi);
	const int kFirstGnssLocationTimeoutSeconds = 10;
	EXPECT_TRUE(gnss_cb_->location_cbq_1_0.retrieve(gnss_cb_->last_location_1_0,kFirstGnssLocationTimeoutSeconds));
	EXPECT_EQ(gnss_cb_->location_cbq_1_0.calledCount(), 1);
	/*开启之后底层上传状态信息、nmea数据、星历状态信息，检测接口是否走通，信息是否符合要求*/
	EXPECT_TRUE(gnss_cb_->status_cbq_.retrieve(gnss_cb_->last_status_, TIMEOUT_SEC));
	EXPECT_TRUE(gnss_cb_->gnss_sv_status_cbq_.retrieve(gnss_cb_->last_gnss_sv_status_, TIMEOUT_SEC));
	EXPECT_TRUE(gnss_cb_->gnss_sv_status_cbq_.retrieve(gnss_cb_->last_gnss_sv_status_, TIMEOUT_SEC));
	EXPECT_EQ(gnss_cb_->nmea_count, 1);

	EXPECT_EQ(gnss_cb_->status_cbq_.calledCount(), 1);//start之后status返回的值
	EXPECT_EQ(gnss_cb_->gnss_sv_status_cbq_.calledCount(), 2);
	EXPECT_EQ(gnss_cb_->requesttime_count, 1);
	/*注入位置信息，检测接口是否走通*/
	auto result_inject = gnss_hal_->injectLocation(80.0, -170.0, 1000.0);
	ASSERT_TRUE(result_inject.isOk());
	EXPECT_TRUE(result_inject);
	/*注入时间信息，检测接口是否走通*/
	auto result_injecttime = gnss_hal_->injectTime(1534567890123L, 123456L, 10000L);
	ASSERT_TRUE(result_injecttime.isOk());
	EXPECT_TRUE(result_injecttime);
	/*删除辅助位置信息，检测接口是否走通*/
	auto resultVoid = gnss_hal_->deleteAidingData(IGnss::GnssAidingData::DELETE_POSITION);
	ASSERT_TRUE(resultVoid.isOk());
	/*删除辅助时间信息，检测接口是否走通*/
	resultVoid = gnss_hal_->deleteAidingData(IGnss::GnssAidingData::DELETE_TIME);
	ASSERT_TRUE(resultVoid.isOk());
	/*关闭gps，并检测当前状态值*/
	auto result_stop = gnss_hal_->stop();
	EXPECT_TRUE(result_stop.isOk());
	EXPECT_TRUE(result_stop);
	EXPECT_TRUE(gnss_cb_->status_cbq_.retrieve(gnss_cb_->last_status_, TIMEOUT_SEC));
	EXPECT_EQ(gnss_cb_->status_cbq_.calledCount(), 2);//stop之后status返回的值
	/*cleanup，并检测当前状态值*/
	auto resultVoid_clean = gnss_hal_->cleanup();
	ASSERT_TRUE(resultVoid_clean.isOk());
	EXPECT_TRUE(gnss_cb_->status_cbq_.retrieve(gnss_cb_->last_status_, TIMEOUT_SEC));
	EXPECT_EQ(gnss_cb_->status_cbq_.calledCount(), 3);//clean之后status返回的值
	/*Gnss 1.1接口*/
	sp<GnssCallback> gnss_cb_1_1 = new GnssCallback();
	ASSERT_NE(gnss_cb_1_1, nullptr);
	auto result_setCallback1_1 = gnss_hal_->setCallback_1_1(gnss_cb_1_1);
	if (!result_setCallback1_1.isOk()) {
		ALOGE("result of failed setCallback %s", result_setCallback1_1.description().c_str());
	}
	EXPECT_TRUE(gnss_cb_1_1->info_cbq_.retrieve(gnss_cb_->last_info_, TIMEOUT_SEC));
	EXPECT_TRUE(gnss_cb_1_1->name_cbq_.retrieve(gnss_cb_1_1->last_name_, TIMEOUT_SEC));
	EXPECT_EQ(gnss_cb_1_1->name_cbq_.calledCount(), 1);
	EXPECT_EQ(gnss_cb_1_1->info_cbq_.calledCount(), 1);
	auto result_start1_1 = gnss_hal_->start();
	EXPECT_TRUE(result_start1_1.isOk());
	EXPECT_TRUE(result_start1_1);
	auto result_setposi1_1 = gnss_hal_->setPositionMode_1_1(
		IGnss::GnssPositionMode::MS_BASED, IGnss::GnssPositionRecurrence::RECURRENCE_PERIODIC, 500, 0, 0, false);
	ASSERT_TRUE(result_setposi1_1.isOk());
	EXPECT_TRUE(result_setposi1_1);
	/*Gnss 2.0接口*/
	sp<GnssCallback> gnss_cb_2_0 = new GnssCallback();
	ASSERT_NE(gnss_cb_2_0, nullptr);
	result = gnss_hal_->setCallback_2_0(gnss_cb_2_0);
	if (!result.isOk()) {
		ALOGE("result of failed setCallback %s", result.description().c_str());
	}
	/*注入bestlocation，检测接口是否走通*/
	auto result_start2_0 = gnss_hal_->start();
	EXPECT_TRUE(result_start2_0.isOk());
	EXPECT_TRUE(result_start2_0);
	EXPECT_TRUE(gnss_cb_2_0->location_cbq_2_0.retrieve(gnss_cb_2_0->last_location_2_0,kFirstGnssLocationTimeoutSeconds));
	EXPECT_EQ(gnss_cb_2_0->location_cbq_2_0.calledCount(), 1);
	auto result_inject2_0 = gnss_hal_->injectBestLocation_2_0(gnss_cb_2_0->last_location_2_0);
	ASSERT_TRUE(result_inject2_0.isOk());
	EXPECT_TRUE(result_inject2_0);
}

/*test for agnss module*/
TEST_F(GnssHalTest, TestAGnssFields) {
	/*AGnss 1.0接口*/
	auto agnss_1_0 = gnss_hal_->getExtensionAGnss();
	ASSERT_TRUE(agnss_1_0.isOk());
	/*AGnss 2.0接口*/
	auto agnss_2_0 = gnss_hal_->getExtensionAGnss_2_0();
	ASSERT_TRUE(agnss_2_0.isOk());
	sp<IAGnss_2_0> iAGnss_2_0 = agnss_2_0;
	if (iAGnss_2_0 == nullptr) {
		ALOGE("getExtensionAGnss_2_0 failed, the result is nullptr");
	}
	/*调用初始化函数，下发回调函数地址*/
	sp<AGnssCallback> callback = new AGnssCallback();
	auto result_setcallback = iAGnss_2_0->setCallback(callback);
	ASSERT_TRUE(result_setcallback.isOk());
	/*检测agnss状态回调接口是否走通*/
	EXPECT_TRUE(callback->agnss_status_cbq_.retrieve(callback->last_agnss_status_, TIMEOUT_SEC));
	EXPECT_EQ(callback->agnss_status_cbq_.calledCount(), 1);
	/*检测设置服务、建立数据连接、释放数据连接、数据连接失败接口是否走通*/
	auto result_setserver = iAGnss_2_0->setServer(IAGnssCallback_2_0::AGnssType::SUPL, "supl.google.com", 7275);
	ASSERT_TRUE(result_setserver.isOk());
	EXPECT_TRUE(result_setserver);
	auto result_dataopen = iAGnss_2_0->dataConnOpen(0, "3gnet", IAGnss_2_0::ApnIpType::IPV4V6);
	ASSERT_TRUE(result_dataopen.isOk());
	EXPECT_TRUE(result_dataopen);
	auto result_dataclose = iAGnss_2_0->dataConnClosed();
	ASSERT_TRUE(result_dataclose.isOk());
	EXPECT_TRUE(result_dataclose);
	auto result_datafailed = iAGnss_2_0->dataConnFailed();
	ASSERT_TRUE(result_datafailed.isOk());
	EXPECT_TRUE(result_datafailed);
}

/*test for gnssmeasurement module*/
TEST_F(GnssHalTest, TestGnssMeasurementFields) {
	/*gnssmeasurement 1.0接口*/
	auto gnssMeasurement_1_0 = gnss_hal_->getExtensionGnssMeasurement();
	if (!gnssMeasurement_1_0.isOk()) {
		return;
	}
	sp<IGnssMeasurement_1_0> ignssMeasurement_1_0 = gnssMeasurement_1_0;
	if (ignssMeasurement_1_0 == nullptr) {
		return;
	}
	sp<GnssMeasurementCallback> callback_1_0 = new GnssHalTest::GnssMeasurementCallback();
	auto result_setcallback = ignssMeasurement_1_0->setCallback(callback_1_0);
	ASSERT_TRUE(result_setcallback.isOk());
	EXPECT_EQ(result_setcallback, IGnssMeasurement_1_0::GnssMeasurementStatus::SUCCESS);

	const int kFirstGnssMeasurementTimeoutSeconds = 20;
	ASSERT_TRUE(callback_1_0->gnssmeasurement_cbq_1_0.retrieve(callback_1_0->last_gnssmeasurement_1_0,
		kFirstGnssMeasurementTimeoutSeconds));
	ASSERT_TRUE(callback_1_0->gnssmeasurement_cbq_1_0.retrieve(callback_1_0->last_gnssmeasurement_1_0,
		kFirstGnssMeasurementTimeoutSeconds));
	EXPECT_EQ(callback_1_0->gnssmeasurement_cbq_1_0.calledCount(), 2);
	ASSERT_TRUE(callback_1_0->last_gnssmeasurement_1_0.measurements.size() > 0);

	ignssMeasurement_1_0->close();
	/*gnssmeasurement 1.1接口*/
	auto gnssMeasurement_1_1 = gnss_hal_->getExtensionGnssMeasurement_1_1();
	if (!gnssMeasurement_1_1.isOk()) {
		return;
	}
	sp<IGnssMeasurement_1_1> iGnssMeasurement_1_1 = gnssMeasurement_1_1;
	if (iGnssMeasurement_1_1 == nullptr) {
		return;
	}
	sp<GnssMeasurementCallback> callback_1_1 = new GnssHalTest::GnssMeasurementCallback();
	auto result_setCallback1_1 = iGnssMeasurement_1_1->setCallback_1_1(callback_1_1, /* enableFullTracking= */ true);
	ASSERT_TRUE(result_setCallback1_1.isOk());
	EXPECT_EQ(result_setCallback1_1, IGnssMeasurement_1_0::GnssMeasurementStatus::SUCCESS);
	/*HIDL 观测量回调1.1接口没有用，调用的是1.0的接口
	ASSERT_TRUE(callback_1_1->gnssmeasurement_cbq_1_1.retrieve(callback_1_1->last_gnssmeasurement_1_1,
		kFirstGnssMeasurementTimeoutSeconds));
	EXPECT_EQ(callback_1_1->gnssmeasurement_cbq_1_1.calledCount(), 1);
	ASSERT_TRUE(callback_1_1->last_gnssmeasurement_1_1.measurements.size() > 0);
	*/
	iGnssMeasurement_1_1->close();
	/*gnssmeasurement 2.0接口*/
	auto gnssMeasurement_2_0 = gnss_hal_->getExtensionGnssMeasurement_2_0();
	if (!gnssMeasurement_2_0.isOk()) {
		return;
	}
	sp<IGnssMeasurement_2_0> iGnssMeasurement_2_0 = gnssMeasurement_2_0;
	if (iGnssMeasurement_2_0 == nullptr) {
		return;
	}
	sp<GnssMeasurementCallback> callback_2_0 = new GnssHalTest::GnssMeasurementCallback();
	auto result_setCallback2_0 = iGnssMeasurement_2_0->setCallback_2_0(callback_2_0, /* enableFullTracking= */ true);
	ASSERT_TRUE(result_setCallback2_0.isOk());
	EXPECT_EQ(result_setCallback2_0, IGnssMeasurement_1_0::GnssMeasurementStatus::SUCCESS);

	ASSERT_TRUE(callback_2_0->gnssmeasurement_cbq_2_0.retrieve(callback_2_0->last_gnssmeasurement_2_0,
		kFirstGnssMeasurementTimeoutSeconds));
	EXPECT_EQ(callback_2_0->gnssmeasurement_cbq_2_0.calledCount(), 1);
	ASSERT_TRUE(callback_2_0->last_gnssmeasurement_2_0.measurements.size() > 0);
	for (auto gnssmeasurement2_0 : callback_2_0->last_gnssmeasurement_2_0.measurements) {
		ASSERT_NE(gnssmeasurement2_0.codeType, "");
		ASSERT_TRUE(static_cast<uint8_t>(gnssmeasurement2_0.constellation) >=
			static_cast<uint8_t>(android::hardware::gnss::V2_0::GnssConstellationType::UNKNOWN) &&
			static_cast<uint8_t>(gnssmeasurement2_0.constellation) <=
			static_cast<uint8_t>(android::hardware::gnss::V2_0::GnssConstellationType::IRNSS));
		ASSERT_TRUE(
			static_cast<uint32_t>(gnssmeasurement2_0.state) >=
			static_cast<uint32_t>(IGnssMeasurementCallback_2_0::GnssMeasurementState::
			STATE_UNKNOWN) &&
			static_cast<uint32_t>(gnssmeasurement2_0.state) <=
			static_cast<uint32_t>(IGnssMeasurementCallback_2_0::GnssMeasurementState::
			STATE_2ND_CODE_LOCK));
	}

	iGnssMeasurement_2_0->close();
}

/*test for gnssmeasurementcorrection module*/
TEST_F(GnssHalTest, TestGnssMeasurementCorrectionsFields) {
	sp<GnssCallback> gnss_cb = new GnssHalTest::GnssCallback();
	if (!(gnss_cb->last_capabilities_ & IGnssCallback::Capabilities::MEASUREMENT_CORRECTIONS)) {
		return;
	}
	auto measurementCorrections = gnss_hal_->getExtensionMeasurementCorrections();
	ASSERT_TRUE(measurementCorrections.isOk());
	sp<IMeasurementCorrections> iMeasurementCorrections = measurementCorrections;
	ASSERT_NE(iMeasurementCorrections, nullptr);
	sp<GnssMeasurementCorrectionsCallback> callback = new GnssHalTest::GnssMeasurementCorrectionsCallback();
	auto result = iMeasurementCorrections->setCallback(callback);
	ASSERT_TRUE(result.isOk());
	EXPECT_TRUE(result);
	const int kMeasurementCorrectionsCapabilitiesTimeoutSeconds = 5;
	callback->capabilities_cbq_.retrieve(callback->last_capabilities_,
		kMeasurementCorrectionsCapabilitiesTimeoutSeconds);
	ASSERT_TRUE(callback->capabilities_cbq_.calledCount() > 0);

	auto result_setcorrect = iMeasurementCorrections->setCorrections(Utils::getMockMeasurementCorrections());
	ASSERT_TRUE(result_setcorrect.isOk());
	EXPECT_TRUE(result_setcorrect);

}

/*test for gnssnavigation module*/
TEST_F(GnssHalTest, TestGnssNavigationMessageFields) {
	const int kFirstGnssNaviTimeoutSeconds = 10;
	auto gnssNavigationMessage = gnss_hal_->getExtensionGnssNavigationMessage();
	if (!gnssNavigationMessage.isOk()) {
		return;
	}
	sp<IGnssNavigationMessage_1_0> iGnssNavigationMessage = gnssNavigationMessage;
	if (iGnssNavigationMessage == nullptr) {
		return;
	}
	sp<GnssNavigationMessageCallback> callback = new GnssHalTest::GnssNavigationMessageCallback();
	auto result = iGnssNavigationMessage->setCallback(callback);
	ASSERT_TRUE(result.isOk());
	ASSERT_TRUE(callback->gnssNavigationMessage_cbq_.retrieve(callback->last_gnssNavigationMessage_,
		kFirstGnssNaviTimeoutSeconds));
	EXPECT_EQ(callback->gnssNavigationMessage_cbq_.calledCount(), 1);
	iGnssNavigationMessage->close();
}

/*test for gnssvisibilityvontrol module*/
TEST_F(GnssHalTest, TestGnssVisibilityControlFields) {
	auto gnssVisibilityControl = gnss_hal_->getExtensionVisibilityControl();
	ASSERT_TRUE(gnssVisibilityControl.isOk());
	sp<IGnssVisibilityControl> iGnssVisibilityControl = gnssVisibilityControl;
	if (iGnssVisibilityControl == nullptr) {
		return;
	}
	hidl_vec<hidl_string> proxyApps{"com.example.ims", "com.example.mdt"};
	auto result_nfw = iGnssVisibilityControl->enableNfwLocationAccess(proxyApps);
	ASSERT_TRUE(result_nfw.isOk());
	sp<GnssVisibilityControlCallback> callback = new GnssHalTest::GnssVisibilityControlCallback();
	auto result_setcallback = iGnssVisibilityControl->setCallback(callback);
	EXPECT_TRUE(result_setcallback);
	int kFirstGnssVisibilityTimeoutSeconds = 10;
	ASSERT_TRUE(callback->nfwnotify_cbq_.retrieve(callback->last_nfwnotify_,
		kFirstGnssVisibilityTimeoutSeconds));
	EXPECT_EQ(callback->nfwnotify_cbq_.calledCount(), 1);
	EXPECT_EQ(callback->isinemergency_count, 1);
}

/*test for gnssni module*/
TEST_F(GnssHalTest, TestGnssNiFields) {
	auto gnssNi_1_0 = gnss_hal_->getExtensionGnssNi();
	ASSERT_TRUE(!gnssNi_1_0.isOk() || ((sp<IGnssNi_1_0>)gnssNi_1_0) == nullptr);
	/*NI已经弃用
	sp<GnssNiCallback> callback = new GnssHalTest::GnssNiCallback();
	auto result_setCallback = ignssNi_1_0->setCallback(callback);
	ASSERT_TRUE(result_setCallback.isOk());
	int kFirstGnssNiTimeoutSeconds = 10;
	ASSERT_TRUE(callback->gnssninotify_cbq_.retrieve(callback->last_gnssninotify_,
													kFirstGnssNiTimeoutSeconds));
	EXPECT_EQ(callback->gnssninotify_cbq_.calledCount(), 1);
	auto result_respond = ignssNi_1_0->respond(0, IGnssNiCallback_1_0::GnssUserResponseType::RESPONSE_ACCEPT);
	ASSERT_TRUE(result_respond.isOk());
	*/
}

/*test for agnssril module*/
TEST_F(GnssHalTest, TestAGnssRilFields) {
	/*AGnss 2.0接口*/
	auto agnssRil_2_0 = gnss_hal_->getExtensionAGnssRil_2_0();
	ASSERT_TRUE(agnssRil_2_0.isOk());
	sp<IAGnssRil_2_0> iagnssRil_2_0 = agnssRil_2_0;
	if (iagnssRil_2_0 == nullptr) {
		return;
	}
	sp<AGnssRilCallback> callback = new GnssHalTest::AGnssRilCallback();
	auto result = iagnssRil_2_0->setCallback(callback);
	ASSERT_TRUE(result.isOk());
	EXPECT_EQ(callback->lastrequestsetid_count, 1);
	EXPECT_EQ(callback->refloc_count, 1);
	IAGnssRil_2_0::AGnssRefLocation agnssreflocation = {
		.type = IAGnssRil_2_0::AGnssRefLocationType::GSM_CELLID,
		.cellID.type = IAGnssRil_2_0::AGnssRefLocationType::GSM_CELLID,
		.cellID.mcc = 1,
		.cellID.mnc = 1,
		.cellID.lac = 1,
		.cellID.cid = 1,
		.cellID.tac = 1,
		.cellID.pcid = 1
	};
	auto resultVoid = iagnssRil_2_0->setRefLocation(agnssreflocation);
	ASSERT_TRUE(resultVoid.isOk());
	auto result_setid = iagnssRil_2_0->setSetId(IAGnssRil_2_0::SetIDType::IMSI, "1");
	ASSERT_TRUE(result_setid.isOk());
	EXPECT_TRUE(result_setid);
	auto result_updatenetstate = iagnssRil_2_0->updateNetworkState(true, IAGnssRil_1_0::NetworkType::MOBILE, true);
	ASSERT_TRUE(result_setid.isOk());
	EXPECT_TRUE(result_setid);
	IAGnssRil_2_0::NetworkAttributes attributes ={
		.networkHandle = static_cast<uint64_t>(7700664333),
		.isConnected = true,
		.capabilities = static_cast<uint16_t>(IAGnssRil_2_0::NetworkCapability::NOT_ROAMING),
		.apn = "dummy-apn"
	};
	auto result_updateNetworkState_2_0 = iagnssRil_2_0->updateNetworkState_2_0(attributes);
	ASSERT_TRUE(result_updateNetworkState_2_0.isOk());
	EXPECT_TRUE(result_updateNetworkState_2_0);
	auto result_updatenetwork = iagnssRil_2_0->updateNetworkAvailability(true, "3gnet");
	ASSERT_TRUE(result_updatenetwork.isOk());
	EXPECT_TRUE(result_updatenetwork);
	attributes.isConnected = false;
	result_updateNetworkState_2_0 = iagnssRil_2_0->updateNetworkState_2_0(attributes);
	ASSERT_TRUE(result_updateNetworkState_2_0.isOk());
	EXPECT_TRUE(result_updateNetworkState_2_0);
	//auto result_nimessag = iagnssRil_2_0->agpsNiMessage("data", 4);
	//ASSERT_TRUE(result_nimessag.isOk());
	//EXPECT_TRUE(result_nimessag);
}

/*test for gnssxtra module*/
TEST_F(GnssHalTest, TestGnssXtraFields) {
	auto gnssXtra_1_0 = gnss_hal_->getExtensionXtra();
	ASSERT_TRUE(gnssXtra_1_0.isOk());
	sp<IGnssXtra_1_0> ignssXtra_1_0 = gnssXtra_1_0;
	if (ignssXtra_1_0 == nullptr) {
		return;
	}
	sp<GnssXtraCallback> callback = new GnssHalTest::GnssXtraCallback();
	auto result_setCallback = ignssXtra_1_0->setCallback(callback);
	ASSERT_TRUE(result_setCallback.isOk());
	auto result_injectXtra = ignssXtra_1_0->injectXtraData("gnssxtra");
	ASSERT_TRUE(result_injectXtra.isOk());
	EXPECT_TRUE(result_injectXtra);
	EXPECT_EQ(callback->downloadrequest_count, 1);
}

/*test for gnssgeofenl module*/
TEST_F(GnssHalTest, TestGnssGeofenceFields) {
	auto gnssGeofen_1_0 = gnss_hal_->getExtensionGnssGeofencing();
	ASSERT_TRUE(gnssGeofen_1_0.isOk());
	sp<IGnssGeofencing_1_0> ignssGeofen_1_0 = gnssGeofen_1_0;
	if (ignssGeofen_1_0 == nullptr) {
		return;
	}
	sp<GnssGeofenceCallback> callback = new GnssHalTest::GnssGeofenceCallback();
	auto result_setcallback = ignssGeofen_1_0->setCallback(callback);
	ASSERT_TRUE(result_setcallback.isOk());
	EXPECT_EQ(callback->geofentransition_count, 1);
	EXPECT_EQ(callback->geofenstatus_count, 1);
	EXPECT_EQ(callback->geofenadd_count, 1);
	EXPECT_EQ(callback->geofenremove_count, 1);
	EXPECT_EQ(callback->geofenpause_count, 1);
	EXPECT_EQ(callback->geofenresume_count, 1);
	auto result_add = ignssGeofen_1_0->addGeofence(1, 26.000000,30.000000,50.000000,IGnssGeofenceCallback_1_0::GeofenceTransition::ENTERED,1,2,3);
	ASSERT_TRUE(result_add.isOk());
	auto result_pause = ignssGeofen_1_0->pauseGeofence(1);
	ASSERT_TRUE(result_pause.isOk());
	auto result_resume = ignssGeofen_1_0->resumeGeofence(1, 1);
	ASSERT_TRUE(result_resume.isOk());
	auto result_remove = ignssGeofen_1_0->removeGeofence(1);
	ASSERT_TRUE(result_remove.isOk());
}

/*test for gnssconfiguration module*/
TEST_F(GnssHalTest, TestGnssConfigurationFields) {
	 /*gnssConfiguration 1.0接口*/
	 auto gnssConfiguration_1_0 = gnss_hal_->getExtensionGnssConfiguration();
	 ASSERT_TRUE(gnssConfiguration_1_0.isOk());
	 sp<IGnssConfiguration_1_0> iGnssConfiguration_1_0 = gnssConfiguration_1_0;
	 ASSERT_NE(iGnssConfiguration_1_0, nullptr);
	 auto result_setsupl = iGnssConfiguration_1_0->setSuplEs(false);
	 ASSERT_TRUE(result_setsupl.isOk());
	 auto result_setlock = iGnssConfiguration_1_0->setGpsLock(0);
	 ASSERT_TRUE(result_setlock.isOk());
	 auto result_setversion = iGnssConfiguration_1_0->setSuplVersion(1);
	 ASSERT_TRUE(result_setversion.isOk());
	 auto result_setmode = iGnssConfiguration_1_0->setSuplMode(1);
	 ASSERT_TRUE(result_setmode.isOk());
	 auto result_setlpp = iGnssConfiguration_1_0->setLppProfile(1);
	 ASSERT_TRUE(result_setlpp.isOk());
	 auto result_setglonass = iGnssConfiguration_1_0->setGlonassPositioningProtocol(1);
	 ASSERT_TRUE(result_setglonass.isOk());
	 auto result_setemerg = iGnssConfiguration_1_0->setEmergencySuplPdn(true);
	 ASSERT_TRUE(result_setemerg.isOk());
	 /*gnssConfiguration 1.1接口*/
	 auto gnssConfiguration_1_1 = gnss_hal_->getExtensionGnssConfiguration_1_1();
	 ASSERT_TRUE(gnssConfiguration_1_1.isOk());
	 sp<IGnssConfiguration_1_1> iGnssConfiguration_1_1 = gnssConfiguration_1_1;
	 IGnssConfiguration_1_1::BlacklistedSource blacklist = {
		.constellation = android::hardware::gnss::V1_0::GnssConstellationType::GLONASS,
		.svid = 1,
	 };
	 hidl_vec<IGnssConfiguration_1_1::BlacklistedSource> sources;
	 sources.resize(1);
	 sources[0] = blacklist;
	 auto result_setblack = iGnssConfiguration_1_1->setBlacklist(sources);
	 ASSERT_TRUE(result_setblack.isOk());
	 /*gnssConfiguration 2.0接口*/
	 auto gnssConfiguration_2_0 = gnss_hal_->getExtensionGnssConfiguration_2_0();
	 ASSERT_TRUE(gnssConfiguration_2_0.isOk());
	 sp<IGnssConfiguration_2_0> iGnssConfiguration_2_0 = gnssConfiguration_2_0;
	 ASSERT_NE(iGnssConfiguration_2_0, nullptr);
	 auto result_setesex = iGnssConfiguration_2_0->setEsExtensionSec(180);
	 ASSERT_TRUE(result_setesex.isOk());
}

/*该接口无法调用
TEST_F(GnssHalTest, TestGnssUnisocExtFields) {
	  vendor_gnss_hal_ = ::testing::VtsHalHidlTargetTestBase::getService<IGnss>(
		GnssHidlEnvironment::Instance()->getServiceName<>(vendor_IGnss));
	ASSERT_NE(gnss_hal_, nullptr);
	auto gnssUnisocExt_1_0 = vendor_gnss_hal_->getExtensionUnisocExt();
	ASSERT_TRUE(gnssUnisocExt_1_0.isOk());
	sp<IGnssUnisocExt_1_0> iGnssUnisocExt_1_0 = gnssUnisocExt_1_0;
	if (iIGnssUnisocExt_1_0 == nullptr) {
		return;
	}
	auto result = iGnssUnisocExt_1_0->setNetWorkHandle(1);
	ASSERT_TRUE(result.isOk());
	EXPECT_TRUE(result);
}*/

/*test for gnssdebug module*/
TEST_F(GnssHalTest, TestGnssDebugFields) {
	/*gnssdebug 1.0接口*/
	auto gnssDebug_1_0 = gnss_hal_->getExtensionGnssDebug();
	ASSERT_TRUE(gnssDebug_1_0.isOk());
	sp<IGnssDebug_1_0> ignssDebug_1_0 = gnssDebug_1_0;
	if (ignssDebug_1_0 == nullptr) {
		return;
	}
	IGnssDebug_1_0::DebugData data;
	ignssDebug_1_0->getDebugData(
		[&data](const IGnssDebug_1_0::DebugData& debugData) { data = debugData; });
	if (data.position.valid) {
		EXPECT_GE(data.position.latitudeDegrees, -90);
		EXPECT_LE(data.position.latitudeDegrees, 90);

		EXPECT_GE(data.position.longitudeDegrees, -180);
		EXPECT_LE(data.position.longitudeDegrees, 180);

		EXPECT_GE(data.position.altitudeMeters, -1000);
		EXPECT_LE(data.position.altitudeMeters, 20000);

		EXPECT_GE(data.position.speedMetersPerSec, 0);
		EXPECT_LE(data.position.speedMetersPerSec, 600);

		EXPECT_GE(data.position.bearingDegrees, -360);
		EXPECT_LE(data.position.bearingDegrees, 360);

		EXPECT_GT(data.position.horizontalAccuracyMeters, 0);
		EXPECT_LE(data.position.horizontalAccuracyMeters, 20000000);

		EXPECT_GT(data.position.verticalAccuracyMeters, 0);
		EXPECT_LE(data.position.verticalAccuracyMeters, 20000);

		EXPECT_GT(data.position.speedAccuracyMetersPerSecond, 0);
		EXPECT_LE(data.position.speedAccuracyMetersPerSecond, 500);

		EXPECT_GT(data.position.bearingAccuracyDegrees, 0);
		EXPECT_LE(data.position.bearingAccuracyDegrees, 180);

		EXPECT_GE(data.position.ageSeconds, 0);
	}
	EXPECT_GE(data.time.timeEstimate, 1483228800000);
	EXPECT_GT(data.time.timeUncertaintyNs, 0);
	EXPECT_GT(data.time.frequencyUncertaintyNsPerSec, 0);
	EXPECT_LE(data.time.frequencyUncertaintyNsPerSec, 2.0e5);
	/*gnssdebug 2.0接口*/
	auto gnssDebug_2_0 = gnss_hal_->getExtensionGnssDebug_2_0();
	ASSERT_TRUE(gnssDebug_2_0.isOk());
	sp<IGnssDebug_2_0> ignssDebug_2_0 = gnssDebug_2_0;
	if (ignssDebug_2_0 == nullptr) {
		return;
	}
	IGnssDebug_2_0::DebugData data2_0;
	ignssDebug_2_0->getDebugData_2_0(
		[&data2_0](const IGnssDebug_2_0::DebugData& debugData) { data2_0 = debugData; });
	if (data2_0.position.valid) {
		EXPECT_GE(data2_0.position.latitudeDegrees, -90);
		EXPECT_LE(data2_0.position.latitudeDegrees, 90);

		EXPECT_GE(data2_0.position.longitudeDegrees, -180);
		EXPECT_LE(data2_0.position.longitudeDegrees, 180);

		EXPECT_GE(data2_0.position.altitudeMeters, -1000);
		EXPECT_LE(data2_0.position.altitudeMeters, 20000);

		EXPECT_GE(data2_0.position.speedMetersPerSec, 0);
		EXPECT_LE(data2_0.position.speedMetersPerSec, 600);

		EXPECT_GE(data2_0.position.bearingDegrees, -360);
		EXPECT_LE(data2_0.position.bearingDegrees, 360);

		EXPECT_GT(data2_0.position.horizontalAccuracyMeters, 0);
		EXPECT_LE(data2_0.position.horizontalAccuracyMeters, 20000000);

		EXPECT_GT(data2_0.position.verticalAccuracyMeters, 0);
		EXPECT_LE(data2_0.position.verticalAccuracyMeters, 20000);

		EXPECT_GT(data2_0.position.speedAccuracyMetersPerSecond, 0);
		EXPECT_LE(data2_0.position.speedAccuracyMetersPerSecond, 500);

		EXPECT_GT(data2_0.position.bearingAccuracyDegrees, 0);
		EXPECT_LE(data2_0.position.bearingAccuracyDegrees, 180);

		EXPECT_GE(data2_0.position.ageSeconds, 0);
	}
	EXPECT_GE(data2_0.time.timeEstimate, 1483228800000);
	EXPECT_GT(data2_0.time.timeUncertaintyNs, 0);
	EXPECT_GT(data2_0.time.frequencyUncertaintyNsPerSec, 0);
	EXPECT_LE(data2_0.time.frequencyUncertaintyNsPerSec, 2.0e5);
}

/*test for gnssbatching module*/
TEST_F(GnssHalTest, TestGnssBatchingFields) {
	auto gnssBatching_2_0 = gnss_hal_->getExtensionGnssBatching_2_0();
	ASSERT_TRUE(gnssBatching_2_0.isOk());
	auto gnssBatching_1_0 = gnss_hal_->getExtensionGnssBatching();
	ASSERT_TRUE(gnssBatching_1_0.isOk());
}

