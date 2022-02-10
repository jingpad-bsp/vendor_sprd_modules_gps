#ifndef GNSS_HAL_TEST_H_
#define GNSS_HAL_TEST_H_

#include <android/hardware/gnss/2.0/IGnss.h>
#include <VtsHalHidlTargetTestBase.h>
#include <VtsHalHidlTargetTestEnvBase.h>
#include <hardware/gps.h>
#include <condition_variable>
#include <deque>
#include <mutex>

using android::hardware::hidl_vec;
using android::hardware::Return;
using android::hardware::Void;
using android::sp;
using android::hardware::gnss::measurement_corrections::V1_0::IMeasurementCorrectionsCallback;
using android::hardware::gnss::V1_0::GnssLocationFlags;
using android::hardware::gnss::V2_0::IGnss;

using GnssLocation_1_0 = android::hardware::gnss::V1_0::GnssLocation;
using GnssLocation_2_0 = android::hardware::gnss::V2_0::GnssLocation;
using IGnssCallback_1_0 = android::hardware::gnss::V1_0::IGnssCallback;
using IGnssCallback_2_0 = android::hardware::gnss::V2_0::IGnssCallback;
using IGnssMeasurementCallback_1_0 = android::hardware::gnss::V1_0::IGnssMeasurementCallback;
using IGnssMeasurementCallback_1_1 = android::hardware::gnss::V1_1::IGnssMeasurementCallback;
using IGnssMeasurementCallback_2_0 = android::hardware::gnss::V2_0::IGnssMeasurementCallback;
using IAGnssCallback_2_0 = android::hardware::gnss::V2_0::IAGnssCallback;
using IGnssNavigationMessageCallback_1_0 = android::hardware::gnss::V1_0::IGnssNavigationMessageCallback;
using IGnssVisibilityControlCallback_1_0 = android::hardware::gnss::visibility_control::V1_0::IGnssVisibilityControlCallback;
using IGnssGeofenceCallback_1_0 = android::hardware::gnss::V1_0::IGnssGeofenceCallback;
using IGnssNiCallback_1_0 = android::hardware::gnss::V1_0::IGnssNiCallback;
using IGnssXtraCallback_1_0 = android::hardware::gnss::V1_0::IGnssXtraCallback;
using IAGnssRilCallback_1_0 = android::hardware::gnss::V1_0::IAGnssRilCallback;

#define TIMEOUT_SEC 10  // for basic commands/responses

// Test environment for GNSS HIDL HAL.
class GnssHidlEnvironment : public ::testing::VtsHalHidlTargetTestEnvBase {
	public:
		// get the test environment singleton
		static GnssHidlEnvironment* Instance() {
			static GnssHidlEnvironment* instance = new GnssHidlEnvironment;
			return instance;
		}
		virtual void registerTestServices() override { registerTestService<IGnss>(); }
	private:
		GnssHidlEnvironment() {}
};

// The main test class for GNSS HAL.
class GnssHalTest : public ::testing::VtsHalHidlTargetTestBase {
	public:
		virtual void SetUp() override;
		virtual void TearDown() override;
		/* Producer/consumer queue for storing/retrieving callback events from GNSS HAL */
		template <class T>
		class CallbackQueue {
			public:
				CallbackQueue(const std::string& name) : name_(name), called_count_(0){};
				~CallbackQueue() { reset(); }
		/* Adds callback event to the end of the queue. */
		void store(const T& event);
		/*
		 * Removes the callack event at the front of the queue, stores it in event parameter
		 * and returns true. Returns false on timeout and event is not populated.
		 */
		bool retrieve(T& event, int timeout_seconds);
		/* Returns the number of events pending to be retrieved from the callback event queue. */
		int size() const;
		/* Returns the number of callback events received since last reset(). */
		int calledCount() const;
		/* Clears the callback event queue and resets the calledCount() to 0. */
		void reset();
	private:
		CallbackQueue(const CallbackQueue&) = delete;
		CallbackQueue& operator=(const CallbackQueue&) = delete;

		std::string name_;
		int called_count_;
		mutable std::recursive_mutex mtx_;
		std::condition_variable_any cv_;
		std::deque<T> events_;
	};

	/* Callback class for data & Event. */
	class GnssCallback : public IGnssCallback_2_0 {
		public:
			IGnssCallback_1_0::GnssSystemInfo last_info_;
			android::hardware::hidl_string last_name_;
			uint32_t last_capabilities_;
			GnssLocation_2_0 last_location_2_0;
			GnssLocation_1_0 last_location_1_0;
			IGnssCallback_1_0::GnssStatusValue last_status_;
			IGnssCallback_1_0::GnssSvStatus last_gnss_sv_status_;
			int nmea_count;
			int requesttime_count;

			CallbackQueue<IGnssCallback_1_0::GnssSystemInfo> info_cbq_;
			CallbackQueue<uint32_t> capabilities_cbq_;
			CallbackQueue<uint32_t> capabilities_cbq_2_0;
			CallbackQueue<GnssLocation_1_0> location_cbq_1_0;
			CallbackQueue<GnssLocation_2_0> location_cbq_2_0;
			CallbackQueue<IGnssCallback_1_0::GnssStatusValue> status_cbq_;
			CallbackQueue<IGnssCallback_1_0::GnssSvStatus> gnss_sv_status_cbq_;
			CallbackQueue<android::hardware::hidl_string> name_cbq_;
			CallbackQueue<bool> request_location_cbq_;

			GnssCallback()
				:
				  nmea_count(0),
				  requesttime_count(0),
				  info_cbq_("system_info"),
				  capabilities_cbq_("capabilities"),
				  capabilities_cbq_2_0("capabilities 2.0"),
				  location_cbq_1_0("location1.0"),
				  location_cbq_2_0("location2.0"),
				  status_cbq_("status"),
				  gnss_sv_status_cbq_("gnss_svstatus"),
				  name_cbq_("name"),
				  request_location_cbq_("request location"){};
			virtual ~GnssCallback() = default;
			// Methods from ::android::hardware::gnss::V1_0::IGnssCallback follow.
			Return<void> gnssStatusCb(IGnssCallback_1_0::GnssStatusValue status) override;
			Return<void> gnssNameCb(const android::hardware::hidl_string& name) override;
			Return<void> gnssRequestTimeCb() override;
			Return<void> gnssAcquireWakelockCb() override { return Void(); }
			Return<void> gnssReleaseWakelockCb() override { return Void(); }
			Return<void> gnssRequestLocationCb(bool request) override;
			Return<void> gnssLocationCb(const GnssLocation_1_0& location) override;
			Return<void> gnssSetCapabilitesCb(uint32_t capabilities) override;
			Return<void> gnssSetSystemInfoCb(const IGnssCallback_1_0::GnssSystemInfo& info) override;
			Return<void> gnssSvStatusCb(const IGnssCallback_1_0::GnssSvStatus& svStatus) override;
			// Methods from ::android::hardware::gnss::V1_1::IGnssCallback follow.
			Return<void> gnssNmeaCb(int64_t timestamp,   const android::hardware::hidl_string& nmea) override;
			// Methods from ::android::hardware::gnss::V2_0::IGnssCallback follow.
			Return<void> gnssLocationCb_2_0(const GnssLocation_2_0& location) override;
			Return<void> gnssSetCapabilitiesCb_2_0(uint32_t capabilities) override;
			Return<void> gnssRequestLocationCb_2_0(bool /* independentFromGnss */,
				bool /* isUserEmergency */) override {
				return Void();
			}
			Return<void> gnssSvStatusCb_2_0(
				const hidl_vec<IGnssCallback_2_0::GnssSvInfo>& /*svInfoList*/) override{ return Void();}
			};

	/* Callback class for GnssMeasurement. */
	class GnssMeasurementCallback : public IGnssMeasurementCallback_2_0 {
		public:
			IGnssMeasurementCallback_1_0::GnssData last_gnssmeasurement_1_0;
			IGnssMeasurementCallback_1_1::GnssData last_gnssmeasurement_1_1;
			IGnssMeasurementCallback_2_0::GnssData last_gnssmeasurement_2_0;

			CallbackQueue<IGnssMeasurementCallback_1_0::GnssData> gnssmeasurement_cbq_1_0;
			CallbackQueue<IGnssMeasurementCallback_1_1::GnssData> gnssmeasurement_cbq_1_1;
			CallbackQueue<IGnssMeasurementCallback_2_0::GnssData> gnssmeasurement_cbq_2_0;

			GnssMeasurementCallback()
				: gnssmeasurement_cbq_1_0("gnssmeasurement1_0"),
				  gnssmeasurement_cbq_1_1("gnssmeasurement1_1"),
				  gnssmeasurement_cbq_2_0("gnssmeasurement2_0"){};
			virtual ~GnssMeasurementCallback() = default;
			Return<void> GnssMeasurementCb(const IGnssMeasurementCallback_1_0::GnssData& data) override;
			Return<void> gnssMeasurementCb(const IGnssMeasurementCallback_1_1::GnssData& data) override;
			Return<void> gnssMeasurementCb_2_0(const IGnssMeasurementCallback_2_0::GnssData&) override;
	};

	/* Callback class for GnssMeasurementCorrections. */
	class GnssMeasurementCorrectionsCallback : public IMeasurementCorrectionsCallback {
		public:
			uint32_t last_capabilities_;

			CallbackQueue<uint32_t> capabilities_cbq_;

			GnssMeasurementCorrectionsCallback()
				: capabilities_cbq_("capabilities"){};
			virtual ~GnssMeasurementCorrectionsCallback() = default;
			Return<void> setCapabilitiesCb(uint32_t capabilities) override;
	};

	/* Callback class for GnssVisibilityControl. */
	class GnssVisibilityControlCallback : public IGnssVisibilityControlCallback_1_0 {
		public:
			IGnssVisibilityControlCallback_1_0::NfwNotification last_nfwnotify_;
			int isinemergency_count;

			CallbackQueue<IGnssVisibilityControlCallback_1_0::NfwNotification> nfwnotify_cbq_;

			GnssVisibilityControlCallback()
				: isinemergency_count(0), nfwnotify_cbq_("nfwnotify"){};
			virtual ~GnssVisibilityControlCallback() = default;
			Return<void> nfwNotifyCb(const IGnssVisibilityControlCallback_1_0::NfwNotification& notification) override;
			Return<bool> isInEmergencySession();
	};

	/* Callback class for AGnss. */
	class AGnssCallback : public IAGnssCallback_2_0 {
		public:
			IAGnssCallback_2_0::AGnssStatusValue last_agnss_status_;

			CallbackQueue<IAGnssCallback_2_0::AGnssStatusValue> agnss_status_cbq_;

			AGnssCallback()
				: agnss_status_cbq_("agnss"){};
			virtual ~AGnssCallback() = default;
			Return<void> agnssStatusCb(IAGnssCallback_2_0::AGnssType type, IAGnssCallback_2_0::AGnssStatusValue status) override;
	};

	/* Callback class for GnssNavigationMessage. */
	class GnssNavigationMessageCallback : public IGnssNavigationMessageCallback_1_0 {
		public:
			IGnssNavigationMessageCallback_1_0::GnssNavigationMessage last_gnssNavigationMessage_;

			CallbackQueue<IGnssNavigationMessageCallback_1_0::GnssNavigationMessage> gnssNavigationMessage_cbq_;

			GnssNavigationMessageCallback()
				: gnssNavigationMessage_cbq_("gnssNavigationMessage"){};
			virtual ~GnssNavigationMessageCallback() = default;
			// Methods from ::android::hardware::gnss::V1_0::IGnssNavigationMessageCallback follow.
			Return<void> gnssNavigationMessageCb(const IGnssNavigationMessageCallback_1_0::GnssNavigationMessage& message) override;
	};

	/* Callback class for GnssGeofence. */
	class GnssGeofenceCallback : public IGnssGeofenceCallback_1_0 {
		public:
			int geofentransition_count;
			int geofenstatus_count;
			int geofenadd_count;
			int geofenremove_count;
			int geofenpause_count;
			int geofenresume_count;
			GnssGeofenceCallback()
				: geofentransition_count(0),
				  geofenstatus_count(0),
				  geofenadd_count(0),
				  geofenremove_count(0),
				  geofenpause_count(0),
				  geofenresume_count(0){};
			virtual ~GnssGeofenceCallback() = default;
			// Methods from ::android::hardware::gnss::V1_0::IGnssGeofenceCallback follow.
			Return<void> gnssGeofenceTransitionCb(int32_t geofenceId, const GnssLocation_1_0& location, IGnssGeofenceCallback_1_0::GeofenceTransition transition, int64_t timestamp) override;
			Return<void> gnssGeofenceStatusCb(IGnssGeofenceCallback_1_0::GeofenceAvailability status, const GnssLocation_1_0& lastLocation) override;
			Return<void> gnssGeofenceAddCb(int32_t geofenceId, IGnssGeofenceCallback_1_0::GeofenceStatus status) override;
			Return<void> gnssGeofenceRemoveCb(int32_t geofenceId, IGnssGeofenceCallback_1_0::GeofenceStatus status) override;
			Return<void> gnssGeofencePauseCb(int32_t geofenceId, IGnssGeofenceCallback_1_0::GeofenceStatus status) override;
			Return<void> gnssGeofenceResumeCb(int32_t geofenceId, IGnssGeofenceCallback_1_0::GeofenceStatus status) override;
	};

	/* Callback class for GnssRil. */
	class AGnssRilCallback : public IAGnssRilCallback_1_0 {
		public:
			int lastrequestsetid_count;
			int refloc_count;

			AGnssRilCallback()
				: lastrequestsetid_count(0),
				  refloc_count(0){};
			virtual ~AGnssRilCallback() = default;
			Return<void> requestRefLocCb() override;
			Return<void> requestSetIdCb(uint32_t flags) override;
	};

	/* Callback class for GnssNi. */
	class GnssNiCallback : public IGnssNiCallback_1_0 {
		public:
			IGnssNiCallback_1_0::GnssNiNotification last_gnssninotify_;

			CallbackQueue<IGnssNiCallback_1_0::GnssNiNotification> gnssninotify_cbq_;

			GnssNiCallback()
				: gnssninotify_cbq_("gnssninotify"){};
			virtual ~GnssNiCallback() = default;
			Return<void> niNotifyCb(const IGnssNiCallback_1_0::GnssNiNotification& notification) override;
	};

	/* Callback class for GnssXtra. */
	class GnssXtraCallback : public IGnssXtraCallback_1_0 {
		public:
			int downloadrequest_count;
			GnssXtraCallback() : downloadrequest_count(0){};
			virtual ~GnssXtraCallback() = default;
			Return<void> downloadRequestCb() override;
	};

	void SetUpGnssCallback();
	bool StartAndCheckFirstLocation();
	void CheckLocation(const GnssLocation_2_0& location, const bool check_speed);
	void StartAndCheckLocations(int count);
	void StopAndClearLocations();
	void SetPositionMode(const int min_interval_msec, const bool low_power_mode);

	sp<IGnss> gnss_hal_;

};

template <class T>
void GnssHalTest::CallbackQueue<T>::store(const T& event) {
	std::unique_lock<std::recursive_mutex> lock(mtx_);
	events_.push_back(event);
	++called_count_;
	lock.unlock();
	cv_.notify_all();
}

template <class T>
bool GnssHalTest::CallbackQueue<T>::retrieve(T& event, int timeout_seconds) {
	std::unique_lock<std::recursive_mutex> lock(mtx_);
	cv_.wait_for(lock, std::chrono::seconds(timeout_seconds), [&] { return !events_.empty(); });
	if (events_.empty()) {
		return false;
	}
	event = events_.front();
	events_.pop_front();
	return true;
}

template <class T>
int GnssHalTest::CallbackQueue<T>::size() const {
	std::unique_lock<std::recursive_mutex> lock(mtx_);
	return events_.size();
}

template <class T>
int GnssHalTest::CallbackQueue<T>::calledCount() const {
	std::unique_lock<std::recursive_mutex> lock(mtx_);
	return called_count_;
}

template <class T>
void GnssHalTest::CallbackQueue<T>::reset() {
	std::unique_lock<std::recursive_mutex> lock(mtx_);
	if (!events_.empty()) {
		ALOGW("%u unprocessed events discarded in callback queue %s", (unsigned int)events_.size(),
			  name_.c_str());
	}
	events_.clear();
	called_count_ = 0;
}

#endif
