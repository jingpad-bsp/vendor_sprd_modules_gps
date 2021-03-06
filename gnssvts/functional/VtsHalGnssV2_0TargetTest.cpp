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
#define LOG_TAG "VtsHalGnssV2_0TargetTest_Mock"

#include <VtsHalHidlTargetTestBase.h>

#include "gnss_hal_test.h"

int main(int argc, char** argv) {
    ::testing::AddGlobalTestEnvironment(GnssHidlEnvironment::Instance());
    ::testing::InitGoogleTest(&argc, argv);
    GnssHidlEnvironment::Instance()->init(&argc, argv);
    // TODO (b/122463165): Expand coverage to include 1.1 and 1.0 VTS tests.
    int status = RUN_ALL_TESTS();
    ALOGI("Test result = %d", status);
    return status;
}
