#!/bin/bash
adb shell find vendor/lib64 >vts_temp_result.txt
if [ -s vts_temp_result.txt ];then
	echo "nativetest64"
	cd Documents/android-vts/testcases/DATA/nativetest64/VtsHalGnssV2_0TargetTest
	mv VtsHalGnssV2_0TargetTest VtsHalGnssV2_0TargetTest_main
	mv VtsHalGnssV2_0TargetTest_Mock VtsHalGnssV2_0TargetTest
	cd ~
	cd Documents/android-vts/testcases/DATA/nativetest/VtsHalGnssV2_0TargetTest
	mv VtsHalGnssV2_0TargetTest VtsHalGnssV2_0TargetTest_main
	cd ~
	adb root
	adb remount
	adb shell mv -i vendor/lib64/libgnssmgt.so vendor/lib64/libgnssmgt_main.so
	adb shell mv -i vendor/lib64/libgnssmgtmock.so vendor/lib64/libgnssmgt.so
	adb shell pkill "gnss"
	echo Dynamic library is ready
	echo VTS_64 is begining
	cd Documents/android-vts/tools
	./vts-tradefed run commandAndExit vts -m VtsHalGnssV2_0Target --skip-device-info
	cd ~
	cd Documents/android-vts/testcases/DATA/nativetest64/VtsHalGnssV2_0TargetTest
	mv VtsHalGnssV2_0TargetTest VtsHalGnssV2_0TargetTest_Mock
	mv VtsHalGnssV2_0TargetTest_main VtsHalGnssV2_0TargetTest
	cd ~
	cd Documents/android-vts/testcases/DATA/nativetest/VtsHalGnssV2_0TargetTest
	mv VtsHalGnssV2_0TargetTest_main VtsHalGnssV2_0TargetTest
	cd ~
	adb root
	adb remount
	adb shell mv -i vendor/lib64/libgnssmgt.so vendor/lib64/libgnssmgtmock.so
	adb shell mv -i vendor/lib64/libgnssmgt_main.so vendor/lib64/libgnssmgt.so
	echo VTS_64 is completed
else
	echo "nativetest32"
	cd Documents/android-vts/testcases/DATA/nativetest/VtsHalGnssV2_0TargetTest
	mv VtsHalGnssV2_0TargetTest VtsHalGnssV2_0TargetTest_main
	mv VtsHalGnssV2_0TargetTest_Mock VtsHalGnssV2_0TargetTest
	cd ~
	adb root
	adb remount
	adb shell mv -i vendor/lib/libgnssmgt.so vendor/lib/libgnssmgt_main.so
	adb shell mv -i vendor/lib/libgnssmgtmock.so vendor/lib/libgnssmgt.so
	adb shell pkill "gnss"
	echo Dynamic library is ready
	echo VTS_32 is begining
	cd Documents/android-vts/tools
	./vts-tradefed run commandAndExit vts -m VtsHalGnssV2_0Target --skip-device-info
	cd ~
	cd Documents/android-vts/testcases/DATA/nativetest/VtsHalGnssV2_0TargetTest
	mv VtsHalGnssV2_0TargetTest VtsHalGnssV2_0TargetTest_Mock
	mv VtsHalGnssV2_0TargetTest_main VtsHalGnssV2_0TargetTest
	cd ~
	adb root
	adb remount
	adb shell mv -i vendor/lib/libgnssmgt.so vendor/lib/libgnssmgtmock.so
	adb shell mv -i vendor/lib/libgnssmgt_main.so vendor/lib/libgnssmgt.so
	echo VTS_32 is completed
fi
rm vts_temp_result.txt
adb reboot


