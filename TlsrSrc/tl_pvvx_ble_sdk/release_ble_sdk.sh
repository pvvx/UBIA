#record sdk version
SDK_VERSION=3.2.0

dir_release=ble_sdk_release

rm -rf ../$dir_release

mkdir ../$dir_release
cp -rf proj proj_lib  ../$dir_release


mkdir ../$dir_release/vendor
cp -rf vendor/826x_ble_remote ../$dir_release/vendor
cp -rf vendor/826x_module ../$dir_release/vendor
cp -rf vendor/826x_ota_boot ../$dir_release/vendor
cp -rf vendor/826x_driver_test ../$dir_release/vendor
cp -rf vendor/826x_feature_test ../$dir_release/vendor
cp -rf vendor/826x_hci ../$dir_release/vendor
cp -rf vendor/826x_hid_sample ../$dir_release/vendor
cp -rf vendor/826x_master_kma_dongle ../$dir_release/vendor
cp -rf vendor/common ../$dir_release/vendor


cp * .project .cproject ../$dir_release

rm -rf ../$dir_release/release_ble_sdk.sh 
rm -rf ../$dir_release/telink_ble_sdk_release GIT log.txt

