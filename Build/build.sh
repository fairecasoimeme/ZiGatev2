 #!/bin/sh
APP_NOTE_NAME=JN-AN-1247-Zigbee-3-0-IoT-ControlBridge

BASE=$(cd ../../..; pwd)
ZIP=winzip32
EXPORT_PATH=SDK/$APP_NOTE_NAME

make -C JN518x_lpx/ TRACE=1 PDM_BUILD_TYPE=_EEPROM NODE=COORDINATOR BAUD=1000000 clean
make -C JN518x_lpx/ TRACE=1 PDM_BUILD_TYPE=_EEPROM NODE=COORDINATOR BAUD=1000000  
make -C JN518x_lpx/ TRACE=1 PDM_BUILD_TYPE=_EEPROM NODE=COORDINATOR BAUD=115200  clean
make -C JN518x_lpx/ TRACE=1 PDM_BUILD_TYPE=_EEPROM NODE=COORDINATOR BAUD=115200  
make -C JN518x_lpx/ TRACE=1 PDM_BUILD_TYPE=_EEPROM NODE=FULL_FUNC_DEVICE BAUD=1000000  clean
make -C JN518x_lpx/ TRACE=1 PDM_BUILD_TYPE=_EEPROM NODE=FULL_FUNC_DEVICE BAUD=1000000  
make -C JN518x_lpx/ TRACE=1 PDM_BUILD_TYPE=_EEPROM NODE=FULL_FUNC_DEVICE BAUD=115200  clean
make -C JN518x_lpx/ TRACE=1 PDM_BUILD_TYPE=_EEPROM NODE=FULL_FUNC_DEVICE BAUD=115200  
mkdir -p SDK

svn export --depth=files ../ $EXPORT_PATH 
mkdir $EXPORT_PATH/Source
mkdir $EXPORT_PATH/Doc
mkdir $EXPORT_PATH/Tools
svn export ../Source/board $EXPORT_PATH/Source/board/
svn export ../Source/Common $EXPORT_PATH/Source/Common/
svn export ../Source/ZigbeeNodeControlBridge $EXPORT_PATH/Source/ZigbeeNodeControlBridge
svn export ../JN518x_lpx $EXPORT_PATH/JN518x_lpx
svn export ../Tools/TestGUI $EXPORT_PATH/Tools/TestGUI
svn export ../Tools/TestGUI/Source $EXPORT_PATH/Tools/TestGUI/Source 
#svn export ../Admin $EXPORT_PATH/Admin
svn export ../Doc/JN-AN-1216-ZigBee-3-0-ControlBridge-UserGuide.pdf  $EXPORT_PATH/Doc/
mkdir $EXPORT_PATH/Build
mkdir $EXPORT_PATH/Build/JN518x_lpx

chmod 777 JN518x_lpx/Makefile
cp JN518x_lpx/*.bin $EXPORT_PATH/Build/JN518x_lpx/
cp JN518x_lpx/Makefile $EXPORT_PATH/Build/JN518x_lpx/Makefile
rm -f $APP_NOTE_NAME.zip
cd SDK
$ZIP -a -r ../$APP_NOTE_NAME.zip * 
cd ..
rm -rf SDK