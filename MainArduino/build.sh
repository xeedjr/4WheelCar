ARDUINO_PATH=../../arduino-1.8.12
$ARDUINO_PATH/arduino-builder -build-path ./output -hardware $ARDUINO_PATH/hardware -tools $ARDUINO_PATH/hardware/tools/avr -tools $ARDUINO_PATH/tools-builder -libraries $ARDUINO_PATH/libraries -libraries ~/mylibs/ -libraries ./  -fqbn arduino:avr:mega ./
