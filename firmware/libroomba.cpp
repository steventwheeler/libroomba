// The MIT License (MIT)
//
// Copyright (c) 2015 Steven Wheeler
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
#include "libroomba.h"

// Initialize the LibRoomba class.
LibRoomba::LibRoomba() {
  // The sensor data should default to zeros.
  for (int i = 0; i < 26; i++) {
    _sensorData[i] = 0;
  }
}

// Setup communication with the Roomba attached to the Photon's Serial1 port.
// This will use the specified digital pin (ddPin) to wake up the Roomba if it
// is powered off.
bool LibRoomba::begin(int ddPin) {
  // Configure the device detect pin.
  _ddPin = ddPin;
  pinMode(_ddPin, OUTPUT);

  // Initialize the serial port.
  Serial1.begin(115200);

  // Wake up the Roomba.
  debug("Waking up...");
  digitalWrite(_ddPin, HIGH);
  delay(500);
  digitalWrite(_ddPin, LOW);
  delay(500);
  digitalWrite(_ddPin, HIGH);

  return true;
}

// Configure the debug mode which will log debugging messages to the Photon's
// USB serial port.
//
// Params:
// debug: whether or not to enable the debug message logging.
void LibRoomba::setDebug(bool debug) {
  _debug = debug;
}

// Get the current debug mode setting, true indicates that debug is enabled and
// false indicates that debug is disabled.
bool LibRoomba::getDebug() {
  return _debug;
}

// This private method is used internally to log debug messages to the Photon's
// USB serial port. It takes the same arguments as printf. Please note that the
// message length is internally limited to 4096 characters.
//
// Params:
// format: the format string.
// ...: the arguments to format and insert into the string.
void LibRoomba::debug(const char *format, ...) {
  if (!_debug) return;
  va_list argp;
  va_start(argp, format);
  char str[4096];
  vsprintf(str, format, argp);
  Serial.print("[LibRoomba] ");
  Serial.println(str);
  va_end(argp);
}

// Send the start command (command #128) to the Roomba. This will return the
// number of bytes written to the serial port.
int LibRoomba::start() {
  debug("Sending: start");
  return write((uint8_t) 128, 0);
}

// Send the set the baud rate (command #129) which the Roomba should use. This
// will return the number of bytes written to the serial port.
//
// Params:
// rate: the baud rate to use. This will be converted to a format the Roomba can
//        understand so use the actual rate here.
int LibRoomba::setBaudRate(int rate) {
  debug("Sending: baud, %d", rate);

  // A list of rates supported by Roomba. Order is important as the index in the
  // array indicates the SCI code to send to the Roomba.
  int rates[] = {
    300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, 115200
  };

  // Find the SCI code which corresponds to specified baud rate.
  int code = -1;
  for (int i = 0; i < 12; i++) {
    if (rates[i] != rate) continue;
    code = i;
    break;
  }

  if (code != -1) {
    // If a SCI code was found then send the command.
    return write(129, 1, code);
  } else {
    // Return zero indicating that no command was sent.
    return 0;
  }
}

// Send the control command (command #130) to the Roomba. This will return the
// number of bytes written to the serial port.
int LibRoomba::setControl() {
  debug("Sending: control");
  return write(130, 0);
}

// Send the safe command (command #131) to the Roomba. This will return the
// number of bytes written to the serial port.
int LibRoomba::setSafe() {
  debug("Sending: safe");
  return write(131, 0);
}

// Send the full command (command #132) to the Roomba. This will return the
// number of bytes written to the serial port.
int LibRoomba::setFull() {
  debug("Sending: full");
  return write(132, 0);
}

// Send the power command (command #133) to the Roomba. This will return the
// number of bytes written to the serial port.
int LibRoomba::powerOff() {
  debug("Sending: power");
  return write(133, 0);
}

// Send the spot command (command #134) to the Roomba. This will return the
// number of bytes written to the serial port.
int LibRoomba::spotClean() {
  debug("Sending: spot");
  return write(134, 0);
}

// Send the clean command (command #135) to the Roomba. This will return the
// number of bytes written to the serial port.
int LibRoomba::clean() {
  debug("Sending: clean");
  return write(135, 0);
}

// Send the max command (command #136) to the Roomba. This will return the
// number of bytes written to the serial port.
int LibRoomba::maxClean() {
  debug("Sending: max");
  return write(136, 0);
}

// Send the drive command (command #137) to the Roomba. This will return the
// number of bytes written to the serial port.
//
// Params:
// velocity: the speed in mm/s to drive at. Negative numbers indicate reverse
//           and positive numbers indicate forward. Zero is stop.
// radius:   the radius of the arc the Roomba should drive on. Negative numbers
//            indicate a left turn and positive numbers indicate a right turn.
//            Special cases:
//            -1:      spin left
//            1:      spin right
//            32768:  straight line (0x8000)
int LibRoomba::drive(uint16_t velocity, uint16_t radius) {
  debug("Sending: drive, %d, %d", velocity, radius);
  uint8_t vH = hiByte(velocity);
  uint8_t vL = loByte(velocity);
  uint8_t rH = hiByte(radius);
  uint8_t rL = loByte(radius);
  return write(137, 4, vH, vL, rH, rL);
}

// Turn the main brush motor on or off. This sends the motors command (command
// #138) to the Roomba. This will return the number of bytes written to the
// serial port.
//
// Params:
// enabled: whether or not the motor should be on.
int LibRoomba::setMainBrush(bool enabled) {
  _motorState[MAIN_BRUSH] = enabled;
  return updateMotors();
}

// Turn the vacuum motor on or off. This sends the motors command (command #138)
// to the Roomba. This will return the number of bytes written to the serial
// port.
//
// Params:
// enabled: whether or not the motor should be on.
int LibRoomba::setVacuum(bool enabled) {
  _motorState[VACUUM] = enabled;
  return updateMotors();
}

// Turn the side brush motor on or off. This sends the motors command (command
// #138) to the Roomba. This will return the number of bytes written to the
// serial port.
//
// Params:
// enabled: whether or not the motor should be on.
int LibRoomba::setSideBrush(bool enabled) {
  _motorState[SIDE_BRUSH] = enabled;
  return updateMotors();
}

// This private method writes the current motor state to the Roomba. This will
// return the number of bytes written to the serial port.
int LibRoomba::updateMotors() {
  uint8_t main = _motorState[MAIN_BRUSH];
  uint8_t vacuum = _motorState[VACUUM];
  uint8_t side = _motorState[SIDE_BRUSH];
  debug("Sending: motors, main: %d, vacuum: %d, side: %d", main, vacuum, side);
  int arg = 0;
  arg = (arg << 1) + (main ? 1 : 0);
  arg = (arg << 1) + (vacuum ? 1 : 0);
  arg = (arg << 1) + (side ? 1 : 0);
  return write(138, 1, arg);
}

// This method can be used to turn a specific LED on or off. This will return
// the number of bytes written to the serial port.
//
// Params:
// led:      the LED to update.
// enabled: whether or not the LED should be turned on.
int LibRoomba::setLED(enum LED led, bool enabled) {
  _ledState[led] = enabled;
  return updateLEDs();
}

// Turn the green status LED on or off. This sends the leds command (command
// #139) to the Roomba. This will return the number of bytes written to the
// serial port.
//
// Params:
// enabled: whether or not the LED should be turned on.
int LibRoomba::setStatusGreenLED(bool enabled) {
  return setLED(STATUS_GREEN, enabled);
}

// Turn the red status LED on or off. This sends the leds command (command #139)
// to the Roomba. This will return the number of bytes written to the serial
// port.
//
// Params:
// enabled: whether or not the LED should be turned on.
int LibRoomba::setStatusRedLED(bool enabled) {
  return setLED(STATUS_RED, enabled);
}

// Turn the spot LED on or off. This sends the leds command (command #139) to
// the Roomba. This will return the number of bytes written to the serial port.
//
// Params:
// enabled: whether or not the LED should be turned on.
int LibRoomba::setSpotLED(bool enabled) {
  return setLED(SPOT, enabled);
}

// Turn the clean LED on or off. This sends the leds command (command #139) to
// the Roomba. This will return the number of bytes written to the serial port.
//
// Params:
// enabled: whether or not the LED should be turned on.
int LibRoomba::setCleanLED(bool enabled) {
  return setLED(CLEAN, enabled);
}

// Turn the max LED on or off. This sends the leds command (command #139) to
// the Roomba. This will return the number of bytes written to the serial port.
//
// Params:
// enabled: whether or not the LED should be turned on.
int LibRoomba::setMaxLED(bool enabled) {
  return setLED(MAX, enabled);
}

// Turn the dirt detect LED on or off. This sends the leds command (command
// #139) to the Roomba. This will return the number of bytes written to the
// serial port.
//
// Params:
// enabled: whether or not the LED should be turned on.
int LibRoomba::setDirtDetectLED(bool enabled) {
  return setLED(DIRT_DETECT, enabled);
}

// Update the power LED color and intensity. This sends the leds command
// (command #139) to the Roomba. This will return the number of bytes written to
// the serial port.
//
// Params:
// color:      what color the LED should be. 0 indicates green and 255 indicates
//             red. Intermediate values indicate a color between green and red.
// intensity: how bright the LED should be. 0 indicates off and 255 indicates
//            full brightness.
int LibRoomba::setPowerLED(uint8_t color, uint8_t intensity) {
  _powerLEDState[COLOR] = color;
  _powerLEDState[INTENSITY] = intensity;
  return updateLEDs();
}

// This private method writes the current LED state to the Roomba. This will
// return the number of bytes written to the serial port.
int LibRoomba::updateLEDs() {
  debug("Sending: leds, statusGreen: %d, statusRed: %d, spot: %d, clean: %d, max: %d, dirtDetect: %d, powerColor: %d, powerIntensity, %d", _ledState[STATUS_GREEN], _ledState[STATUS_RED], _ledState[SPOT], _ledState[CLEAN], _ledState[MAX], _ledState[DIRT_DETECT], _powerLEDState[COLOR], _powerLEDState[INTENSITY]);
  int arg1 = 0;
  arg1 = (arg1 << 1) + (_ledState[STATUS_GREEN] ? 1 : 0);
  arg1 = (arg1 << 1) + (_ledState[STATUS_RED] ? 1 : 0);
  arg1 = (arg1 << 1) + (_ledState[SPOT] ? 1 : 0);
  arg1 = (arg1 << 1) + (_ledState[CLEAN] ? 1 : 0);
  arg1 = (arg1 << 1) + (_ledState[MAX] ? 1 : 0);
  arg1 = (arg1 << 1) + (_ledState[DIRT_DETECT] ? 1 : 0);
  return write(139, 3, arg1, _powerLEDState[COLOR], _powerLEDState[INTENSITY]);
}

// Send the song command (command #140) to the Roomba to create a song. This
// will return the number of bytes written to the serial port.
//
// Params:
// songNumber: the ID of the song to write.
// noteCount:   the number of notes in the song.
// ...:          a series of bytes containing the song data. Note the length of
//              the array must be exactly 2 * noteCount.
int LibRoomba::writeSong(uint8_t songNumber, uint8_t noteCount, ...) {
  va_list songData;
  va_start(songData, noteCount);
  int n = vWriteSong(songNumber, noteCount, songData);
  va_end(songData);
  return n;
}

// Send the song command (command #140) to the Roomba to create a song. This
// will return the number of bytes written to the serial port.
//
// Params:
// songNumber: the ID of the song to write.
// noteCount:   the number of notes in the song.
// songData:   a va_list containing containing the song data. Note the length of
//              the va_list must be exactly 2 * noteCount.
int LibRoomba::vWriteSong(uint8_t songNumber, uint8_t noteCount, va_list songData) {
  debug("Sending song: #%d, notes: %d", songNumber, noteCount);
  int len = 2 * noteCount + 3;
  uint8_t array[35];  // The maximum length is 16 notes + 3 header bytes.
  array[0] = (uint8_t) 140;
  array[1] = songNumber;
  array[2] = noteCount;

  // Copy the song data bytes into the new array.
  for (int i = 0; i < noteCount * 2; i++) {
    array[i + 3] = va_arg(songData, uint8_t);
  }

  // Write the command and song data to the serial port.
  return writeArray(array, len);
}

// Send the play song command (command #140) to the Roomba. This will return the
// number of bytes written to the serial port.
//
// Params:
// songNumber: the ID of the song to play.
int LibRoomba::playSong(uint8_t songNumber) {
  debug("Sending play: #%d", songNumber);
  return write(141, 0);
}

// Get the current value of the caster wheel drop sensor, true indicates that
// the caster wheel has dropped.
//
// Note: you must call updateSensors() before calling this method.
bool LibRoomba::getCasterWheelDrop() {
  return bitmask(_sensorData[0], 4);
}

// Get the current value of the left wheel drop sensor, true indicates that
// the left wheel has dropped.
//
// Note: you must call updateSensors() before calling this method.
bool LibRoomba::getLeftWheelDrop() {
  return bitmask(_sensorData[0], 3);
}

// Get the current value of the right wheel drop sensor, true indicates that
// the right wheel has dropped.
//
// Note: you must call updateSensors() before calling this method.
bool LibRoomba::getRightWheelDrop() {
  return bitmask(_sensorData[0], 2);
}

// Get the current value of the left bump sensor, true indicates that the left
// bumper has been triggered.
//
// Note: you must call updateSensors() before calling this method.
bool LibRoomba::getLeftBump() {
  return bitmask(_sensorData[0], 1);
}

// Get the current value of the right bump sensor, true indicates that the right
// bumper has been triggered.
//
// Note: you must call updateSensors() before calling this method.
bool LibRoomba::getRightBump() {
  return bitmask(_sensorData[0], 0);
}

// Get the current value of the wall sensor, true indicates that a wall has been
// detected.
//
// Note: you must call updateSensors() before calling this method.
bool LibRoomba::getWallDetected() {
  return bitmask(_sensorData[1], 0);
}

// Get the current value of the left cliff sensor, true indicates a cliff has
// been detected.
//
// Note: you must call updateSensors() before calling this method.
bool LibRoomba::getLeftCliffDetected() {
  return bitmask(_sensorData[2], 0);
}

// Get the current value of the left front cliff sensor, true indicates a cliff
// has been detected.
//
// Note: you must call updateSensors() before calling this method.
bool LibRoomba::getLeftFrontCliffDetected() {
  return bitmask(_sensorData[3], 0);
}

// Get the current value of the right front cliff sensor, true indicates a cliff
// has been detected.
//
// Note: you must call updateSensors() before calling this method.
bool LibRoomba::getRightFrontCliffDetected() {
  return bitmask(_sensorData[4], 0);
}

// Get the current value of the right cliff sensor, true indicates a cliff has
// been detected.
//
// Note: you must call updateSensors() before calling this method.
bool LibRoomba::getRightCliffDetected() {
  return bitmask(_sensorData[5], 0);
}

// Get the current value of the virtual wall sensor, true indicates a virtual
// wall has been detected.
//
// Note: you must call updateSensors() before calling this method.
bool LibRoomba::getVirtualWallDetected() {
  return bitmask(_sensorData[6], 0);
}

// Get the current value of the left drive overcurrent sensor, true indicates
// the left drive is overcurrent.
//
// Note: you must call updateSensors() before calling this method.
bool LibRoomba::getLeftDriveOvercurrent() {
  return bitmask(_sensorData[7], 4);
}

// Get the current value of the right drive overcurrent sensor, true indicates
// the right drive is overcurrent.
//
// Note: you must call updateSensors() before calling this method.
bool LibRoomba::getRightDriveOvercurrent() {
  return bitmask(_sensorData[7], 3);
}

// Get the current value of the main brush overcurrent sensor, true indicates
// the main brush is overcurrent.
//
// Note: you must call updateSensors() before calling this method.
bool LibRoomba::getMainBrushOvercurrent() {
  return bitmask(_sensorData[7], 2);
}

// Get the current value of the vacuum overcurrent sensor, true indicates the
// vacuum is overcurrent.
//
// Note: you must call updateSensors() before calling this method.
bool LibRoomba::getVacuumOvercurrent() {
  return bitmask(_sensorData[7], 1);
}

// Get the current value of the side brush overcurrent sensor, true indicates
// the side brush is overcurrent.
//
// Note: you must call updateSensors() before calling this method.
bool LibRoomba::getSideBrushOvercurrent() {
  return bitmask(_sensorData[7], 0);
}

// Get the current value of the left dirt detect sensor, zero indicates no dirt
// is detected. Higher levels indicate more dirt is detected.
//
// Note: you must call updateSensors() before calling this method.
uint8_t LibRoomba::getLeftDirtDetector() {
  return bitmask(_sensorData[8], 0);
}

// Get the current value of the right dirt detect sensor, zero indicates no dirt
// is detected. Higher levels indicate more dirt is detected.
//
// Note: you must call updateSensors() before calling this method.
// Note 2: not all vacuums have a right dirt detector.
uint8_t LibRoomba::getRightDirtDetector() {
  return bitmask(_sensorData[9], 0);
}

// Get the current IR remote command being received, 255 indicates no command is
// being received. All other values indicate a specific command.
//
// Note: you must call updateSensors() before calling this method.
uint8_t LibRoomba::getRemoteCommand() {
  return _sensorData[10];
}

// Get the current power button status, true indicates the button is being
// pressed.
//
// Note: you must call updateSensors() before calling this method.
bool LibRoomba::getPowerButtonDown() {
  return bitmask(_sensorData[11], 3);
}

// Get the current spot button status, true indicates the button is being
// pressed.
//
// Note: you must call updateSensors() before calling this method.
bool LibRoomba::getSpotButtonDown() {
  return bitmask(_sensorData[11], 2);
}

// Get the current clean button status, true indicates the button is being
// pressed.
//
// Note: you must call updateSensors() before calling this method.
bool LibRoomba::getCleanButtonDown() {
  return bitmask(_sensorData[11], 1);
}

// Get the current max button status, true indicates the button is being
// pressed.
//
// Note: you must call updateSensors() before calling this method.
bool LibRoomba::getMaxButtonDown() {
  return bitmask(_sensorData[11], 0);
}

// Get the distance traveled in milimeters since the last update. If not updated
// frequently enough this might be capped at -32768 or 32767.
//
// Note: you must call updateSensors() before calling this method.
int16_t LibRoomba::getDistance() {
  return ((_sensorData[12] & 0xFF) << 8) + (_sensorData[13] & 0xFF);
}

// This private method gets the angle that the Roomba has turned through since
// the last update. Negative values indicate clockwise rotation and positive
// numbers indicate counter-clockwise roatation. If not updated frequently
// enough this might be capped at -32768 or 32767.
//
// Note: you must call updateSensors() before calling this method.
int16_t LibRoomba::getAngleCM() {
  return ((_sensorData[14] & 0xFF) << 8) + (_sensorData[15] & 0xFF);
}

// Get the angle in degrees that the Roomba has turned through since the last
// update. Negative values indicate clockwise rotation and positive numbers
// indicate counter-clockwise roatation.
//
// Note: you must call updateSensors() before calling this method.
double LibRoomba::getAngleDegrees() {
  int16_t angle = getAngleCM();
  return (360.0 * angle) / (258.0 * 3.14159);
}

// Get the angle in radians that the Roomba has turned through since the last
// update. Negative values indicate clockwise rotation and positive numbers
// indicate counter-clockwise roatation.
//
// Note: you must call updateSensors() before calling this method.
double LibRoomba::getAngleRadians() {
  int16_t angle = getAngleCM();
  return (2 * angle) / 258.0;
}

// Get the current charging state code. The codes are:
//   0: Not Charging
//  1: Charging Recovery
//   2: Charging
//   3: Trickle Charging
//   4: Waiting
//  5: Charging Error
//
// Note: you must call updateSensors() before calling this method.
uint8_t LibRoomba::getChargingState() {
  return _sensorData[16];
}

// Get the battery's current charge level in millivolts (mV).
//
// Note: you must call updateSensors() before calling this method.
uint16_t LibRoomba::getVoltage() {
  return ((_sensorData[17] & 0xFF) << 8) + (_sensorData[18] & 0xFF);
}

// Get the battery's current charge/discharge rate in milliamps (mA).
//
// Note: you must call updateSensors() before calling this method.
int16_t LibRoomba::getCurrent() {
  return ((_sensorData[19] & 0xFF) << 8) + (_sensorData[20] & 0xFF);
}

// Get the battery's temperature in celcius.
//
// Note: you must call updateSensors() before calling this method.
int8_t LibRoomba::getTemperature() {
  return _sensorData[21];
}

// Get the current charge of the battery in milliamp-hours (mAh).
//
// Note: you must call updateSensors() before calling this method.
uint16_t LibRoomba::getCharge() {
  return ((_sensorData[22] & 0xFF) << 8) + (_sensorData[23] & 0xFF);
}

// Get the estimated batery capacity when fully charged in milliamp-hours (mAh).
uint16_t LibRoomba::getCapacity() {
  return ((_sensorData[24] & 0xFF) << 8) + (_sensorData[25] & 0xFF);
}

// This method is used to request updated sensor data from the Roomba. This
// sends the sensors command (command #142). This will return the number of
// bytes written to the serial port.
int LibRoomba::updateSensors() {
  debug("Sending sensors: %d", 0);
  int sent = write(142, 1, 0);
  if (sent == -1) return -1;

  // Read the response data.
  Serial1.flush();
  for (int i = 0; i < 26; i++) {
    _sensorData[i] = (uint8_t) Serial1.read();
    debug("Read byte; #%d, %d", i, _sensorData[i]);
  }
  return sent;
}

// Output debug messages containing the current sensor data.
void LibRoomba::debugSensors() {
  debug("Caster Wheel Drop: %d", getCasterWheelDrop());
  debug("Left Wheel Drop: %d", getLeftWheelDrop());
  debug("Right Wheel Drop: %d", getRightWheelDrop());
  debug("Left Bump: %d", getLeftBump());
  debug("Right Bump: %d", getRightBump());
  debug("Wall Detected: %d", getWallDetected());
  debug("Left Cliff Detected: %d", getLeftCliffDetected());
  debug("Left Front Cliff Detected: %d", getLeftFrontCliffDetected());
  debug("Right Front Cliff Detected: %d", getRightFrontCliffDetected());
  debug("Right Cliff Detected: %d", getRightCliffDetected());
  debug("Virtual Wall Detected: %d", getVirtualWallDetected());
  debug("Left Drive Overcurrent: %d", getLeftDriveOvercurrent());
  debug("Right Drive Overcurrent: %d", getRightDriveOvercurrent());
  debug("Main Brush Overcurrent: %d", getMainBrushOvercurrent());
  debug("Vacuum Overcurrent: %d", getVacuumOvercurrent());
  debug("Side Brush Overcurrent: %d", getSideBrushOvercurrent());
  debug("Left Dirt Detector: %d", getLeftDirtDetector());
  debug("Right Dirt Detector: %d", getRightDirtDetector());
  debug("Remote Command: %d", getRemoteCommand());
  debug("Power Button Down: %d", getPowerButtonDown());
  debug("Spot Button Down: %d", getSpotButtonDown());
  debug("Clean Button Down: %d", getCleanButtonDown());
  debug("Max Button Down: %d", getMaxButtonDown());
  debug("Distance: %d", getDistance());
  debug("Angle: %d", getAngleDegrees());
  debug("Charging State: %d", getChargingState());
  debug("Voltage: %d", getVoltage());
  debug("Current: %d", getCurrent());
  debug("Temperature: %d", getTemperature());
  debug("Charge: %d", getCharge());
  debug("Capacity: %d", getCapacity());
}

// Send the dock command (command #143) to the Roomba. This will return the
// number of bytes written to the serial port.
int LibRoomba::dock() {
  return write(143, 0);
}

// This private method is used to write commands to the Roomba's serial port.
//
// Params:
// command: the command number.
// argc:    the number of additional bytes to send.
// ...:      the additional bytes to send.
int LibRoomba::write(uint8_t command, int argc, ...) {
  va_list argp;
  va_start(argp, command);
  int n = vwrite(command, argc, argp);
  va_end(argp);
  return n;
}

// This private method is used to write commands to the Roomba's serial port.
//
// Params:
// command: the command number.
// argc:    the number of additional bytes to send.
// argp:    the additional bytes to send.
int LibRoomba::vwrite(uint8_t command, int argc, va_list argp) {
  int len = argc + 1;
  uint8_t array[4096];
  array[0] = command;

  // Copy the argument bytes into the new array.
  for (int i = 0; i < argc; i++) {
    array[i + 1] = va_arg(argp, int);
  }

  // Write the array to the serial port.
  return writeArray(array, len);
}

// This private method is used to write commands to the Roomba's serial port.
//
// Params:
// buf: the array of bytes to write to the serial port.
// len:  the number of bytes to send.
int LibRoomba::writeArray(uint8_t* buf, int len) {
  int written = 0;

  // Write the bytes to the Roomba's serial port.
  for (int i = 0; i < len; i++) {
    uint8_t arg = buf[i];
    written += Serial1.write(arg);
    debug("Writing: %u", arg);
  }

  // Check if a delay is needed after this command.
  if (len > 0) commandDelay(buf[0]);

  // Return the number of bytes written to the serial port.
  return written;
}

// This private method checks the command number and delays execution if
// required. According to the SCI specification commands which change the mode
// must delay 20ms before the next command and changing the baud rate requires
// a delay of 100ms before the next command.
//
// Params:
// command: the command number which was sent.
void LibRoomba::commandDelay(uint8_t command) {
  switch (command) {
  case 128:
  case 130:
  case 131:
  case 132:
  case 133:
  case 134:
  case 135:
  case 136:
  case 143:
    delay(20);
    break;
  case 129:
    delay(100);
    break;
  }
}

// This private method gets the high byte (first 8 bits) from a 16 bit unsigned
// integer.
//
// Params:
// value: the 16 bit unsigned integer.
uint8_t LibRoomba::hiByte(uint16_t value) {
  return value >> 8;
}

// This private method gets the low byte (last 8 bits) from a 16 bit unsigned
// integer.
//
// Params:
// value: the 16 bit unsigned integer.
uint8_t LibRoomba::loByte(uint16_t value) {
  return value & 0xFF;
}

// This private method gets a single bit from an 8 bit unsigned integer. This
// returns true if the bit is set (equal to 1) or false if the bit is not set
// (equal to 0).
//
// Params:
// byte:     the 8 bit unsigned integer containing the bit.
// bitIndex: the bit to check.
bool LibRoomba::bitmask(uint8_t byte, int bitIndex) {
  int mask = 1 << bitIndex;
  return (byte & mask) == mask;
}
