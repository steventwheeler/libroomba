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
#ifndef _LIBROOMBA
#define _LIBROOMBA

#include "application.h"
#include <stdarg.h>

enum SensorGroup { ALL = 0, PHYSICAL = 1, INTERNAL = 2, BATTERY = 3 };
enum Motor { MAIN_BRUSH = 0, VACUUM = 1, SIDE_BRUSH = 2 };
enum LED { STATUS_GREEN = 0, STATUS_RED = 1, SPOT = 2, CLEAN = 3, MAX = 4, DIRT_DETECT = 5 };
enum PowerLED { COLOR, INTENSITY };

class LibRoomba {
  public:
    LibRoomba();
    bool begin(int ddPin);

    // Debug methods.
    void setDebug(bool debug);
    bool getDebug();
    void debugSensors();

    // Mode control methods.
    int start();
    int setBaudRate(int rate);
    int setControl();
    int setSafe();
    int setFull();
    int powerOff();

    // Button emulation methods.
    int spotClean();
    int clean();
    int maxClean();
    int dock();

    // Motor control methods.
    int drive(uint16_t velocity, uint16_t radius);
    int setMainBrush(bool enabled);
    int setVacuum(bool enabled);
    int setSideBrush(bool enabled);

    // LED control methods.
    int setLED(enum LED led, bool enabled);
    int setStatusGreenLED(bool enabled);
    int setStatusRedLED(bool enabled);
    int setSpotLED(bool enabled);
    int setCleanLED(bool enabled);
    int setMaxLED(bool enabled);
    int setDirtDetectLED(bool enabled);
    int setPowerLED(uint8_t color, uint8_t intensity);

    // Song control methods.
    int writeSong(uint8_t songNumber, uint8_t noteCount, ...);
    int vWriteSong(uint8_t songNumber, uint8_t noteCount, va_list songData);
    int playSong(uint8_t songNumber);

    // Sensor reading accessors.
    bool getCasterWheelDrop();
    bool getLeftWheelDrop();
    bool getRightWheelDrop();
    bool getLeftBump();
    bool getRightBump();
    bool getWallDetected();
    bool getLeftCliffDetected();
    bool getLeftFrontCliffDetected();
    bool getRightFrontCliffDetected();
    bool getRightCliffDetected();
    bool getVirtualWallDetected();
    bool getLeftDriveOvercurrent();
    bool getRightDriveOvercurrent();
    bool getMainBrushOvercurrent();
    bool getVacuumOvercurrent();
    bool getSideBrushOvercurrent();
    uint8_t getLeftDirtDetector();
    uint8_t getRightDirtDetector();
    uint8_t getRemoteCommand();
    bool getPowerButtonDown();
    bool getSpotButtonDown();
    bool getCleanButtonDown();
    bool getMaxButtonDown();
    int16_t getDistance();
    double getAngleDegrees();
    double getAngleRadians();
    uint8_t getChargingState();
    uint16_t getVoltage();
    int16_t getCurrent();
    int8_t getTemperature();
    uint16_t getCharge();
    uint16_t getCapacity();

    // Sensor control.
    int updateSensors();
  private:
    // Private variables.
    int _ddPin;
    bool _debug;
    bool _motorState[3];
    bool _ledState[6];
    uint8_t _powerLEDState[2];
    uint8_t _sensorData[26];

    // Private methods.
    void debug(const char *format, ...);
    int updateMotors();
    int updateLEDs();
    int16_t getAngleCM();
    int write(uint8_t command, int argc, ...);
    int vwrite(uint8_t command, int argc, va_list argp);
    int writeArray(uint8_t* buf, int len);
    uint8_t hiByte(uint16_t value);
    uint8_t loByte(uint16_t value);
    bool bitmask(uint8_t byte, int bitIndex);
    void commandDelay(uint8_t command);
};

#endif
