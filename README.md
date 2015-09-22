Spark Core library for communicating with iRobot<sup>&reg;</sup> Roomba<sup>&reg;</sup> vacuums.
===

> This library has been tested on the following devices:
> 
> 
> * Roomba<sup>&reg;</sup> 530
>
>
> You will need to create a cable to connect your Photon to the Roomba<sup>&reg;</sup>'s serial port.


### Example code
```c++
#include "libroomba.h"

LibRoomba roomba;

void setup() {
  // Enable the USB serial port for debug messages.
  Serial.begin(115200);
  Serial.println("Starting Roomba!");

  // Setup the Roomba.
  roomba.setDebug(true);
  roomba.begin(D6);
  
  // Enable user control.
  roomba.start();
  roomba.setControl();
  roomba.setSafe();

  // Play a song to indicate setup is complete.
  roomba.writeSong(0, 8, 60, 32, 62, 32, 64, 32, 65, 32, 67, 32, 69, 32, 71, 32, 72, 32);
  roomba.playSong(0);
}

void loop() {
  roomba.updateSensors();
  roomba.debugSensors();
  delay(1000);
}
```
