#include "libroomba.h"

LibRoomba roomba;

void setup() {
  // Configure the remote API.
  Spark.function("roomba", roombaControl);

  // Enable the USB serial port for debug messages.
  Serial.begin(115200);
  Serial.println("Starting Roomba!");

  // Setup the Roomba.
  roomba.setDebug(true);
  roomba.begin(D6);
  enableControl();

  // Play a song to indicate setup is complete.
  roomba.writeSong(0, 8, 60, 32, 62, 32, 64, 32, 65, 32, 67, 32, 69, 32, 71, 32, 72, 32);
  roomba.playSong(0);
}

void loop() {
  roomba.updateSensors();
  roomba.debugSensors();
  delay(1000);
}

void enableControl() {
  roomba.start();
  roomba.setControl();
  roomba.setSafe();
}

int roombaControl(String command) {
  if (command == "STOP") {
    enableControl();
    roomba.drive(0, 0);
  } else if (command == "REVERSE") {
    enableControl();
    roomba.drive(-200, 0x8000);
  } else if (command == "FORWARD") {
    enableControl();
    roomba.drive(500, 0x8000);
  } else if (command == "RIGHT") {
    enableControl();
    roomba.drive(200, 0xffff);
  } else if (command == "LEFT") {
    enableControl();
    roomba.drive(200, 0x0001);
  } else if (command == "VACUUM_ON") {
    enableControl();
    roomba.setVacuum(true);
  } else if (command == "VACUUM_OFF") {
    enableControl();
    roomba.setVacuum(false);
  } else if (command == "CONTROL") {
    enableControl();
  } else if (command == "CLEAN") {
    enableControl();
    roomba.clean();
  } else if (command == "DOCK") {
    enableControl();
    roomba.clean();
    roomba.dock();
  } else {
    return -1;
  }
  return 1;
}
