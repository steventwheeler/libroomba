Particle Photon library for communicating with iRobot Roomba vacuums.
===

> This library has been tested on the following devices:
>
>
> * Roomba 530
>
>
> You will need to create a cable to connect your Photon to the Roomba's serial port.

## Usage

Connect XYZ hardware, add the libroomba library to your project and follow this simple example:

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
See the [examples](examples) folder for more details.

## Contributing

Fork this library on [GitHub](https://github.com/steventwheeler/libroomba)

Here's how you can make changes to this library and eventually contribute those changes back.

To get started, [clone the library from GitHub to your local machine](https://help.github.com/articles/cloning-a-repository/).

Change the name of the library in `library.properties` to something different. You can add your name at then end.

Modify the sources in <src> and <examples> with the new behavior.

To compile an example, use `particle compile examples/usage` command in [Particle CLI](https://docs.particle.io/guide/tools-and-features/cli#update-your-device-remotely) or use [Desktop IDE](https://docs.particle.io/guide/tools-and-features/dev/#compiling-code).

After your changes are done you can upload them with `particle library upload` or `Upload` command in the IDE. This will create a private (only visible by you) library that you can use in other projects. Do `particle library add libroomba_myname` to add the library to a project on your machine or add the libroomba_myname library to a project on the Web IDE or Desktop IDE.

At this point, you can create a [GitHub pull request](https://help.github.com/articles/about-pull-requests/) with your changes to the original library.

If you wish to make your library public, use `particle library publish` or `Publish` command.

## LICENSE
Copyright 2017 Steven Wheeler <steventwheeler@gmail.com>

Licensed under the MIT license
