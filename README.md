
# Smart Shirt

This is the development for class project on making a smart shirt.

MCU:
* Nordic Semiconductor NRF52, dev board PCA10040

Potential IMUs:
* Bosch BN055
* InvenSens MPU-9250

## Getting started with Nordic NRF52

First install the tools.  These are pretty basic and are supported on most systems.

* [GNU ARM Embedded Toolchain](https://launchpad.net/gcc-arm-embedded/+download)
* [nRFx Toolset](http://www.nordicsemi.com/eng/Products/Bluetooth-low-energy/nRF52832) (nrfjprog)
* [JLink tool](https://www.segger.com/downloads/jlink)

Normally you would download the Nordic SDK too but this is include in git repo.

If you're on Linux or OS X, Change `GNU_INSTALL_ROOT` in `src/Makefile` to match the place you downloaded 
[GNU ARM Embedded Toolchain](https://launchpad.net/gcc-arm-embedded/+download) to.

```make
# Change these if needed
GNU_INSTALL_ROOT := /home/pp/gcc-arm-none-eabi-5_4-2016q3
```

Plug in the NRF52 PCA10040 board and run:

```bash
cd src/
make
make flash_softdevice  # only need to do this once per NRF52
make flash
```
Now it's running the code.

### See the output

The USB has a UART bridge.  Use a terminal emulator or other serial reader to see output.

E.g.:

```bash
$ picocom -b115200 /dev/ttyACMO

accel:
    x:5540 y:2684 z:-15708
gyro:
    x:193 y:19 z:-73
magno:
    x:-192 y:-124 z:-238
temperature:
    t:144
…
```

### Adding more code

Start with `src/main.c` and edit `src/Makefile` somewhere to support more files.  For driver code, just look at `nrf_sdk/examples` and copy code from there.  Most things are separated into `pca*` folders because of different dev board revisions.  Use `pca10040`.

# Installing Bluetooth Windows Dependencies

### Prereqs

* Install Nodejs
* Install Visual Studio Community *2015*
* Need a USB Bluetooth dongle to bypass windows b.s.

### Steps

* Follow directions [here](https://github.com/sandeepmistry/node-bluetooth-hci-socket#windows)

Then run 
```bash
cd scripts
npm i    # only do this once
node bluetooth.js
```

It will connect to the sensor MCU.  Restart sensor MCU if it hangs.  

### Interfacing with the data

The bluetooth.js will host a http server you can poll on port 3000.  Data is returned in json and flushed.

```bash
$ curl http://localhost:3000
```

#### Output units

Default unit scale:
* Accelerometer: +/- 2g
* Gyroscope:     +/- 250 deg/sec 
* Magnetometer:  0.6 uT/bit
* Time:          miliseconds
* Temperature: -40 to 80 degrees C, +/-4%.

The Accelerometer and Gyroscope units are the maximum ranges for the ADC output.  Since each one is signed 16 bit data,
you would calculate the real value as `2*<signed-value>/2^15 [g]`.  If you find that the ranges saturate easily, then let me know
and I can easily update it to be like `+/- 4g` or `+/-16g`, etc.  Temperature is also a range.

The magnetometer is calculated simply as` <signed-value> * 0.6 [uT]`.

### Notes

The `npm` installation of `bluetooth-hci-socket` didn't work for me at first.  See [this fix](http://stackoverflow.com/questions/38149603/npm-install-fails-with-error-c2373-with-vs2015-update-3/38149604#38149604) if it doesn't work.  Try the accepted answer and try just running `npm install npm -g` all in administer cmd lines.

# Usage

## Calibration

You need to calibrate the magnetometer for it to work properly.  Simply run the program and follow directions.
```bash
python visual.py   # or sensor.py
```

All calibration is done python side for convenience.  Firmware/bluetooth just dumps the raw values.  See output units section.

You will have to move the board from left to right 180 degrees and then the same moving it up and down.
The program will save the configuration to a file called `calibration.json` so you don't need to do this
each time you run the program.  Simply delete `calibration.json` to recalibrate next time  you run the program.

## Visualization

We are doing the graphics manually with OpenGL.  The orange cubes represent joints and the blue cubes
represent places inbetween the joints that the sensors are located.

![](http://i.imgur.com/bubSQV7.png)

Right now only one sensor is being used and that's on the upper arm location (between shoulder joint and elbow joint).
Next we should add a sensor to the wrist (between elbow joint and hand joint).  This can be done by reading the main
loop in visual.py and seeing lines 188,199.  Right now `arm.set_uparm` and `arm.set_wrist` sets the normalized XYZ point
on the imaginary unit sphere around the sholder and elbow joints, respectively.  `arm.set_wrist` needs real data input
and not fake data.  Next, more joints and sensors should be added.

### Sensor IDs

The sensor IDs are 22, 23, 24, 25.  They correspond to the "chip select" GPIO they are each plugged into.
See wiring section.


# Wiring

See picture of bread board for reference.  SCA connects to MCU pin 30 and SCL connects to MCU pin 31.  Each AD0 pin needs to connect to a specific GPIO on the MCU as it acts like a chip select.  Connect each AD0 to MCU pins 22, 23, 24, 25.  By connecting a sensor to a pin, it is also assigning it an ID.  E.g. Sensor with AD0 connected to 23 will have ID 23.  Sensors can be removed or added if necessary.

![](http://i.imgur.com/u7qW0R2.jpg)


