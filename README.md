
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

#### Output units

Default unit scale:
* Accelerometer: +/- 2g
* Gyroscope:     +/- 250 deg/sec 
* Magnetometer:  0.6 uT/bit
* Time:          miliseconds
* Temperature: -40 to 80 degrees C, +/-4%.

The Accelerometer and Gyroscope units are the maximum ranges for the ADC output.  Since each one is signed 16 bit data,
you would calculate the real value as \<signed-value\>/2^15 [g].  If you find that the ranges saturate easily, then let me know
and I can easily update it to be like +/- 4g or +/-16g, etc.  Temperature is also a range.

The magnetometer is calculated simply as \<signed-value\> * 0.6 [uT].

### Notes

The `npm` installation of `bluetooth-hci-socket` didn't work for me at first.  See [this fix](http://stackoverflow.com/questions/38149603/npm-install-fails-with-error-c2373-with-vs2015-update-3/38149604#38149604) if it doesn't work.  Try the accepted answer and try just running `npm install npm -g` all in administer cmd lines.

# Wiring

See picture of bread board for reference.  SCA connects to MCU pin 30 and SCL connects to MCU pin 31.  Each AD0 pin needs to connect to a specific GPIO on the MCU as it acts like a chip select.  Connect each AD0 to MCU pins 22, 23, 24, 25.  By connecting a sensor to a pin, it is also assigning it an ID.  E.g. Sensor with AD0 connected to 23 will have ID 23.  Sensors can be removed or added if necessary.

![](http://i.imgur.com/u7qW0R2.jpg)
