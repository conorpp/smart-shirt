
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



