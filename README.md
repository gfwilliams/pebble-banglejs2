# PebbleOS (Bangle.js 2 port)

This repository contains the source code of PebbleOS.

**This has been hacked around to add a 'banglejs2' board type**

**Current state is:**

* `./waf configure --board banglejs2 --nojs --nohash` works
* `./waf build` builds a binary
* We have what appear to be the correct GPIOs for some of the Bangle.js functionality
* `jdi_lpm013m126c.h` driver is implemented (however unsure if it works or not - even the commented out debugging code has no effect)

Appears to crash in:

```
#0  0x0007a4e0 in vPortExitCritical () at ../src/fw/vendor/FreeRTOS/Source/portable/GCC/ARM_CM4_PEBBLE/port.c:372
#1  0x00079d30 in xTaskResumeAll () at ../src/fw/vendor/FreeRTOS/Source/tasks.c:1764
#2  0x00078fc4 in xLightMutexLock (xMutex=0xa81, xTicksToWait=<optimized out>, xTicksToWait@entry=4294967295) at ../src/fw/vendor/FreeRTOS/Source/light_mutex.c:299
#3  0x00078cba in mutex_lock (handle=0x0) at ../src/libos/mutex_freertos.c:83
#4  0x0005ffa6 in vibe_service_set_enabled (enable=false) at ../src/fw/services/common/vibe_pattern.c:204
#5  0x00076e7c in services_common_set_runlevel (runlevel=runlevel@entry=RunLevel_BareMinimum) at ../src/fw/services/common/services_common.c:107
#6  0x00076b0c in services_set_runlevel (runlevel=runlevel@entry=RunLevel_BareMinimum) at ../src/fw/services/services.c:48
#7  0x000804fc in prv_reset_into_prf () at ../src/fw/kernel/util/fw_reset.c:34
#8  0x00080506 in fw_reset_into_prf () at ../src/fw/kernel/util/fw_reset.c:39
#9  0x00033e84 in system_resource_init () at ../src/fw/resource/system_resource.c:48
#10 0x0004598a in prv_main_task_init () at ../src/fw/main.c:459
#11 0x0008780a in main_task (parameter=<optimized out>) at ../src/fw/main.c:523
```

Which appears to be further than the `asterix_vla_dvb1` gets on the Bangle.js board (that crashes because the flash wasn't found) but I still don't understand why.

**TODO**

* Find out why we have the crash above 
* Bangle.js has all 4 button IO pins assigned to the same number (might this cause issues?) Ideally we implement our own touchscreen handler and fake buttons using the touchscreen
* Accelerometer/Magnetometer/etc not implemented




## How PebbleOS works
Read more in the very detailed [PebbleOS architecture presentation](https://docs.google.com/presentation/d/1wfyBRwbrv5YtSnvNRnEPz5tRx9y7VGcFsuHbi1X-D7I/edit?usp=sharing)

Then read this [presentation on how PebbleOS works](https://docs.google.com/presentation/d/1M--yoEJBO-uckvY5CTFfHT4srw6RCj9RTGT57RcogX8/edit?usp=sharing)!

Join the [Rebble Discord](https://discordapp.com/invite/aRUAYFN) #firmware-dev channel to discuss.
### Architecture
![CleanShot 2025-01-31 at 14 38 16@2x](https://github.com/user-attachments/assets/23d13a36-55e6-4e3a-87ab-4fb1fd1fca5a)

![Arc 2025-01-31 11 44 47](https://github.com/user-attachments/assets/804bc6b9-47c1-4af5-b698-6078aca467ee)


**WARNING**: Codebase is being refactored/modernized, so only certain features
may work right now.

## Getting Started

- Use Linux (tested: Ubuntu 24.04, Fedora 41) or macOS (tested: Sequoia 15.2)
- Clone the submodules:
  ```shell
  git submodule init
  git submodule update
  ```
- Install GNU ARM Embedded toolchain from
  https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads. Make
  sure it is available on your `PATH` by checking `arm-none-eabi-gcc --version`
  returns the expected version.
- If using Ubuntu, install `gcc-multilib` and `gettext`
- Install `nrfjprog` from
  https://www.nordicsemi.com/Products/Development-tools/nRF-Command-Line-Tools.
- Create a Python venv:

  ```shell
  python -m venv .venv
  ```
- Activate the Python venv (also every time you start working):
  ```shell
  source .venv/bin/activate
  ```
- Install dependencies:
  ```shell
  pip install -r requirements-linux.txt
  ```
- Install local dependencies:
  ```shell
  pip install -e \
    python_libs/pblprog \
    python_libs/pebble-commander \
    python_libs/pulse2 \
    python_libs/pebble-loghash
  ```

## Building

First, configure the project like this:

```shell
./waf configure --board banglejs2 --nojs --nohash
```

At this moment, only `asterix_vla_dvb1` and `banglejs2` board target may compile and boot.

Then build:

```shell
./waf build
```

PRF can be also be built:

```shell
./waf build_prf
```

## Flashing

First make sure Nordic S140 Softdevice is flashed (only do this once):

```shell
nrfjprog --program src/fw/vendor/nrf5-sdk/components/softdevice/s140/hex/s140_nrf52_7.2.0_softdevice.hex --sectoranduicrerase --reset
```

Flash the firmware:

```shell
nrfjprog --program build/src/fw/tintin_fw.elf --sectorerase --reset
```

First time you should see a "sad watch" screen because resources are not
flashed. To flash resources, run:

```shell
python tools/pulse_flash_imaging.py -t /dev/$PORT -p resources build/system_resources.pbpack
```

## Viewing logs

```shell
python tools/pulse_console.py -t /dev/$PORT
```
