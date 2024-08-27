Firmware Organization
=====================

The PAL 9000 firmware is based on PlatformIO, so the structure follows the PlatformIO project structure.

Environments
------------

The :code:`environments` directory contains the PlatformIO environments for the different PAL 9000 flight computers. 
Each environment is a differnt version of the PAL hardware and contains a :code:`platformio.ini` file which specifies
board specific settings for the environment.

Lib
---

The :code:`lib` directory contains libraries which can be shared between all the PlatformIO environments for the PAL 9000.
These libraries are most often sensor drivers and headers for peripheral wrappers like I2C and SPI. 

Src
---

The :code:`src` directory contains the platform specific code for each PAL board. The code written here is specific to the
pinouts and available peripherals on a board. It contains drivers for peripherals that differ between the different
microcontrollers used on the boards. This directory also contains the :doc:`main program <main>` for the firmware.

Doc
---

The :code:`doc` directory contains this Sphinx documentation for the PAL 9000 firmware. The documentation is hosted
on Read the Docs at https://psp-pal-9000-firmware.readthedocs.io/en/latest/.