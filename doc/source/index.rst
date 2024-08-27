PAL 9000 Firmware Reference
===========================

Summary
-------

PAL 9000 is a series of custom flight computers designed and manufactured by Purdue Space Program High Altitude (PSP HA).
Although there are several PAL 9000 flight computers, their firmware is shared with the PSP-HA-Firmware repository.
This firmware is written in C and its purpose is to collect and record data from the multiple sensors on the flight computer,
to use this data to determine the state of the flight computer, to trigger events and deployments, and to transmit and
receive telemetry and flight data.

Hardware
--------

The latest version of PAL is PAL 9000 Version 5 (PAL 9K5). This version is based on the STM32H735 microcontroller and has
1.25" x 3.25" footprint with mounting holes that are compatible with the TeleMega. The board has the following sensors
and peripheral devices:

    * BMI088 6-axis IMU for rotation and low-acceleration
    * KX134-1211 3-axis accelerometer for high-acceleration
    * MS5637 barometer for altitude determination
    * IIS2MDC magentometer for compass heading
    * MAX-M10S-00B GPS module
    * MT29F4G 4Gbit SPI flash memory
    * STM32WLE5 MCU with LoRa for telemetry

Table of Contents
-----------------

.. toctree::
   :maxdepth: 2

   Usage <usage>
   Firmware Reference <code>