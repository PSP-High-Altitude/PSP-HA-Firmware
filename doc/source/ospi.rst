SPI
===

The OSPI driver is contained within its header :code:`ospi.h` and its source :code:`ospi.c` in 
:ref:`Src <src_section>`. The driver allows for new code to easily read and write to an OSPI interface 
on the board. Note that the driver is used for Quad-SPI over the OSPI peripheral.

Types
-----

.. c:enum:: OSpiPeriph

    The OSPI interface to use. Only one OSPI peripheral can be used at one time, and only one
    device should use the bus.

.. c:enum:: OSpiSpeed

    The desired speed of the OSPI interface in Hz. The possible options are:

    .. c:macro:: OSPI_SPEED_INVALID

    .. c:macro:: OSPI_SPEED_1MHz

    .. c:macro:: OSPI_SPEED_5MHz

    .. c:macro:: OSPI_SPEED_10MHz

    .. c:macro:: OSPI_SPEED_20MHz

    .. c:macro:: OSPI_SPEED_40MHz

    .. c:macro:: OSPI_SPEED_80MHz

.. c:struct:: OSpiDevice

    Configuration for communicating with a specific device on the bus.

    .. c:member:: OSpiPeriph periph

        Peripheral which the device is connected to.

    .. c:member:: OSpiSpeed clk

        The clock speed of the OSPI interface.

    .. c:member:: uint8_t sck

        Pin number for the SCK wire (PIN_PXX).

    .. c:member:: uint8_t ncs

        Pin number for the CS wire (PIN_PXX).

    .. c:member:: uint8_t io0

        Pin number for the MOSI wire (PIN_PXX).

    .. c:member:: uint8_t io1

        Pin number for the MISO wire (PIN_PXX).

    .. c:member:: uint8_t io2

        Pin number for the IO2 wire (PIN_PXX).

    .. c:member:: uint8_t io3

        Pin number for the IO3 wire (PIN_PXX).

    .. c:member:: uint8_t device_size

        Describes the size of the device in bytes following the formula :code:`2^(device_size+1)`.

Functions
---------