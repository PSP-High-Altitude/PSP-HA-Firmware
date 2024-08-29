SPI
===

The SPI driver is contained within its header :code:`spi.h` in :ref:`Lib <lib_section>` and its board-specific 
source :code:`spi.c` in :ref:`Src <src_section>`. The driver allows for new code to easily read and write to an 
SPI interface on the board.

Types
-----

.. c:enum:: SpiPeriph

    The SPI interface to use. A particular sensor will always interface with the same SPI peripheral.

.. c:enum:: SpiSpeed

    The desired speed of the SPI interface in Hz. The possible options are:

    .. c:macro:: SPI_SPEED_INVALID

    .. c:macro:: SPI_SPEED_100kHz

    .. c:macro:: SPI_SPEED_500kHz

    .. c:macro:: SPI_SPEED_1MHz

    .. c:macro:: SPI_SPEED_10MHz

    .. c:macro:: SPI_SPEED_20MHz

.. c:struct:: SpiDevice

    Configuration for communicating with a specific device on the bus.

    .. c:member:: SpiSpeed clk

        The clock speed of the SPI interface.

    .. c:member:: SpiPeriph periph

        Peripheral which the device is connected to.

    .. c:member:: bool cpol

        Clock polarity configuration.

    .. c:member:: bool cpha

        Clock phase configuration.

    .. c:member:: uint8_t mosi

        Pin number for the MOSI wire (PIN_PXX). Note that every device on the same peripheral must use
        the same pin number.

    .. c:member:: uint8_t miso

        Pin number for the MISO wire (PIN_PXX). Note that every device on the same peripheral must use
        the same pin number.

    .. c:member:: uint8_t sck

        Pin number for the SCK wire (PIN_PXX). Note that every device on the same peripheral must use
        the same pin number.

    .. c:member:: uint8_t cs

        Pin number for the CS wire (PIN_PXX). Note that every device on the same peripheral must use a
        **different** pin number.

Functions
---------

.. c:function:: Status spi_exchange(SpiDevice* dev, uint8_t* tx_buf, uint8_t* rx_buf, uint16_t len)

    Exchanges data with an SPI device. If the peripheral is not yet initialized, it will be initialized.

    :param dev: The device to echange data with.
    :param tx_buf: Pointer to the data to write.
    :param rx_buf: Pointer to a buffer to store incoming data.
    :param len: Number of bytes to exchange.

    :return: STATUS_OK if the write was successful, STATUS_PARAMETER_ERROR if a parameter is invalid, 
        or STATUS_HARDWARE_ERROR if the write failed.

.. c:function:: Status spi_exchange_nosetup(SpiDevice* dev, uint8_t* tx_buf, uint8_t* rx_buf, uint16_t len)

    Exchanges data with an SPI device. The peripheral must be initialized before calling this function.

    :param dev: The device to echange data with.
    :param tx_buf: Pointer to the data to write.
    :param rx_buf: Pointer to a buffer to store incoming data.
    :param len: Number of bytes to exchange.

    :return: STATUS_OK if the write was successful, STATUS_PARAMETER_ERROR if a parameter is invalid, 
        or STATUS_HARDWARE_ERROR if the write failed.