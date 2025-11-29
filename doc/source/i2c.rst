I2C
===

The I2C driver is contained within its header :code:`i2c.h` in :ref:`Lib <lib_section>` and its board-specific 
source :code:`i2c.c` in :ref:`Src <src_section>`. The driver allows for new code to easily read and write to an 
I2C interface on the board.

Types
-----

.. c:enum:: I2cPeriph

    The I2C interface to use. A particular sensor will always interface with the same I2C peripheral.

.. c:enum:: I2cSpeed

    The desired speed of the I2C interface in Hz. The possible options are:

    .. c:macro:: I2C_SPEED_INVALID

        Invalid speed.

    .. c:macro:: I2C_SPEED_STANDARD

        Standard speed (100 kHz).

    .. c:macro:: I2C_SPEED_FAST

        Fast speed (400 kHz).

    .. c:macro:: I2C_SPEED_FAST_PLUS

        Fast Plus speed (1 MHz).

.. c:struct:: I2cDevice

    Configuration for communicating with a specific device on the bus.

    .. c:member:: uint8_t address

        The 7-bit address of the I2C device to be communicated with.

    .. c:member:: I2cSpeed speed

        Speed to communicate with the device at.

    .. c:member:: I2cPeriph periph

        Peripheral which the device is connected to.

    .. c:member:: uint8_t scl

        Pin number for the SCL wire (PIN_PXX). Note that every device on the same peripheral must use
        the same pin number.

    .. c:member:: uint8_t sda

        Pin number for the SDA wire (PIN_PXX). Note that every device on the same peripheral must use
        the same pin number.
    
Functions
---------

.. c:function:: Status i2c_write(I2cDevice *device, uint8_t *tx_buf, size_t len)

    Writes data to an I2C device. If the peripheral is not yet initialized, it will be initialized.

    :param device: The device to write to.
    :param tx_buf: Pointer to the data to write.
    :param len: Number of bytes to write.

    :return: STATUS_OK if the write was successful, STATUS_PARAMETER_ERROR if a parameter is invalid, 
        or STATUS_ERROR if the write failed.

.. c:function:: Status i2c_read(I2cDevice *device, uint8_t *rx_buf, size_t len)

    Reads data from an I2C device. If the peripheral is not yet initialized, it will be initialized.

    :param device: The device to read from.
    :param rx_buf: Pointer to a buffer to store incoming data.
    :param len: Number of bytes to read.

    :return: STATUS_OK if the read was successful, STATUS_PARAMETER_ERROR if a parameter is invalid, 
        or STATUS_ERROR if the read failed.
