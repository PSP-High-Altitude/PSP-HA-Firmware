The Storage Driver
==================

The storage driver is responsible for reading and writing data to the flash memory on the PAL 9000.
This code is contained within the :code:`storage.c` and :code:`storage.h` files.

.. c:function:: Status init_storage()

    This function initializes the flash memory. It initializes the filesystem diskio driver,
    then initializes the flash hardware. It also initializes queues for data storage.

    :return: A status indicating success or failure.

.. c:function:: void storage_task()

    This task is responsible for writing data to the flash memory. It waits for data to be written to the
    queues and then writes it to the flash memory. Data is written using the protobuf 2 format. It checks the
    status of the write operation. Finally the task also checks the pause flag and handles stopping and 
    restarting of the storage.