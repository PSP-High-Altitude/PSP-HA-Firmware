The Main Program
================

The main program is the entry point for the firmware when the board initializes. 
It is responsible for setting up the core functions of the microcontroller and the board, and then entering the tasks
of the OS.

Macros
------

.. c:macro:: TASK_CREATE(func, pri, ss)

    This macro creates a FreeRTOS task with the given function, priority, and stack size.

    :param func: The function to run in the task.
    :param pri: The priority of the task.
    :param ss: The stack size of the task.

.. c:function:: int _write(int file, char *data, int len)

    This function redefines the C stdio write function to allow serial output. It is possible to try printing
    before the USB peripheral is initialized. Because we still want to capture this output, we buffer any prints
    that occurred before initialization. If the USB peripheral is initialized, we print anything that may be in
    the buffer as well as the current print. Note that the prints in the buffer will only be transmitted if the
    USB peripheral is initialized successfully; therefore, any debugging before this should be done with a 
    proper debugger.

.. c:function:: void handle_pause()

    This function is called in the SysTick handler to poll the pause button. If this button is pressed, we save
    the storage and stop writing to avoid corruption. We also restart logging if the button is released. Proper
    pausing consists of pressing the button and then pressing reset while the pause button is held.

.. c:function:: void init_task()

    This task initializes the board and peripherals and starts all other tasks. It first initializes every
    subsystem. The :code:`EXPECT_OK` macro is used to check for and report any errors. All of the OS tasks are
    then initialized. If the MTP mode is selected, its tasks will be started instead. An infinitely looping
    delay then ensures that the task does not exit.
    
    The initializations performed are:
        * :code:`init_storage`
        * :code:`init_sensors`
        * :code:`init_state_est`
        * :code:`init_pyros`
        * :code:`pspcom_init`
        * :code:`MX_USB_DEVICE_Init`

    The tasks started are:
        * :code:`pyros_task`
        * :code:`read_sensors_task`
        * :code:`state_est_task`
        * :code:`pspcom_process_bytes`
        * :code:`pspcom_send_standard`
        * :code:`read_gps_task`
        * :code:`storage_task`
        * :code:`mtp_readwrite_file_task` If MTP mode is selected

.. c:function:: int main(void)

    The main function first initializes the STM HAL, clocks, and timers. It configures some essential IO. It then
    determines the startup mode (MTP or normal). Finally, it configures the priority grouping 
    (https://www.freertos.org/RTOS-Cortex-M3-M4.html) and starts the init task.

Other functions contained within main.c are interrupt handlers which allow for the SysTick and purposes.