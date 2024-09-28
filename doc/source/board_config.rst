Board Configuration
===================

The board configuration system is implemented to manage settings for things such as state estimation and recovery.
The configuration is stored in the backup SRAM such that it should persist through reboots. 
The validity of the configuration in SRAM is verified with a checksum stored with the configuration.
If the checksum is invalid, that implies that the SRAM must have lost power, so the config is loaded from non-volatile flash instead.
The module also defines a default config that is loaded in case both the copy in SRAM and in flash have problems.

Users of the module can get a pointer to the config object by calling :c:func:`get_config_ptr()`.
Modifications can be made to the in-memory copy of the config through this pointer, but in order for changes to persist through a reboot, :c:func:`commit_config()` must be called.
In order to force loading from flash, :c:func:`invalidate_config()` can be called to manually invalidate the in-memory copy of the config.

Types
-----

.. c:struct:: BoardConfig

    Contains settings related to state estimation, recovery, and system constraints.

    .. c:member:: uint32_t sampling_rate_ms

        The period in milliseconds between state estimation update steps.

    .. c:member:: uint32_t state_init_time_ms

        The time in milliseconds used to determine the baseline value for the sensors during initialization.

    .. c:member:: float min_boost_acc_ms2

        The minimum acceleration, in m/s², above which the system is considered to be in boost mode.

    .. c:member:: float max_coast_acc_ms2

        The maximum acceleration, in m/s², below which the system is considered to be in coast mode.

    .. c:member:: float main_height_m

        The height above ground, in meters, at which the main pyro charge is fired during recovery.

    .. c:member:: uint32_t drogue_delay_ms

        The delay in milliseconds from apogee detection to drogue pyro activation.

    .. c:member:: uint32_t deploy_lockout_ms

        The time in milliseconds during which pyros cannot fire after launch detection.

    .. c:member:: uint32_t checksum

        The CRC-32 checksum of the configuration, used to verify data integrity.


Functions
---------

.. c:function:: BoardConfig* get_config_ptr()

    Returns a pointer to the currently loaded board configuration.

    :return: Pointer to the in-memory :c:struct:`BoardConfig` object, or NULL if no valid configuration is loaded.

.. c:function:: Status load_config()

    Loads the configuration from SRAM, flash, or defaults. The function checks the validity of the configuration using the checksum. If the checksum fails, it attempts to load from flash, and if that fails, the default configuration is loaded.

    :return: :code:`STATUS_OK` if the configuration was successfully loaded, or a relevant error code otherwise.

.. c:function:: Status commit_config()

    Saves changes to the configuration by updating the checksum and storing the configuration to flash memory.

    :return: :code:`STATUS_OK` if the configuration was successfully saved, or an error if the flash operation failed.

.. c:function:: Status invalidate_config()

    Invalidates the current configuration, forcing the next call to :code:`load_config()` to load from flash instead of SRAM.

    :return: :code:`STATUS_OK` if the invalidation was successful.
