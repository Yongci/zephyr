.. zephyr:board:: envdaq_bot

Overview
********

EnvDAQ Bot is a compact development board based on ESP32-C6-MINI-1U chip with integrated
4 MB flash. This board integrates complete Wi-Fi, Bluetooth LE, Zigbee, and Thread functions.

For more information, check `EnvDAQ Bot`_.

Hardware
********

The EnvDAQ Bot is a compact board with the ESP32-C6-MINI-1U chip directly mounted, featuring
a 4 MB SPI flash. The board includes a USB Type-C connector, boot and reset buttons, and an RGB LED.

.. include:: ../../../espressif/common/soc-esp32c6-features.rst
   :start-after: espressif-soc-esp32c6-features

Supported Features
==================

.. zephyr:board-supported-hw::

System Requirements
*******************

.. include:: ../../../espressif/common/system-requirements.rst
   :start-after: espressif-system-requirements

Programming and Debugging
*************************

.. zephyr:board-supported-runners::

.. include:: ../../../espressif/common/building-flashing.rst
   :start-after: espressif-building-flashing

MCUboot (Board Default)
=======================

For this board, MCUboot is enabled at board level for sysbuild.
The board defaults select ``BOOTLOADER_MCUBOOT`` in ``Kconfig.sysbuild``.

Important:
Building without sysbuild creates an application-only image and does not build
or flash the MCUboot bootloader domain.

To build and flash with MCUboot, use sysbuild:

.. zephyr-app-commands::
   :zephyr-app: samples/drivers/led/led_strip
   :board: envdaq_bot/esp32c6/hpcore
   :gen-args: --sysbuild
   :goals: build flash

Before flashing, verify the sysbuild output contains MCUboot artifacts:

1. ``domains.yaml`` exists in the build directory.
2. ``mcuboot/zephyr/`` exists in the build directory.
3. ``zephyr/`` exists in the build directory for the application domain.

If these are missing, rebuild with ``--sysbuild`` and flash from that build
directory.

.. include:: ../../../espressif/common/board-variants.rst
   :start-after: espressif-board-variants

Debugging
=========

.. include:: ../../../espressif/common/openocd-debugging.rst
   :start-after: espressif-openocd-debugging

References
**********

.. target-notes::

.. _`EnvDAQ Bot`: https://github.com/Yongci/EnvDaqBox-PCB
