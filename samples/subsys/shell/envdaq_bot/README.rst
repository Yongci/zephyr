EnvDAQ Bot
######

EnvDAQ Bot is a small GPIO control test application for the EnvDAQ Bot board.

The sample blinks the user LED continuously and enables the relay output once
every 30 seconds. The relay stays active for 5 seconds and is then turned off.

Runtime state changes are reported through the Zephyr logging subsystem.

Supported board
***************

Only ``envdaq_bot`` is supported.

Build and run
*************

.. code-block:: console

   west build -b envdaq_bot/esp32c6/hpcore -p always --sysbuild samples/subsys/shell/envdaq_bot
   west flash -d build --domain mcuboot -- --esp-device /dev/ttyACM1 --esp-baud-rate 115200
   west flash -d build --domain led_strip -- --esp-device /dev/ttyACM1 --esp-baud-rate 115200