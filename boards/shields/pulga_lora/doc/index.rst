.. _semtech_sx1272mb2das:

Pulga LoRa Shield
################################

Overview
********


Pin Assignment
=======================================================

+-----------------------+-----------------+
| Shield Connector Pin  | Function        |
+=======================+=================+
|                    | SX1272 RESET    |
+-----------------------+-----------------+
|                  | SX1272 DIO0     |
+-----------------------+-----------------+
|                    | SX1272 SPI NSS  |
+-----------------------+-----------------+
|                    | SX1272 SPI MOSI |
+-----------------------+-----------------+
|                    | SX1272 SPI MISO |
+-----------------------+-----------------+
|                    | SX1272 SPI SCK  |
+-----------------------+-----------------+


Requirements
************

Programming
***********

Set ``-DSHIELD=pulga_lora`` when you invoke ``west build``. For
example:

.. zephyr-app-commands::
   :zephyr-app: samples/lorawan/class_a
   :board: pulga
   :shield: pulga_lora
   :goals: build

References
**********
lol nope
