.. _pulga_lora:

Pulga LoRa Shield
################################

Overview
********
The Pulga LoRa shield uses an SX1272 transceiver through SPI
in a shield board with the same form-factor as the main board.

Pin Assignment
=======================================================

+-----------------------+-----------------+
| Shield Connector Pin  | Function        |
+=======================+=================+
|                    | SX1272 RESET    |
+-----------------------+-----------------+
|                   | SX1272 DIO0     |
+-----------------------+-----------------+
|                    | SX1272 SPI NSS  |
+-----------------------+-----------------+
|                    | SX1272 SPI MOSI |
+-----------------------+-----------------+
|                    | SX1272 SPI MISO |
+-----------------------+-----------------+
|                    | SX1272 SPI SCK  |
+-----------------------+-----------------+


Programming
***********

Set ``-DSHIELD=pulga_lora`` when you invoke ``west build``. For
example:

``$ west build -b pulga -- -DSHIELD=pulga_lora``

or

.. zephyr-app-commands::
   :zephyr-app: samples/lorawan/class_a
   :board: pulga
   :shield: pulga_lora
   :goals: build
