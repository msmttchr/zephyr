.. _nucleo_wba65ri_comprehensive_setup:

Getting Started with Nucleo-WBA65RI on Ubuntu
#############################################

This guide describes how to:

* Prepare the **NUCLEO-WBA65RI** hardware (flow control configuration).
* Configure **Ubuntu** to work with the board without requiring root.
* Use the **Python UART multiplexer** to share a single UART with multiple
  software components (BLE, OpenThread, system console).

After following these steps you will be able to connect to the board,
debug with ST-LINK, and attach multiple protocol stacks to the same UART.

Prerequisites
*************

Before you start, ensure you have:

* A NUCLEO-WBA65RI board.
* A Linux host running Ubuntu.
* Python 3 installed on the host.

Step 1: NUCLEO Board Setup
***************************

Hardware Flow Control Configuration
===================================

To enable **RTS/CTS** hardware flow control on the secondary VCP, modify the
solder bridges on the back of the PCB as follows:

+-----------------+----------------+--------------------------------+
| Solder Bridge   | Position       | Function                       |
+=================+================+================================+
| **SB4**         | **ON** (Closed)| Connects MCU_CTS to ST-Link RTS|
+-----------------+----------------+--------------------------------+
| **SB7**         | **OFF** (Open) | Disconnects ETM.JTMI           |
+-----------------+----------------+--------------------------------+

.. warning::
   Use a fine-tip soldering iron and work under magnification if possible.
   Ensure SB7 is fully open to avoid signal contention on the ETM/JTMI line.

Step 2: Ubuntu Setup
********************

In this step you configure Ubuntu so that:

* A normal user can access the serial ports.
* The system does not interfere with the Nucleo serial interface.
* ST-LINK can be used without :code:`sudo`.

System Permissions & Conflicts
==============================

1. **Add the user to the dialout group**

   Grant permission to access serial devices:

   .. code-block:: bash

      sudo usermod -aG dialout $USER

   .. note::
      You must log out and log back in for this change to take effect.

2. **Disable ModemManager**

   Prevent the system from sending AT commands to the Nucleo:

   .. code-block:: bash

      sudo systemctl stop ModemManager
      sudo systemctl disable ModemManager

ST-LINK Udev Rules
==================

Create a udev rule to allow ST-LINK/V3 access without :code:`sudo`:

1. **Create the udev rules file**

   .. code-block:: bash

      sudo nano /etc/udev/rules.d/49-stlinkv3.rules

2. **Add the following content:**

   .. code-block:: text

      # ST-LINK/V3
      SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374d", MODE="0666", GROUP="dialout"
      SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374e", MODE="0666", GROUP="dialout"
      SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374f", MODE="0666", GROUP="dialout"

3. **Reload udev rules:**

   .. code-block:: bash

      sudo udevadm control --reload-rules
      sudo udevadm trigger

At this point, reconnect the Nucleo board’s USB cable to ensure the
new rules are applied.

Step 3: Python UART Multiplexer
*******************************

Overview
========

The :file:`uart_mux_asyncio.py` script bridges a single physical UART port
to multiple virtual PTYs. This allows you to run:

* A BLE stack (e.g., BlueZ via HCI)
* An OpenThread stack (via Spinel)
* A system console or management interface

all over the same physical UART.

Installing Dependencies
=======================

Install the asynchronous serial dependency:

.. code-block:: bash

   pip install pyserial-asyncio

Basic Usage
===========

The script maps the physical port (for example, :file:`/dev/ttyACM1`) to
three virtual channels.

.. code-block:: bash

   # Basic launch
   python3 uart_mux_asyncio.py --port /dev/ttyACM1 --baudrate 2000000

   # Launch with automatic Bluetooth and OpenThread attachment
   sudo python3 uart_mux_asyncio.py --bt-attach --ot-manager ot-daemon

Key Command-Line Arguments
==========================

* :code:`--port`:
  Serial port device (default: :file:`/dev/ttyACM0`).

* :code:`--baudrate`:
  Communication speed (default: :code:`2000000`).

* :code:`--rtscts`:
  Enable hardware flow control (requires SB4/SB7 hardware modification).

* :code:`--bt-attach`:
  Spawns :code:`btattach` on the BLE channel (requires root).

* :code:`--ot-manager`:
  Spawns :code:`ot-daemon` or :code:`otbr-agent` on the OpenThread channel
  (requires root).

Interactive Mode
================

While the script is running, you can interact with it directly from the
terminal:

* **Press 'i'**:
  Displays the current status of the PTYs and the PIDs of attached daemons.

* **Press 'q'**:
  Safely shuts down the multiplexer and cleans up virtual devices.

Virtual Device Mapping
======================

The script creates three PTYs. Look for the following lines in the terminal
output to identify the virtual device paths:

* **BLE**:
  Use this for BlueZ / HCI communication.

* **OT**:
  Use this for OpenThread Spinel communication.

* **System**:
  Use this for console or management frames.

.. tip::
   If you see "staircase" text in your terminal, the script includes a
   custom logger to handle the translation between Linux LF and Serial CR/LF
   while in raw/cbreak mode.