.. _nucleo_wbaxx_comprehensive_setup:

Getting Started with Nucleo-WBA65RI and Nucleo-WBA55CG on Ubuntu
################################################################

This guide describes how to:

* Prepare the **NUCLEO-WBA65RI** and **NUCLEO-WBA55CG** hardware
  (flow control configuration).
* Configure **Ubuntu** to work with these boards without requiring root.
* Use the **Python UART multiplexer** to share a single UART with multiple
  software components (BLE, OpenThread, system console).

After following these steps you will be able to connect to the boards,
debug with ST-LINK, and attach multiple protocol stacks to the same UART.

Prerequisites
*************

Before you start, ensure you have:

* A **NUCLEO-WBA65RI** or **NUCLEO-WBA55CG** board.
* A Linux host running Ubuntu.
* Python 3 installed on the host.

Step 1: NUCLEO Board Setup
***************************

Hardware Flow Control Configuration
===================================

To enable **RTS/CTS** hardware flow control on the secondary Virtual COM
port (VCP2), the solder bridges must be configured differently on
**NUCLEO-WBA65RI** and **NUCLEO-WBA55CG**.

NUCLEO-WBA65RI (USART2 / VCP2)
-------------------------------

On NUCLEO-WBA65RI, VCP2 is connected to **USART2** of STM32WBA65RI.
To use **PA15** as RTS (default as per UM3448, Table 7), configure the
solder bridges on the mezzanine board **MB1801** as follows:

+-----------------+----------------+-------------------------------------------+
| Solder Bridge   | Position       | Function                                  |
+=================+================+===========================================+
| **SB4**         | **ON** (Closed)| Connects USART2_RTS (PA15) to ST-LINK CTS |
+-----------------+----------------+-------------------------------------------+
| **SB7**         | **OFF** (Open) | Disconnects alternative PA15 routing      |
+-----------------+----------------+-------------------------------------------+

Other VCP2 signals (per UM3448, Table 7):

- **USART2_RX (PA11)** → ST-LINK TX (PC10) via CN4/pin 37
- **USART2_TX (PA12)** → ST-LINK RX (PB11) via CN4/pin 35
  (`SB16 ON`, `SB13 OFF` by default)
- **USART2_CTS (PB15)** → ST-LINK RTS (PD12) via CN4/pin 26

NUCLEO-WBA55CG (LPUART1 / VCP2)
--------------------------------

On NUCLEO-WBA55CG, the second Virtual COM port (VCP2) uses **LPUART1**
of STM32WBA55CG (see UM3301, Section 7.4.5, Table 8). VCP2 replaces the
mass‑storage interface when enabled.

1. **Enable VCP2 TX/RX on MB1801 (mezzanine)**

   Close the following solder bridges on **MB1801**:

   +-----------------+----------------+--------------------------------------+
   | Solder Bridge   | Position       | Function                             |
   +=================+================+======================================+
   | **SB7**         | **ON** (Closed)| Connects LPUART1_RX (PA10)           |
   |                 |                | to ST-LINK TX (PC10)                 |
   +-----------------+----------------+--------------------------------------+
   | **SB8**         | **ON** (Closed)| Connects LPUART1_TX (PB5)            |
   |                 |                | to ST-LINK RX (PB11)                 |
   +-----------------+----------------+--------------------------------------+

2. **Route CTS/RTS on MB1801 (mezzanine)**

   On **MB1801**, these bridges control RTS/CTS connection between
   ST-LINK and the SoC (see UM3301, Table 8):

   +-----------------+----------------+--------------------------------------+
   | Solder Bridge   | Position       | Function                             |
   +=================+================+======================================+
   | **SB25**        | **ON** (Closed)| Routes ST-LINK_RTS (PD12)            |
   |                 |                | towards the SoC CTS path             |
   +-----------------+----------------+--------------------------------------+
   | **SB23**        | **ON** (Closed)| Routes ST-LINK_CTS (PD11)            |
   |                 |                | towards the SoC RTS path             |
   +-----------------+----------------+--------------------------------------+

3. **Select CTS/RTS pins on MB1803 (MCU RF board)**

   On **MB1803** you choose which STM32WBA55CG pins carry CTS and RTS:

   **CTS (LPUART1_CTS, from ST-LINK_RTS):**

   - To use **PB15** as CTS:

     +-----------------+----------------+----------------------------------+
     | Solder Bridge   | Position       | Function                         |
     +=================+================+==================================+
     | **SB32**        | **ON** (Closed)| Connects CTS signal to PB15      |
     +-----------------+----------------+----------------------------------+
     | **SB31**        | **OFF** (Open) | Disconnects alternative path     |
     +-----------------+----------------+----------------------------------+

   **RTS (LPUART1_RTS on PB9, from ST-LINK_CTS):**

   - To use **PB9** as RTS:

     +-----------------+----------------+----------------------------------+
     | Solder Bridge   | Position       | Function                         |
     +=================+================+==================================+
     | **SB25**        | **ON** (Closed)| Connects RTS signal to PB9       |
     +-----------------+----------------+----------------------------------+
     | **SB23**        | **OFF** (Open) | Disconnects alternative path     |
     +-----------------+----------------+----------------------------------+

   With this configuration, VCP2 signals are:

   - **TX / RX**: LPUART1_TX (PB5) / LPUART1_RX (PA10)  
   - **CTS**: LPUART1_CTS on PB15 (via SB32/SB31)  
   - **RTS**: LPUART1_RTS on PB9 (via SB25/SB23)

.. note::
   Enabling VCP2 on either board replaces the ST-LINK mass‑storage
   interface with a second Virtual COM port. Follow UM3448 (WBA65RI) and
   UM3301 (WBA55CG) for the required ST-LINK firmware configuration.

.. warning::
   Use a fine-tip soldering iron and appropriate ESD precautions. Always
   verify solder bridge locations and default states against the latest
   user manuals (UM3448 for **NUCLEO-WBA65RI**, UM3301 for
   **NUCLEO-WBA55CG**) before modification.

Step 2: Ubuntu Setup
********************

In this step you configure Ubuntu so that:

* A normal user can access the serial ports.
* The system does not interfere with the Nucleo serial interface.
* ST-LINK can be used without :code:`sudo`.

These steps are common to both **NUCLEO-WBA65RI** and **NUCLEO-WBA55CG**.

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

      # ST-LINK/V3 (used by NUCLEO-WBA65RI and NUCLEO-WBA55CG)
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

The usage is identical for **NUCLEO-WBA65RI** and **NUCLEO-WBA55CG**.

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

   # Basic launch (same for WBA65RI and WBA55CG)
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
