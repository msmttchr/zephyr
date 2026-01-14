=========================
OpenThread and BLE RCP
=========================

TL;DR (Quick Start)
*******************

This repository contains a **main OpenThread + BLE RCP application** and a
**UART multiplexer test side application** that share the same UART
multiplexer implementation.

Build the **main RCP application** (recommended for normal use):

.. code-block:: console

   west build -b <board> app_rcp

Build the **UART mux test side application** (for boards without radio
or for UART multiplexer validation):

.. code-block:: console

   west build -b nucleo_l152re app_uart_mux_test

For UART multiplexer details, see ``UART_MUX.md``.

---

Overview
********

This repository provides an **application-level UART multiplexer** and
two applications that use it:

- **app_rcp/**: the main **OpenThread and BLE Radio Co-Processor (RCP) application**
- **app_uart_mux_test/**: a **UART multiplexer test side application**
  intended for validation and for boards without radio hardware

The UART multiplexer allows multiple independent virtual UART devices
to share a single physical UART using a framed protocol and the Zephyr
asynchronous UART API.

All routing, prioritization, and framing behavior is configured via
Devicetree.

---

Repository Structure
********************

::

   rcp_ble_ot/
   ├── src/
   │   ├── uart_mux.c
   │   └── uart_mux.h
   ├── scripts/
   │   └── uart_asyncio.py
   ├── dts/
   │   └── bindings/
   │       └── uart/
   │           └── st,uart-mux.yaml
   ├── app_rcp/
   │   ├── prj.conf
   │   ├── src/main.c
   │   └── boards/
   ├── app_uart_mux_test/
   │   ├── prj.conf
   │   ├── src/main.c
   │   └── boards/
   ├── UART_MUX.md
   ├── README.rst
   └── sample.yaml

The UART multiplexer implementation is shared between both applications
and resides in the common ``src/`` directory.

---

UART Multiplexer
****************

The UART multiplexer provides:

- Multiple virtual UART devices
- Priority-based TX arbitration
- Framed transport with CRC protection
- Deterministic memory usage (no heap allocation)
- Protocol-agnostic routing using PLC values

The multiplexer is implemented entirely at application level and does
not modify Zephyr core drivers.

Detailed documentation is provided in ``UART_MUX.md``.

---

Building the Applications
*************************

This repository contains two applications:

- The **main OpenThread + BLE RCP application** (:file:`app_rcp/`)
- The **UART mux test side application** (:file:`app_uart_mux_test/`)

Each must be built explicitly by selecting its directory.

---

Building the OpenThread and BLE RCP Application (main)
======================================================

The main RCP application is located in the ``app_rcp/`` directory.

It implements the full device functionality and uses the UART
multiplexer to expose multiple logical channels to a host processor,
typically:

- An OpenThread RCP interface (Spinel) over one virtual UART channel
- A BLE HCI interface over another virtual UART channel
- Optional additional channels (e.g. system console or diagnostics)

To build the RCP application:

.. code-block:: console

   west build -b <board> app_rcp

Replace ``<board>`` with your target board name.

Board-specific Devicetree overlays are used to configure:

- The physical UART used by the multiplexer
- Virtual UART channels
- Channel priorities and routing

Refer to the board overlay files in ``app_rcp/boards/`` for details.

The following diagram shows the connection of a NUCLEO-WBA65RI to a Ubuntu PC.

.. image:: app_rcp_diagram.png
   :alt: RCP connection to Ubuntu PC
   :width: 400px
   :align: center

---

Building the UART Mux Test Side Application
===========================================

The UART multiplexer test application is located in the
``app_uart_mux_test/`` directory.

This side application is intended for:

- Functional validation of the UART multiplexer
- Stress testing TX arbitration and RX routing
- Development and debug **on boards without radio hardware**
- Host‑side implementation bring‑up of the framing protocol

To build the test application:

.. code-block:: console

   west build -b nucleo_l152re app_uart_mux_test

The test application instantiates multiple virtual UART channels and
runs independent threads that generate and consume traffic on each
channel. It focuses on exercising the UART multiplexer itself rather
than providing full OpenThread/BLE RCP functionality.

---

Requirements
************

- Board with asynchronous UART API support
- One available UART peripheral
- For the main RCP application:
  - A board with radio hardware supported by :file:`app_rcp`
- Host-side software implementing the framing protocol (for RCP usage)

---

Limitations
***********

- Only one frame may be in flight on the physical UART at a time
- The framing protocol is proprietary and requires host-side support
