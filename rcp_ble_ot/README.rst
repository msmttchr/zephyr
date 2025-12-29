.. zephyr:code-sample:: uart_mux_rcp
   :name: UART multiplexer with RCP and test applications
   :relevant-api: uart async

===============================
UART Multiplexer RCP Repository
===============================

TL;DR (Quick Start)
******************

This repository contains **two independent applications** sharing the
same UART multiplexer implementation.

Build the **RCP application**:

.. code-block:: console

   west build -b <board> app_rcp

Build the **UART mux test application** (recommended for validation):

.. code-block:: console

   west build -b nucleo_l152re app_uart_mux_test

For UART multiplexer details, see ``UART_MUX.md``.

---

Overview
********

This repository provides an **application-level UART multiplexer** and
two applications that use it:

- A full **Radio Co-Processor (RCP)** application
- A **UART multiplexer test application** for validation and stress
  testing

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

This repository contains two independent applications. Each must be
built explicitly by selecting its directory.

---

Building the RCP Application
============================

The RCP application is located in the ``app_rcp/`` directory.

It implements the full device functionality and uses the UART
multiplexer to expose multiple logical channels to a host processor.

To build the RCP application:

.. code-block:: console

   west build -b <board> app_rcp

Replace ``<board>`` with your target board name.

Board-specific Devicetree overlays are used to configure:
- The physical UART used by the multiplexer
- Virtual UART channels
- Channel priorities and routing

Refer to the board overlay files in ``app_rcp/boards/`` for details.

---

Building the UART Mux Test Application
=====================================

The UART multiplexer test application is located in the
``app_uart_mux_test/`` directory.

This application is intended for:
- Functional validation of the UART multiplexer
- Stress testing TX arbitration and RX routing
- Development without requiring real radio hardware

To build the test application:

.. code-block:: console

   west build -b nucleo_l152re app_uart_mux_test

The test application instantiates multiple virtual UART channels and
runs independent threads that generate and consume traffic on each
channel.

---

Requirements
************

- Board with asynchronous UART API support
- One available UART peripheral
- Host-side software implementing the framing protocol (for RCP usage)

---

Limitations
***********

- Only one frame may be in flight on the physical UART at a time
- The framing protocol is proprietary and requires host-side support

