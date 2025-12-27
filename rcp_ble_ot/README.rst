.. zephyr:code-sample:: rcp_ble_ot
   :name: RCP BLE / OT with UART multiplexer
   :relevant-api: uart async

Overview
********

This application implements a Radio Co-Processor (RCP) that exposes
multiple protocol stacks to a host processor using a single physical
UART interface.

A proprietary UART multiplexing layer is used to transport multiple
independent virtual UART channels over one asynchronous UART instance.
Channel routing and prioritization are fully configured via Devicetree.

The UART multiplexer is protocol-agnostic and may be reused for other
multi-channel UART use cases.

Architecture
************

- One physical UART using Zephyr's asynchronous UART API
- Multiple virtual UART devices instantiated from Devicetree
- Frame-based multiplexing protocol with CRC protection
- Priority-based TX arbitration between virtual UARTs

Each virtual UART appears to the application as a normal Zephyr UART
device and can be bound using ``chosen`` Devicetree properties.

UART Multiplexer
****************

The UART multiplexer is implemented entirely at application level and
does not modify Zephyr core drivers.

Key features:
- Framed transport with CRC-16-CCITT
- Deterministic memory usage (no heap)
- Configurable channel routing via PLC fields
- Channel priority defined in Devicetree
- Compatible with asynchronous UART operation

For detailed documentation, see ``UART_MUX.md``.

Building and Running
********************

Build the application as usual for your target board:

.. code-block:: console

   west build -b <board> rcp_ble_ot

Devicetree overlays are used to configure:
- The physical UART used by the multiplexer
- Virtual UART instances
- Channel priority and routing

Refer to the board overlay file for an example configuration.

Requirements
************

- Board with UART asynchronous API support
- One available UART peripheral
- Host-side software implementing the same framing protocol

Limitations
***********

- One frame in flight at a time on the physical UART
- Framing protocol is proprietary and host support is required

