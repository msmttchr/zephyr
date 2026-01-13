.. _uart_multiplexer:

UART Multiplexer
################

Overview
********

The UART multiplexer allows multiple independent **virtual UART devices**
to share a single **physical asynchronous UART**.

Each virtual UART appears to the system as a normal Zephyr UART device,
while all data is transported over a single physical UART using a
framed multiplexing protocol.

The multiplexer is implemented at the **application level** and does not
modify or extend Zephyr core UART drivers.

Configuration, routing, and prioritization are entirely driven by
Devicetree.

User Point of View
******************

What problem does the UART mux solve?
=====================================

Some systems need to expose multiple logical UART channels to a host
while having only one physical UART available.

The UART multiplexer solves this by:

* Serializing traffic from multiple virtual UARTs
* Framing data on a shared physical UART
* Routing received frames back to the correct virtual UART

The user interacts only with virtual UART devices and does not need to
handle framing or routing manually.

Virtual UARTs
=============

Each virtual UART:

* Is instantiated from Devicetree
* Appears as a standard Zephyr UART device
* Can be referenced using ``chosen`` nodes
* Has independent RX and TX paths

From the user’s perspective, using a virtual UART is no different from
using a physical UART.

Configuration via Devicetree
============================

Virtual UARTs are configured entirely in Devicetree.

Key properties include:

Priority
   Determines TX arbitration order (lower value = higher priority).

PLC TX
   Determines how the PLC field is set on transmission.

PLC RX
   List of PLC values that are routed to this virtual UART.

.. note::
   No application code changes are required to add, remove, or reorder
   virtual UART channels.

Adding a new virtual UART
=========================

To add a new virtual UART channel:

1. Add a new UART mux child node in the board overlay.
2. Assign a unique priority.
3. Configure PLC TX and PLC RX values.
4. Optionally bind it using a ``chosen`` node.

The UART multiplexer automatically includes the new channel at runtime.

What the user does NOT need to handle
=====================================

The user does **not** need to:

* Handle framing bytes
* Compute or verify CRCs
* Manage RX reassembly
* Coordinate access to the physical UART
* Implement arbitration logic

All of this is handled internally by the multiplexer.

Software Design Point of View
*****************************

High-level architecture
=======================

.. code-block:: text

   +----------------------+
   | Virtual UART Device  |
   +---------+------------+
             |
   +---------v------------+
   |   UART Mux Driver    |
   |----------------------|
   | TX scheduler         |
   | RX state machine     |
   | PLC routing          |
   +---------+------------+
             |
   +---------v------------+
   | Physical UART (ASYNC)|
   +----------------------+

Framing protocol
================

All data on the physical UART is exchanged using framed packets:

+---------------+----+----+-----------+-----+---------+-----------+
| Field name    | FI | FF | LEN(LE16) | PLC | PAYLOAD | CRC16(LE) |
+===============+====+====+===========+=====+=========+===========+
| Size in Bytes | 1  | 1  | 2         | 1   | LEN     | 2         |
+---------------+----+----+-----------+-----+---------+-----------+
| Example       | C0 | 02 | 04 00     | 40  | payload | ED 55     |
+---------------+----+----+-----------+-----+---------+-----------+

Where:

FI / FF
   Fixed framing bytes.
LEN
   Length of ``PAYLOAD`` in bytes (little-endian).
PLC
   Protocol Link Context (routing field).
PAYLOAD
   Channel-specific data.
CRC16
   CRC-16-CCITT (little-endian), including the FI byte.

PLC handling
============

The PLC field is used for **RX routing** and **TX identification**. 
PLC behavior is fully defined by Devicetree:

TX
   * PLC can be injected from a fixed value.
   * Or extracted from the first byte of the virtual UART payload.

RX
   * Each channel defines which PLC values it accepts.
   * Delivery may include or exclude the PLC byte depending on channel type.

The multiplexer itself remains protocol-agnostic.

TX path
=======

1. A virtual UART submits a TX buffer.
2. The multiplexer determines the PLC value.
3. A frame descriptor is allocated from a fixed slab.
4. Frame header, payload, and CRC are transmitted sequentially.
5. TX completion is reported only after the full frame is sent.

Only one frame is transmitted at a time on the physical UART. TX arbitration between channels is priority-based.

RX path
=======

1. Bytes are received asynchronously from the physical UART.
2. A state machine reconstructs frames.
3. Length and CRC are validated.
4. PLC value is extracted.
5. The destination channel is selected.
6. Payload is delivered to the corresponding virtual UART.

.. warning::
   Frames with invalid CRC or unknown PLC values are dropped silently.

Memory model
============

The UART multiplexer uses deterministic memory allocation:

* Fixed-size RX buffer
* Fixed-size TX slab
* No heap allocation
* No unbounded queues

This ensures predictable RAM usage and real-time behavior.

Concurrency model
=================

* Physical UART operates using Zephyr’s asynchronous UART API.
* Virtual UARTs do not need to use async APIs.
* Internal serialization ensures safe access to the physical UART.
* All callbacks are short and non-blocking.

Design goals
============

* Protocol-agnostic transport.
* Devicetree-driven configuration.
* Deterministic memory usage.
* Clear separation between transport and application logic.
* Minimal coupling to Zephyr internals.

Summary
*******

The UART multiplexer provides a flexible and reusable solution for
transporting multiple virtual UART channels over a single physical UART.

Its behavior is fully configurable via Devicetree, requires no runtime
reconfiguration, and integrates cleanly with existing Zephyr UART APIs.