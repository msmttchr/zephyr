====================================================
Design Level and Advanced Information
====================================================

Building from Source
********************
While pre-compiled binaries are provided for standard setups, developers can customize the application or the UART multiplexer configuration by building from source using the Zephyr toolchain.

Building the RCP Application
============================
To build the main Radio Co-Processor application for your specific board:

.. code-block:: console

   # For NUCLEO-WBA65RI
   west build -b nucleo_wba65ri app_rcp

   # For NUCLEO-WBA55CG
   west build -b nucleowba55cg app_rcp

Board-specific Devicetree overlays located in ``app_rcp/boards/`` define the physical UART peripheral used and the virtual channel mapping.

UART Multiplexer Architecture
*****************************
The UART multiplexer allows multiple independent **virtual UART devices** to share a single **physical asynchronous UART**. 



Key Design Principles
=====================
* **Application Level**: Implemented entirely at the application level without modifying Zephyr core drivers.
* **Devicetree Driven**: Configuration, routing, and prioritization are defined in Devicetree.
* **Deterministic**: Uses fixed-size buffers and memory slabs with no heap allocation to ensure predictable RAM usage.
* **Priority-based**: TX arbitration is handled based on assigned channel priority; only one frame is transmitted at a time.

Framing Protocol
================
Data is exchanged using a framed packet structure to allow the host to distinguish between BLE, OpenThread, and System data:

+-------+-------+-----------+-------+---------+-----------+
| FI(1) | FF(1) | LEN(2)    | PLC(1)| PAYLOAD | CRC16(2)  |
+-------+-------+-----------+-------+---------+-----------+

* **FI (0xC0) / FF (0x02)**: Fixed framing start bytes.
* **LEN**: 2-byte little-endian value representing the payload length.
* **PLC**: 1-byte Protocol Link Context used for routing to the correct virtual UART.
* **CRC16**: 2-byte CRC-16-CCITT (little-endian) for data integrity.

Advanced Configuration
**********************

Adding a Virtual UART
=====================
The multiplexer behavior is fully configurable. To add a new channel, you must modify the Devicetree overlay for the multiplexer:

1. Define a new child node under the ``st,uart-mux`` compatible device.
2. **Priority**: Assign a unique integer (lower values have higher priority).
3. **PLC TX**: Set the fixed PLC value the MCU will send to the host.
4. **PLC RX**: Define a list of PLC values the MCU should route to this specific virtual device.

Below is an example of an overlay file defining three virtual channels (BLE, OpenThread, and System) sharing a single physical UART:

.. code-block:: dts

    / {
        uart_mux_bt: uart-mux-bt {
            compatible = "st,uart-mux";
            priority = <0>;   /* BLE HCI – highest priority */
            plc_tx = <0x91>;  /* Protocol Link Context for transmission */
            plc_rx = [91];    /* Accepted PLC values for routing */
            status = "okay";
        };

        uart_mux_ot: uart-mux-ot {
            compatible = "st,uart-mux";
            priority = <1>;   /* OpenThread */
            plc_tx = <0x92>;
            plc_rx = [92];
            status = "okay";
        };

        uart_mux_sys: uart-mux-sys {
            compatible = "st,uart-mux";
            priority = <2>;   /* System / Logs – lowest priority */
            plc_tx = <0x90>;
            plc_rx = [90];
            status = "okay";
        };

        chosen {
            zephyr,bt-c2h-uart = &uart_mux_bt;
            zephyr,ot-uart = &uart_mux_ot;
            uart-mux = &usart_mux;
        };
    };



Kconfig Options
***************

The multiplexer's behavior and memory footprint are managed through the following Kconfig parameters:

* **CONFIG_UART_MUX**: Master switch to enable the application-level UART multiplexer.
* **CONFIG_UART_MUX_PHY_RX_BUF_SIZE**: Sets the RX buffer size for the physical UART. Default is **64 bytes** (range 32-256).
* **CONFIG_UART_MUX_RX_BUF_SIZE**: Sets the payload buffer size for virtual UART devices. Default is **256 bytes** (range 64-4096).
* **CONFIG_UART_MUX_RX_TIMEOUT_MS**: Specifies the timeout in milliseconds for the RX state machine to resynchronize after an error. Default is **50 ms**.


Validation and Testing
**********************
The repository includes an auxiliary application ``app_uart_mux_test/`` for functional validation.

* **Purpose**: Stress tests the TX arbitration and RX routing logic.
* **Hardware Requirements**: Intended for development on boards without radio hardware (e.g., NUCLEO-L152RE).
* **Operation**: Runs independent threads that generate synthetic traffic to verify the framing protocol and host-side script stability.


