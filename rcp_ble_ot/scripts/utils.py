import asyncio
import re
import os

async def wait_for_hci(timeout=20):
    from sdbus import (
        sd_bus_open_system,
        set_default_bus,
        DbusObjectManagerInterfaceAsync,
    )
    bus = sd_bus_open_system()
    set_default_bus(bus)
    # Create ObjectManager proxy
    manager = DbusObjectManagerInterfaceAsync()

    manager._connect("org.bluez", "/")

    async def listener():
        async for path, interfaces in manager.interfaces_added:
            if "org.bluez.Adapter1" in interfaces:
                return path.split('/')[-1]

    try:
        return await asyncio.wait_for(listener(), timeout=timeout)
    except asyncio.TimeoutError:
        return "Not found"

def hci_interfaces(timeout=20):
    from sdbus import (
        sd_bus_open_system,
        set_default_bus,
        DbusObjectManagerInterfaceAsync,
    )

    # Enumerate existing adapters
    bus = sd_bus_open_system()
    set_default_bus(bus)
    # Create ObjectManager proxy
    manager = DbusObjectManagerInterfaceAsync()

    manager._connect("org.bluez", "/")
    print ("Active interfaces:")
    for path, interfaces in objects.items():
        if "org.bluez.Adapter1" in interfaces:
            print(f"📡 Existing adapter: {path}")


def get_hci_status(hci_dev):
    from pydbus import SystemBus
    from sdbus import (
        sd_bus_open_system,
        set_default_bus,
        DbusObjectManagerInterfaceAsync,
    )
    bus = SystemBus()
    manager = bus.get("org.bluez", "/")
    objects = manager.GetManagedObjects()

    try:
        props = objects[f'/org/bluez/{hci_dev}']["org.bluez.Adapter1"]
        results = {
            "address": props.get("Address"),
            "powered": "ON" if props.get("Powered") else "OFF"
        }
    except:
        results = {
            "address": "Not available",
            "powered": "Not available"
        }
    return results

async def get_last_hci_interface():
    # 1. Run hciconfig to get the current state
    h_proc = await asyncio.create_subprocess_exec(
        "hciconfig", 
        stdout=asyncio.subprocess.PIPE, 
        stderr=asyncio.subprocess.DEVNULL,
        stdin=asyncio.subprocess.DEVNULL
    )
    stdout, _ = await h_proc.communicate()
    output = stdout.decode()

    # 2. Find ALL occurrences (e.g., ['hci0', 'hci1'])
    # Using findall ensures we see the full list in the order the kernel reports them
    interfaces = re.findall(r"(hci\d+):", output)

    if not interfaces:
        print("No HCI interfaces found.")
        return None

    # 3. Pick the last one (the most recently added/highest index)
    last_hci = interfaces[-1]
    
    return last_hci

def get_process_info(pid):
    """
    Returns the status and command line for a given PID.
    """
    try:
        # 1. Get the Command Line
        # The cmdline file uses null bytes (\x00) as delimiters
        with open(f"/proc/{pid}/cmdline", "r") as f:
            cmdline_raw = f.read()
            # Replace null bytes with spaces for readability
            command = cmdline_raw.replace('\x00', ' ').strip()

        # 2. Get the Status
        # We look for the "State:" line in the status file
        status = "Unknown"
        with open(f"/proc/{pid}/status", "r") as f:
            for line in f:
                if line.startswith("State:"):
                    # Format is usually: State: S (sleeping)
                    status = line.split(":", 1)[1].strip()
                    break

        return {
            "pid": pid,
            "status": status,
        }

    except FileNotFoundError:
        return {"pid": pid, "status": "Terminated"}
    except Exception as e:
        return {"pid": pid, "status": f"Error: {e}"}
