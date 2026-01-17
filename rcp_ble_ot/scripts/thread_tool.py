import sys
import time
import argparse
import subprocess
import serial
import re

# This is a sample Dataset TLV. It contains:
# Channel 15, PANID 0x1234, Network Name "MyThread", and a default Master Key.
DEFAULT_DATASET = "0e080000000000010000000300000f35060004001fffe00208dead00beef00cafe0708fd00dead00beef00051011112222333344445555666677778888030e4d795468726561644e6574010212340410445566778899aa00bbccddeeff0011220c0402a0f7b8"

class ThreadNode:
    def __init__(self, port, baud, mode):
        self.port = port
        self.baud = baud
        self.mode = mode # 'leader' or 'child'
        self.ser = None

    def connect(self):
        if self.mode == 'child':
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
        else:
            print("Leader mode: System-wide ot-ctl active.")

    def run_command(self, cmd):
        if self.mode == 'leader':
            full_cmd = f"sudo ot-ctl {cmd}"
            try:
                return subprocess.check_output(full_cmd.split(), stderr=subprocess.STDOUT).decode().strip()
            except: return "Error"
        else:
            if not self.ser: return "No Serial"
            self.ser.read_all() # Flush
            self.ser.write(f"{cmd}\r\n".encode())

            # --- IMPROVED READ LOGIC ---
            response = ""
            timeout = time.time() + 10  # 10-second max wait for slow pings
            while time.time() < timeout:
                if self.ser.in_waiting > 0:
                    chunk = self.ser.read(self.ser.in_waiting).decode(errors='ignore')
                    response += chunk
                    # Look for the sentinel: Done, Error, or Invalid
                    if "Done" in response or "Error" in response or "Invalid" in response:
                        break
                time.sleep(0.1) # Small poll interval to save CPU

            # Clean up the echo
            lines = response.strip().splitlines()
            if lines and cmd in lines[0]:
                return "\n".join(lines[1:]).strip()
            return response.strip()

    def factory_reset(self):
        print(f"Factory resetting {self.mode}...")
        self.run_command("factoryreset")
        # Give the device time to reboot
        time.sleep(3)
        print("Reboot complete.")

    def setup_network(self, network_name="MyThread", panid="0x1234", channel="15", key="00112233445566778899aabbccddeeff"):
        print(f"Configuring {self.mode} manually...")

        # 1. Stop and Clear
        self.run_command("thread stop")
        self.run_command("dataset clear")

        # 2. Set Components
        self.run_command(f"dataset init new")
        self.run_command(f"dataset networkname {network_name}")
        self.run_command(f"dataset panid {panid}")
        self.run_command(f"dataset channel {channel}")
        self.run_command(f"dataset networkkey {key}")
        self.run_command("dataset extpanid 40ef2efe431ed698")
        self.run_command("dataset meshlocalprefix fd73:b95a:b17b:52c0::")

        # 3. Commit and Start
        self.run_command("dataset commit active")
        self.run_command("ifconfig up")

        if self.mode == 'child':
            print("Forcing End Device mode (disabling router eligibility)...")
            # Using 'routereligible disable' as it's standard for most Zephyr/WBA builds
            self.run_command("routereligible disable")

        self.run_command("thread start")
        print(f"{self.mode.capitalize()} is now starting with Name: {network_name}, PAN: {panid}")
        print(self.run_command("dataset active"))

    def get_ip_addresses(self):
        """Returns a list of all IPv6 addresses assigned to this node."""
        output = self.run_command("ipaddr")
        # Filters out 'Done' and empty lines, returns list of IPs
        return [line.strip() for line in output.split('\n') if ":" in line]

    def get_child_table(self):
        """Parses the child table into a list of dictionaries."""
        output = self.run_command("child table")
        print(f"\n--- Child Table for {self.mode.upper()} ---")
        if "Done" in output and len(output.split('\n')) <= 1:
            print("No children connected.")
            return []

        print(output) # Print the raw table for visual reference

        # Logic to extract the RLOC16 or ID from the table
        children = []
        for line in output.split('\n'):
            if "|" in line or ":" in line: # Basic filter for table rows
                children.append(line.strip())
        return children

    def ping(self, target_ip, count=3):
        print(f"Pinging {target_ip}...")
        out = self.run_command(f"ping {target_ip} 56 {count}")

        # Check for immediate errors
        if "Error" in out or "invalid" in out.lower():
            print(f"Ping failed: {out.strip()}")
            return False

        # Robust Regex: looks for the number followed by 'packets received'
        # re.IGNORECASE and re.MULTILINE help if the echo is still present
        match = re.search(r"(\d+)\s+packets\s+received", out, re.IGNORECASE)

        if match:
            rx_count = int(match.group(1))
            if rx_count > 0:
                print(f"SUCCESS: {rx_count}/{count} packets received.")
                return True
            else:
                print(f"FAILURE: 0/{count} packets received.")
                return False
        else:
            print(f"ERROR: Could not parse response.\nRaw output: {out}")
            return False

    def discover_and_ping_children(self):
        if self.mode != 'leader':
            return

        print("Searching for children...")
        # Get the Child Table
        table = self.run_command("child table")

        # Extract RLOC16s using regex
        import re
        rlocs = re.findall(r"0x[0-9a-fA-F]{4}", table)

        if not rlocs:
            print("No children found in table.")
            return

        for rloc in rlocs:
            # Most CLI versions allow: child ip <rloc16>
            # If that fails, we can ping the RLOC16 directly if we have the prefix
            # But let's try to get the full EID first:
            print(f"Testing connectivity to Child {rloc}...")
            # We can use the RLOC16 as the target for the ping command directly!
            # OpenThread will resolve this if it's in the neighbor table.
            self.ping(rloc)

    def auto_ping(self):
        if self.mode != 'leader':
            print("Auto-ping discovery is only available in Leader mode.")
            return

        print("Checking EID Cache for connected children...")
        output = self.run_command("eidcache list")

        # Find all IPv6 addresses mapped to RLOCs (e.g., ec01)
        # Format: fd73:b95a... ec01 cache ...
        entries = re.findall(r"([a-f0-9:]+)\s+([a-f0-9]{4})\s+cache", output)

        if not entries:
            # The Leader Anycast address is the prefix + 0000:00ff:fe00:fc00
            leader_anycast = "fd73:b95a:b17b:52c0:0:ff:fe00:fc00"
            print("\n[!] EID Cache is empty.")
            print(f"    Hint: Run this command on the Child (STM32) to register its IP:")
            print(f"    ping {leader_anycast}")
            return

        for ip, rloc in entries:
            print(f"\n[Child 0x{rloc} detected]")
            self.ping(ip)

def main():
    parser = argparse.ArgumentParser(description="Unified Thread Setup Tool")
    parser.add_argument("-u", "--uart", help="Serial port for Child")
    parser.add_argument("-m", "--mode", choices=['leader', 'child'], required=True)
    parser.add_argument("-c", "--command",
                        choices=['factory-reset', 'setup', 'state', 'ping', 'discover'],
                        required=True)
    parser.add_argument("-p", "--param", help="Target IP for ping")

    # Optional arguments for the setup command
    parser.add_argument("--name", default="MyThread", help="Network Name")
    parser.add_argument("--panid", default="0x1234", help="PAN ID (hex)")
    parser.add_argument("--channel", default="15", help="Channel (11-26)")
    parser.add_argument("--key", default="00112233445566778899aabbccddeeff", help="Network Key (32 hex chars)")

    args = parser.parse_args()
    node = ThreadNode(args.uart, 115200, args.mode)
    node.connect()

    if args.command == 'factory-reset':
        node.factory_reset()

    elif args.command == 'setup':
        # Now passing individual arguments instead of one dataset blob
        node.setup_network(
            network_name=args.name,
            panid=args.panid,
            channel=args.channel,
            key=args.key
        )
        time.sleep(5) # Give it a moment to attach
        print(f"Final State: {node.run_command('state')}")

    elif args.command == 'state':
        print(f"Current State: {node.run_command('state')}")

    elif args.command == 'ping':
        if not args.param:
            # Auto-mode: find children and ping them
            if args.mode == 'leader':
                node.auto_ping()
            else:
                print("For child-to-leader, please provide leader IP with -p")
        else:
            # Manual mode
            node.ping(args.param)

    elif args.command == 'discover':
        print(f"Local IP Addresses for {args.mode}:")
        for ip in node.get_ip_addresses():
            print(f"  > {ip}")

        if args.mode == 'leader':
            node.get_child_table()
if __name__ == "__main__":
    main()
