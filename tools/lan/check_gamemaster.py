"""
check_gamemaster.py -- run on a relay laptop (red / blue) to see whether it can reach the gamemaster.

    py -3 tools\\lan\\check_gamemaster.py              # listen for the gamemaster's LAN announcement
    py -3 tools\\lan\\check_gamemaster.py 192.168.1.20  # test a specific gamemaster IP

It tries the same two things field_relay.py does:
  1. UDP 5801  - the gamemaster broadcasts "FCGM <port>" once a second (automatic discovery)
  2. TCP 5800  - the relay's connection to the gamemaster
and says which one is blocked, so you know whether to open the firewall, use the
same Wi-Fi / switch, or pass --host <ip> to the relay.
"""
import socket
import sys

DISCOVERY_PORT = 5801
TCP_PORT = 5800


def discover(seconds=6.0):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.bind(("", DISCOVERY_PORT))
    except OSError as e:
        print(f"  can't listen on UDP {DISCOVERY_PORT}: {e}")
        return None
    s.settimeout(seconds)
    try:
        while True:
            data, addr = s.recvfrom(64)
            parts = data.decode(errors="ignore").split()
            if len(parts) == 2 and parts[0] == "FCGM" and parts[1].isdigit():
                return addr[0], int(parts[1])
    except socket.timeout:
        return None
    finally:
        s.close()


def tcp_ok(host, port):
    try:
        socket.create_connection((host, port), timeout=3).close()
        return True
    except OSError as e:
        print(f"  can't connect to {host}:{port} -- {e}")
        return False


def main():
    host = sys.argv[1] if len(sys.argv) > 1 else None
    if host is None:
        print(f"Listening for the gamemaster's announcement on UDP {DISCOVERY_PORT} (6 s)...")
        found = discover()
        if not found:
            print("NOT FOUND. Either the gamemaster isn't running, the laptops are on different\n"
                  "networks (or the Wi-Fi isolates devices), or this laptop's firewall blocks UDP 5801\n"
                  "(run tools\\lan\\open_firewall.bat as administrator).\n"
                  "You can still connect by IP:  check_gamemaster.py <gamemaster IP>, then start the\n"
                  "relay with --host <gamemaster IP>.")
            return 1
        host, port = found
        print(f"Found the gamemaster at {host} (announced TCP port {port}).")
    else:
        port = TCP_PORT
    print(f"Connecting to {host}:{port} ...")
    if tcp_ok(host, port):
        print(f"OK - the relay can reach the gamemaster. Start it with:\n"
              f"  start_red.bat   (or start_blue.bat)   [add  --host {host}  if discovery was skipped]")
        return 0
    print("The gamemaster is visible but TCP 5800 is blocked: open the firewall on the GAMEMASTER laptop "
          "(tools\\lan\\open_firewall.bat as administrator).")
    return 1


if __name__ == "__main__":
    sys.exit(main())
