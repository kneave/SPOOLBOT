import socket
import serial
import time
import sys

# ------------------ CONFIG ------------------
COM_PORT = "COM31"
BAUD = 115200
UDP_PORT = 9000

PRINT_UDP_TEXT = False
PRINT_COM_TEXT = False
HEX_DUMP = False            # set True for raw bytes dumps
NEWLINE_FIX = True          # append \n if missing for plotters

LOOP_SLEEP_S = 0.001
# --------------------------------------------

def hexdump(data: bytes, maxlen=64):
    d = data[:maxlen]
    return " ".join(f"{b:02X}" for b in d) + (" ..." if len(data) > maxlen else "")

print(f"[INIT] Opening COM {COM_PORT} @ {BAUD} ...")
ser = serial.Serial(COM_PORT, BAUD, timeout=0)
print("[INIT] COM opened OK")

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", UDP_PORT))
sock.setblocking(False)

print(f"[INIT] UDP bound to 0.0.0.0:{UDP_PORT}")
print("[RUN] Bridge running: UDP <-> COM")
print("      UDP -> COM forwards telemetry to plotter")
print("      COM -> UDP sends commands back to last UDP sender\n")

last_sender = None
udp_count = 0
com_count = 0
last_stat = time.time()

try:
    while True:
        # ---------------- UDP -> COM ----------------
        try:
            data, addr = sock.recvfrom(4096)
            if data:
                udp_count += 1
                last_sender = addr

                if HEX_DUMP:
                    print(f"[UDP RX] {addr}  {len(data)} bytes  {hexdump(data)}")
                elif PRINT_UDP_TEXT:
                    try:
                        txt = data.decode("utf-8", errors="replace").rstrip("\r\n")
                        print(f"[UDP RX] {addr}  '{txt}'")
                    except Exception:
                        print(f"[UDP RX] {addr}  {len(data)} bytes")

                if NEWLINE_FIX and not data.endswith(b"\n"):
                    data += b"\n"

                ser.write(data)

        except BlockingIOError:
            pass

        # ---------------- COM -> UDP ----------------
        out = ser.read(1024)
        if out:
            com_count += 1

            if HEX_DUMP:
                print(f"[COM RX] {len(out)} bytes  {hexdump(out)}")
            elif PRINT_COM_TEXT:
                try:
                    txt = out.decode("utf-8", errors="replace").rstrip("\r\n")
                    print(f"[COM RX] '{txt}'")
                except Exception:
                    print(f"[COM RX] {len(out)} bytes")

            if last_sender is None:
                print("[WARN] COM data received but no UDP sender seen yet (can't reply).")
            else:
                sock.sendto(out, last_sender)
                if PRINT_COM_TEXT:
                    print(f"[UDP TX] -> {last_sender}")

        # ---------------- periodic stats ----------------
        now = time.time()
        if now - last_stat > 5.0:
            print(f"[STAT] udp_rx={udp_count} com_rx={com_count} last_sender={last_sender}")
            last_stat = now

        time.sleep(LOOP_SLEEP_S)

except KeyboardInterrupt:
    print("\n[EXIT] Ctrl+C")
finally:
    try:
        ser.close()
    except:
        pass
    try:
        sock.close()
    except:
        pass
    print("[EXIT] Closed.")
