# ===============================================================
#   RP2350 4IO + W5500 TCP CONTROLLER
#   AUTOSTART – NO COMMANDS NEEDED
#   Allowed IP: 192.168.10.99
# ===============================================================

import _thread
import time, json, utime
from machine import Pin, SPI, WDT
import w5500

# ===============================================================
# CONFIG
# ===============================================================
ALLOWED_IP = (192, 168, 10, 99)

MY_IP      = (192, 168, 10, 140)
SUBNET     = (255, 255, 255, 0)
GATEWAY    = (192, 168, 10, 1)
MAC        = (0xDE,0xAD,0xBE,0xEF,0x00,0x01)

PORT = 5005
HEARTBEAT_MS = 10000
MAIN_SLEEP_MS = 2
MAX_PULSE_MS = 5000

OUTPUT_PINS = [2, 3, 5, 6]
INPUT_PINS  = [29, 28, 27, 26]

# ===============================================================
# HARDWARE INIT
# ===============================================================
outs = [Pin(p, Pin.OUT, value=0) for p in OUTPUT_PINS]
out_state = [0, 0, 0, 0]
auto_off_deadline = [0, 0, 0, 0]

ins = [Pin(p, Pin.IN, Pin.PULL_UP) for p in INPUT_PINS]
in_stable = [ins[i].value() for i in range(4)]
in_last_raw = in_stable[:]

wdt = WDT(timeout=2500)

# ===============================================================
# JSON HELPERS
# ===============================================================
telemetry_id = 1

def telemetry_json(type_id):
    global telemetry_id
    s = {
        "Type": type_id,
        "MessageID": telemetry_id,
        "R1": out_state[0],
        "R2": out_state[1],
        "R3": out_state[2],
        "R4": out_state[3],
        "T1": in_stable[0],
        "T2": in_stable[1],
        "T3": in_stable[2],
        "T4": in_stable[3]
    }
    telemetry_id = (telemetry_id + 1) & 0x7FFFFFFF
    return (json.dumps(s) + "\n").encode()

def ack_json(cmd, status):
    s = {
        "MessageID": cmd["MessageID"],
        "Type": cmd["Type"],
        "Mode": cmd["Mode"],
        "SignalLenght": cmd["SignalLenght"],
        "R1": cmd["R1"],
        "R2": cmd["R2"],
        "R3": cmd["R3"],
        "R4": cmd["R4"],
        "Status": status
    }
    return (json.dumps(s) + "\n").encode()

# ===============================================================
# OUTPUT LOGIC
# ===============================================================
def set_output(i, v, send_event=True):
    out_state[i] = 1 if v else 0
    outs[i].value(out_state[i])
    if send_event:
        server_send(telemetry_json(1))

def process_auto_off(now):
    for i in range(4):
        d = auto_off_deadline[i]
        if d and utime.ticks_diff(now, d) >= 0:
            auto_off_deadline[i] = 0
            set_output(i, 0, send_event=True)

# ===============================================================
# INPUT LOGIC
# ===============================================================
def process_inputs(now):
    global in_last_raw, in_stable
    change = False

    for i in range(4):
        raw = ins[i].value()
        if raw != in_last_raw[i]:
            in_last_raw[i] = raw
        else:
            if raw != in_stable[i]:
                in_stable[i] = raw
                change = True

    if change:
        server_send(telemetry_json(1))

# ===============================================================
# COMMAND PARSER
# ===============================================================
def parse_cmd(line):
    try:
        return json.loads(line)
    except:
        return None

def handle_cmd(cmd):
    ok = 1

    for i in range(4):
        if cmd[f"R{i+1}"] == 1:
            if cmd["Mode"] == 1:   # permanent
                auto_off_deadline[i] = 0
                set_output(i, 1)
            else:
                # pulse
                set_output(i, 1)
                ms = cmd["SignalLenght"]
                if ms > MAX_PULSE_MS: ms = MAX_PULSE_MS
                if ms <= 0: ms = 1000
                auto_off_deadline[i] = utime.ticks_add(utime.ticks_ms(), ms)
        else:
            auto_off_deadline[i] = 0
            set_output(i, 0)

    server_send(ack_json(cmd, ok))

# ===============================================================
# W5500 SETUP
# ===============================================================
SCK=10; MOSI=11; MISO=12; CS=13; RESET=14

spi = SPI(1, baudrate=6_000_000, polarity=0, phase=0,
          sck=Pin(SCK), mosi=Pin(MOSI), miso=Pin(MISO))

w = w5500.W5500(spi, Pin(CS), Pin(RESET))
w.set_ip(MY_IP, SUBNET, GATEWAY, MAC)

# ===============================================================
# SERVER (THREAD)
# ===============================================================
client_connected = False

def server_send(data):
    global client_connected
    if client_connected:
        try:
            w.socket0_send(data)
        except:
            client_connected = False

def server_thread():
    global client_connected

    w.socket0_open(PORT)
    w.socket0_listen()

    last_hb = utime.ticks_ms()

    while True:
        now = utime.ticks_ms()
        wdt.feed()

        status = w.socket0_status()

        if status == 0x17:   # ESTABLISHED
            if not client_connected:
                client_connected = True

            # Heartbeat
            if utime.ticks_diff(now, last_hb) >= HEARTBEAT_MS:
                server_send(telemetry_json(0))
                last_hb = now

            # RECEIVE
            data = w.socket0_recv()
            if data:
                try:
                    line = data.decode().strip()
                    cmd = parse_cmd(line)
                    if cmd:
                        handle_cmd(cmd)
                except:
                    pass

        else:
            client_connected = False
            w.socket0_open(PORT)
            w.socket0_listen()

        process_auto_off(now)
        process_inputs(now)

        utime.sleep_ms(MAIN_SLEEP_MS)

# ===============================================================
# AUTO-START SERVER
# ===============================================================
_thread.start_new_thread(server_thread, ())
print("SERVER STARTED on port", PORT)
