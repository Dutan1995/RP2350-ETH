# RP2350 4IO JSON controller + W5500 TCP SERVER
# Final version – heartbeat Type:0, event Type:1

from machine import Pin, SPI, WDT
import utime, sys
import w5500    # your fixed w5500.py

# -------------------- NETWORK CONFIG --------------------
IP      = (192,168,10,140)
SUBNET  = (255,255,255,0)
GATEWAY = (192,168,10,1)
MAC     = (0xDE,0xAD,0xBE,0xEF,0x00,0x01)
TCP_PORT = 5005

HB_INTERVAL_MS = 10000
MAIN_LOOP_SLEEP_MS = 2
DEBOUNCE_MS = 30
MAX_PULSE_MS = 5000
WDT_TIMEOUT_MS = 2000

# -------------------- I/O CONFIG -------------------------
OUTPUT_PINS = [2,3,5,6]
INPUT_PINS  = [29,28,27,26]

# -------------------- INIT HARDWARE ----------------------
outs = [Pin(p, Pin.OUT, value=0) for p in OUTPUT_PINS]
out_state = [0,0,0,0]
auto_off_deadline = [0,0,0,0]

ins = [Pin(p, Pin.IN, Pin.PULL_UP) for p in INPUT_PINS]
in_raw = [ins[i].value() for i in range(4)]
in_stable = in_raw[:]
in_last_change = [utime.ticks_ms()] * 4

# -------------------- WATCHDOG --------------------------
wdt = WDT(timeout=WDT_TIMEOUT_MS)

# -------------------- W5500 INIT ------------------------
print("Initializing W5500...")

# HW reset
rst = Pin(14, Pin.OUT)
rst.value(0)
utime.sleep_ms(200)
rst.value(1)
utime.sleep_ms(200)

# SPI init
spi = SPI(1,
          baudrate=6_000_000,
          polarity=0,
          phase=0,
          sck=Pin(10),
          mosi=Pin(11),
          miso=Pin(12))

w = w5500.W5500(spi, Pin(13), Pin(14))
w.set_ip(IP, SUBNET, GATEWAY, MAC)

print("Ethernet ready:", IP)

# -------------------- TCP SERVER ------------------------
def open_socket():
    w.socket0_close()
    utime.sleep_ms(10)
    w.socket0_open(TCP_PORT)
    utime.sleep_ms(10)
    w.socket0_listen()
    print("Listening on port", TCP_PORT)

open_socket()
print("READY. Waiting for client...")

# --------------------- JSON HELPERS ----------------------
telemetry_id = 1

def j01(x): return "1" if x else "0"

def telemetry_json(event_type):   # event_type: 0=heartbeat, 1=event
    global telemetry_id
    s = (
        '{"MessageID":' + str(telemetry_id) +
        ',"Type":' + str(event_type) +
        ',"R1":' + j01(out_state[0]) +
        ',"R2":' + j01(out_state[1]) +
        ',"R3":' + j01(out_state[2]) +
        ',"R4":' + j01(out_state[3]) +
        ',"T1":' + j01(in_stable[0]) +
        ',"T2":' + j01(in_stable[1]) +
        ',"T3":' + j01(in_stable[2]) +
        ',"T4":' + j01(in_stable[3]) +
        '}\n'
    )
    telemetry_id = (telemetry_id % 2147483647) + 1
    return s

def send_tcp(msg):
    try:
        w.socket0_send(msg.encode())
    except:
        pass

# --------------------- OUTPUT CONTROL --------------------
def validate_pin_index(i):
    return 0 <= i < 4

def out_perm_on(i):
    return out_state[i] == 1 and auto_off_deadline[i] == 0

def set_output(i, v, *, from_auto=False):
    if not validate_pin_index(i): return False
    v = 1 if v else 0
    changed = (out_state[i] != v)
    outs[i].value(v)
    out_state[i] = v
    if changed or from_auto:
        send_tcp(telemetry_json(1))  # event type=1
    return changed

def arm_auto_off(i, ms):
    if ms > MAX_PULSE_MS: ms = MAX_PULSE_MS
    auto_off_deadline[i] = utime.ticks_add(utime.ticks_ms(), ms)

def clear_auto_off(i):
    auto_off_deadline[i] = 0

def check_auto_offs(now):
    for i in range(4):
        d = auto_off_deadline[i]
        if d and utime.ticks_diff(now, d) >= 0:
            clear_auto_off(i)
            set_output(i, 0, from_auto=True)

# --------------------- INPUT DEBOUNCE --------------------
def scan_inputs(now):
    global in_raw, in_stable, in_last_change
    raw_now = [ins[i].value() for i in range(4)]
    for i in range(4):
        if raw_now[i] != in_raw[i]:
            in_raw[i] = raw_now[i]
            in_last_change[i] = now
        else:
            if raw_now[i] != in_stable[i] and utime.ticks_diff(now, in_last_change[i]) >= DEBOUNCE_MS:
                in_stable[i] = raw_now[i]
                send_tcp(telemetry_json(1))  # input event

# --------------------- COMMAND PARSER --------------------
def parse_cmd(s):
    try:
        d = eval(s)   # safe because your PC always sends valid JSON dict
        return d
    except:
        return None

def handle_cmd(cmd):
    try:
        Mode  = int(cmd.get("Mode",1))
        Type  = int(cmd.get("Type",1))
        Sig   = int(cmd.get("SignalLenght",1000))

        targets = [
            int(cmd.get("R1",0)),
            int(cmd.get("R2",0)),
            int(cmd.get("R3",0)),
            int(cmd.get("R4",0))
        ]

        for i,t in enumerate(targets):
            if t != 1: continue

            if Mode == 1:
                clear_auto_off(i)
                set_output(i, 1 if Type==1 else 0)
            else:
                if Type == 1:
                    set_output(i,1)
                    arm_auto_off(i,Sig)
                else:
                    clear_auto_off(i)
                    set_output(i,0)

    except:
        pass

# --------------------- MAIN LOOP ------------------------
buf = ""
last_hb = utime.ticks_ms()

while True:
    now = utime.ticks_ms()

    # watchdog
    try: wdt.feed()
    except: pass

    # TCP status monitor
    st = w.socket0_status()

    # ---- ESTABLISHED ----
    if st == 0x17:
        # Heartbeat
        if utime.ticks_diff(now, last_hb) >= HB_INTERVAL_MS:
            send_tcp(telemetry_json(0))  # HB Type=0
            last_hb = now

        # Incoming data
        data = w.socket0_recv()
        if data:
            try:
                s = data.decode().strip()
                buf += s
                if "\n" in buf or "}" in buf:
                    cmd = parse_cmd(buf)
                    if cmd: handle_cmd(cmd)
                    buf = ""
            except:
                buf = ""

    # ---- CLOSED → reopen ----
    elif st == 0x00:
        open_socket()

    # housekeeping
    check_auto_offs(now)
    scan_inputs(now)

    utime.sleep_ms(MAIN_LOOP_SLEEP_MS)

