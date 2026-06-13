# EV4 CAN Dashboard (Raspberry Pi native)

Live dashboard for the BER EV4 Vehicle Bus, running **directly on a Raspberry Pi**
with two MCP2515 CAN HATs. The Pi reads both CAN buses natively over SocketCAN
(`python-can`), decodes every frame against the bundled DBCs, and displays them
live. It can also **write** to the Cascadia inverters (command + parameter
frames) for motor calibration, and repeats the inverter command "heartbeat"
itself (the job the ESP32 firmware used to do) with a deadman safety.

```
  Vehicle Bus (CAN 500k) ◄─► MCP2515 HAT ─► can0 ┐
  Inverter 1 bus (500k)  ◄─► MCP2515 HAT ─► can1 ┴► SocketCAN ─► ev4_dashboard.py
                                                                 (this app, on the Pi)
```

Internally the dashboard uses **bus index 0 and 1**, mapped to the two
interfaces in channel order:

- **bus 0** → first channel (`can0`): Vehicle bus + Inverter 2 + IMD
- **bus 1** → second channel (`can1`): Inverter 1 (isolated onto its own bus)

> The legacy ESP32-over-USB-serial bridge
> (`../firmware/EV4_CAN_Serial_Forwarder`) still exists for running the
> dashboard from a laptop, but this app now talks to the Pi's CAN HATs
> directly — no ESP32 needed.

## 1. Wire up the CAN HATs (one-time)

Each MCP2515 HAT needs an `mcp251x` device-tree overlay so the kernel exposes it
as a `canN` interface. Edit `/boot/firmware/config.txt` (older Pi OS:
`/boot/config.txt`) — the exact `oscillator` and `interrupt` GPIO values depend
on **your** HAT, so check its docs:

```
dtparam=spi=on
# HAT 1 -> can0 (CS0, INT on GPIO25, 8 MHz crystal)
dtoverlay=mcp2515-can0,oscillator=8000000,interrupt=25
# HAT 2 -> can1 (CS1, INT on GPIO23)
dtoverlay=mcp2515-can1,oscillator=8000000,interrupt=23
```

Reboot, then confirm both interfaces enumerated:

```sh
ip link show | grep can      # should list can0 and can1 (state DOWN)
dmesg | grep -i mcp251x      # confirms the chips were detected
```

## 2. Bring the buses up @ 500 kbps

```sh
cd dashboard
chmod +x setup_can.sh
sudo ./setup_can.sh          # can0 + can1 at 500000 bps
```

Re-run it after every reboot, or wire it into a systemd unit / `cron @reboot`
(see "Auto-start on boot" below) so the buses come up automatically.

## 3. Install deps and run

```sh
cd dashboard
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python3 ev4_dashboard.py     # auto-connects to can0, can1
```

The app auto-connects on launch. To pick interfaces or stop auto-connecting:

```sh
python3 ev4_dashboard.py --channels can0 can1 --bitrate 500000
python3 ev4_dashboard.py --no-connect          # click Connect in the UI
```

The connection bar lets you edit the two interface names (`bus0`/`bus1`) and the
bitrate (used only for the bus-load %) and Connect/Disconnect live.

### Auto-start on boot

Bring the CAN buses up automatically with a systemd unit
(`/etc/systemd/system/can-up.service`):

```ini
[Unit]
Description=Bring up EV4 CAN interfaces
After=network.target

[Service]
Type=oneshot
ExecStart=/home/pi/CAN_CAN_Hat/dashboard/setup_can.sh 500000
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

`sudo systemctl enable --now can-up.service`. (The Tkinter dashboard itself
needs a desktop session, so launch it from the Pi's desktop or autostart.)

### Try it with no hardware

```sh
python3 ev4_dashboard.py --demo
```

Generates plausible fake telemetry for **every** message on all three buses
(values sweep through each signal's range, with occasional fault blips), so all
tabs and panels populate. Handy for checking the layout without the car.
Demo values are synthetic and can read out-of-range — they only prove the UI
works, not real behavior.

## What you see

- **Top strip** – big readouts for SOC, power, pack voltage, max cell temp,
  APPS %, and torque command.
- **Tabs** – the panels are split into pages: **Vehicle**, **Inverter 1**,
  **Inverter 2**, and **🔍 Lookup**. The top strip and connection bar stay
  visible on every tab.
- **🔍 Lookup tab** – type part of a signal name, message, bus, description, **or
  CAN address** (e.g. `0xA5`, `165`) to instantly filter all signals. Each result
  shows its bus, message + CAN ID, **live value**, scale/offset/range, and the DBC
  description (including enum/state meanings). Multi-word search is AND
  (e.g. `inv2 temp`). Click the **📌** on any row to pin it to the Watch panel.
- **📌 Watch panel** (docked on the right, visible on every tab) – a live list of
  the signals you pinned. Remove one with `×`, **Clear** all, or **Save** the
  current set as a named group and **Load** it back later. Your pins
  auto-restore on the next launch. Groups are stored **per-user, outside the
  repo** at `%APPDATA%\EV4_CAN_Viewer\watch_groups.json` (Windows) /
  `~/.config/EV4_CAN_Viewer/watch_groups.json`, so updating or re-flashing the
  project never resets them. An older `dashboard/watch_groups.json` is migrated
  automatically on first run.
  - **Drag the divider** between the tabs and the Watch panel to resize it. The
    name↔value spacing grows with the panel, so widen it to read large values in
    full.
  - **Pop ↗** detaches the Watch list into a floating **always-on-top** window so
    it stays visible over other apps; click it again (or close the window) to
    re-dock.
  - **Right-click any watched signal** → **📈 Open chart** for a live rolling plot
    of that one signal, with now / min / max / Δ readouts. Charts are opt-in —
    nothing is plotted unless you ask for it — and each opens in its own window.
  - Pinned items are **grouped by bus** (Vehicle, Inverter 1, Inverter 2) with a
    header per group, regardless of the order you pinned them.
  - **Inverter parameters can be watched too:** in the Parameter Manager, select a
    parameter and click **📌 Watch**. While a parameter is pinned the dashboard
    auto-reads it (~3 Hz round-robin across all watched params) so its value stays
    live, decoded into engineering units like everything else.
  - **Right-click → ⚙ Send to Control** jumps to the Control tab with that
    parameter's inverter + address pre-loaded (and its last value prefilled) so you
    can Read/Write it immediately. (Plain broadcast signals are read-only, so it
    just tells you so.)
- **Panels** – one per CAN message, every signal decoded with its DBC units.
  Fault signals turn **red** when set, **green** when clear. A panel title goes
  **yellow** if that message stops arriving (>1 s stale), **grey** if never seen.
  **Right-click any signal** in a bus panel to **📌 Add to Watch** (or open a
  chart) — no need to go through the Lookup tab.
- **Top-right** – frames/sec, **estimated load per bus** (`b0 X%  b1 Y%`), count
  of unknown IDs (frames not in the DBC), and the **Log CSV** button. The load
  color follows the busiest bus: green (<50%) / yellow (<80%) / red (≥80%). It's estimated from the traffic the
  dashboard actually receives at 500 kbps (frame overhead + data, bit-stuffing
  not modelled), so it slightly under-reports and won't count frames the
  CAN driver/socket drops under heavy load — treat it as a close approximation,
  not a bus analyzer.
- **⚙ Control tab** – write to the Cascadia inverters (command + parameter
  messages) for motor calibration. See "Writing to the inverters" below.

## Writing to the inverters (Control tab)

> ⚠️ **This spins the motor.** Drive wheels off the ground, area clear, physical
> e-stop within reach. Demo mode cannot transmit — you must be connected to real
> CAN interfaces.

The Pi transmits directly on the right bus (`can0`/`can1`) for the selected
inverter. The transport handles three things:

- **Heartbeat** – the repeating inverter Command frame, re-sent on CAN every
  **20 ms**.
- **One-shot frames** – parameter reads/writes and fault clears, sent once.
- **Disable** – a final Command-disable frame on STOP/disconnect/close.

**Heartbeat + deadman safety:** the inverter faults if the Command message stops
for >1 s, so the dashboard's CAN transport (`CanReader`) auto-repeats it every
20 ms. The GUI refreshes that frame ~10 Hz; if the GUI hangs and stops refreshing
for >500 ms, the transport **drops** the heartbeat (deadman) and the inverter
disables the motor via its own CAN timeout. The **STOP** button, disconnecting,
and closing the app all send a Disable and halt the heartbeat.

**Command panel** (`0xC0` INV1 / `0xF0` INV2): ARM (starts the heartbeat with the
inverter disabled — this also releases the enable lockout), then ENABLE, set
direction and a capped torque command (type a value or use the slider), DISABLE,
or STOP.

### Spinning the motor with torque commands

1. **Put the inverter in CAN mode first.** In its default **VSM mode the inverter
   ignores all CAN command messages** — the motor won't move no matter what you
   send. Click **Enter CAN + Torque mode** (writes param 143=0 and 142=0), then
   **power-cycle the inverter**. Confirm the **Inverter status** panel shows
   *Command mode: CAN ✓* (it shows *VSM — CMDs IGNORED* in red otherwise).
2. HV bus connected/pre-charged (needed to actually make torque), faults clear
   (**Fault Clear** quick button), wheels off the ground.
3. Set the **Torque cap** to something gentle, pick **direction**.
4. **ARM** → **ENABLE** → raise the **torque** (slider/entry). The motor spins.
5. **STOP** (big red) disables and halts the heartbeat at any time.

The **Inverter status** panel (live) shows Command mode, Run mode, Enable state,
Lockout, Inverter state, and active run faults for the selected inverter — use it
to see exactly why the motor will or won't spin.

**Parameter panel** (`0xC1` INV1 / `0xF1` INV2): read/write any parameter by
address, plus quick buttons for the calibration parameters and Fault Clear.

**Parameter Manager window** (button on the Control tab): a searchable table of
all ~100 Cascadia command + EEPROM parameters from the CAN Protocol doc. Look a
parameter up by **name or address**, select it, and **Read** or **Write/Save**
it. You enter values in **engineering units** (°C, V, Nm, deg, rpm…) and the
tool applies the correct scale/offset on the wire (e.g. Gamma Adjust −4.3° is
sent as −43; over-temp 85 °C as 850). Read shows the inverter's decoded
response with `write_success`. EEPROM parameters are blocked from writing while
the inverter is enabled (they must be written motor-off).

## Motor calibration (Cascadia resolver / gamma)

Full procedure from Cascadia's *Resolver Calibration Process*. Do this once per
inverter before ever running the motor. **Gamma Adjust applies to all inverter
generations; Resolver PWM Delay only applies to PM Gen3** (skip it on Gen5/CM).

1. **Set up safely.** Wheels off the ground. Motor Type EEPROM already set for
   your motor. Connect the dashboard to the CAN buses and pick the inverter on the
   Control tab.
2. **Clear faults** with the Fault Clear quick button.
3. *(PM Gen3 only)* **Resolver PWM Delay:** with the motor still, watch
   `INV_*` resolver signals while writing addr **11** (try values around 1100)
   to maximize the cosine reading, then save to EEPROM addr **151**.
4. **Verify direction:** slowly hand-spin the motor forward and confirm
   `Motor Angle Electrical` increases and `Motor Speed` reads positive. If it
   decreases / goes negative, the resolver wiring is reversed.
5. **Gamma Adjust:** spin the motor to ~¼–⅓ of base speed (≈1000 rpm) using a
   **small torque command** in torque mode, then DISABLE so it coasts. While
   coasting (inverter disabled), read **Delta Resolver (deg)** on the Control
   tab. Goal: **+90° forward** (or −90° reverse), held steady within ±0.7°.
6. **Adjust:** write Gamma Adjust (addr **12**, degrees) to drive Delta Resolver
   toward 90°. *Increasing gamma decreases delta resolver.* Example: delta reads
   82.8°, you need +7.2°, current gamma is 2.9° → new gamma = 2.9 − 7.2 = −4.3°.
   Re-spin, re-read, repeat until delta = 90° ±0.7°.
7. **Save** the final gamma to EEPROM (addr **152**), power-cycle the inverter,
   and re-verify.

If the motor won't spin at any gamma value, the resolver direction doesn't match
the motor phase order — swap both SIN with both COS, or swap two motor phases.

## Logging to CSV

Click **Log CSV** to start recording; click **Stop Log** to finish (it also
stops automatically when you close the app). Files are written to
`dashboard/logs/ev4_log_<date>_<time>.csv`.

Each received frame writes one row:

| datetime | elapsed_s | trigger_msg | APPS_Pct | BMS_SOC | ... |
|----------|-----------|-------------|----------|---------|-----|
| ISO timestamp | seconds since log start | which message arrived | every signal in the DBC |

Every signal gets its own column (the latest value is repeated each row, so any
column is a complete time series). `trigger_msg` tells you which message caused
that row. A cell is blank until that signal has been seen at least once. Opens
directly in Excel, Google Sheets, MATLAB, or pandas (`pd.read_csv`).

## DBC files (multiple buses)

The dashboard decodes against **three** DBCs at once, listed in `DBC_SOURCES`
near the top of `ev4_dashboard.py`:

| File | Prefix | CAN IDs |
|------|--------|---------|
| `EV4_Vehicle_Bus.dbc` | *(none)* | 2–7 |
| `Inverter_1.dbc` | `INV1` | 160–514 |
| `Inverter_2.dbc` | `INV2` | 208–562 |

The two inverters reuse the **same message and signal names** (e.g. both have
`INV_Motor_Speed`), so each source gets a prefix. In the UI, inverter panels are
titled `INV1 M165_...` / `INV2 M165_...` and colored differently; in the CSV the
columns are `INV1_INV_Motor_Speed`, `INV2_INV_Motor_Speed`, etc. The vehicle bus
keeps its plain names. Frame IDs don't overlap between the three files.

To add another bus, drop the `.dbc` next to the script and add a
`("file.dbc", "PREFIX")` line to `DBC_SOURCES`.

### IMD (Bender iso175)

The vehicle-bus DBC fully decodes the iso175's `IMD_Info_General` message, which
this device is configured to send at the **29-bit extended** ID `0x18FF01F4`
(`IMD_Info`, 100 ms cyclic). Decoded per the iso175 CAN spec
(`Documents/iso175_CAN_D00415_N_XXEN.pdf`):

| Signal | Bytes | Meaning |
|--------|-------|---------|
| `IMD_R_iso` | 0–1 | Insulation resistance R_iso_corrected, **kΩ** (0–40500; 65535 = invalid) |
| `IMD_R_iso_Status` | 2 | Measurement status (254 = normal operation; 252/253 = startup; 255 = invalid) |
| `IMD_Meas_Counter` | 3 | Increments each new measurement |
| `IMD_Device_Error` … `IMD_Earthlift_Open` | 4–5 | 11 warning/alarm bits: device error, HV± / earth connection failure, iso alarm/warning, iso outdated, unbalance alarm, undervoltage alarm, unsafe-to-start, earthlift open |
| `IMD_Device_Activity` | 6 | 0 init / 1 normal operation / 2 self test |

The connection-failure and alarm bits color **red** when set (like other fault
signals). Status/activity enums carry value tables (visible in the 🔍 Lookup
tab). Extended (29-bit) IDs are handled automatically — SocketCAN flags them on
the frame and the dashboard decodes them the same as standard IDs.

> The device's other iso175 messages (`IMD_Info_IsolationDetail` 0x38,
> `IMD_Info_Voltage` 0x39 — HV bus & HV±-to-earth voltages, `IMD_Info_IT-System`
> 0x3A) are **deactivated by default**. Enable them on the iso175 (CAN `Set`
> command, index 0x78) and add matching `BO_`/`SG_` entries to the DBC if you
> want HV voltage / per-rail resistance on the dashboard.

## Keeping it in sync with the bus

The DBCs are the single source of truth. When the team edits a bus layout,
replace the corresponding `.dbc` here (masters live in the team's
`EV4_Software/CAN/` folder) and restart the app — panels rebuild automatically.
No code changes needed for new/changed signals.

> Note: the EV3 dash firmware referenced a message ID `0x8` (speed / TS voltage)
> that isn't in any of these DBCs, so it won't decode until it's added. Any
> unknown ID is counted (top-right) but otherwise ignored.
