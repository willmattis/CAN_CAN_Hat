"""
Cascadia Motion (PM / RM / CM) inverter parameter catalog.

Transcribed from "CAN Protocol" Revision 5.9, sections 2.3.3 (Command
Parameters) and 2.3.4 (EEPROM Parameters).  Used by the dashboard's
Parameter Manager window to look parameters up by name or address, read
them, and write them over the Read/Write Parameter message (0x_C1).

Each parameter is read/written via the parameter message with its
*address* in bytes 0-1 and the value (16-bit) in bytes 4-5.  The value on
the wire is the engineering value times `scale`; `signed` says whether the
16-bit field is signed.

    raw_on_wire = round(engineering_value * scale)
    engineering_value = raw_on_wire / scale
"""

# format key -> (scale, signed, unit)   (CAN Protocol §1.5 Data Formats)
_FMT = {
    "temp":   (10,    True,  "C"),
    "lv":     (100,   True,  "V"),     # Low Voltage
    "torque": (10,    True,  "Nm"),
    "hv":     (10,    True,  "V"),     # High Voltage
    "current":(10,    True,  "A"),
    "angle":  (10,    True,  "deg"),
    "speed":  (1,     True,  "rpm"),   # Angular velocity
    "bool":   (1,     False, ""),
    "freq":   (10,    True,  "Hz"),
    "power":  (10,    True,  "kW"),
    "flux":   (1000,  True,  "Wb"),
    "pgain":  (10000, False, ""),      # Proportional gain ×10000
    "igain":  (10000, False, ""),      # Integral gain ×10000
    "dgain":  (100,   False, ""),      # Derivative gain ×100
    "lpgain": (10000, False, ""),      # Low-pass filter gain ×10000
    "cnt100": (100,   False, ""),      # Counts ×100
    "adc":    (1,     False, "cnt"),
    "uint":   (1,     False, ""),
}

# (address, name, fmt, category, note)
#   category: "CMD" = command parameter (0-99), "EEP" = EEPROM (100-499)
_RAW = [
    # --- Command parameters (§2.3.3) ---
    (1,  "Relay Command",            "uint", "CMD", "0xAA00=normal run; 0x55nn=external relay control"),
    (10, "Flux Command",             "flux", "CMD", "Modify the flux command"),
    (11, "Resolver PWM Delay (live)","uint", "CMD", "Live resolver A/D timing (PM Gen3). Default 1100, range 0-6250"),
    (12, "Gamma Adjust (live)",      "angle","CMD", "Live resolver angle offset used during calibration"),
    (20, "Fault Clear",              "bool", "CMD", "Write 0 to clear active faults"),
    (21, "Set PWM Frequency",        "uint", "CMD", "Gen5/CM override 6-24 kHz; reverts on power cycle"),
    (22, "AIN Pull-up Control",      "uint", "CMD", "CM only: bit0=AIN1, bit1=AIN2, bit2=AIN3"),
    (23, "Shudder Comp Gain Control","uint", "CMD", "0 disables; >0 sets Kp_Shudder/100 and enables"),
    (31, "Diag Data Trigger",        "uint", "CMD", "Nonzero triggers a diagnostic data dump (v651E+)"),

    # --- EEPROM: Motor configuration (§2.3.4.1) ---
    (150, "Motor Parameter Set",     "uint", "EEP", "Parameter set for the motor type"),
    (151, "Resolver PWM Delay",      "uint", "EEP", "Resolver A/D timing. Immediate. Range 0-6250"),
    (152, "Gamma Adjust",            "angle","EEP", "Resolver angle offset (calibration). Immediate"),
    (154, "Sin Offset",              "lv",   "EEP", "SIN/COS encoder offset"),
    (155, "Cos Offset",              "lv",   "EEP", "SIN/COS encoder offset"),
    (156, "Sin ADC Offset",          "adc",  "EEP", "ADC count offset (not normally used)"),
    (157, "Cos ADC Offset",          "adc",  "EEP", "ADC count offset (not normally used)"),

    # --- EEPROM: System configuration (§2.3.4.2) ---
    (140, "Pre-charge Bypassed",     "bool", "EEP", "1 = pre-charge bypassed"),
    (142, "Inverter Run Mode",       "bool", "EEP", "0 = Torque mode, 1 = Speed mode. Immediate"),
    (143, "Inverter Command Mode",   "bool", "EEP", "0 = CAN mode, 1 = VSM mode (default)"),
    (149, "Key Switch Mode",         "uint", "EEP", "0=on/off switch, 1=ignition START (VSM only)"),
    (170, "Relay Output State",      "uint", "EEP", "Relay normal function vs CAN control"),
    (173, "Discharge Enable",        "uint", "EEP", "See Inverter Discharge Process"),
    (204, "Analog Output Function",  "uint", "EEP", "Analog output select (Gen3 only)"),
    (174, "Serial Number",           "uint", "EEP", "Unit serial number (read-only)"),

    # --- EEPROM: Current (§2.3.4.3) ---
    (100, "Iq Limit",                "current","EEP", "Q-axis current limit"),
    (101, "Id Limit",                "current","EEP", "D-axis current limit"),
    (107, "Ia Offset",               "adc",  "EEP", "ADC offset, set 2048 (auto-calibrated)"),
    (108, "Ib Offset",               "adc",  "EEP", "ADC offset, set 2048 (auto-calibrated)"),
    (109, "Ic Offset",               "adc",  "EEP", "ADC offset, set 2048 (auto-calibrated)"),

    # --- EEPROM: Voltage & Flux (§2.3.4.4) ---
    (102, "DC Voltage Limit",        "hv",   "EEP", "Over-voltage protection limit"),
    (103, "DC Voltage Hysteresis",   "hv",   "EEP", "Over-voltage recovery hysteresis"),
    (104, "DC Under-voltage Limit",  "hv",   "EEP", "Under-voltage fault limit; 0 disables"),
    (106, "Vehicle Flux Command",    "flux", "EEP", "Back-EMF flux constant. Immediate"),

    # --- EEPROM: Temperature (§2.3.4.5) ---
    (112, "Inverter Over-Temperature","temp","EEP", "Inverter temp fault limit"),
    (113, "Motor Over-Temperature",  "temp", "EEP", "Motor temp fault limit"),
    (114, "Zero Torque Temperature", "temp", "EEP", "Temp where torque = 0"),
    (115, "Full Torque Temperature", "temp", "EEP", "Temp where full torque is available"),
    (203, "RTD Selection",           "bool", "EEP", "Gen3: bit0 RTD1, bit1 RTD2 (0=1k, 1=100ohm)"),

    # --- EEPROM: Accelerator pedal (§2.3.4.6) ---
    (120, "ACCEL Pedal Low",         "lv",   "EEP", "Below this = ACCEL SHORTED fault"),
    (121, "ACCEL Pedal Min",         "lv",   "EEP", "Pedal min (regen-limit region)"),
    (122, "ACCEL Coast Low",         "lv",   "EEP", "Coast low"),
    (123, "ACCEL Coast High",        "lv",   "EEP", "Coast high"),
    (124, "ACCEL Pedal Max",         "lv",   "EEP", "Pedal max (driving range)"),
    (125, "ACCEL Pedal High",        "lv",   "EEP", "Above this = ACCEL OPEN fault"),
    (132, "Accel Pedal Flipped",     "bool", "EEP", "0=increases, 1=decreases with press"),

    # --- EEPROM: Torque (§2.3.4.7) ---
    (129, "Motor Torque Limit",      "torque","EEP", "Upper motoring torque limit"),
    (130, "REGEN Torque Limit",      "torque","EEP", "Pedal-release regen torque limit"),
    (131, "Braking Torque Limit",    "torque","EEP", "Torque applied when brake active"),
    (164, "Kp Torque",               "pgain","EEP", "Torque regulator P gain (×10000)"),
    (165, "Ki Torque",               "igain","EEP", "Torque regulator I gain (×10000)"),
    (166, "Kd Torque",               "dgain","EEP", "Torque regulator D gain (×100)"),
    (167, "Klp Torque",              "lpgain","EEP","Torque regulator low-pass gain (×10000)"),
    (168, "Torque Rate Limit",       "torque","EEP", "Torque ramp limit, 0.1-250 Nm"),

    # --- EEPROM: Speed (§2.3.4.8) ---
    (111, "Motor Over-speed",        "speed","EEP", "Over-speed fault limit"),
    (128, "Max Speed",               "speed","EEP", "Above this, torque command -> 0"),
    (126, "REGEN Fade Speed",        "speed","EEP", "Speed where available regen reduces"),
    (127, "Break Speed",             "speed","EEP", "Field-weakening break speed"),
    (160, "Kp Speed",                "pgain","EEP", "Speed regulator P gain (×10000)"),
    (161, "Ki Speed",                "igain","EEP", "Speed regulator I gain (×10000)"),
    (162, "Kd Speed",                "dgain","EEP", "Speed regulator D gain (×100)"),
    (163, "Klp Speed",               "lpgain","EEP","Speed regulator low-pass gain (×10000)"),
    (169, "Speed Rate Limit",        "speed","EEP", "Speed ramp limit, 100-5100 rpm"),

    # --- EEPROM: Shudder compensation (§2.3.4.9) ---
    (187, "Shudder Comp Enable",     "bool", "EEP", "0 = off, 1 = on"),
    (188, "Kp Shudder",              "cnt100","EEP","Shudder gain (×100)"),
    (189, "TCLAMP Shudder",          "torque","EEP", "Max compensation torque"),
    (190, "Shudder Filter Frequency","freq", "EEP", "Shudder filter frequency"),
    (191, "Shudder Speed Fade",      "speed","EEP", "Speed where comp fades from 0"),
    (192, "Shudder Speed Low",       "speed","EEP", "Speed where comp begins to fade"),
    (193, "Shudder Speed High",      "speed","EEP", "Speed where comp = 0"),

    # --- EEPROM: Brake pedal (§2.3.4.10) ---
    (180, "Brake Mode",              "bool", "EEP", "0 = switch, 1 = pot"),
    (181, "Brake Low",               "lv",   "EEP", "Brake pot low (mode 1)"),
    (182, "Brake Min",               "lv",   "EEP", "Brake min (mode 1)"),
    (183, "Brake Max",               "lv",   "EEP", "Brake max (mode 1)"),
    (184, "Brake High",              "lv",   "EEP", "Brake high (mode 1)"),
    (185, "REGEN Ramp Period",       "uint", "EEP", "Regen ramp-down time, counts ×0.001 s"),
    (186, "Brake Pedal Flipped",     "bool", "EEP", "0 = 0V released, 1 = opposite"),
    (199, "Brake Input Bypassed",    "bool", "EEP", "1 = ignore brake input (VSM)"),

    # --- EEPROM: PWM (Gen5/CM, §2.3.4.1 cont.) ---
    (241, "PWM Frequency",               "uint",   "EEP", "Gen5/CM default PWM frequency"),
    (242, "PWM High Current Limit",      "current","EEP", "Gen5/CM"),
    (243, "PWM High Current Speed Limit","speed",  "EEP", "Gen5/CM"),
    (244, "High Current PWM Frequency",  "uint",   "EEP", "Gen5/CM"),
    (245, "PWM High Speed Limit",        "speed",  "EEP", "Gen5/CM"),
    (246, "High Speed PWM Frequency",    "uint",   "EEP", "Gen5/CM"),

    # --- EEPROM: CAN configuration ---
    (141, "CAN ID Offset",                "uint", "EEP", "Base CAN id offset (default 0xA0)"),
    (144, "CAN Extended Message Id",      "bool", "EEP", "0 = 11-bit, 1 = 29-bit"),
    (171, "CAN J1939 Option Active",      "bool", "EEP", "1 = SAE J1939 format"),
    (145, "CAN Term Resistor Present",    "bool", "EEP", "PM internal terminator"),
    (146, "CAN Command Message Active",   "bool", "EEP", "1 = enable CAN timeout / heartbeat fault"),
    (147, "CAN Bit Rate",                 "uint", "EEP", "125/250/500/1000 kbps (needs power cycle)"),
    (148, "CAN Active Messages Lo Word",  "uint", "EEP", "Broadcast-enable bitfield (low word)"),
    (237, "CAN Active Messages Hi Word",  "uint", "EEP", "Mailbox disable; default 0xFFFF"),
    (158, "CAN Diagnostic Data Tx Active","bool", "EEP", "Diagnostic broadcast"),
    (159, "CAN Inverter Enable Switch",   "bool", "EEP", "1 = require DIN1 to enable"),
    (172, "CAN Timeout",                  "uint", "EEP", "Timeout in counts of 3 ms (333 = 999 ms)"),
    (177, "CAN OBD2 Enable",              "bool", "EEP", "OBD2 support"),
    (178, "CAN BMS Limit Enable",         "bool", "EEP", "BMS CAN torque limiting"),
    (233, "CAN Slave Cmd ID",             "uint", "EEP", "Slave-mode command id (0 disables)"),
    (234, "CAN Slave Dir",                "uint", "EEP", "Slave direction (0 same, 1 opposite)"),
    (235, "CAN Fast Msg Rate",            "uint", "EEP", "Fast broadcast rate, ms (v2025+)"),
    (236, "CAN Slow Msg Rate",            "uint", "EEP", "Slow broadcast rate, ms (v2025+)"),
    (238, "CAN Debounce Counter Max",     "uint", "EEP", "Gen5/CM rolling counter (0 disables)"),
    (239, "CAN Debounce Up Count",        "uint", "EEP", "Gen5/CM"),
    (240, "CAN Debounce Down Count",      "uint", "EEP", "Gen5/CM"),
]


def _build():
    params = []
    for addr, name, fmt, cat, note in _RAW:
        scale, signed, unit = _FMT[fmt]
        params.append({
            "addr": addr, "name": name, "fmt": fmt, "cat": cat, "note": note,
            "scale": scale, "signed": signed, "unit": unit,
        })
    params.sort(key=lambda p: p["addr"])
    return params


PARAMS = _build()
BY_ADDR = {p["addr"]: p for p in PARAMS}


def to_wire(param, eng_value):
    """Engineering value -> 16-bit value to send (clamped to the field range)."""
    raw = int(round(float(eng_value) * param["scale"]))
    if param["signed"]:
        return max(-32768, min(32767, raw))
    return max(0, min(65535, raw))


def from_wire(param, raw):
    """16-bit value received -> engineering value."""
    v = raw / param["scale"]
    return int(v) if param["scale"] == 1 else round(v, 4)
