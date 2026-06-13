# CAN_CAN_Hat

EV4 CAN dashboard that runs **directly on a Raspberry Pi** with two MCP2515 CAN
HATs. The Pi reads both vehicle CAN buses natively over SocketCAN (`can0` +
`can1`), decodes every frame against the bundled DBCs, shows a live Tkinter
dashboard, and can write to the Cascadia inverters for motor calibration.

See **[dashboard/README.md](dashboard/README.md)** for full setup (CAN HAT
overlays, bringing the buses up, running the app) and usage.

Quick start on the Pi:

```sh
cd dashboard
sudo ./setup_can.sh                 # bring up can0 + can1 @ 500 kbps
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
python3 ev4_dashboard.py            # auto-connects to can0, can1
# no hardware: python3 ev4_dashboard.py --demo
```

> The original ESP32 + MCP2515 USB-serial bridge
> (`firmware/EV4_CAN_Serial_Forwarder`) is kept as a legacy option for running
> the dashboard from a laptop, but the app now talks to the Pi's CAN HATs
> directly and no longer needs it.
