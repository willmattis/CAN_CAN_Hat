# Raspberry Pi 4 B — EV4 Dashboard Setup

How the dashboard Pi is set up to run headless, reachable over a direct Ethernet
cable from a Windows PC. Commands are tagged **[PC]** (Windows PowerShell) or
**[Pi]** (bash, over SSH).

## Final state / key facts
- Pi static IP on the cable: **`192.168.50.2`** · PC side: **`192.168.50.1`**
- Connect with: **`ssh pitwall`** (key-based, no password)
- Dashboard virtualenv: **`~/CAN_CAN_Hat/dashboard/.venv`**
- Board is a **Pi 4 B**, OS is **Debian trixie**, Ethernet iface is `eth0`.

---

## 1. SD card / boot config
On the card's boot partition, `config.txt` carries the dual MCP2515 CAN HAT
overlays (oscillators differ per board):

```
dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25   # CS0, 12 MHz Waveshare -> can0
dtoverlay=mcp2515-can1,oscillator=16000000,interrupt=16   # CS1, 16 MHz board     -> can1
```

`user-data` sets hostname `pitwall1`, user `pitwall1` (groups `sudo,spi,gpio,
dialout,...`), SSH + password auth on. `network-config` brings up `eth0` + Wi-Fi.

## 2. Stable connection over the direct cable (static IPs both ends)
A direct PC↔Pi cable has no DHCP server; link-local auto-addressing is unstable
(NetworkManager keeps bouncing `eth0` and resetting SSH). Fix = static IPs.

**[PC]** elevated PowerShell:
```powershell
netsh interface ip set address name="Ethernet" static 192.168.50.1 255.255.255.0
```

**[Pi]** (eth0 connection = `Wired connection 1`,
UUID `4246dfca-5f69-3ad3-ac3b-62f18d9d8a69`):
```bash
sudo nmcli con mod "Wired connection 1" ipv4.method manual ipv4.addresses 192.168.50.2/24
sudo nmcli con up "Wired connection 1"
```

> ⚠️ Never set `eth0` to `ipv4.method auto` while no DHCP server is on the link —
> NM deactivates the interface and the Pi becomes unreachable. Recover by
> power-cycling and catching it on the boot window via its IPv6 link-local
> address (`fe80::…%<iface-index>`), then re-applying the static config.

## 3. SSH alias, key login, passwordless sudo
**[PC]** `C:\Users\<you>\.ssh\config`:
```
Host pitwall
    HostName 192.168.50.2
    User pitwall1
    StrictHostKeyChecking accept-new
    ServerAliveInterval 15
    ServerAliveCountMax 4
    TCPKeepAlive yes
```

**[PC]** generate a key and install it (one password entry):
```powershell
ssh-keygen -t ed25519 -C "pitwall" -f "$env:USERPROFILE\.ssh\id_ed25519" --% -N ""
Get-Content "$env:USERPROFILE\.ssh\id_ed25519.pub" | ssh pitwall "umask 077; mkdir -p ~/.ssh && cat >> ~/.ssh/authorized_keys"
```

**[Pi]** passwordless sudo (so setup commands don't each prompt):
```bash
echo "pitwall1 ALL=(ALL) NOPASSWD:ALL" | sudo tee /etc/sudoers.d/010-pitwall1-nopasswd
```

## 4. Get the repo onto the Pi
**[PC]** copy directly from the PC (no GitHub needed):
```powershell
scp -r "C:\Users\<you>\Documents\GitHub\CAN_CAN_Hat" pitwall:~/
```
**[Pi]** normalize the shell script's line endings (it came from Windows):
```bash
cd ~/CAN_CAN_Hat/dashboard
tr -d '\r' < setup_can.sh > setup_can.sh.tmp && mv setup_can.sh.tmp setup_can.sh && chmod +x setup_can.sh
```

## 5. Give the headless Pi internet over the cable (PC proxy)
No Wi-Fi/hotspot is available, and Windows **ICS** and **`New-NetNat`** both fail
on Win11 Home + enterprise Wi-Fi. Working approach: a tiny HTTP/CONNECT proxy on
the PC; the Pi reaches it over the cable (no NAT/routing needed).

**[PC]** save `piproxy.py` (see `docs/piproxy.py`) and run it:
```powershell
python piproxy.py        # listens on 0.0.0.0:8899
```
The Pi then uses `http://192.168.50.1:8899` as its proxy for `apt` and `pip`.

## 6. Install dependencies
**[Pi]** fix the clock first — a headless Pi with no NTP has a past clock, which
makes `apt` reject "future-dated" repo signatures and can break TLS:
```bash
sudo date -u -s "<current UTC time>"
```
**[Pi]** install Tk (apt) + the Python packages (venv + pip), both through the proxy:
```bash
sudo env http_proxy=http://192.168.50.1:8899 https_proxy=http://192.168.50.1:8899 \
  apt-get install -y python3-tk

cd ~/CAN_CAN_Hat/dashboard
python3 -m venv .venv
.venv/bin/pip install --proxy http://192.168.50.1:8899 -r requirements.txt
```
Installs `cantools` + `python-can` into the venv; `tkinter` comes from `python3-tk`.

> The "externally-managed-environment" error comes from the **bare** `pip`. Always
> use the venv: `source .venv/bin/activate` first, or call `.venv/bin/python` /
> `.venv/bin/pip` directly.

## 7. View + run the dashboard
The dashboard is a GUI, so it needs the Pi's desktop. Raspberry Pi OS (trixie)
uses **wayvnc** — **RealVNC Viewer won't connect; use TigerVNC Viewer.**

**[Pi]** start a LAN VNC server on the running desktop:
```bash
XDG_RUNTIME_DIR=/run/user/1000 WAYLAND_DISPLAY=wayland-0 \
  nohup wayvnc 0.0.0.0 5900 >/tmp/wayvnc.log 2>&1 &
```
**[PC]** connect TigerVNC Viewer to `192.168.50.2`, open a terminal there, then:
```bash
cd ~/CAN_CAN_Hat/dashboard
.venv/bin/python ev4_dashboard.py
```
(`wayvnc` started this way is not persistent — it stops on reboot.)

## Reconnecting later (cheat sheet)
1. Plug in the cable, power the Pi.
2. **[PC]** `ssh pitwall`
3. To run the GUI: start wayvnc (step 7), connect TigerVNC to `192.168.50.2`,
   run `.venv/bin/python ev4_dashboard.py` from a desktop terminal.
4. For internet (apt/pip): run `python piproxy.py` on the PC and use the proxy.

---

## ⚠️ Outstanding — the CAN HATs
The Pi **hangs / drops off when the HATs are attached** (onboard green LED dies,
Ethernet link stays up); it is rock-solid bare. Power tested clean
(`vcgencmd get_throttled` = `0x0`) on the 5 V/3 A brick (the Pi 4 B's spec
rating), so the suspects are a **transient inrush brownout** or the **marginal
16 MHz `can1` board / its bus termination**.

To resolve:
- Use a higher-quality / known-good 5 V supply and a good USB-C cable.
- Reattach the HATs **one at a time** (start with the 12 MHz `can0` board) while
  watching `dmesg` and `vcgencmd get_throttled`.
- Once both come up: `sudo ~/CAN_CAN_Hat/dashboard/setup_can.sh 500000`, verify
  with `candump can0` / `candump can1`.

Until then, run the dashboard with the HATs off (it shows "no data").
