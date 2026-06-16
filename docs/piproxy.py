#!/usr/bin/env python3
"""Minimal forward proxy: relays plain HTTP and tunnels HTTPS (CONNECT).
Lets the Pi reach the internet through this PC's connection over the cable.
Run: python piproxy.py   (listens on 0.0.0.0:8899)
"""
import socket, threading, select
from urllib.parse import urlsplit

LISTEN = ("0.0.0.0", 8899)


def pipe(a, b):
    try:
        while True:
            r, _, _ = select.select([a, b], [], [], 120)
            if not r:
                break
            for s in r:
                data = s.recv(65536)
                if not data:
                    return
                (b if s is a else a).sendall(data)
    finally:
        for s in (a, b):
            try:
                s.close()
            except Exception:
                pass


def handle(client):
    upstream = None
    try:
        client.settimeout(30)
        req = b""
        while b"\r\n\r\n" not in req:
            chunk = client.recv(4096)
            if not chunk:
                client.close()
                return
            req += chunk
        head_end = req.index(b"\r\n\r\n") + 4
        lines = req[:head_end].decode("latin1").split("\r\n")
        method, target, ver = lines[0].split(" ", 2)
        if method.upper() == "CONNECT":
            host, _, port = target.partition(":")
            upstream = socket.create_connection((host, int(port or 443)), timeout=30)
            client.sendall(b"HTTP/1.1 200 Connection established\r\n\r\n")
            client.settimeout(None)
            pipe(client, upstream)
        else:
            u = urlsplit(target)
            host = u.hostname
            port = u.port or 80
            path = u.path or "/"
            if u.query:
                path += "?" + u.query
            upstream = socket.create_connection((host, port), timeout=30)
            out = method + " " + path + " " + ver + "\r\n"
            for l in lines[1:]:
                low = l.lower()
                if low.startswith("proxy-connection") or low.startswith("connection"):
                    continue
                if l:
                    out += l + "\r\n"
            out += "Connection: close\r\n\r\n"
            upstream.sendall(out.encode("latin1") + req[head_end:])
            client.settimeout(None)
            pipe(client, upstream)
    except Exception:
        try:
            client.close()
        except Exception:
            pass
        if upstream:
            try:
                upstream.close()
            except Exception:
                pass


def main():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(LISTEN)
    srv.listen(64)
    print("proxy listening on %s:%d" % LISTEN, flush=True)
    while True:
        c, _ = srv.accept()
        threading.Thread(target=handle, args=(c,), daemon=True).start()


if __name__ == "__main__":
    main()
