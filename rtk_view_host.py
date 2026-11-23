#!/usr/bin/env python3
"""本机控制播放容器。网页按当前输入的路径把对应目录挂进容器并重启。"""

import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

ALLOWED = []
STATE = {
    "image": "ros2-jazzy:dev",
    "container": "lidar-rtk-view",
    "port": 8768,
    "calib": "",
    "mcap": "",
    "yaml": "",
    "map": "",
    "roots": [],
    "lock": threading.Lock(),
}


def run(command):
    return subprocess.run(command, check=False, text=True, capture_output=True)


def real_dir(path: Path):
    if path.is_dir():
        return path.resolve()
    return path.resolve().parent


def allowed(path: Path):
    resolved = path.resolve()
    return any(resolved == root or root in resolved.parents for root in ALLOWED)


def add_root(roots, path: Path):
    if not path.is_dir():
        return False
    root = path.resolve()
    if not allowed(root):
        raise SystemExit(f"不允许挂载: {root}")
    for existing in roots:
        if root == existing or existing in root.parents:
            return False
    roots[:] = [item for item in roots if root not in item.parents]
    roots.append(root)
    return True


def wait_port(port, timeout):
    import socket

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            socket.create_connection(("127.0.0.1", port), 0.5).close()
            return True
        except OSError:
            time.sleep(0.25)
    return False


def start_container():
    roots = STATE["roots"]
    data_roots = ":".join(str(root) for root in roots)
    command = [
        "docker", "run", "-d", "--rm",
        "--name", STATE["container"],
        "--init",
        "--ipc=host",
        "-p", f"{STATE['port']}:{STATE['port']}",
        "-e", f"PORT={STATE['port']}",
        "-e", f"SUGGEST_MCAP={STATE['mcap']}",
        "-e", f"SUGGEST_YAML={STATE['yaml']}",
        "-e", f"SUGGEST_MAP={STATE['map']}",
        "-e", f"DATA_ROOTS={data_roots}",
        "-e", f"CONTROL_URL=http://127.0.0.1:{STATE['control_port']}",
        "-e", "PYTHONPATH=/calib:/opt/calib/lib/python3.12/site-packages",
        "-e", "PYTHONDONTWRITEBYTECODE=1",
        "-v", f"{STATE['calib']}:/calib",
    ]
    for root in roots:
        command.extend(["-v", f"{root}:{root}:ro"])
    command.extend([
        STATE["image"],
        "bash", "-lc",
        "source /opt/ros/jazzy/setup.bash && "
        "python3 -c 'import flask' >/dev/null 2>&1 || "
        "pip3 install --break-system-packages -q flask && "
        "exec python3 -u /calib/web_calib/rtk_view.py",
    ])
    result = run(command)
    if result.returncode != 0:
        raise SystemExit(result.stderr.strip() or "容器没有启动")
    if not wait_port(STATE["port"], 60):
        raise SystemExit("网页没有起来")
    print(f"播放容器已启动，挂载 {len(roots)} 个目录", flush=True)


def restart(paths, mcap, yaml_path, map_path):
    with STATE["lock"]:
        added = []
        for raw in paths:
            text = str(raw or "").strip()
            if not text:
                continue
            path = Path(text)
            if not path.exists():
                raise SystemExit(f"找不到路径: {text}")
            if not allowed(path):
                raise SystemExit(f"路径不在允许范围内: {text}")
            directory = real_dir(path)
            if add_root(STATE["roots"], directory):
                added.append(str(directory))
        STATE["mcap"] = mcap
        STATE["yaml"] = yaml_path
        STATE["map"] = map_path
        if not added:
            return {"ok": True, "restarted": False, "roots": [str(item) for item in STATE["roots"]]}
        print("重新挂载: " + ", ".join(added), flush=True)
        run(["docker", "rm", "-f", STATE["container"]])
        start_container()
        return {
            "ok": True,
            "restarted": True,
            "added": added,
            "roots": [str(item) for item in STATE["roots"]],
        }


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        return

    def send_json(self, code, payload):
        body = json.dumps(payload, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Access-Control-Allow-Methods", "POST, OPTIONS")
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Access-Control-Allow-Methods", "POST, OPTIONS")
        self.end_headers()

    def do_POST(self):
        if self.path.split("?", 1)[0] != "/remount":
            self.send_json(404, {"error": "没有这个接口"})
            return
        length = int(self.headers.get("Content-Length", "0"))
        try:
            payload = json.loads(self.rfile.read(length) or b"{}")
            result = restart(
                payload.get("paths", []),
                str(payload.get("mcap", "") or ""),
                str(payload.get("yaml", "") or ""),
                str(payload.get("map", "") or ""),
            )
        except SystemExit as exc:
            self.send_json(400, {"error": str(exc)})
            return
        except Exception as exc:
            self.send_json(500, {"error": str(exc)})
            return
        self.send_json(200, result)


def stop_container(*_args):
    run(["docker", "rm", "-f", STATE["container"]])
    sys.exit(0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", default="ros2-jazzy:dev")
    parser.add_argument("--container", default="lidar-rtk-view")
    parser.add_argument("--port", type=int, default=8768)
    parser.add_argument("--control-port", type=int, default=8769)
    parser.add_argument("--calib", required=True)
    parser.add_argument("--mcap", default="")
    parser.add_argument("--yaml", default="")
    parser.add_argument("--map", default="")
    parser.add_argument("--root", action="append", default=[])
    parser.add_argument("--allow", action="append", default=[])
    args = parser.parse_args()
    ALLOWED.extend(Path(item).resolve() for item in args.allow if Path(item).is_dir())
    if not ALLOWED:
        raise SystemExit("没有允许挂载的范围")
    STATE.update({
        "image": args.image,
        "container": args.container,
        "port": args.port,
        "control_port": args.control_port,
        "calib": str(Path(args.calib).resolve()),
        "mcap": args.mcap,
        "yaml": args.yaml,
        "map": args.map,
        "roots": [],
    })
    for raw in args.root:
        path = Path(raw)
        if path.is_dir():
            add_root(STATE["roots"], path.resolve())
    for raw in (args.mcap, args.yaml, args.map):
        if raw and Path(raw).exists():
            add_root(STATE["roots"], real_dir(Path(raw)))
    signal.signal(signal.SIGINT, stop_container)
    signal.signal(signal.SIGTERM, stop_container)
    run(["docker", "rm", "-f", STATE["container"]])
    start_container()
    server = ThreadingHTTPServer(("127.0.0.1", args.control_port), Handler)
    print(f"挂载控制: http://127.0.0.1:{args.control_port}/remount", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
