#!/usr/bin/env python3
"""把标定 yaml 写成文件，并按 GNOME 的文件复制放进剪贴板。"""

import argparse
import ctypes
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
X11 = ctypes.CDLL("libX11.so.6")
CurrentTime = 0
PropModeReplace = 0
SelectionClear = 29
SelectionRequest = 30
SelectionNotify = 31


class XSelectionRequestEvent(ctypes.Structure):
    _fields_ = [
        ("type", ctypes.c_int),
        ("serial", ctypes.c_ulong),
        ("send_event", ctypes.c_int),
        ("display", ctypes.c_void_p),
        ("owner", ctypes.c_ulong),
        ("requestor", ctypes.c_ulong),
        ("selection", ctypes.c_ulong),
        ("target", ctypes.c_ulong),
        ("property", ctypes.c_ulong),
        ("time", ctypes.c_ulong),
    ]


class XSelectionEvent(ctypes.Structure):
    _fields_ = [
        ("type", ctypes.c_int),
        ("serial", ctypes.c_ulong),
        ("send_event", ctypes.c_int),
        ("display", ctypes.c_void_p),
        ("requestor", ctypes.c_ulong),
        ("selection", ctypes.c_ulong),
        ("target", ctypes.c_ulong),
        ("property", ctypes.c_ulong),
        ("time", ctypes.c_ulong),
    ]


class XEvent(ctypes.Union):
    _fields_ = [
        ("type", ctypes.c_int),
        ("xselectionrequest", XSelectionRequestEvent),
        ("xselection", XSelectionEvent),
        ("pad", ctypes.c_char * 192),
    ]


X11.XInitThreads.restype = ctypes.c_int
X11.XOpenDisplay.restype = ctypes.c_void_p
X11.XOpenDisplay.argtypes = [ctypes.c_char_p]
X11.XDefaultRootWindow.restype = ctypes.c_ulong
X11.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
X11.XCreateSimpleWindow.restype = ctypes.c_ulong
X11.XCreateSimpleWindow.argtypes = [
    ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_int,
    ctypes.c_uint, ctypes.c_uint, ctypes.c_uint, ctypes.c_ulong, ctypes.c_ulong,
]
X11.XInternAtom.restype = ctypes.c_ulong
X11.XInternAtom.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
X11.XSetSelectionOwner.restype = None
X11.XSetSelectionOwner.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_ulong, ctypes.c_ulong]
X11.XGetSelectionOwner.restype = ctypes.c_ulong
X11.XGetSelectionOwner.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
X11.XNextEvent.restype = ctypes.c_int
X11.XNextEvent.argtypes = [ctypes.c_void_p, ctypes.POINTER(XEvent)]
X11.XChangeProperty.restype = None
X11.XChangeProperty.argtypes = [
    ctypes.c_void_p, ctypes.c_ulong, ctypes.c_ulong, ctypes.c_ulong,
    ctypes.c_int, ctypes.c_int, ctypes.c_void_p, ctypes.c_int,
]
X11.XSendEvent.restype = ctypes.c_int
X11.XSendEvent.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_long, ctypes.POINTER(XEvent)]
X11.XFlush.restype = None
X11.XFlush.argtypes = [ctypes.c_void_p]


class FileClipboard:
    def __init__(self):
        X11.XInitThreads()
        self.display = X11.XOpenDisplay(None)
        if not self.display:
            raise SystemExit("打不开 X 显示，无法把文件放进剪贴板")
        root = X11.XDefaultRootWindow(self.display)
        self.window = X11.XCreateSimpleWindow(self.display, root, 0, 0, 1, 1, 0, 0, 0)
        self.clipboard = self.atom("CLIPBOARD")
        self.targets = self.atom("TARGETS")
        self.gnome = self.atom("x-special/gnome-copied-files")
        self.uri_list = self.atom("text/uri-list")
        self.utf8 = self.atom("UTF8_STRING")
        self.atom_type = self.atom("ATOM")
        self.lock = threading.Lock()
        self.gnome_bytes = b""
        self.uri_bytes = b""
        self.text_bytes = b""
        threading.Thread(target=self.serve, daemon=True).start()

    def atom(self, name):
        return X11.XInternAtom(self.display, name.encode(), 0)

    def offer(self, path: Path):
        uri = path.resolve().as_uri()
        with self.lock:
            self.gnome_bytes = ("copy\n" + uri).encode()
            self.uri_bytes = (uri + "\n").encode()
            self.text_bytes = uri.encode()
            X11.XSetSelectionOwner(self.display, self.clipboard, self.window, CurrentTime)
            X11.XFlush(self.display)
            owned = X11.XGetSelectionOwner(self.display, self.clipboard) == self.window
        if not owned:
            raise SystemExit("剪贴板没有接受这次文件复制")

    def serve(self):
        event = XEvent()
        while True:
            X11.XNextEvent(self.display, ctypes.byref(event))
            if event.type != SelectionRequest:
                continue
            request = event.xselectionrequest
            reply = XEvent()
            reply.xselection.type = SelectionNotify
            reply.xselection.display = request.display
            reply.xselection.requestor = request.requestor
            reply.xselection.selection = request.selection
            reply.xselection.target = request.target
            reply.xselection.property = request.property
            reply.xselection.time = request.time
            with self.lock:
                self.fill(request, reply)
            X11.XSendEvent(self.display, request.requestor, 0, 0, ctypes.byref(reply))
            X11.XFlush(self.display)

    def fill(self, request, reply):
        target = request.target
        if request.property == 0:
            reply.xselection.property = 0
            return
        if target == self.targets:
            values = (ctypes.c_ulong * 4)(self.targets, self.gnome, self.uri_list, self.utf8)
            X11.XChangeProperty(
                self.display, request.requestor, request.property, self.atom_type,
                32, PropModeReplace, ctypes.cast(values, ctypes.c_void_p), 4,
            )
        elif target == self.gnome:
            self.put_bytes(request, self.gnome, self.gnome_bytes)
        elif target == self.uri_list:
            self.put_bytes(request, self.uri_list, self.uri_bytes)
        elif target == self.utf8:
            self.put_bytes(request, self.utf8, self.text_bytes)
        else:
            reply.xselection.property = 0

    def put_bytes(self, request, atom, data):
        buffer = ctypes.create_string_buffer(data)
        X11.XChangeProperty(
            self.display, request.requestor, request.property, atom,
            8, PropModeReplace, ctypes.cast(buffer, ctypes.c_void_p), len(data),
        )


def safe_name(name):
    base = Path(name).name
    if not base.endswith(".yaml") or base.startswith("."):
        raise ValueError("文件名必须是 yaml")
    if not all(char.isalnum() or char in "._-" for char in base):
        raise ValueError("文件名含有不能使用的字符")
    return base


class Handler(BaseHTTPRequestHandler):
    directory = Path(".")
    clipboard = None

    def log_message(self, fmt, *args):
        return

    def header(self, status, body=b""):
        self.send_response(status)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        if body:
            self.wfile.write(body)

    def do_OPTIONS(self):
        self.header(204)

    def do_POST(self):
        if self.path != "/copy":
            self.header(404, b'{"error":"unknown"}')
            return
        length = int(self.headers.get("Content-Length", "0"))
        try:
            payload = json.loads(self.rfile.read(length).decode("utf-8"))
            name = safe_name(payload.get("filename", ""))
            text = payload.get("text", "")
            if not isinstance(text, str) or not text.strip():
                raise ValueError("没有标定文件内容")
            path = self.directory / name
            path.write_text(text if text.endswith("\n") else text + "\n", encoding="utf-8")
            self.clipboard.offer(path)
        except (ValueError, OSError, SystemExit, json.JSONDecodeError) as exc:
            body = json.dumps({"error": str(exc)}, ensure_ascii=False).encode()
            self.header(400, body)
            return
        body = json.dumps({"path": str(path)}, ensure_ascii=False).encode()
        self.header(200, body)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dir", required=True)
    parser.add_argument("--port", type=int, default=8767)
    args = parser.parse_args()
    directory = Path(args.dir).resolve()
    directory.mkdir(parents=True, exist_ok=True)
    Handler.directory = directory
    Handler.clipboard = FileClipboard()
    server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    print(f"标定文件剪贴板 {args.port}，目录 {directory}", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
