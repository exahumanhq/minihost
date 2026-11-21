"""Camera server: MJPG frames from a USB camera straight to the operator's
browser, with the Pi never decoding a pixel.

Why a separate process from host.py: a slow or stuck camera must never delay
a joint command. They share nothing; the operator opens the video in a
browser tab (or the operator UI fetches it) and the control link is UDP on
its own.

Why ffmpeg and not OpenCV: cv2.VideoCapture decodes every frame to BGR, and
then the old host re-encoded it to JPEG. On a Pi 4 that decode+encode is the
single biggest CPU cost in the whole system. ffmpeg with `-c:v copy` asks the
camera for MJPG and hands the compressed frames through untouched. ffmpeg is
already an apt dependency in Skyler's Pi install, so this adds nothing new.

Why HTTP multipart/x-mixed-replace: it is the oldest trick for live JPEG in a
browser (every IP webcam does it), needs no client code, and any <img> tag
shows it. For a far-away operator WebRTC would adapt bitrate and ride NAT;
that is the step after this one, and this version is what WebRTC would feed.

Nothing is captured until a client connects, and ffmpeg is stopped when the
last client leaves. An idle robot should cost nothing.

    python -m minihost.camera --dev /dev/video0 --fps 15 --port 8080
    then open http://<pi>:8080/ in a browser
"""
import argparse
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

SOI = b"\xff\xd8"   # JPEG start of image
EOI = b"\xff\xd9"   # JPEG end of image


class Camera:
    """Runs ffmpeg when someone is watching, splits its MJPEG byte stream into
    frames, and keeps the newest one plus a counter so clients can wait for a
    new frame instead of polling."""

    def __init__(self, dev, width=640, height=480, fps=15, test=False):
        if test:
            # synthetic source for development without a camera
            self.cmd = ["ffmpeg", "-hide_banner", "-loglevel", "error", "-re", "-f", "lavfi",   # -re: real time, like a camera
                        "-i", f"testsrc=size={width}x{height}:rate={fps}", "-c:v", "mjpeg", "-q:v", "6", "-f", "mjpeg", "-"]
        else:
            self.cmd = ["ffmpeg", "-hide_banner", "-loglevel", "error", "-f", "v4l2", "-input_format", "mjpeg",
                        "-video_size", f"{width}x{height}", "-framerate", str(fps), "-i", dev,
                        "-c:v", "copy", "-f", "mjpeg", "-"]
        self.frame = None
        self.n = 0
        self.cond = threading.Condition()
        self.clients = 0
        self.proc = None
        self.lock = threading.Lock()

    def _reader(self, proc):
        buf = b""
        while True:
            chunk = proc.stdout.read(65536)
            if not chunk:
                break
            buf += chunk
            # a frame is SOI ... EOI; there may be several or a partial one in buf
            while True:
                s = buf.find(SOI)
                if s < 0:
                    buf = b""
                    break
                e = buf.find(EOI, s + 2)
                if e < 0:
                    buf = buf[s:]
                    break
                with self.cond:
                    self.frame = buf[s:e + 2]
                    self.n += 1
                    self.cond.notify_all()
                buf = buf[e + 2:]

    def attach(self):
        with self.lock:
            self.clients += 1
            if self.proc is None:
                self.proc = subprocess.Popen(self.cmd, stdout=subprocess.PIPE)
                threading.Thread(target=self._reader, args=(self.proc,), daemon=True).start()
                print(f"camera: started ffmpeg for first client", flush=True)

    def detach(self):
        with self.lock:
            self.clients -= 1
            if self.clients == 0 and self.proc is not None:
                self.proc.terminate()
                self.proc = None
                print("camera: no clients, stopped ffmpeg", flush=True)

    def wait_frame(self, seen, timeout=2.0):
        """Block until a frame newer than `seen` exists; return (frame, n) or (None, seen)."""
        with self.cond:
            if self.n == seen:
                self.cond.wait(timeout)
            return (self.frame, self.n) if self.n != seen else (None, seen)


def make_handler(cam):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):   # the default logs every request; one line per client is enough
            pass

        def do_GET(self):
            if self.path != "/stream":
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.end_headers()
                self.wfile.write(b'<html><body style="margin:0;background:#000"><img src="/stream" style="width:100%"></body></html>')
                return
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            cam.attach()
            print(f"camera: client {self.client_address[0]} connected", flush=True)
            seen = 0
            try:
                while True:
                    frame, seen = cam.wait_frame(seen)
                    if frame is None:
                        continue
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(frame))
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                cam.detach()
    return H


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dev", default="/dev/video0")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=15)
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--test", action="store_true", help="ffmpeg test pattern instead of a camera")
    a = ap.parse_args()
    cam = Camera(a.dev, a.width, a.height, a.fps, test=a.test)
    srv = ThreadingHTTPServer(("0.0.0.0", a.port), make_handler(cam))
    print(f"camera: http://0.0.0.0:{a.port}/  ({'test pattern' if a.test else a.dev}, {a.width}x{a.height}@{a.fps})", flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
