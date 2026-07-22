#!/usr/bin/env python3
import argparse
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

import cv2


def build_pipeline(args):
    return (
        "nvarguscamerasrc sensor-mode={sensor_mode} ! "
        "video/x-raw(memory:NVMM), width={capture_width}, height={capture_height}, "
        "format=(string)NV12, framerate=(fraction){fps}/1 ! "
        "nvvidconv flip-method={flip_method} ! "
        "video/x-raw, width=(int){width}, height=(int){height}, format=(string)BGRx ! "
        "videoconvert ! video/x-raw, format=(string)BGR ! appsink drop=true max-buffers=1 sync=false"
    ).format(
        sensor_mode=args.sensor_mode,
        capture_width=args.capture_width,
        capture_height=args.capture_height,
        fps=args.fps,
        flip_method=args.flip_method,
        width=args.width,
        height=args.height,
    )


class CameraWorker:
    def __init__(self, args, pipeline, jpeg_quality):
        self.args = args
        self.pipeline = pipeline
        self.jpeg_quality = jpeg_quality
        self.lock = threading.Lock()
        self.latest_jpeg = None
        self.last_error = ""
        self.running = True

    def start(self):
        thread = threading.Thread(target=self.run, daemon=True)
        thread.start()

    def run(self):
        if self.args.backend in {"auto", "jetbot"}:
            try:
                self.run_jetbot()
                return
            except Exception as exc:
                self.last_error = (
                    "jetbot backend failed: {0}; set JETBOT_REPO_PATH=/path/to/jetbot "
                    "or clone http://github.com/NVIDIA-AI-IOT/jetbot.git"
                ).format(exc)
                print(self.last_error, flush=True)
                print("python_path=" + ":".join(sys.path), flush=True)
                print("falling back to OpenCV CSI pipeline", flush=True)
        self.run_opencv()

    def run_jetbot(self):
        from jetbot import Camera

        camera = Camera.instance(width=self.args.width, height=self.args.height)
        print("backend=jetbot", flush=True)
        while self.running:
            frame = getattr(camera, "value", None)
            if frame is None:
                self.last_error = "jetbot no frame"
                time.sleep(0.1)
                continue
            ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
            if ok:
                self.last_error = ""
                with self.lock:
                    self.latest_jpeg = encoded.tobytes()
            time.sleep(1.0 / max(float(self.args.fps), 1.0))

    def run_opencv(self):
        print("backend=opencv", flush=True)
        while self.running:
            cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)
            if not cap.isOpened():
                self.last_error = "opencv open failed"
                print(self.last_error, flush=True)
                time.sleep(2.0)
                continue
            self.last_error = ""
            while self.running:
                ok, frame = cap.read()
                if not ok or frame is None:
                    self.last_error = "opencv read failed"
                    print(self.last_error, flush=True)
                    break
                ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
                if ok:
                    with self.lock:
                        self.latest_jpeg = encoded.tobytes()
            cap.release()
            time.sleep(1.0)

    def snapshot(self):
        with self.lock:
            return self.latest_jpeg


class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


def make_handler(worker):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            return

        def do_GET(self):
            if self.path.startswith("/health"):
                body = ("ok" if worker.snapshot() else worker.last_error or "no frame").encode("utf-8")
                self.send_response(200 if worker.snapshot() else 503)
                self.send_header("Content-Type", "text/plain")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if self.path.startswith("/snapshot.jpg"):
                frame = worker.snapshot()
                if frame is None:
                    self.send_error(503, worker.last_error or "no frame")
                    return
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(frame)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(frame)
                return
            if self.path.startswith("/stream.mjpg"):
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                while True:
                    frame = worker.snapshot()
                    if frame is not None:
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(("Content-Length: %d\r\n\r\n" % len(frame)).encode("ascii"))
                        self.wfile.write(frame)
                        self.wfile.write(b"\r\n")
                    time.sleep(0.1)
                return
            self.send_error(404)

    return Handler


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--sensor-mode", type=int, default=3)
    parser.add_argument("--capture-width", type=int, default=816)
    parser.add_argument("--capture-height", type=int, default=616)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--width", type=int, default=320)
    parser.add_argument("--height", type=int, default=240)
    parser.add_argument("--flip-method", type=int, default=0)
    parser.add_argument("--jpeg-quality", type=int, default=45)
    parser.add_argument("--backend", choices=["auto", "jetbot", "opencv"], default="auto")
    args = parser.parse_args()

    pipeline = build_pipeline(args)
    print("pipeline=" + pipeline, flush=True)
    worker = CameraWorker(args, pipeline, args.jpeg_quality)
    worker.start()
    server = ThreadingHTTPServer((args.host, args.port), make_handler(worker))
    print("serving=http://{0}:{1}/stream.mjpg".format(args.host, args.port), flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
