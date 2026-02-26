#!/usr/bin/env python3
"""MJPEG camera stream server.
Usage:
    python3 camera_stream_server.py                          # defaults: camera 0, port 8090
    python3 camera_stream_server.py --camera 1 --port 9000   # custom camera & port
    python3 camera_stream_server.py --width 640 --height 480 # custom resolution
"""

import argparse
import signal
import sys
import threading
import time
from http.server import HTTPServer, BaseHTTPRequestHandler

import cv2


class MJPEGStreamHandler(BaseHTTPRequestHandler):
    """Serves an MJPEG stream from a shared camera capture."""

    camera: cv2.VideoCapture = None   
    lock: threading.Lock = None       
    jpeg_quality: int = 80

    def do_GET(self):
        if self.path == "/video":
            self._stream_video()
        elif self.path == "/snapshot":
            self._serve_snapshot()
        elif self.path == "/health":
            self._serve_health()
        elif self.path == "/":
            self._serve_index()
        else:
            self.send_error(404)

    def _stream_video(self):
        """Continuous MJPEG stream."""
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()

        encode_params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]

        try:
            while True:
                with self.lock:
                    ret, frame = self.camera.read()
                if not ret:
                    time.sleep(0.01)
                    continue

                _, jpeg = cv2.imencode(".jpg", frame, encode_params)
                payload = jpeg.tobytes()

                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(payload)}\r\n\r\n".encode())
                self.wfile.write(payload)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass  # client disconnected

    def _serve_snapshot(self):
        """Single JPEG frame."""
        with self.lock:
            ret, frame = self.camera.read()
        if not ret:
            self.send_error(503, "Camera not available")
            return

        _, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        payload = jpeg.tobytes()
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def _serve_health(self):
        """Health check."""
        ok = self.camera.isOpened() if self.camera else False
        status = 200 if ok else 503
        body = f'{{"status": "{"ok" if ok else "error"}", "camera_open": {str(ok).lower()}}}'
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(body.encode())

    def _serve_index(self):
        """Landing page."""
        html = """<!DOCTYPE html>
<html><head><title>Camera Stream</title></head>
<body style="background:#111;color:#eee;font-family:monospace;text-align:center">
  <h2>ROB-GY 6213 — Camera Stream</h2>
  <img src="/video" style="max-width:90vw;border:2px solid #444;border-radius:8px">
  <p>Stream: <code>http://&lt;this-ip&gt;:{port}/video</code></p>
  <p>Snapshot: <a href="/snapshot" style="color:cyan">/snapshot</a>
   | Health: <a href="/health" style="color:cyan">/health</a></p>
</body></html>"""
        body = html.replace("{port}", str(self.server.server_address[1])).encode()
        self.send_response(200)
        self.send_header("Content-Type", "text/html")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format, *args):  # noqa: A002
        """Logging."""
        if "/video" not in str(args):
            super().log_message(format, *args)


def main():
    parser = argparse.ArgumentParser(description="MJPEG camera stream server")
    parser.add_argument("--camera", type=int, default=0, help="Camera device index (default: 0)")
    parser.add_argument("--port", type=int, default=8090, help="HTTP port (default: 8090)")
    parser.add_argument("--width", type=int, default=None, help="Frame width (optional)")
    parser.add_argument("--height", type=int, default=None, help="Frame height (optional)")
    parser.add_argument("--quality", type=int, default=80, help="JPEG quality 1-100 (default: 80)")
    parser.add_argument("--host", type=str, default="0.0.0.0", help="Bind address (default: 0.0.0.0)")
    args = parser.parse_args()

    # Open camera
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f"[ERROR] Cannot open camera {args.camera}")
        sys.exit(1)

    if args.width:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    if args.height:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)

    print(f"[INFO] Camera {args.camera}: {w}x{h} @ {fps:.0f}fps")
    print(f"[INFO] JPEG quality: {args.quality}")

    # Set up handler
    lock = threading.Lock()
    MJPEGStreamHandler.camera = cap
    MJPEGStreamHandler.lock = lock
    MJPEGStreamHandler.jpeg_quality = args.quality

    server = HTTPServer((args.host, args.port), MJPEGStreamHandler)
    print(f"[INFO] Streaming at http://{args.host}:{args.port}/video")
    print(f"[INFO] Press Ctrl+C to stop")

    def shutdown(sig, frame):
        print("\n[INFO] Shutting down...")
        cap.release()
        server.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    server.serve_forever()


if __name__ == "__main__":
    main()
