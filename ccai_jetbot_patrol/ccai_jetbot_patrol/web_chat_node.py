import json
import asyncio
import threading
from collections import deque
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import uvicorn
    from fastapi import FastAPI
    from fastapi.responses import HTMLResponse, Response
    from pydantic import BaseModel
except ImportError:  # pragma: no cover
    uvicorn = None
    FastAPI = None
    HTMLResponse = None
    Response = None
    BaseModel = object

from sensor_msgs.msg import CompressedImage


class ChatRequest(BaseModel):
    message: str


class WebChatNode(Node):
    def __init__(self) -> None:
        super().__init__("web_chat_node")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)
        self.declare_parameter("camera_topic", "/image_raw/compressed")
        self.admin_text_pub = self.create_publisher(String, "/ccai/admin_text", 10)
        self.create_subscription(String, "/ccai/status", self.on_status, 10)
        self.create_subscription(String, "/ccai/events", self.on_event, 10)
        self.create_subscription(String, "/ccai/llm_status", self.on_llm_status, 10)
        self.create_subscription(String, "/ccai/llm_response", self.on_llm_response, 10)
        self.create_subscription(String, "/ccai/vision_status", self.on_vision_status, 10)
        self.create_subscription(String, "/ccai/camera_status", self.on_camera_status, 10)
        self.create_subscription(CompressedImage, str(self.get_parameter("camera_topic").value), self.on_camera_frame, 2)
        self.messages = deque(maxlen=200)
        self.latest_status = "{}"
        self.latest_llm_status = "{}"
        self.latest_vision_status = "{}"
        self.latest_camera_status = "{}"
        self.latest_camera_frame = None
        self.app = self.build_app()
        self.start_server()
        self.get_logger().info("web_chat_node ready")

    def on_status(self, msg: String) -> None:
        self.latest_status = msg.data

    def on_event(self, msg: String) -> None:
        self.messages.append({"source": "robot", "message": msg.data})

    def on_llm_status(self, msg: String) -> None:
        self.latest_llm_status = msg.data

    def on_llm_response(self, msg: String) -> None:
        self.messages.append({"source": "llm", "message": msg.data})

    def on_vision_status(self, msg: String) -> None:
        self.latest_vision_status = msg.data

    def on_camera_frame(self, msg: CompressedImage) -> None:
        self.latest_camera_frame = bytes(msg.data)

    def on_camera_status(self, msg: String) -> None:
        self.latest_camera_status = msg.data

    def build_app(self):
        if FastAPI is None:
            self.get_logger().warning("fastapi/uvicorn unavailable; using stdlib HTTP server fallback")
            return None

        app = FastAPI(title="CCAI JetBot Patrol")

        @app.get("/", response_class=HTMLResponse)
        def index():
            return HTML_PAGE

        @app.get("/api/status")
        def status():
            try:
                status_payload = json.loads(self.latest_status)
            except json.JSONDecodeError:
                status_payload = {"raw": self.latest_status}
            try:
                llm_status_payload = json.loads(self.latest_llm_status)
            except json.JSONDecodeError:
                llm_status_payload = {"raw": self.latest_llm_status}
            try:
                vision_status_payload = json.loads(self.latest_vision_status)
            except json.JSONDecodeError:
                vision_status_payload = {"raw": self.latest_vision_status}
            try:
                camera_status_payload = json.loads(self.latest_camera_status)
            except json.JSONDecodeError:
                camera_status_payload = {"raw": self.latest_camera_status}
            return {
                "status": status_payload,
                "llm_status": llm_status_payload,
                "vision_status": vision_status_payload,
                "camera_status": camera_status_payload,
                "messages": list(self.messages),
            }

        @app.get("/api/camera.jpg")
        def camera_jpg():
            if self.latest_camera_frame is None:
                return Response(content=EMPTY_JPEG, media_type="image/jpeg")
            return Response(content=self.latest_camera_frame, media_type="image/jpeg")

        @app.post("/api/chat")
        def chat(req: ChatRequest):
            self.messages.append({"source": "admin", "message": req.message})
            self.admin_text_pub.publish(String(data=req.message))
            return {"accepted": True}

        return app

    def start_server(self) -> None:
        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)
        if self.app is None:
            self.start_stdlib_server(host, port)
            return

        def run() -> None:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            uvicorn.run(self.app, host=host, port=port, log_level="warning")

        threading.Thread(target=run, daemon=True).start()

    def start_stdlib_server(self, host: str, port: int) -> None:
        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):
                return

            def do_GET(self):
                if self.path == "/" or self.path.startswith("/?"):
                    self.send_bytes(HTML_PAGE.encode("utf-8"), "text/html; charset=utf-8")
                elif self.path.startswith("/api/status"):
                    self.send_json(node.status_payload())
                elif self.path.startswith("/api/camera.jpg"):
                    self.send_bytes(node.latest_camera_frame or EMPTY_JPEG, "image/jpeg")
                else:
                    self.send_error(404)

            def do_POST(self):
                if not self.path.startswith("/api/chat"):
                    self.send_error(404)
                    return
                length = int(self.headers.get("Content-Length", "0") or "0")
                body = self.rfile.read(length)
                try:
                    payload = json.loads(body.decode("utf-8"))
                    message = str(payload.get("message", ""))
                except Exception:
                    message = ""
                if message:
                    node.messages.append({"source": "admin", "message": message})
                    node.admin_text_pub.publish(String(data=message))
                self.send_json({"accepted": bool(message)})

            def send_json(self, payload):
                self.send_bytes(json.dumps(payload).encode("utf-8"), "application/json; charset=utf-8")

            def send_bytes(self, body: bytes, content_type: str):
                self.send_response(200)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

        class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
            daemon_threads = True

        def run() -> None:
            server = ThreadingHTTPServer((host, port), Handler)
            server.serve_forever()

        threading.Thread(target=run, daemon=True).start()

    def status_payload(self):
        return {
            "status": self.safe_json(self.latest_status),
            "llm_status": self.safe_json(self.latest_llm_status),
            "vision_status": self.safe_json(self.latest_vision_status),
            "camera_status": self.safe_json(self.latest_camera_status),
            "messages": list(self.messages),
        }

    def safe_json(self, text: str):
        try:
            return json.loads(text)
        except json.JSONDecodeError:
            return {"raw": text}


HTML_PAGE = """
<!doctype html>
<html lang="ko">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>CCAI JetBot Patrol</title>
  <style>
    body { margin: 0; font-family: system-ui, -apple-system, BlinkMacSystemFont, sans-serif; background: #f5f7fb; color: #17202a; }
    main { max-width: 960px; margin: 0 auto; padding: 24px; }
    header { display: flex; justify-content: space-between; gap: 16px; align-items: center; margin-bottom: 20px; }
    h1 { font-size: 24px; margin: 0; }
    #state { padding: 6px 10px; border-radius: 6px; background: #17202a; color: white; font-size: 14px; }
    #camera { width: 320px; max-width: 100%; height: auto; border: 1px solid #d7dee8; border-radius: 8px; background: #111; display: block; margin-bottom: 12px; }
    #log { height: 36vh; overflow: auto; background: white; border: 1px solid #d7dee8; border-radius: 8px; padding: 16px; }
    .msg { margin: 0 0 12px; line-height: 1.45; }
    .src { font-weight: 700; margin-right: 6px; }
    form { display: flex; gap: 8px; margin-top: 12px; }
    input { flex: 1; padding: 12px; border: 1px solid #c7d0dd; border-radius: 6px; font-size: 16px; }
    button { padding: 0 16px; border: 0; border-radius: 6px; background: #1463ff; color: white; font-weight: 700; }
  </style>
</head>
<body>
<main>
  <header><h1>CCAI JetBot Patrol</h1><span id="state">loading</span><span id="cameraState">camera</span><span id="vision">vision</span><span id="llm">llm</span></header>
  <img id="camera" src="/api/camera.jpg" alt="JetBot camera">
  <section id="log"></section>
  <form id="form"><input id="message" autocomplete="off" placeholder="status, patrol start, inspect entrance"><button>Send</button></form>
</main>
<script>
const log = document.getElementById('log');
const state = document.getElementById('state');
const llm = document.getElementById('llm');
const vision = document.getElementById('vision');
const cameraState = document.getElementById('cameraState');
const camera = document.getElementById('camera');
async function refresh() {
  const res = await fetch('/api/status');
  const data = await res.json();
  state.textContent = data.status.state || 'unknown';
  llm.textContent = data.llm_status && data.llm_status.connected ? 'LLM online' : 'LLM offline';
  vision.textContent = data.vision_status && data.vision_status.state ? data.vision_status.state : 'vision unknown';
  cameraState.textContent = data.camera_status && data.camera_status.backend ? `${data.camera_status.backend} ${data.camera_status.last_error || 'ok'}` : 'camera unknown';
  log.innerHTML = data.messages.map(m => `<p class="msg"><span class="src">${m.source}</span>${m.message}</p>`).join('');
  log.scrollTop = log.scrollHeight;
}
function refreshCamera() {
  camera.src = '/api/camera.jpg?t=' + Date.now();
}
document.getElementById('form').addEventListener('submit', async (event) => {
  event.preventDefault();
  const input = document.getElementById('message');
  await fetch('/api/chat', {method: 'POST', headers: {'Content-Type':'application/json'}, body: JSON.stringify({message: input.value})});
  input.value = '';
  refresh();
});
setInterval(refresh, 1000);
setInterval(refreshCamera, 150);
refresh();
refreshCamera();
</script>
</body>
</html>
"""


EMPTY_JPEG = (
    b"\xff\xd8\xff\xe0\x00\x10JFIF\x00\x01\x01\x01\x00H\x00H\x00\x00"
    b"\xff\xdb\x00C\x00\x08\x06\x06\x07\x06\x05\x08\x07\x07\x07\t\t"
    b"\x08\n\x0c\x14\r\x0c\x0b\x0b\x0c\x19\x12\x13\x0f\x14\x1d\x1a"
    b"\x1f\x1e\x1d\x1a\x1c\x1c $.' \",#\x1c\x1c(7),01444\x1f'9="
    b"82<.342\xff\xc0\x00\x0b\x08\x00\x01\x00\x01\x01\x01\x11\x00"
    b"\xff\xc4\x00\x14\x00\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    b"\x00\x00\x00\x00\x00\x00\x00\x08\xff\xc4\x00\x14\x10\x01\x00"
    b"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
    b"\x00\xff\xda\x00\x08\x01\x01\x00\x00?\x00\xd2\xcf \xff\xd9"
)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WebChatNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
