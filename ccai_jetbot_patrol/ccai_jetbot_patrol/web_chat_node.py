import json
import threading
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import uvicorn
    from fastapi import FastAPI
    from fastapi.responses import HTMLResponse
    from pydantic import BaseModel
except ImportError:  # pragma: no cover
    uvicorn = None
    FastAPI = None
    HTMLResponse = None
    BaseModel = object


class ChatRequest(BaseModel):
    message: str


class WebChatNode(Node):
    def __init__(self) -> None:
        super().__init__("web_chat_node")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)
        self.command_pub = self.create_publisher(String, "/ccai/mission_command", 10)
        self.create_subscription(String, "/ccai/status", self.on_status, 10)
        self.create_subscription(String, "/ccai/events", self.on_event, 10)
        self.messages: deque[dict[str, str]] = deque(maxlen=200)
        self.latest_status = "{}"
        self.app = self.build_app()
        self.start_server()
        self.get_logger().info("web_chat_node ready")

    def on_status(self, msg: String) -> None:
        self.latest_status = msg.data

    def on_event(self, msg: String) -> None:
        self.messages.append({"source": "robot", "message": msg.data})

    def build_app(self):
        if FastAPI is None:
            raise RuntimeError("fastapi and uvicorn are required: pip install fastapi uvicorn")

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
            return {"status": status_payload, "messages": list(self.messages)}

        @app.post("/api/chat")
        def chat(req: ChatRequest):
            self.messages.append({"source": "admin", "message": req.message})
            self.command_pub.publish(String(data=req.message))
            return {"accepted": True}

        return app

    def start_server(self) -> None:
        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)

        def run() -> None:
            uvicorn.run(self.app, host=host, port=port, log_level="warning")

        threading.Thread(target=run, daemon=True).start()


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
    #log { height: 52vh; overflow: auto; background: white; border: 1px solid #d7dee8; border-radius: 8px; padding: 16px; }
    .msg { margin: 0 0 12px; line-height: 1.45; }
    .src { font-weight: 700; margin-right: 6px; }
    form { display: flex; gap: 8px; margin-top: 12px; }
    input { flex: 1; padding: 12px; border: 1px solid #c7d0dd; border-radius: 6px; font-size: 16px; }
    button { padding: 0 16px; border: 0; border-radius: 6px; background: #1463ff; color: white; font-weight: 700; }
  </style>
</head>
<body>
<main>
  <header><h1>CCAI JetBot Patrol</h1><span id="state">loading</span></header>
  <section id="log"></section>
  <form id="form"><input id="message" autocomplete="off" placeholder="status, patrol start, inspect entrance"><button>Send</button></form>
</main>
<script>
const log = document.getElementById('log');
const state = document.getElementById('state');
async function refresh() {
  const res = await fetch('/api/status');
  const data = await res.json();
  state.textContent = data.status.state || 'unknown';
  log.innerHTML = data.messages.map(m => `<p class="msg"><span class="src">${m.source}</span>${m.message}</p>`).join('');
  log.scrollTop = log.scrollHeight;
}
document.getElementById('form').addEventListener('submit', async (event) => {
  event.preventDefault();
  const input = document.getElementById('message');
  await fetch('/api/chat', {method: 'POST', headers: {'Content-Type':'application/json'}, body: JSON.stringify({message: input.value})});
  input.value = '';
  refresh();
});
setInterval(refresh, 1000);
refresh();
</script>
</body>
</html>
"""


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

