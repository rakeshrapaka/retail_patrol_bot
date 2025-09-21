#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from nav_msgs.msg import Odometry
from threading import Thread
from flask import Flask, jsonify, render_template_string

HTML = """
<!doctype html>
<html>
<head>
  <title>Smart Retail Dashboard</title>
  <style>
    body { font-family: Arial, sans-serif; }
    #log { border:1px solid #ccc; padding:8px; height:200px; overflow-y:scroll; background:#f9f9f9; }
  </style>
</head>
<body>
  <h2>🛒 Smart Retail Patrol — Live Demo</h2>
  <div>Patrol state: <b id="state">-</b></div>
  <div>Person count: <b id="persons">-</b></div>
  <div>Odom: <pre id="odom">-</pre></div>
  <h3>📜 Live Log</h3>
  <div id="log"></div>

<script>
let lastLogLength = 0;

async function fetchStatus(){
  try {
    let r = await fetch('/status');
    let j = await r.json();
    document.getElementById('state').innerText = j.patrol_state;
    document.getElementById('persons').innerText = j.person_count;
    document.getElementById('odom').innerText = JSON.stringify(j.odom, null, 2);

    // update log
    if (j.log && j.log.length > lastLogLength) {
      let logDiv = document.getElementById('log');
      for(let i=lastLogLength; i<j.log.length; i++){
        let entry = document.createElement('div');
        entry.textContent = j.log[i];
        logDiv.appendChild(entry);
        logDiv.scrollTop = logDiv.scrollHeight; // auto-scroll
      }
      lastLogLength = j.log.length;
    }
  } catch(e) { console.log(e); }
  setTimeout(fetchStatus, 1000);
}
fetchStatus();
</script>
</body>
</html>
"""

def run_flask(app):
    app.run(host='0.0.0.0', port=5000, threaded=True)

class DashboardNode(Node):
    def __init__(self):
        super().__init__('dashboard_node')
        self.patrol_state = 'unknown'
        self.person_count = 0
        self.odom = {}
        self.log_messages = []   # store messages for dashboard log

        self.create_subscription(String, '/patrol_state', self.patrol_cb, 10)
        self.create_subscription(Int32, '/person_count', self.person_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        self.app = Flask(__name__)
        @self.app.route('/')
        def index():
            return render_template_string(HTML)

        @self.app.route('/status')
        def status():
            return jsonify({
                'patrol_state': self.patrol_state,
                'person_count': int(self.person_count),
                'odom': self.odom,
                'log': self.log_messages[-50:]  # send last 50 entries
            })

        t = Thread(target=run_flask, args=(self.app,), daemon=True)
        t.start()
        self.get_logger().info('Dashboard: Flask server started on port 5000')

    def add_log(self, text: str):
        self.get_logger().info(text)
        self.log_messages.append(text)
        if len(self.log_messages) > 200:  # keep log buffer size
            self.log_messages = self.log_messages[-200:]

    def patrol_cb(self, msg: String):
        self.patrol_state = msg.data
        self.add_log(f"Patrol state: {msg.data}")

    def person_cb(self, msg: Int32):
        self.person_count = msg.data
        self.add_log(f"Person count: {msg.data}")

    def odom_cb(self, msg: Odometry):
        self.odom = {
            'x': round(msg.pose.pose.position.x, 2),
            'y': round(msg.pose.pose.position.y, 2),
        }
        self.add_log(f"Odom update: x={self.odom['x']}, y={self.odom['y']}")


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
