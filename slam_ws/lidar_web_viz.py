#!/usr/bin/env python3
"""
Standalone web server for SICK TIM561 polar scan visualization
Serves HTML page that connects to ROS2 via rosbridge and displays scan data
Similar to L1_lidar_GUI_tunnel but for rosboard/web viewing

Usage:
    Terminal 1: Run SLAM
    sudo bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch slam_config slam_usb.launch.py"
    
    Terminal 2: Run rosbridge (websocket server for ROS2)
    sudo bash -c "source /opt/ros/humble/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
    
    Terminal 3: Run this web server
    cd ~/Adversary_DRONE/slam_ws && python3 lidar_web_viz.py
    
    Then open: http://10.250.240.81:8080/?rosbridge_port=9090
"""

import http.server
import socketserver
import os
import sys
import argparse
from urllib.parse import urlparse

PORT = 8080

# HTML content with embedded visualization
HTML_CONTENT = """<!DOCTYPE html>
<html>
<head>
    <title>SICK TIM561 LiDAR - Polar Scan Viewer</title>
    <meta charset="UTF-8">
    <style>
        body {
            margin: 0;
            padding: 20px;
            background: #1a1a2e;
            font-family: Arial, sans-serif;
            color: white;
        }
        #container {
            max-width: 1200px;
            margin: 0 auto;
        }
        h1 {
            text-align: center;
            color: #16c79a;
            margin-bottom: 20px;
        }
        #canvas {
            background: #16213e;
            border-radius: 10px;
            display: block;
            margin: 0 auto;
            box-shadow: 0 0 20px rgba(22, 199, 154, 0.3);
        }
        #status {
            text-align: center;
            margin: 20px 0;
            font-size: 18px;
            font-weight: bold;
            padding: 15px;
            border-radius: 5px;
        }
        .danger { background: #d32f2f; }
        .warning { background: #f57c00; }
        .caution { background: #fbc02d; color: #000; }
        .clear { background: #388e3c; }
        .waiting { background: #555; }
        #info {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 15px;
            margin-top: 20px;
        }
        .info-box {
            background: #16213e;
            padding: 15px;
            border-radius: 8px;
            border: 2px solid #0f3460;
        }
        .info-label {
            color: #16c79a;
            font-size: 12px;
            text-transform: uppercase;
            margin-bottom: 5px;
        }
        .info-value {
            font-size: 20px;
            font-weight: bold;
        }
    </style>
</head>
<body>
    <div id="container">
        <h1>🎯 SICK TIM561 LiDAR - 270° Polar Scan (ROS2)</h1>
        <canvas id="canvas" width="800" height="800"></canvas>
    <div id="status" class="waiting">Connecting to ROS2...</div>
    <div id="wsinfo" style="text-align:center; font-size:13px; color:#9ec; margin-top:6px;"></div>
        <div id="info">
            <div class="info-box">
                <div class="info-label">Points</div>
                <div class="info-value" id="points">-</div>
            </div>
            <div class="info-box">
                <div class="info-label">Nearest Obstacle</div>
                <div class="info-value" id="nearest">-</div>
            </div>
            <div class="info-box">
                <div class="info-label">FOV / Resolution</div>
                <div class="info-value" id="fov">270° / 1.0°</div>
            </div>
            <div class="info-box">
                <div class="info-label">Range</div>
                <div class="info-value" id="range">0.05m - 10.0m</div>
            </div>
            <div class="info-box">
                <div class="info-label">Scan Rate</div>
                <div class="info-value" id="rate">-</div>
            </div>
            <div class="info-box">
                <div class="info-label">Close Objects (<2m)</div>
                <div class="info-value" id="close">-</div>
            </div>
        </div>
    </div>

    <script src="https://cdn.jsdelivr.net/npm/roslib@1/build/roslib.min.js"></script>
    <script>
        // ROS connection - auto-detect hostname and allow custom port via query string
        const wsHost = window.location.hostname || 'localhost';
        const params = new URLSearchParams(window.location.search);
        const wsPort = params.get('rosbridge_port') || '9090';
        const wsURL = `ws://${wsHost}:${wsPort}`;
        document.getElementById('wsinfo').textContent = `WebSocket: ${wsURL}`;
        const ros = new ROSLIB.Ros({ url: wsURL });

        ros.on('connection', () => {
            console.log('Connected to ROS2');
            document.getElementById('status').textContent = 'Connected - Waiting for scan data...';
            document.getElementById('status').className = 'waiting';
        });

        ros.on('error', (error) => {
            console.error('Error connecting to ROS:', error);
            const msg = (error && (error.message || error.toString())) || 'Unknown error';
            document.getElementById('status').textContent = `❌ ROS Connection Error: ${msg}`;
            document.getElementById('status').className = 'danger';
        });

        ros.on('close', () => {
            console.log('Connection to ROS closed');
            document.getElementById('status').textContent = '❌ ROS Connection Closed';
            document.getElementById('status').className = 'danger';
        });

        // Subscribe to /scan topic
        const scanTopic = new ROSLIB.Topic({
            ros: ros,
            name: '/scan',
            messageType: 'sensor_msgs/LaserScan'
        });

        const canvas = document.getElementById('canvas');
        const ctx = canvas.getContext('2d');
        const centerX = canvas.width / 2;
        const centerY = canvas.height / 2;
        const maxRadius = Math.min(centerX, centerY) - 50;

        let lastScanTime = Date.now();
        let scanCount = 0;
        let scanRate = 0;

        function drawGrid() {
            ctx.strokeStyle = '#0f3460';
            ctx.lineWidth = 1;
            
            // Distance circles (1m intervals up to 10m)
            for (let r = 1; r <= 10; r++) {
                const radius = (r / 10) * maxRadius;
                ctx.beginPath();
                ctx.arc(centerX, centerY, radius, 0, 2 * Math.PI);
                ctx.stroke();
                
                // Distance labels
                ctx.fillStyle = '#16c79a';
                ctx.font = '12px Arial';
                ctx.fillText(`${r}m`, centerX + radius + 5, centerY);
            }
            
            // Angle lines (every 45°, covering -135° to +135°)
            ctx.strokeStyle = '#0f3460';
            for (let angle = -135; angle <= 135; angle += 45) {
                const rad = (angle - 90) * Math.PI / 180;  // -90 to orient 0° forward
                const x1 = centerX;
                const y1 = centerY;
                const x2 = centerX + maxRadius * Math.cos(rad);
                const y2 = centerY + maxRadius * Math.sin(rad);
                
                ctx.beginPath();
                ctx.moveTo(x1, y1);
                ctx.lineTo(x2, y2);
                ctx.stroke();
                
                // Angle labels
                ctx.fillStyle = '#16c79a';
                ctx.font = '12px Arial';
                const labelX = centerX + (maxRadius + 20) * Math.cos(rad);
                const labelY = centerY + (maxRadius + 20) * Math.sin(rad);
                ctx.fillText(`${angle}°`, labelX - 15, labelY + 5);
            }
            
            // Center point
            ctx.fillStyle = '#16c79a';
            ctx.beginPath();
            ctx.arc(centerX, centerY, 5, 0, 2 * Math.PI);
            ctx.fill();
            
            // Forward indicator
            ctx.fillStyle = '#16c79a';
            ctx.font = 'bold 16px Arial';
            ctx.fillText('⬆ FORWARD', centerX - 40, 30);
        }

        function drawScan(msg) {
            // Calculate scan rate
            scanCount++;
            const now = Date.now();
            if (now - lastScanTime > 1000) {
                scanRate = scanCount;
                scanCount = 0;
                lastScanTime = now;
                document.getElementById('rate').textContent = `${scanRate} Hz`;
            }
            
            // Clear canvas
            ctx.fillStyle = '#16213e';
            ctx.fillRect(0, 0, canvas.width, canvas.height);
            
            // Draw grid
            drawGrid();
            
            // Extract scan data
            const ranges = msg.ranges;
            const angleMin = msg.angle_min;
            const angleIncrement = msg.angle_increment;
            const rangeMin = msg.range_min;
            const rangeMax = msg.range_max;
            
            let validPoints = 0;
            let minDist = Infinity;
            let closeCount = 0;
            
            // Draw scan points
            for (let i = 0; i < ranges.length; i++) {
                const range = ranges[i];
                
                // Skip invalid points
                if (range < rangeMin || range > rangeMax || !isFinite(range)) {
                    continue;
                }
                
                validPoints++;
                if (range < minDist) minDist = range;
                if (range < 2.0) closeCount++;
                
                // Calculate angle (LiDAR angles relative to forward = 0°)
                const angle = angleMin + i * angleIncrement;
                
                // Convert to canvas coordinates (rotate -90° so forward is up)
                const canvasAngle = angle - Math.PI / 2;
                const radius = (range / 10) * maxRadius;  // Scale to 10m max
                const x = centerX + radius * Math.cos(canvasAngle);
                const y = centerY + radius * Math.sin(canvasAngle);
                
                // Color based on distance
                let color;
                if (range < 1.0) color = '#d32f2f';      // Red: <1m
                else if (range < 2.0) color = '#f57c00'; // Orange: <2m
                else if (range < 6.0) color = '#fbc02d'; // Yellow: <6m
                else color = '#16c79a';                   // Green: >6m
                
                ctx.fillStyle = color;
                ctx.beginPath();
                ctx.arc(x, y, 3, 0, 2 * Math.PI);
                ctx.fill();
            }
            
            // Update status
            let statusText, statusClass;
            if (minDist < 0.5) {
                statusText = `⚠️ DANGER! Obstacle at ${minDist.toFixed(2)}m`;
                statusClass = 'danger';
            } else if (minDist < 1.0) {
                statusText = `⚠️ WARNING: Close obstacle at ${minDist.toFixed(2)}m`;
                statusClass = 'warning';
            } else if (minDist < 2.0) {
                statusText = `⚠️ CAUTION: Obstacle at ${minDist.toFixed(2)}m (${closeCount} points < 2m)`;
                statusClass = 'caution';
            } else {
                statusText = `✅ Clear - Nearest: ${minDist.toFixed(2)}m`;
                statusClass = 'clear';
            }
            
            document.getElementById('status').textContent = statusText;
            document.getElementById('status').className = statusClass;
            document.getElementById('points').textContent = validPoints;
            document.getElementById('nearest').textContent = `${minDist.toFixed(2)}m`;
            document.getElementById('close').textContent = closeCount;
            
            // Update FOV info
            const fovDeg = ((msg.angle_max - msg.angle_min) * 180 / Math.PI).toFixed(0);
            const resDeg = (msg.angle_increment * 180 / Math.PI).toFixed(2);
            document.getElementById('fov').textContent = `${fovDeg}° / ${resDeg}°`;
            document.getElementById('range').textContent = `${rangeMin.toFixed(2)}m - ${rangeMax.toFixed(1)}m`;
        }

        scanTopic.subscribe((message) => {
            drawScan(message);
        });

        // Initial grid
        drawGrid();
    </script>
</body>
</html>
"""

class MyHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        path = urlparse(self.path).path  # strip query string
        if path == '/' or path == '/index.html':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(HTML_CONTENT.encode('utf-8'))
        else:
            super().do_GET()
    
    def log_message(self, format, *args):
        # Suppress logging
        pass

def main():
    global PORT
    # Allow overriding port via CLI or env var
    try:
        parser = argparse.ArgumentParser(add_help=False)
        parser.add_argument('--port', type=int, default=None)
        args, _ = parser.parse_known_args()
        if args.port:
            PORT = args.port
        else:
            PORT = int(os.environ.get('WEB_VIZ_PORT', PORT))
    except Exception:
        pass
    # Allow port reuse to prevent "Address already in use" errors
    socketserver.TCPServer.allow_reuse_address = True
    
    with socketserver.TCPServer(("0.0.0.0", PORT), MyHTTPRequestHandler) as httpd:
        import socket
        hostname = socket.gethostname()
        ip = socket.gethostbyname(hostname)
        
        print(f"╔════════════════════════════════════════════════════════════╗")
        print(f"║  SICK TIM561 Web Visualization Server                     ║")
        print(f"╠════════════════════════════════════════════════════════════╣")
        print(f"║  Server running at:                                        ║")
        print(f"║    http://localhost:{PORT}                                    ║")
        print(f"║    http://{ip}:{PORT}                             ║")
        print(f"╠════════════════════════════════════════════════════════════╣")
        print(f"║  Prerequisites:                                            ║")
        print(f"║  1. SLAM running with /scan topic                          ║")
        print(f"║  2. Rosbridge websocket running (default port 9090)        ║")
        print(f"║                                                            ║")
        print(f"║  Start rosbridge (default port 9090):                      ║")
        print(f"║    sudo bash -c \"source /opt/ros/humble/setup.bash &&      ║")
        print(f"║                ros2 launch rosbridge_server \\              ║")
        print(f"║                rosbridge_websocket_launch.xml\"             ║")
        print(f"║                                                            ║")
        print(f"║  If using a non-default port, append to URL:               ║")
        print(f"║    http://<host>:8080/?rosbridge_port=<PORT>               ║")
        print(f"╚════════════════════════════════════════════════════════════╝")
        print()
        
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\n[*] Shutting down server...")
            httpd.shutdown()

if __name__ == "__main__":
    main()
