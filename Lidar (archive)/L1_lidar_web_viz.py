"""
L1_lidar_web_viz.py — Web-based real-time visualizer for SICK TiM LiDAR
Author: Boo Man & GPT-5

Usage:
------
python3 L1_lidar_web_viz.py --ethernet   # use Ethernet (default)
python3 L1_lidar_web_viz.py --usb        # use USB

Then open browser: http://localhost:8080

For remote access via SSH tunnel:
ssh -L 8080:localhost:8080 eagle@jetty
Then open: http://localhost:8080
"""

import asyncio
import json
import threading
import time
from aiohttp import web
import numpy as np
from L1_lidar import Lidar as EthernetLidar
from L1_lidar_usb import Lidar as UsbLidar

SENSOR_IP = "168.254.15.1"
PORT = 8080

# Global lidar instance
lidar = None
processor = None

async def websocket_handler(request):
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    
    print(f"[+] WebSocket client connected from {request.remote}")
    
    try:
        while True:
            if lidar and lidar.ds is not None:
                num_points = len(lidar.ds)
                angles = np.linspace(-135, 135, num_points).tolist()
                distances = lidar.ds.tolist()
                
                data = {
                    'angles': angles,
                    'distances': distances,
                    'num_points': num_points
                }
                await ws.send_json(data)
            
            await asyncio.sleep(0.05)  # 20 Hz update rate
            
    except Exception as e:
        print(f"[!] WebSocket error: {e}")
    finally:
        print("[-] WebSocket client disconnected")
    
    return ws


async def index_handler(request):
    html = """
<!DOCTYPE html>
<html>
<head>
    <title>SICK TiM561 LiDAR Visualizer</title>
    <style>
        body {
            margin: 0;
            padding: 20px;
            background: #1a1a1a;
            color: #fff;
            font-family: 'Courier New', monospace;
            overflow: hidden;
        }
        #container {
            display: flex;
            flex-direction: column;
            align-items: center;
            height: 100vh;
        }
        #info {
            padding: 10px;
            background: #2a2a2a;
            border-radius: 5px;
            margin-bottom: 10px;
            font-size: 14px;
        }
        canvas {
            border: 2px solid #00ffff;
            background: #0a0a0a;
            border-radius: 10px;
        }
        .status {
            color: #00ff00;
        }
        .label {
            color: #888;
        }
    </style>
</head>
<body>
    <div id="container">
        <div id="info">
            <span class="label">Status:</span> <span id="status" class="status">Connecting...</span> |
            <span class="label">Points:</span> <span id="points">0</span> |
            <span class="label">FPS:</span> <span id="fps">0</span>
        </div>
        <canvas id="lidar" width="800" height="800"></canvas>
    </div>

    <script>
        const canvas = document.getElementById('lidar');
        const ctx = canvas.getContext('2d');
        const centerX = canvas.width / 2;
        const centerY = canvas.height / 2;
        const maxRadius = Math.min(centerX, centerY) - 20;
        const maxRange = 10.0; // meters (matches tunnel GUI)
        
        let frameCount = 0;
        let lastTime = Date.now();
        
        const ws = new WebSocket('ws://' + window.location.host + '/ws');
        
        ws.onopen = () => {
            document.getElementById('status').textContent = 'Connected';
            document.getElementById('status').style.color = '#00ff00';
        };
        
        ws.onerror = () => {
            document.getElementById('status').textContent = 'Error';
            document.getElementById('status').style.color = '#ff0000';
        };
        
        ws.onclose = () => {
            document.getElementById('status').textContent = 'Disconnected';
            document.getElementById('status').style.color = '#ff8800';
        };
        
        ws.onmessage = (event) => {
            const data = JSON.parse(event.data);
            drawLidar(data.angles, data.distances, data.num_points);
            
            // Update info
            document.getElementById('points').textContent = data.num_points;
            
            // Calculate FPS
            frameCount++;
            const now = Date.now();
            if (now - lastTime >= 1000) {
                document.getElementById('fps').textContent = frameCount;
                frameCount = 0;
                lastTime = now;
            }
        };
        
        function drawLidar(angles, distances, numPoints) {
            // Clear canvas
            ctx.fillStyle = '#0a0a0a';
            ctx.fillRect(0, 0, canvas.width, canvas.height);
            
            // Draw range circles
            ctx.strokeStyle = '#222';
            ctx.lineWidth = 1;
            for (let i = 1; i <= 10; i++) {
                const r = (i / maxRange) * maxRadius;
                ctx.beginPath();
                ctx.arc(centerX, centerY, r, 0, Math.PI * 2);
                ctx.stroke();
                
                // Range labels (every 2m)
                if (i % 2 === 0) {
                    ctx.fillStyle = '#555';
                    ctx.font = '12px monospace';
                    ctx.fillText(i + 'm', centerX + 5, centerY - r + 15);
                }
            }
            
            // Draw angle lines
            ctx.strokeStyle = '#333';
            for (let angle = -135; angle <= 135; angle += 45) {
                const rad = angle * Math.PI / 180;
                const x = centerX + Math.cos(rad - Math.PI/2) * maxRadius;
                const y = centerY + Math.sin(rad - Math.PI/2) * maxRadius;
                ctx.beginPath();
                ctx.moveTo(centerX, centerY);
                ctx.lineTo(x, y);
                ctx.stroke();
                
                // Angle labels
                ctx.fillStyle = '#666';
                ctx.font = '11px monospace';
                const labelX = centerX + Math.cos(rad - Math.PI/2) * (maxRadius + 15);
                const labelY = centerY + Math.sin(rad - Math.PI/2) * (maxRadius + 15);
                ctx.fillText(angle + '°', labelX - 15, labelY + 5);
            }
            
            // Draw LiDAR points
            ctx.fillStyle = '#00ffff';
            for (let i = 0; i < numPoints; i++) {
                const angle = angles[i];
                const distance = distances[i];
                
                if (distance > 0 && distance < maxRange) {
                    // Mirror the angle to fix left/right reversal
                    const rad = -angle * Math.PI / 180;
                    const r = (distance / maxRange) * maxRadius;
                    const x = centerX + r * Math.cos(rad - Math.PI/2);
                    const y = centerY + r * Math.sin(rad - Math.PI/2);
                    
                    ctx.beginPath();
                    ctx.arc(x, y, 2, 0, Math.PI * 2);
                    ctx.fill();
                }
            }
            
            // Draw center point
            ctx.fillStyle = '#ff0000';
            ctx.beginPath();
            ctx.arc(centerX, centerY, 4, 0, Math.PI * 2);
            ctx.fill();
        }
    </script>
</body>
</html>
"""
    return web.Response(text=html, content_type='text/html')


async def init_app():
    app = web.Application()
    app.router.add_get('/', index_handler)
    app.router.add_get('/ws', websocket_handler)
    return app


def run_web_server(port):
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    
    app = loop.run_until_complete(init_app())
    runner = web.AppRunner(app)
    loop.run_until_complete(runner.setup())
    
    site = web.TCPSite(runner, '0.0.0.0', port)
    loop.run_until_complete(site.start())
    
    print(f"[+] Web server running on http://0.0.0.0:{port}")
    print(f"[+] Access locally: http://localhost:{port}")
    print(f"[+] For remote access, use SSH tunnel:")
    print(f"    ssh -L {port}:localhost:{port} eagle@<jetson-ip>")
    
    loop.run_forever()


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Web-based LiDAR visualizer")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--ethernet", action="store_true", help="use Ethernet (default)")
    group.add_argument("--usb", action="store_true", help="use USB")
    parser.add_argument("--ip", default=SENSOR_IP, help="sensor IP for Ethernet mode")
    parser.add_argument("--port", type=int, default=PORT, help="web server port")
    args = parser.parse_args()
    
    mode = 'usb' if args.usb else 'ethernet'
    
    # Start LiDAR
    if mode == 'ethernet':
        lidar = EthernetLidar(IP=args.ip)
        lidar.connect()
        processor = lidar.run()
        time.sleep(1)
        print(f"[+] Using Ethernet LiDAR at {args.ip}")
    else:
        lidar = UsbLidar()
        lidar.connect()
        processor = lidar.run()
        time.sleep(1)
        print("[+] Using USB LiDAR")
    
    # Start web server
    try:
        run_web_server(args.port)
    except KeyboardInterrupt:
        print("\n[*] Shutting down...")
    finally:
        if lidar and processor:
            lidar.kill(processor)
        print("[*] Stopped")
