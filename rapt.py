import asyncio
import json
import time
import os
import struct
from datetime import datetime, timedelta, timezone
from contextlib import asynccontextmanager

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Request
from fastapi.responses import HTMLResponse
from bleak import BleakScanner

# --- Configuration & State ---
CONFIG_FILE = "pill_config.json"
MASTER_LOG = "master_log.json"

state = {
    "is_logging": False, 
    "current_session_id": None, 
    "last_data": {"g": 0.0, "temp": 0.0, "bat": 0, "t": 0}, 
    "offset": 0.0
}

# Load existing config on startup
if os.path.exists(CONFIG_FILE):
    try:
        with open(CONFIG_FILE, "r") as f:
            state.update(json.load(f))
    except Exception:
        pass

# --- Helper Functions ---
def get_aest_time():
    """Calculates current time in AEST for session naming."""
    aest = timezone(timedelta(hours=10))
    return datetime.now(aest).strftime("%A %-d %B %H:%M")

def save_config():
    """Saves logging state and offsets to disk."""
    with open(CONFIG_FILE, "w") as f:
        # Exclude transient 'last_data' from JSON file
        persist_data = {k: v for k, v in state.items() if k != "last_data"}
        json.dump(persist_data, f)

def log_to_master(data):
    """Writes a new data point to the append-only master log."""
    if not state["is_logging"] or not state["current_session_id"]:
        return
    
    entry = {
        "session_id": state["current_session_id"], 
        "t": int(time.time() * 1000), 
        "g": data["g"], 
        "temp": data["temp"], 
        "bat": data["bat"]
    }
    with open(MASTER_LOG, "a") as f:
        f.write(json.dumps(entry) + "\n")

def parse_rapt(advertisement_data):
    """Parses the specific BLE manufacturer data format for RAPT Pill."""
    m_data = advertisement_data.manufacturer_data.get(16722)
    if not m_data or len(m_data) < 23:
        return None
    try:
        # Temperature: Big Endian Half-float conversion
        t_raw = struct.unpack(">H", m_data[9:11])[0] / 128 - 273.15
        # Gravity: Big Endian Float conversion
        g_raw = struct.unpack(">f", m_data[11:15])[0] / 1000
        # Battery: Big Endian Half-float conversion
        bat_raw = struct.unpack(">H", m_data[21:23])[0] / 256
        
        return {
            "g": round(g_raw + state["offset"], 4), 
            "temp": round(t_raw, 1), 
            "bat": int(min(100, max(0, bat_raw)))
        }
    except Exception:
        return None

# --- Background Services ---
async def ble_scanner():
    """Continuous BLE scanning task."""
    def callback(device, adv):
        data = parse_rapt(adv)
        if data:
            state["last_data"] = {**data, "t": time.time()}
            log_to_master(data)
            
    async with BleakScanner(callback):
        while True:
            await asyncio.sleep(5)

@asynccontextmanager
async def lifespan(app: FastAPI):
    scanner_task = asyncio.create_task(ble_scanner())
    yield
    scanner_task.cancel()

app = FastAPI(lifespan=lifespan)

# --- API Endpoints ---
@app.get("/", response_class=HTMLResponse)
async def get_index():
    return HTML_CONTENT

@app.post("/toggle-logging")
async def toggle(request: Request):
    data = await request.json()
    brew_name = data.get("name", "").strip()
    
    state["is_logging"] = not state["is_logging"]
    
    if state["is_logging"] and not state["current_session_id"]:
        date_str = get_aest_time()
        state["current_session_id"] = f"{brew_name} | {date_str}" if brew_name else date_str
        
    save_config()
    return state

@app.post("/archive")
async def archive():
    state["is_logging"], state["current_session_id"] = False, None
    save_config()
    return {"status": "success"}

@app.get("/history")
async def get_history():
    sessions = {}
    if os.path.exists(MASTER_LOG):
        with open(MASTER_LOG, "r") as f:
            for line in f:
                try:
                    e = json.loads(line)
                    sid = e["session_id"]
                    if sid not in sessions:
                        sessions[sid] = {"name": sid, "start": e["t"], "data": []}
                    sessions[sid]["data"].append(e)
                except: continue
    return sorted(list(sessions.values()), key=lambda x: x["start"], reverse=True)

@app.delete("/history/{session_id}")
async def delete_history(session_id: str):
    if not os.path.exists(MASTER_LOG): return {"status": "error"}
    temp_file = MASTER_LOG + ".tmp"
    with open(MASTER_LOG, "r") as f, open(temp_file, "w") as tf:
        for line in f:
            try:
                if json.loads(line)["session_id"] != session_id:
                    tf.write(line)
            except: continue
    os.replace(temp_file, MASTER_LOG)
    return {"status": "deleted"}

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            history = []
            if state["current_session_id"] and os.path.exists(MASTER_LOG):
                with open(MASTER_LOG, "r") as f:
                    for line in f:
                        try:
                            l = json.loads(line)
                            if l["session_id"] == state["current_session_id"]:
                                history.append(l)
                        except: continue
            
            await websocket.send_json({
                "live": state["last_data"], 
                "is_logging": state["is_logging"], 
                "session_id": state["current_session_id"] or "STATION READY", 
                "logs": history
            })
            await asyncio.sleep(5)
    except WebSocketDisconnect:
        pass

# --- Embedded Frontend ---
HTML_CONTENT = """
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>RAPT PILL MONITOR</title>
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/date-fns"></script>
    <script src="https://cdn.jsdelivr.net/npm/chartjs-adapter-date-fns"></script>
    <style>
        :root {
            --r: #e30613; --b: #3498db; --d: #1a1a1a; --g: #2d2d2d; --t: #fff;
        }
        body {
            font-family: sans-serif; background: var(--d); color: var(--t);
            margin: 0; padding: 10px; display: flex; flex-direction: column; align-items: center;
        }
        .container { width: 100%; max-width: 800px; }
        header { border-left: 4px solid var(--r); padding-left: 15px; margin-bottom: 15px; }
        .status-bar {
            background: var(--g); padding: 10px; border-radius: 4px;
            display: flex; justify-content: space-between; margin-bottom: 15px; font-size: 12px;
        }
        .dot { height: 10px; width: 10px; background: #444; border-radius: 50%; display: inline-block; margin-right: 5px; }
        .active .dot { background: #27ae60; box-shadow: 0 0 8px #27ae60; }
        #dashboard { display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; margin-bottom: 15px; }
        .card { background: var(--g); padding: 15px; border-radius: 8px; text-align: center; border-bottom: 3px solid #333; }
        .value { font-size: 24px; font-weight: bold; font-family: monospace; }
        .label { font-size: 10px; text-transform: uppercase; color: #a0a0a0; margin-bottom: 4px; }
        .chart-wrapper { background: var(--g); padding: 15px; border-radius: 8px; margin-bottom: 20px; height: 250px; display: flex; flex-direction: column; }
        button { padding: 10px; border: none; border-radius: 4px; font-weight: bold; cursor: pointer; background: #444; color: white; font-size: 11px; }
        .btn-main { background: var(--r); }
        input { background: #444; border: 1px solid #555; color: #fff; padding: 8px; border-radius: 4px; width: 180px; }
        .modal { display: none; position: fixed; z-index: 100; left: 0; top: 0; width: 100%; height: 100%; background: rgba(0,0,0,.85); }
        .modal-content { background: var(--g); margin: 10% auto; padding: 20px; width: 85%; max-width: 500px; border-radius: 8px; }
        .history-item { padding: 12px; border-bottom: 1px solid #444; display: flex; align-items: center; justify-content: space-between; }
    </style>
</head>
<body>
    <div class="container">
        <header>
            <h1 style="margin:0; font-size: 22px">RAPT PILL</h1>
            <p id="sessionDisplay" style="color:#a0a0a0; margin:0; font-size: 12px">READY</p>
        </header>

        <div style="display:flex; gap:10px; margin-bottom:15px; align-items:center">
            <input type="text" id="brewName" placeholder="Brew Name (optional)">
            <button id="logBtn" class="btn-main" onclick="toggleLogging()">START LOGGING</button>
            <button onclick="openHistory()">HISTORY</button>
            <button onclick="archive()">ARCHIVE</button>
        </div>

        <div id="statusBox" class="status-bar">
            <span><span class="dot"></span> <span id="statusText">NOT LOGGING</span></span>
            <span id="lastUpdate">--:--:--</span>
        </div>

        <div id="dashboard">
            <div class="card" style="border-bottom-color:var(--r)">
                <div class="label" style="color:var(--r)">Gravity</div>
                <div class="value" id="gVal">--</div>
            </div>
            <div class="card" style="border-bottom-color:var(--b)">
                <div class="label" style="color:var(--b)">Temp</div>
                <div class="value"><span id="tVal">--</span>&deg;C</div>
            </div>
            <div class="card">
                <div class="label">Battery</div>
                <div class="value"><span id="bVal">--</span>%</div>
            </div>
        </div>

        <div class="chart-wrapper">
            <canvas id="gravChart"></canvas>
        </div>
        <div class="chart-wrapper">
            <canvas id="tempChart"></canvas>
        </div>
    </div>

    <div id="historyModal" class="modal">
        <div class="modal-content">
            <h3 style="margin-top:0; color:var(--r)">HISTORY</h3>
            <div id="historyList" style="max-height: 300px; overflow-y: auto; margin-bottom: 15px;"></div>
            <button onclick="closeHistory()" style="width:100%">CLOSE</button>
        </div>
    </div>

    <script>
        let gravChart, tempChart, isViewingHistory = false;
        const ws = new WebSocket("ws://" + window.location.host + "/ws");

        function initChart(id, color, label) {
            return new Chart(document.getElementById(id).getContext("2d"), {
                type: "line",
                data: { datasets: [{ borderColor: color, data: [], tension: 0.3, pointRadius: 0, borderWidth: 2 }] },
                options: {
                    responsive: true, maintainAspectRatio: false,
                    scales: {
                        x: { type: "time", time: { unit: "hour" }, grid: { color: "#333" } },
                        y: { grid: { color: "#333" }, title: { display: true, text: label, color: "#888" } }
                    },
                    plugins: { legend: { display: false } }
                }
            });
        }

        gravChart = initChart("gravChart", "#e30613", "Specific Gravity");
        tempChart = initChart("tempChart", "#3498db", "Temperature (°C)");

        ws.onmessage = e => {
            const data = JSON.parse(e.data);
            if (!isViewingHistory) updateUI(data);
        };

        function updateUI(data) {
            document.getElementById("gVal").innerText = data.live.g > 0 ? data.live.g.toFixed(4) : "--";
            document.getElementById("tVal").innerText = data.live.temp > 0 ? data.live.temp : "--";
            document.getElementById("bVal").innerText = data.live.bat > 0 ? data.live.bat : "--";
            
            if (data.live.t > 0) {
                document.getElementById("lastUpdate").innerText = new Date(data.live.t * 1000).toLocaleTimeString();
            }

            document.getElementById("logBtn").innerText = data.is_logging ? "STOP LOGGING" : "START LOGGING";
            document.getElementById("statusText").innerText = data.is_logging ? "LOGGING ACTIVE" : "NOT LOGGING";
            document.getElementById("statusBox").className = data.is_logging ? "status-bar active" : "status-bar";
            document.getElementById("sessionDisplay").innerText = data.session_id;
            document.getElementById("brewName").style.display = data.is_logging ? "none" : "inline-block";

            gravChart.data.datasets[0].data = data.logs.map(l => ({ x: l.t, y: l.g }));
            tempChart.data.datasets[0].data = data.logs.map(l => ({ x: l.t, y: l.temp }));
            gravChart.update("none");
            tempChart.update("none");
        }

        async function toggleLogging() {
            const name = document.getElementById("brewName").value;
            isViewingHistory = false;
            await fetch("/toggle-logging", {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify({ name: name })
            });
        }

        async function archive() {
            if (confirm("End session and archive?")) {
                await fetch("/archive", { method: "POST" });
                location.reload();
            }
        }

        async function openHistory() {
            const res = await fetch("/history");
            const sessions = await res.json();
            const list = document.getElementById("historyList");
            list.innerHTML = "";
            sessions.forEach(s => {
                const div = document.createElement("div");
                div.className = "history-item";
                div.innerHTML = `
                    <div onclick="viewPast('${s.name}')" style="cursor:pointer">
                        <strong>${s.name}</strong>
                    </div>
                    <button onclick="deleteSession('${s.name}')" style="background:#888; padding:4px 8px">DEL</button>
                `;
                list.appendChild(div);
            });
            document.getElementById("historyModal").style.display = "block";
        }

        async function viewPast(id) {
            const res = await fetch("/history");
            const sessions = await res.json();
            const session = sessions.find(s => s.name === id);
            isViewingHistory = true;
            updateUI({ live: {}, is_logging: false, session_id: "[HISTORY] " + id, logs: session.data });
            closeHistory();
        }

        async function deleteSession(id) {
            if (confirm("Delete " + id + "?")) {
                await fetch("/history/" + encodeURIComponent(id), { method: "DELETE" });
                openHistory();
            }
        }

        function closeHistory() { document.getElementById("historyModal").style.display = "none"; }
    </script>
</body>
</html>
"""

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=5050)
