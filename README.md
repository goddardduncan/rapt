## RAPT Pill Connectivity Suite

This repository contains various tools and firmware to interface with the **RAPT Pill** hydrometer, ranging from direct browser connections to smart home integrations via ESP32 and Raspberry Pi.

---

### 📂 Project Components

| Component | Platform | Description |
| :--- | :--- | :--- |
| **`index.html`** | Web Browser | Allows a **direct Bluetooth connection** to the RAPT Pill via Web Bluetooth API. No server required. |
| **`gravity_sinric.ino`** | ESP32 | Firmware to bridge the Pill and **Sinric Pro**. Reports gravity data for integration with **Google Home**. |
| **`temp_sinric.ino`** | ESP32 | Firmware to bridge the Pill and **Sinric Pro**. Reports temperature data for smart home monitoring. |
| **`rapt.py`** | Raspberry Pi | A robust logging station designed for the **Pi Zero 2W**. |

---

### 🚀 Raspberry Pi Station (`rapt.py`)

The Python station provides a persistent monitoring dashboard and data logging.

* **Web Dashboard:** Hosts a local server accessible at `http://<your-pi-ip>:5050`.
* **Data Persistence:** * `pill_config.json`: Stores current logging states, session names, and calibration offsets.
    * `master_log.json`: An append-only file containing historical gravity and temperature data points.
* **Real-time Tracking:** Uses Bluetooth Low Energy (BLE) to scan for advertisements and broadcasts data via WebSockets for instant dashboard updates.

---

### 🛠 Hardware Requirements

* **ESP32:** Required for Sinric Pro / Google Home integration.
* **Raspberry Pi Zero 2W:** (or similar) Recommended for the Python logging station.
* **Bluetooth Connectivity:** The host device must have BLE capabilities to see the RAPT Pill.

---

### 📖 Usage Note

For the **Sinric Pro** sketches, ensure you have your `APP_KEY`, `APP_SECRET`, and `DEVICE_ID` configured in the `.ino` files before flashing to your ESP32.
