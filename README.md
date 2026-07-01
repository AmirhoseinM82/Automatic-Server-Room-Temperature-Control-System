### The Fan Control Rule

The Master applies a linear proportional control formula to map the ambient temperature ($T$ in $^\\circ\\text{C}$) to the required fan speed percentage ($S$):

$$S = \\max\\left(0.0, \\min\\left(100.0, \\frac{T - 16}{2} \\times 5\\right)\\right)$$

- **At or below $16^\\circ\\text{C}$ (Ideal Temp):** Fans remain completely off (0%).
- **Temperature Proportionality:** Every $2^\\circ\\text{C}$ rise in temperature increases fan speed by $5\\%$.
- **Critical Threshold:** The maximum allowable safe room temperature is $60^\\circ\\text{C}$. Exceeding this triggers the system-wide emergency alarm state.

---

## 🗂️ File Structure

The project is encapsulated inside a single, self-contained file:
- `server_room_control.py` — Contains the implementation of system constants, the `WirelessChannel`, the `ServerRoom` physics engine, the `ESP32` micro-controller emulator, the `Simulation` runner thread-coordinator, the background `Siren` acoustics, and the `Dashboard` Tkinter view.

---

## 🛠️ Requirements & Installation

1. **Python 3.6+** must be installed on your operating system.
2. No external libraries are needed. Standard distribution modules are leveraged.

Clone this repository or download the source script directly:
```bash
git clone [https://github.com/your-username/server-room-temperature-control.git](https://github.com/your-username/server-room-temperature-control.git)
cd server-room-temperature-control

```

---

## 💻 How to Run & Test

Execute the main simulation script directly from your terminal:

```bash
python server_room_control.py

```

### Recommended Interactive Evaluation Steps:

1. **Manipulate Heat Load:** Drag the **Server Heat Level Slider** in the bottom control dock. Push it upwards to simulate a sudden server processing load. Watch the temperature gauge rise and see the animated fan accelerate.
2. **Trigger Emergency Mode:** Move the heat slider to its maximum level. Once the ambient temperature gauge punches past $60^\circ\text{C}$, the audio-visual alarm will engage with flashing red indicators and an alternating acoustic siren.
3. **Mute Audio:** Click the **Mute Siren Sound** checkbox to silence the acoustic alert while maintaining the visual warning layout.
4. **Induce Network Disruption:** Uncheck the **Wireless Connection** box. The Master will immediately detect packet loss, print timeout failures into the live console log, and flag a `LINK FAULT` status warning.
5. **Sensor Realism:** Toggle the **Sensor Noise** checkbox to switch between clean mathematical curves and a fluctuating real-world sensor data stream with mechanical ripple.

---

## 🧠 Thread Safety & Concurrent Architecture

To accurately simulate separate computational chips, the system distributes processing tasks across three parallel contexts:

1. **Main UI Thread:** Handles the Tkinter frame execution, interface rendering, and periodically reads state updates from the thread-safe communication buffers.
2. **ESP32 Worker Thread:** Manages physics updates for the server room's thermodynamic properties and listens for incoming downlink messages.
3. **Raspberry Pi Master Thread:** Drives the deterministic polling mechanism, executes control math equations, and dispatches command flags.

All Inter-Process Communications (IPC) utilize synchronized `queue.Queue` structures, completely eliminating data race conditions and avoiding the need for complex mutex resource-locking.

---

