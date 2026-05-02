# GonioPro - Home Care Rehabilitation Analytics

**Live Demo:** [https://kamronsoltani.github.io/goniometer](https://kamronsoltani.github.io/goniometer)

A web-based goniometer system for monitoring and tracking range of motion (ROM) exercises during home-based rehabilitation. Built with Three.js for 3D visualization and designed for telehealth integration.


### Play with Demo Mode
1. Open [the live demo](https://kamronsoltani.github.io/goniometer)
2. Click **"Start Exercise"**
3. Watch the animated arm move through a demo range
4. Try dragging the 3D arm to manual control
5. Submit the survey at the end

### Usage

#### Starting a Session
1. Select an exercise from the prescribed goals
2. Set your target angle (default 120°) and hold time (default 3s)
3. Click **"Start Exercise"** → device calibrates
4. Move your limb through the motion:
   - Motion must exceed 6° to register
   - Hold position within target zone (±10° for precision tier)
   - Return to start to count the repetition

#### Exercise Presets
- **Elbow Flex**: 120° target
- **Wrist Extension**: 45° target
- **Knee Flexion**: 90° target
- **Shoulder**: 160° target

#### Session Data
- **Live Angle**: Current joint angle in degrees
- **Peak ROM**: Highest angle achieved in session
- **Stability**: Consistency metric (%)
- **Hold Target**: Current hold duration vs goal

## Hardware Connection

### For BLE Device (Seeed XIAO ESP32-C3)
1. Upload the firmware from `src/main.cpp` to your device
2. Flash the device with the goniometer firmware
3. On the web interface, click **"Connect Device"**
4. Select your GonioPro device from the Bluetooth list
5. Perform device calibration (follow on-screen prompts)

### BLE Protocol
The device communicates via Web Bluetooth with:
- **Service UUID**: `6f4d0001-7c4d-4b8c-9a7a-3f4c2b6d0001`
- **Data Characteristic**: `6f4d0002-7c4d-4b8c-9a7a-3f4c2b6d0001`
- **Command Characteristic**: `6f4d0003-7c4d-4b8c-9a7a-3f4c2b6d0001`

## Configuration

### Patient Settings
- **Patient ID**: Reference identifier (stored in exports)
- **Target Reps**: Number of repetitions per session
- **Limb Side**: Left or Right

### Angle Controls
- **Target Angle**: 10° - 175° (adjustable slider)
- **Hold Time**: 0 - 10 seconds

## Data Export

Click **"Save Clinical CSV"** to download session data including:
- Patient reference ID
- Repetition count with timestamps
- Max angle achieved per rep
- Hold duration per rep
- Post-session survey responses

Example filename: `Home_Rehab_Log_1714689320000.csv`

## Post-Session Survey

After completing a session, you'll be prompted to fill out a home care assessment:
- Joint discomfort rating (0-10)
- Ease of movement (Stiff/Moderate/Fluid)
- Setup difficulty
- Confidence level
- Notes for physical therapist

## Technical Stack

- **Frontend**: HTML5, TailwindCSS, Chart.js
- **3D Graphics**: Three.js r128
- **Backend Communication**: Web Bluetooth API
- **Hosting**: GitHub Pages
- **Device OS**: Arduino (Seeed XIAO ESP32-C3)

## File Structure

```
goniometer/
├── docs/                    # GitHub Pages deployment
│   └── index.html          # Main web app (production)
├── src/
│   ├── main.cpp            # ESP32 firmware
│   ├── GUI                 # Original GUI (deprecated)
│   └── index.html          # Original HTML
├── include/
│   └── gui_html.h          # HTML header file
├── platformio.ini          # Project configuration
└── README.md               # This file
```

## Browser Support

- **Chrome/Edge**: Full support (Web Bluetooth required)
- **Firefox**: Demo mode only (no BLE)
- **Safari**: Demo mode only (no BLE)

*Note: Web Bluetooth requires HTTPS or localhost*

## Deployment

The web interface is automatically deployed via GitHub Pages when you push to the `main` branch. The `docs/` folder is served at:
```
https://kamronsoltani.github.io/goniometer
```

## Troubleshooting

**"Web Bluetooth requires Chrome or Edge"**
- Use Chrome, Edge, or Opera for BLE connectivity
- Other browsers can use demo mode

**"BLE device not found"**
- Ensure device is powered and in range
- Check Bluetooth is enabled on your system
- Verify device name starts with "GonioPro"

**"No sensor data"**
- Calibrate device first
- Check I2C connections (IMU sensors)
- Verify firmware upload was successful

## Project Structure

- **Platform**: Seeed XIAO ESP32-C3
- **Sensors**: 2x Adafruit LSM6DSOX IMU (forearm + bicep)
- **Communication**: BLE 5.0
- **Protocol**: Text-based packet format (DATA,key=value,...)

## License

Open source rehabilitation analytics project

## Questions?

For issues, feature requests, or technical questions, open an issue on GitHub.
