# MANEUVER Hardware Support

> **Complete Guide to Supported Platforms, Sensors, and Actuators**

[![Platforms](https://img.shields.io/badge/Platforms-20+-blue)]()
[![Sensors](https://img.shields.io/badge/Sensors-50+-green)]()
[![Status](https://img.shields.io/badge/Status-Expanding-yellow)]()

---

## 📋 Table of Contents

1. [Development Platforms](#development-platforms)
2. [Deployment Platforms](#deployment-platforms)
3. [Sensors](#sensors)
4. [Actuators](#actuators)
5. [Communication Protocols](#communication-protocols)
6. [Robot Platforms](#robot-platforms)
7. [Industrial Systems](#industrial-systems)
8. [Configuration](#configuration)

---

## 💻 Development Platforms

### Supported Operating Systems

| OS | Architecture | Status | Notes |
|----|--------------|--------|-------|
| **Ubuntu 20.04+** | x86-64, ARM64 | ✅ Fully Supported | Primary development platform |
| **Debian 11+** | x86-64, ARM64 | ✅ Fully Supported | |
| **Fedora 36+** | x86-64 | ✅ Fully Supported | |
| **Arch Linux** | x86-64 | ✅ Fully Supported | |
| **macOS 11+** | x86-64, ARM64 | ✅ Fully Supported | Intel and Apple Silicon |
| **Windows 10/11** | x86-64 | ✅ Fully Supported | via WSL2 recommended |
| **Windows Native** | x86-64 | ⚠️ Partial | Limited hardware access |
| **FreeBSD** | x86-64 | 🔄 In Progress | |

### Minimum Requirements

**For Development:**
- CPU: Any modern x86-64 or ARM64
- RAM: 4GB (8GB recommended)
- Disk: 2GB free space
- Internet: For package downloads

**For Compilation:**
- CPU: 2+ cores recommended
- RAM: 2GB minimum (4GB for large projects)

---

## 🤖 Deployment Platforms

### Single-Board Computers

#### Raspberry Pi

| Model | Status | Performance | Use Cases |
|-------|--------|-------------|-----------|
| **Raspberry Pi 5** | ✅ Fully Supported | Excellent | Advanced projects, AI |
| **Raspberry Pi 4** | ✅ Fully Supported | Excellent | Most projects |
| **Raspberry Pi 3** | ✅ Fully Supported | Good | Basic robots, learning |
| **Raspberry Pi Zero 2 W** | ✅ Fully Supported | Good | Small robots |
| **Raspberry Pi Zero W** | ⚠️ Limited | Fair | Very basic projects |
| **Raspberry Pi Pico** | 🔄 In Progress | N/A | Microcontroller |

**Recommended:** Raspberry Pi 4 (4GB RAM) for most projects

**Configuration:**
```toml
# robot_config.toml
[hardware]
platform = "raspberry-pi-4"

[gpio]
# BCM pin numbering
left_motor = 17
right_motor = 18
ultrasonic_trigger = 23
ultrasonic_echo = 24
```

---

#### NVIDIA Jetson

| Model | Status | GPU | Use Cases |
|-------|--------|-----|-----------|
| **Jetson AGX Orin** | ✅ Fully Supported | 2048 CUDA cores | High-end AI robots, AVs |
| **Jetson Orin Nano** | ✅ Fully Supported | 1024 CUDA cores | AI robots, drones |
| **Jetson Xavier NX** | ✅ Fully Supported | 384 CUDA cores | Computer vision |
| **Jetson Nano** | ✅ Fully Supported | 128 CUDA cores | Learning, prototypes |

**Best for:** Computer vision, deep learning, autonomous vehicles

**Performance:** 
- GPU acceleration automatic
- TensorRT integration
- 10-50x faster vision processing

---

#### Other SBCs

| Platform | Status | Notes |
|----------|--------|-------|
| **BeagleBone Black** | ✅ Supported | Good for real-time |
| **BeagleBone AI** | ✅ Supported | Has EVE accelerator |
| **Odroid** (various) | ✅ Supported | High performance |
| **Rock Pi** | ✅ Supported | RP compatible |
| **Banana Pi** | ⚠️ Partial | Community support |
| **Orange Pi** | ⚠️ Partial | Community support |

---

### Microcontrollers

#### Arduino

| Board | Status | Capabilities |
|-------|--------|--------------|
| **Arduino Uno** | ✅ Supported | Basic robots, learning |
| **Arduino Mega** | ✅ Supported | More I/O pins |
| **Arduino Due** | ✅ Supported | 32-bit ARM, faster |
| **Arduino Nano** | ✅ Supported | Compact projects |
| **Arduino MKR** series | ✅ Supported | IoT, WiFi/BLE |

**Limitations:** 
- Limited memory
- No dynamic allocation
- Subset of MANEUVER features

---

#### ESP32 / ESP8266

| Board | Status | Features |
|-------|--------|----------|
| **ESP32** | ✅ Supported | WiFi, BLE, dual-core |
| **ESP32-S2** | ✅ Supported | USB, WiFi |
| **ESP32-S3** | ✅ Supported | AI acceleration |
| **ESP32-C3** | ✅ Supported | RISC-V, WiFi, BLE |
| **ESP8266** | ⚠️ Limited | WiFi only, less memory |

**Best for:** IoT robots, wireless control, remote sensing

---

#### STM32

| Series | Status | Notes |
|--------|--------|-------|
| **STM32F4** | ✅ Supported | Popular, good performance |
| **STM32F7** | ✅ Supported | High performance |
| **STM32H7** | ✅ Supported | Very high performance |
| **STM32L4** | ✅ Supported | Low power |
| **STM32G4** | ✅ Supported | Motor control optimized |

---

### Industrial Computers

| Type | Status | Use Cases |
|------|--------|-----------|
| **x86 Industrial PC** | ✅ Supported | Factory robots, AVs |
| **ARM Industrial** | ✅ Supported | Embedded systems |
| **CompactRIO (NI)** | 🔄 Planned | Industrial automation |
| **Beckhoff CX** series | 🔄 Planned | Industrial control |

---

## 📡 Sensors

### Distance Sensors

#### Ultrasonic

| Model | Status | Range | Interface |
|-------|--------|-------|-----------|
| **HC-SR04** | ✅ Supported | 2cm - 400cm | GPIO |
| **HC-SR05** | ✅ Supported | 2cm - 450cm | GPIO |
| **JSN-SR04T** | ✅ Supported | 20cm - 600cm | GPIO (waterproof) |
| **MaxBotix MB1xxx** | ✅ Supported | Various | Analog/Serial/PWM |

**Configuration:**
```maneuver
sensor distance:
    type: ultrasonic_hcsr04
    trigger_pin: GPIO_23
    echo_pin: GPIO_24
    timeout: 30ms

distance_cm = distance.read()
```

---

#### LIDAR

| Model | Status | Range | Points/Sec | Notes |
|-------|--------|-------|------------|-------|
| **Velodyne VLS-128** | ✅ Supported | 300m | 2.4M | High-end AV |
| **Velodyne VLP-16** | ✅ Supported | 100m | 300K | Popular |
| **Ouster OS1** | ✅ Supported | 120m | 655K | Great value |
| **Livox Mid-360** | ✅ Supported | 40m | 200K | Affordable |
| **RPLIDAR A1** | ✅ Supported | 12m | 8K | Entry-level 2D |
| **RPLIDAR A2** | ✅ Supported | 18m | 8K | 2D |
| **RPLIDAR A3** | ✅ Supported | 25m | 16K | 2D |
| **Slamtec MAPPER** | ✅ Supported | 20m | 16K | 3D |

**Configuration:**
```maneuver
sensor lidar:
    type: velodyne_vlp16
    ip: "192.168.1.201"
    port: 2368
    frame_id: "velodyne"
    frequency: 10 Hz

cloud: PointCloud = lidar.read()
```

---

#### Time-of-Flight (ToF)

| Model | Status | Range | Interface |
|-------|--------|-------|-----------|
| **VL53L0X** | ✅ Supported | 2m | I2C |
| **VL53L1X** | ✅ Supported | 4m | I2C |
| **VL53L4CD** | ✅ Supported | 6m | I2C |
| **TF-Luna** | ✅ Supported | 8m | UART/I2C |
| **TF-Mini** | ✅ Supported | 12m | UART |

---

### Vision Sensors

#### Cameras (RGB)

| Type | Status | Resolutions | Interface |
|------|--------|-------------|-----------|
| **Raspberry Pi Camera v2** | ✅ Supported | 1080p @ 30fps | CSI |
| **Raspberry Pi Camera v3** | ✅ Supported | 1080p @ 50fps | CSI |
| **Raspberry Pi HQ Camera** | ✅ Supported | 4K @ 10fps | CSI |
| **USB Webcams** | ✅ Supported | Various | USB |
| **Logitech C920** | ✅ Supported | 1080p @ 30fps | USB |
| **Logitech C270** | ✅ Supported | 720p @ 30fps | USB |
| **ArduCam** series | ✅ Supported | Various | SPI/I2C |
| **IP Cameras** | ✅ Supported | Various | Ethernet/WiFi |

---

#### Depth Cameras

| Model | Status | Range | Technology | FPS |
|-------|--------|-------|------------|-----|
| **Intel RealSense D435** | ✅ Supported | 10m | Stereo | 90 |
| **Intel RealSense D455** | ✅ Supported | 20m | Stereo | 90 |
| **Intel RealSense L515** | ✅ Supported | 9m | LiDAR | 30 |
| **Kinect v2** | ✅ Supported | 4.5m | ToF | 30 |
| **Kinect Azure** | ✅ Supported | 5m | ToF | 30 |
| **Orbbec Astra** | ✅ Supported | 8m | Structured Light | 30 |
| **ZED 2** | ✅ Supported | 20m | Stereo AI | 60 |
| **OAK-D** | ✅ Supported | 10m | Stereo AI | 60 |

---

#### Thermal Cameras

| Model | Status | Resolution | Interface |
|-------|--------|------------|-----------|
| **FLIR Lepton** | ✅ Supported | 80x60 | SPI |
| **FLIR Lepton 3.5** | ✅ Supported | 160x120 | SPI |
| **MLX90640** | ✅ Supported | 32x24 | I2C |
| **AMG8833** | ✅ Supported | 8x8 | I2C |

---

### Inertial Measurement Units (IMU)

| Model | DOF | Status | Interface | Features |
|-------|-----|--------|-----------|----------|
| **MPU6050** | 6 | ✅ Supported | I2C | Accel + Gyro |
| **MPU9250** | 9 | ✅ Supported | I2C/SPI | + Magnetometer |
| **BMI088** | 6 | ✅ Supported | I2C/SPI | High performance |
| **ICM-20948** | 9 | ✅ Supported | I2C/SPI | Advanced |
| **BNO055** | 9 | ✅ Supported | I2C | Sensor fusion onboard |
| **BNO085** | 9 | ✅ Supported | I2C/SPI | Latest version |
| **ADIS16470** | 6 | ✅ Supported | SPI | Industrial grade |
| **VectorNav VN-100** | 9 | ✅ Supported | UART/SPI | High-end |

**Configuration:**
```maneuver
sensor imu:
    type: mpu6050
    address: 0x68
    frequency: 100 Hz
    
reading: IMUReading = imu.read()
accel = reading.acceleration  // m/s²
gyro = reading.angular_velocity  // rad/s
```

---

### GPS / GNSS

| Model | Status | Accuracy | Protocols |
|-------|--------|----------|-----------|
| **u-blox NEO-M8N** | ✅ Supported | 2.5m | GPS, GLONASS |
| **u-blox NEO-M9N** | ✅ Supported | 1.5m | GPS, GLONASS, Galileo |
| **u-blox ZED-F9P** | ✅ Supported | 0.01m (RTK) | Multi-GNSS + RTK |
| **Emlid Reach M2** | ✅ Supported | 0.01m (RTK) | RTK module |
| **SparkFun GPS modules** | ✅ Supported | Various | Various |

---

### Other Sensors

#### Encoders

| Type | Status | Interface |
|------|--------|-----------|
| **Quadrature Encoders** | ✅ Supported | GPIO |
| **Absolute Encoders** | ✅ Supported | SPI/SSI |
| **Magnetic Encoders (AS5600)** | ✅ Supported | I2C |

#### Force/Torque

| Model | Status | Axes | Max Force |
|-------|--------|------|-----------|
| **ATI Mini40** | ✅ Supported | 6-axis | 40N |
| **ATI Gamma** | ✅ Supported | 6-axis | 130N |
| **Robotiq FT 300** | ✅ Supported | 6-axis | 300N |
| **Load Cells (HX711)** | ✅ Supported | 1-axis | Various |

#### Proximity/Touch

| Model | Status | Technology | Range |
|-------|--------|------------|-------|
| **IR Proximity** | ✅ Supported | Infrared | 3-80cm |
| **Capacitive Touch** | ✅ Supported | Capacitive | Contact |
| **Inductive Sensors** | ✅ Supported | Magnetic | 0-15mm |

#### Environmental

| Type | Models | Status |
|------|--------|--------|
| **Temperature** | DS18B20, DHT22, BME280 | ✅ Supported |
| **Humidity** | DHT22, BME280, SHT31 | ✅ Supported |
| **Pressure** | BMP280, BME280, MS5611 | ✅ Supported |
| **Gas/Air Quality** | MQ-series, BME680, SGP30 | ✅ Supported |
| **Light** | BH1750, TSL2561, VEML7700 | ✅ Supported |

---

## ⚙️ Actuators

### Motors

#### DC Motors

| Type | Driver | Status | Control |
|------|--------|--------|---------|
| **Brushed DC** | L298N | ✅ Supported | PWM |
| **Brushed DC** | L293D | ✅ Supported | PWM |
| **Brushed DC** | TB6612FNG | ✅ Supported | PWM |
| **Brushed DC** | DRV8833 | ✅ Supported | PWM |
| **Brushless DC** | ESCs | ✅ Supported | PWM/OneShot |

**Configuration:**
```maneuver
motor left_motor:
    type: dc_brushed
    driver: l298n
    pwm_pin: GPIO_12
    dir_pin: GPIO_16
    max_voltage: 12V

left_motor.set_speed(0.75)  // 75% forward
```

---

#### Servo Motors

| Type | Protocol | Status | Angle Range |
|------|----------|--------|-------------|
| **Standard Servos** | PWM | ✅ Supported | 0-180° |
| **Continuous Servos** | PWM | ✅ Supported | N/A (rotation) |
| **Digital Servos** | PWM | ✅ Supported | Various |
| **Smart Servos (Dynamixel)** | TTL/RS485 | ✅ Supported | 360° or more |

**Configuration:**
```maneuver
servo shoulder:
    type: standard_servo
    pin: GPIO_18
    min_pulse: 1000μs
    max_pulse: 2000μs
    range: 0° to 180°

shoulder.set_angle(90°)
```

---

#### Stepper Motors

| Type | Driver | Status | Resolution |
|------|--------|--------|------------|
| **Bipolar** | A4988 | ✅ Supported | Full/micro |
| **Bipolar** | DRV8825 | ✅ Supported | 1/32 step |
| **Bipolar** | TMC2208 | ✅ Supported | 1/256 step |
| **Unipolar** | ULN2003 | ✅ Supported | Full/half |

---

### Grippers & End Effectors

| Type | Status | Interface | Use Case |
|------|--------|-----------|----------|
| **Parallel Grippers** | ✅ Supported | Servo/Pneumatic | General purpose |
| **Vacuum Grippers** | ✅ Supported | Digital I/O | Flat objects |
| **Soft Grippers** | ✅ Supported | Servo | Delicate objects |
| **Electromagnets** | ✅ Supported | Digital I/O | Ferrous objects |

---

### Other Actuators

| Type | Status | Interface |
|------|--------|-----------|
| **LEDs** | ✅ Supported | GPIO/PWM |
| **RGB LEDs (WS2812)** | ✅ Supported | SPI |
| **Buzzers** | ✅ Supported | GPIO/PWM |
| **Relays** | ✅ Supported | GPIO |
| **Solenoids** | ✅ Supported | GPIO + Driver |
| **Pneumatic Valves** | ✅ Supported | GPIO + Driver |

---

## 🔌 Communication Protocols

### Hardware Protocols

| Protocol | Status | Typical Use |
|----------|--------|-------------|
| **GPIO** | ✅ Supported | Digital I/O |
| **PWM** | ✅ Supported | Motors, servos |
| **ADC** | ✅ Supported | Analog sensors |
| **I2C** | ✅ Supported | Sensors, displays |
| **SPI** | ✅ Supported | High-speed sensors |
| **UART/Serial** | ✅ Supported | GPS, modules |
| **CAN Bus** | ✅ Supported | Automotive, industrial |
| **1-Wire** | ✅ Supported | Temperature sensors |

---

### Network Protocols

| Protocol | Status | Use Case |
|----------|--------|----------|
| **TCP/IP** | ✅ Supported | General networking |
| **UDP** | ✅ Supported | Fast, unreliable |
| **WebSocket** | ✅ Supported | Real-time web |
| **MQTT** | ✅ Supported | IoT messaging |
| **ROS 1** | ✅ Supported | Robot Operating System |
| **ROS 2** | ✅ Supported | Next-gen ROS |
| **MAVLink** | ✅ Supported | Drones |
| **Modbus** | ✅ Supported | Industrial |

---

### Wireless

| Technology | Status | Range | Use Case |
|------------|--------|-------|----------|
| **WiFi** | ✅ Supported | 50-100m | General purpose |
| **Bluetooth** | ✅ Supported | 10-100m | Short range |
| **BLE** | ✅ Supported | 10-50m | Low power |
| **Zigbee** | 🔄 Planned | 10-100m | Mesh networks |
| **LoRa** | 🔄 Planned | 2-15km | Long range IoT |
| **5G** | 🔄 Planned | N/A | High bandwidth |

---

## 🤖 Robot Platforms

### Educational Robots

| Robot | Status | Platform | Use Case |
|-------|--------|----------|----------|
| **TurtleBot3** | ✅ Supported | ROS | Learning, research |
| **Duckiebot** | ✅ Supported | Custom | Autonomous driving education |
| **Thymio** | ⚠️ Partial | Custom | K-12 education |
| **Makeblock mBot** | ✅ Supported | Arduino | K-12 STEM |

---

### Research Platforms

| Robot | Status | Type | Capabilities |
|-------|--------|------|--------------|
| **Clearpath Jackal** | ✅ Supported | UGV | Outdoor navigation |
| **Clearpath Husky** | ✅ Supported | UGV | Heavy-duty outdoor |
| **Boston Dynamics Spot** | 🔄 Planned | Quadruped | Advanced mobility |
| **PAL Robotics TIAGo** | ✅ Supported | Mobile manipulator | Service robot |

---

### Drones

| Type | Firmware | Status |
|------|----------|--------|
| **Multirotor** | PX4 | ✅ Supported |
| **Multirotor** | ArduPilot | ✅ Supported |
| **Fixed-wing** | PX4/ArduPilot | ✅ Supported |
| **VTOL** | PX4 | ✅ Supported |

---

### Robotic Arms

| Arm | DOF | Status | Features |
|-----|-----|--------|----------|
| **Universal Robots UR5** | 6 | ✅ Supported | Collaborative |
| **Universal Robots UR10** | 6 | ✅ Supported | Larger reach |
| **Franka Emika Panda** | 7 | ✅ Supported | Research platform |
| **ABB IRB series** | 6 | 🔄 Planned | Industrial |
| **KUKA LBR iiwa** | 7 | 🔄 Planned | Collaborative |
| **Dobot Magician** | 4 | ✅ Supported | Educational |

---

## 🏭 Industrial Systems

### PLCs

| Brand | Status | Protocol |
|-------|--------|----------|
| **Siemens S7** | 🔄 Planned | S7 Protocol |
| **Allen-Bradley** | 🔄 Planned | EtherNet/IP |
| **Modicon** | 🔄 Planned | Modbus |

### Industrial Protocols

| Protocol | Status | Use Case |
|----------|--------|----------|
| **OPC UA** | 🔄 Planned | Industrial IoT |
| **EtherCAT** | 🔄 Planned | Real-time motion |
| **PROFINET** | 🔄 Planned | Siemens ecosystem |

---

## ⚙️ Configuration

### Hardware Configuration File

```toml
# robot_config.toml

[hardware]
platform = "raspberry-pi-4"
cpu_governor = "performance"  # performance | powersave

[motors]
left_motor = { type = "dc", driver = "l298n", pwm_pin = 12, dir_pin = 16, max_voltage = 12.0 }
right_motor = { type = "dc", driver = "l298n", pwm_pin = 13, dir_pin = 19, max_voltage = 12.0 }

[sensors]
ultrasonic_front = { type = "hcsr04", trigger_pin = 23, echo_pin = 24, timeout_ms = 30 }
imu = { type = "mpu6050", i2c_address = 0x68, frequency_hz = 100 }
camera = { type = "raspberry_pi_v2", resolution = [1920, 1080], fps = 30 }

[communication]
wifi_ssid = "RobotNetwork"
wifi_password = "secure_password"
ros_master_uri = "http://192.168.1.100:11311"

[power]
battery_cells = 3
cell_voltage_min = 3.0
cell_voltage_max = 4.2
low_battery_threshold = 20.0  # percent
```

---

## 🔧 Adding New Hardware

### Contributing Hardware Support

Want to add support for your hardware?

1. **Check existing drivers** - Maybe it's already supported
2. **Create driver** - Implement sensor/actuator interface
3. **Test thoroughly** - On actual hardware
4. **Document** - Add to this file
5. **Submit PR** - Share with community

See [CONTRIBUTING.md](./CONTRIBUTING.md) for details.

---

## 📞 Hardware Support

### Getting Help

- **Hardware not working?** [Troubleshooting Guide](./docs/troubleshooting.md)
- **Need driver?** [Request on GitHub](https://github.com/maneuver-lang/maneuver/issues)
- **Found bug?** [Report issue](https://github.com/maneuver-lang/maneuver/issues)
- **Questions?** [Ask on Discord](https://discord.gg/maneuver)

### Commercial Support

For commercial hardware integration:
- Email: hardware@maneuver-lang.org
- Custom driver development available

---

## 📊 Legend

- ✅ **Fully Supported** - Tested, documented, stable
- ⚠️ **Partial Support** - Works but limited or beta
- 🔄 **In Progress** - Currently being implemented
- 📋 **Planned** - On roadmap
- ❌ **Not Supported** - No current plans

---

<div align="center">

**Don't see your hardware? Let us know!**

[Request Support](https://github.com/maneuver-lang/maneuver/issues/new?template=hardware_request.md) • [Contribute Driver](./CONTRIBUTING.md) • [Commercial Support](mailto:hardware@maneuver-lang.org)

</div>
