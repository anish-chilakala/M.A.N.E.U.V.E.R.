import React, { useState, useEffect } from 'react';
import { Play, Save, Download, Copy, Check, AlertCircle, Terminal, Zap, Code2 } from 'lucide-react';

const ManeuverPlayground = () => {
    const [code, setCode] = useState('');
    const [output, setOutput] = useState('');
    const [isRunning, setIsRunning] = useState(false);
    const [copied, setCopied] = useState(false);
    const [selectedExample, setSelectedExample] = useState('basic');
    const [showTranslations, setShowTranslations] = useState(false);
    const [selectedLang, setSelectedLang] = useState('cpp');

    const examples = {
        basic: {
            name: 'Basic Movement',
            code: `robot mobile_bot:
    // Move forward 50cm at 0.2 m/s
    move forward 50cm at 0.2 m/s
    turn right 90 degrees
    move forward 30cm
    stop
    
    say "Movement complete!"`
        },
        sensor: {
            name: 'Sensor-Based Control',
            code: `robot obstacle_avoider:
    sensor distance_sensor:
        type: ultrasonic
        location: front
    
    loop:
        distance = read distance_sensor
        
        if distance < 20cm:
            stop robot
            say "Obstacle detected at" distance
            turn left 45 degrees
            move forward 10cm
        else:
            move forward at 0.3 m/s`
        },
        arm: {
            name: 'Robotic Arm',
            code: `arm manipulator:
    joints: 6
    reach: 80cm
    
task pick_up_cube:
    // Move to object
    move arm to position (x: 30cm, y: 15cm, z: 10cm)
    open gripper
    
    // Grasp object
    move arm down 5cm smoothly
    close gripper with gentle force
    
    // Place object
    lift arm 10cm
    move arm to position (x: 0cm, y: 40cm, z: 20cm)
    open gripper
    
    // Return home
    return arm to home
    say "Pick and place complete"`
        },
        units: {
            name: 'Physical Units',
            code: `// MANEUVER's type-safe unit system
distance: 10 meters + 50 centimeters
// Result: 10.5 meters

angle: 90 degrees + π radians
// Result: 270 degrees

speed: 5 meters / 2 seconds
// Result: 2.5 m/s

// This would cause a compile error:
// mistake: 5 meters + 3 seconds
// ERROR: Incompatible units!

say "Distance:" distance
say "Angle:" angle
say "Speed:" speed`
        },
        safety: {
            name: 'Safety Constraints',
            code: `robot safe_bot:
    // Define safe operating ranges
    motor_speed: m/s where 0 ≤ value ≤ 2.0
    joint_angle: degrees where -180° ≤ value ≤ 180°
    
task controlled_movement:
    requires: battery_level > 20%
    ensures: all_sensors.operational
    
    // These are verified at compile time
    speed = 1.5 m/s  // ✓ Valid
    angle = 90°      // ✓ Valid
    
    move at speed
    turn angle
    
    say "Safety checks passed!"
    
    // This would cause compile error:
    // bad_speed = 3.0 m/s  // ✗ Exceeds max`
        },
        realtime: {
            name: 'Real-Time Control',
            code: `task motor_control:
    frequency: 100 Hz
    deadline: 8ms
    priority: critical
    
    loop:
        // Read sensors (proven: 0.5ms)
        position = read encoder
        
        // Compute control (proven: 2ms)
        error = target_position - position
        control = PID(error)
        
        // Send commands (proven: 0.3ms)
        send motor_command(control)
        
        // Total: 2.8ms < 8ms ✓
        
    say "Control loop running at 100Hz"`
        },
        advanced: {
            name: 'Autonomous Navigation',
            code: `autonomous_vehicle nav_system:
    sensors:
        lidar: 10 Hz
        camera: 30 Hz
        gps: 1 Hz
    
perception_system:
    deadline: 50ms
    
    process:
        // Fuse sensor data
        objects = detect_objects(camera.image)
        obstacles = process_lidar(lidar.cloud)
        position = localize(gps.data)
        
        // Plan path
        path = plan_trajectory:
            start: position
            goal: destination
            avoid: obstacles
            
        say "Detected" len(objects) "objects"
        say "Planned path with" len(path) "waypoints"
        
        return path`
        }
    };

    useEffect(() => {
        setCode(examples[selectedExample].code);
    }, [selectedExample]);

    const simulateExecution = (code) => {
        const lines = code.split('\n').filter(l => l.trim());
        let result = '🤖 MANEUVER Simulator v1.0\n';
        result += '━'.repeat(50) + '\n\n';

        // Simple simulation logic
        let position = { x: 0, y: 0, angle: 0 };
        let distance = 100; // cm
        let variables = {};

        for (let line of lines) {
            const trimmed = line.trim();

            if (trimmed.startsWith('//') || !trimmed) continue;

            // Movement commands
            if (trimmed.includes('move forward')) {
                const match = trimmed.match(/(\d+)(cm|m|meters)/);
                if (match) {
                    const dist = match[1];
                    const unit = match[2];
                    const distCm = unit === 'm' || unit === 'meters' ? dist * 100 : dist;
                    position.x += distCm * Math.cos(position.angle * Math.PI / 180);
                    position.y += distCm * Math.sin(position.angle * Math.PI / 180);
                    result += `✓ Moved forward ${dist}${unit}\n`;
                    result += `  Position: (${position.x.toFixed(1)}, ${position.y.toFixed(1)})\n\n`;
                }
            }

            if (trimmed.includes('turn')) {
                const match = trimmed.match(/(left|right)\s+(\d+)\s*degrees/);
                if (match) {
                    const direction = match[1];
                    const angle = parseInt(match[2]);
                    position.angle += direction === 'right' ? angle : -angle;
                    result += `✓ Turned ${direction} ${angle}°\n`;
                    result += `  Heading: ${position.angle}°\n\n`;
                }
            }

            if (trimmed.includes('stop')) {
                result += `✓ Robot stopped\n\n`;
            }

            // Say command
            if (trimmed.includes('say')) {
                const match = trimmed.match(/say\s+"([^"]+)"/);
                if (match) {
                    result += `💬 "${match[1]}"\n\n`;
                } else {
                    const parts = trimmed.split('say ')[1];
                    result += `💬 ${parts}\n\n`;
                }
            }

            // Variable assignments
            if (trimmed.includes('=') && !trimmed.includes('==')) {
                const match = trimmed.match(/(\w+)\s*[:=]\s*(.+)/);
                if (match) {
                    const varName = match[1];
                    const value = match[2];
                    variables[varName] = value;
                    result += `📊 ${varName} = ${value}\n\n`;
                }
            }

            // Sensor reading
            if (trimmed.includes('read distance_sensor')) {
                distance = Math.floor(Math.random() * 100) + 20;
                result += `📡 Distance sensor: ${distance}cm\n\n`;
            }

            // Conditionals
            if (trimmed.includes('if') && trimmed.includes('<')) {
                const match = trimmed.match(/if\s+\w+\s*<\s*(\d+)/);
                if (match) {
                    const threshold = parseInt(match[1]);
                    if (distance < threshold) {
                        result += `⚠️  Condition met: distance (${distance}cm) < ${threshold}cm\n\n`;
                    }
                }
            }

            // Arm movements
            if (trimmed.includes('move arm')) {
                const match = trimmed.match(/position\s*\(([^)]+)\)/);
                if (match) {
                    result += `🦾 Arm moved to ${match[0]}\n\n`;
                } else if (trimmed.includes('down')) {
                    result += `🦾 Arm moved down\n\n`;
                } else if (trimmed.includes('up')) {
                    result += `🦾 Arm moved up\n\n`;
                }
            }

            if (trimmed.includes('open gripper')) {
                result += `🤏 Gripper opened\n\n`;
            }

            if (trimmed.includes('close gripper')) {
                result += `🤏 Gripper closed\n\n`;
            }

            // Task definitions
            if (trimmed.startsWith('task ')) {
                const taskName = trimmed.match(/task\s+(\w+)/)[1];
                result += `▶️  Starting task: ${taskName}\n`;
                result += '─'.repeat(40) + '\n\n';
            }

            // Frequency/deadline info
            if (trimmed.includes('frequency:')) {
                const match = trimmed.match(/(\d+)\s*Hz/);
                if (match) {
                    result += `⚡ Control frequency: ${match[1]} Hz\n\n`;
                }
            }

            if (trimmed.includes('deadline:')) {
                const match = trimmed.match(/(\d+)ms/);
                if (match) {
                    result += `⏱️  Deadline: ${match[1]}ms\n\n`;
                }
            }
        }

        result += '━'.repeat(50) + '\n';
        result += '✅ Execution complete!\n';
        result += `\nFinal Position: (${position.x.toFixed(1)}, ${position.y.toFixed(1)})\n`;
        result += `Final Heading: ${position.angle}°\n`;

        return result;
    };

    const runCode = () => {
        setIsRunning(true);
        setOutput('Compiling and executing...\n');

        setTimeout(() => {
            try {
                const result = simulateExecution(code);
                setOutput(result);
            } catch (error) {
                setOutput(`❌ Error: ${error.message}`);
            }
            setIsRunning(false);
        }, 500);
    };

    const copyCode = () => {
        navigator.clipboard.writeText(code);
        setCopied(true);
        setTimeout(() => setCopied(false), 2000);
    };

    const translateToLanguage = (maneuverCode, targetLang) => {
        const lines = maneuverCode.split('\n');

        if (targetLang === 'cpp') {
            return translateToCpp(lines);
        } else if (targetLang === 'python') {
            return translateToPython(lines);
        } else if (targetLang === 'rust') {
            return translateToRust(lines);
        }
    };

    const translateToCpp = (lines) => {
        let cpp = `#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>

class Robot {
private:
    double x = 0.0, y = 0.0, angle = 0.0;
    double distance_sensor = 100.0;
    
public:
    void moveForward(double distance, double speed = 0.2) {
        std::cout << "Moving forward " << distance << "m at " << speed << "m/s" << std::endl;
        x += distance * cos(angle * M_PI / 180.0);
        y += distance * sin(angle * M_PI / 180.0);
        std::this_thread::sleep_for(std::chrono::milliseconds((int)(distance / speed * 1000)));
    }
    
    void turn(std::string direction, double degrees) {
        std::cout << "Turning " << direction << " " << degrees << " degrees" << std::endl;
        angle += (direction == "right" ? degrees : -degrees);
    }
    
    void stop() {
        std::cout << "Robot stopped" << std::endl;
    }
    
    void say(std::string message) {
        std::cout << message << std::endl;
    }
    
    double readDistanceSensor() {
        distance_sensor = rand() % 100 + 20;
        return distance_sensor;
    }
    
    void openGripper() {
        std::cout << "Gripper opened" << std::endl;
    }
    
    void closeGripper() {
        std::cout << "Gripper closed" << std::endl;
    }
    
    void moveArmTo(double x, double y, double z) {
        std::cout << "Moving arm to position (" << x << ", " << y << ", " << z << ")" << std::endl;
    }
};

int main() {
    Robot robot;
    
`;

        let inTask = false;
        for (let line of lines) {
            const trimmed = line.trim();
            if (!trimmed || trimmed.startsWith('//')) {
                cpp += '    ' + line + '\n';
                continue;
            }

            if (trimmed.startsWith('robot ')) continue;
            if (trimmed.startsWith('sensor ')) continue;
            if (trimmed.startsWith('type:')) continue;
            if (trimmed.startsWith('location:')) continue;
            if (trimmed.startsWith('arm ')) continue;
            if (trimmed.startsWith('joints:')) continue;
            if (trimmed.startsWith('reach:')) continue;

            if (trimmed.startsWith('task ')) {
                inTask = true;
                continue;
            }

            if (trimmed.includes('move forward')) {
                const match = trimmed.match(/(\d+)(cm|m|meters)/);
                if (match) {
                    const dist = match[1];
                    const unit = match[2];
                    const distM = unit === 'cm' ? parseFloat(dist) / 100 : dist;
                    const speedMatch = trimmed.match(/at\s+([\d.]+)\s*m\/s/);
                    const speed = speedMatch ? speedMatch[1] : '0.2';
                    cpp += `    robot.moveForward(${distM}, ${speed});\n`;
                }
            } else if (trimmed.includes('turn')) {
                const match = trimmed.match(/(left|right)\s+(\d+)\s*degrees/);
                if (match) {
                    cpp += `    robot.turn("${match[1]}", ${match[2]});\n`;
                }
            } else if (trimmed === 'stop' || trimmed === 'stop robot') {
                cpp += `    robot.stop();\n`;
            } else if (trimmed.includes('say')) {
                const match = trimmed.match(/say\s+"([^"]+)"/);
                if (match) {
                    cpp += `    robot.say("${match[1]}");\n`;
                }
            } else if (trimmed.includes('read distance_sensor')) {
                cpp += `    double distance = robot.readDistanceSensor();\n`;
            } else if (trimmed.includes('if distance')) {
                const match = trimmed.match(/if\s+distance\s*<\s*(\d+)cm/);
                if (match) {
                    cpp += `    if (distance < ${match[1]}) {\n`;
                }
            } else if (trimmed === 'else:' || trimmed === 'else') {
                cpp += `    } else {\n`;
            } else if (trimmed.includes('open gripper')) {
                cpp += `    robot.openGripper();\n`;
            } else if (trimmed.includes('close gripper')) {
                cpp += `    robot.closeGripper();\n`;
            } else if (trimmed.includes('move arm to position')) {
                const match = trimmed.match(/x:\s*([\d.]+)cm.*y:\s*([\d.]+)cm.*z:\s*([\d.]+)cm/);
                if (match) {
                    cpp += `    robot.moveArmTo(${match[1]}, ${match[2]}, ${match[3]});\n`;
                }
            } else if (trimmed.includes('loop:')) {
                cpp += `    while (true) {\n`;
            }
        }

        cpp += `    
    return 0;
}`;

        return cpp;
    };

    const translateToPython = (lines) => {
        let py = `import math
import time
import random

class Robot:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.angle = 0.0
        self.distance_sensor = 100.0
    
    def move_forward(self, distance, speed=0.2):
        print(f"Moving forward {distance}m at {speed}m/s")
        self.x += distance * math.cos(math.radians(self.angle))
        self.y += distance * math.sin(math.radians(self.angle))
        time.sleep(distance / speed)
    
    def turn(self, direction, degrees):
        print(f"Turning {direction} {degrees} degrees")
        self.angle += degrees if direction == "right" else -degrees
    
    def stop(self):
        print("Robot stopped")
    
    def say(self, message):
        print(message)
    
    def read_distance_sensor(self):
        self.distance_sensor = random.randint(20, 120)
        return self.distance_sensor
    
    def open_gripper(self):
        print("Gripper opened")
    
    def close_gripper(self):
        print("Gripper closed")
    
    def move_arm_to(self, x, y, z):
        print(f"Moving arm to position ({x}, {y}, {z})")

def main():
    robot = Robot()
    
`;

        for (let line of lines) {
            const trimmed = line.trim();
            if (!trimmed || trimmed.startsWith('//')) {
                py += '    # ' + trimmed + '\n';
                continue;
            }

            if (trimmed.startsWith('robot ')) continue;
            if (trimmed.startsWith('sensor ')) continue;
            if (trimmed.startsWith('type:')) continue;
            if (trimmed.startsWith('location:')) continue;
            if (trimmed.startsWith('arm ')) continue;
            if (trimmed.startsWith('joints:')) continue;
            if (trimmed.startsWith('reach:')) continue;
            if (trimmed.startsWith('task ')) continue;

            if (trimmed.includes('move forward')) {
                const match = trimmed.match(/(\d+)(cm|m|meters)/);
                if (match) {
                    const dist = match[1];
                    const unit = match[2];
                    const distM = unit === 'cm' ? parseFloat(dist) / 100 : dist;
                    const speedMatch = trimmed.match(/at\s+([\d.]+)\s*m\/s/);
                    const speed = speedMatch ? speedMatch[1] : '0.2';
                    py += `    robot.move_forward(${distM}, ${speed})\n`;
                }
            } else if (trimmed.includes('turn')) {
                const match = trimmed.match(/(left|right)\s+(\d+)\s*degrees/);
                if (match) {
                    py += `    robot.turn("${match[1]}", ${match[2]})\n`;
                }
            } else if (trimmed === 'stop' || trimmed === 'stop robot') {
                py += `    robot.stop()\n`;
            } else if (trimmed.includes('say')) {
                const match = trimmed.match(/say\s+"([^"]+)"/);
                if (match) {
                    py += `    robot.say("${match[1]}")\n`;
                }
            } else if (trimmed.includes('read distance_sensor')) {
                py += `    distance = robot.read_distance_sensor()\n`;
            } else if (trimmed.includes('if distance')) {
                const match = trimmed.match(/if\s+distance\s*<\s*(\d+)cm/);
                if (match) {
                    py += `    if distance < ${match[1]}:\n`;
                }
            } else if (trimmed === 'else:') {
                py += `    else:\n`;
            } else if (trimmed.includes('open gripper')) {
                py += `    robot.open_gripper()\n`;
            } else if (trimmed.includes('close gripper')) {
                py += `    robot.close_gripper()\n`;
            } else if (trimmed.includes('move arm to position')) {
                const match = trimmed.match(/x:\s*([\d.]+)cm.*y:\s*([\d.]+)cm.*z:\s*([\d.]+)cm/);
                if (match) {
                    py += `    robot.move_arm_to(${match[1]}, ${match[2]}, ${match[3]})\n`;
                }
            } else if (trimmed.includes('loop:')) {
                py += `    while True:\n`;
            }
        }

        py += `
if __name__ == "__main__":
    main()`;

        return py;
    };

    const translateToRust = (lines) => {
        let rust = `use std::thread;
use std::time::Duration;
use rand::Rng;

struct Robot {
    x: f64,
    y: f64,
    angle: f64,
    distance_sensor: f64,
}

impl Robot {
    fn new() -> Self {
        Robot {
            x: 0.0,
            y: 0.0,
            angle: 0.0,
            distance_sensor: 100.0,
        }
    }
    
    fn move_forward(&mut self, distance: f64, speed: f64) {
        println!("Moving forward {}m at {}m/s", distance, speed);
        self.x += distance * self.angle.to_radians().cos();
        self.y += distance * self.angle.to_radians().sin();
        thread::sleep(Duration::from_secs_f64(distance / speed));
    }
    
    fn turn(&mut self, direction: &str, degrees: f64) {
        println!("Turning {} {} degrees", direction, degrees);
        self.angle += if direction == "right" { degrees } else { -degrees };
    }
    
    fn stop(&self) {
        println!("Robot stopped");
    }
    
    fn say(&self, message: &str) {
        println!("{}", message);
    }
    
    fn read_distance_sensor(&mut self) -> f64 {
        let mut rng = rand::thread_rng();
        self.distance_sensor = rng.gen_range(20.0..120.0);
        self.distance_sensor
    }
    
    fn open_gripper(&self) {
        println!("Gripper opened");
    }
    
    fn close_gripper(&self) {
        println!("Gripper closed");
    }
    
    fn move_arm_to(&self, x: f64, y: f64, z: f64) {
        println!("Moving arm to position ({}, {}, {})", x, y, z);
    }
}

fn main() {
    let mut robot = Robot::new();
    
`;

        for (let line of lines) {
            const trimmed = line.trim();
            if (!trimmed || trimmed.startsWith('//')) {
                rust += '    ' + line + '\n';
                continue;
            }

            if (trimmed.startsWith('robot ')) continue;
            if (trimmed.startsWith('sensor ')) continue;
            if (trimmed.startsWith('type:')) continue;
            if (trimmed.startsWith('location:')) continue;
            if (trimmed.startsWith('arm ')) continue;
            if (trimmed.startsWith('joints:')) continue;
            if (trimmed.startsWith('reach:')) continue;
            if (trimmed.startsWith('task ')) continue;

            if (trimmed.includes('move forward')) {
                const match = trimmed.match(/(\d+)(cm|m|meters)/);
                if (match) {
                    const dist = match[1];
                    const unit = match[2];
                    const distM = unit === 'cm' ? parseFloat(dist) / 100 : dist;
                    const speedMatch = trimmed.match(/at\s+([\d.]+)\s*m\/s/);
                    const speed = speedMatch ? speedMatch[1] : '0.2';
                    rust += `    robot.move_forward(${distM}, ${speed});\n`;
                }
            } else if (trimmed.includes('turn')) {
                const match = trimmed.match(/(left|right)\s+(\d+)\s*degrees/);
                if (match) {
                    rust += `    robot.turn("${match[1]}", ${match[2]}.0);\n`;
                }
            } else if (trimmed === 'stop' || trimmed === 'stop robot') {
                rust += `    robot.stop();\n`;
            } else if (trimmed.includes('say')) {
                const match = trimmed.match(/say\s+"([^"]+)"/);
                if (match) {
                    rust += `    robot.say("${match[1]}");\n`;
                }
            } else if (trimmed.includes('read distance_sensor')) {
                rust += `    let distance = robot.read_distance_sensor();\n`;
            } else if (trimmed.includes('if distance')) {
                const match = trimmed.match(/if\s+distance\s*<\s*(\d+)cm/);
                if (match) {
                    rust += `    if distance < ${match[1]}.0 {\n`;
                }
            } else if (trimmed === 'else:' || trimmed === 'else') {
                rust += `    } else {\n`;
            } else if (trimmed.includes('open gripper')) {
                rust += `    robot.open_gripper();\n`;
            } else if (trimmed.includes('close gripper')) {
                rust += `    robot.close_gripper();\n`;
            } else if (trimmed.includes('move arm to position')) {
                const match = trimmed.match(/x:\s*([\d.]+)cm.*y:\s*([\d.]+)cm.*z:\s*([\d.]+)cm/);
                if (match) {
                    rust += `    robot.move_arm_to(${match[1]}, ${match[2]}, ${match[3]});\n`;
                }
            } else if (trimmed.includes('loop:')) {
                rust += `    loop {\n`;
            }
        }

        rust += `}`;

        return rust;
    };

    return (
        <div className="min-h-screen bg-gradient-to-br from-slate-900 via-purple-900 to-slate-900 p-6">
            <div className="max-w-7xl mx-auto">
                {/* Header */}
                <div className="mb-6 text-center">
                    <div className="flex items-center justify-center gap-3 mb-3">
                        <Zap className="w-10 h-10 text-yellow-400" />
                        <h1 className="text-4xl font-bold text-white">MANEUVER</h1>
                    </div>
                    <p className="text-purple-300 text-lg">
                        Interactive Programming Playground for Robotics
                    </p>
                    <p className="text-purple-400 text-sm mt-1">
                        Motion Algorithm Naturally Expressed with Unit-Verified Execution Reasoning
                    </p>
                </div>

                {/* Example Selector */}
                <div className="mb-4">
                    <div className="flex flex-wrap gap-2 justify-center">
                        {Object.entries(examples).map(([key, ex]) => (
                            <button
                                key={key}
                                onClick={() => setSelectedExample(key)}
                                className={`px-4 py-2 rounded-lg font-medium transition-all ${
                                    selectedExample === key
                                        ? 'bg-purple-600 text-white shadow-lg'
                                        : 'bg-slate-800 text-purple-300 hover:bg-slate-700'
                                }`}
                            >
                                {ex.name}
                            </button>
                        ))}
                    </div>
                </div>

                <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
                    {/* Code Editor */}
                    <div className="bg-slate-800 rounded-xl shadow-2xl overflow-hidden border border-purple-500/30">
                        <div className="bg-slate-900 px-4 py-3 flex items-center justify-between border-b border-purple-500/30">
                            <div className="flex items-center gap-2">
                                <Terminal className="w-5 h-5 text-purple-400" />
                                <span className="text-white font-semibold">Code Editor</span>
                            </div>
                            <div className="flex gap-2">
                                <button
                                    onClick={() => setShowTranslations(!showTranslations)}
                                    className={`px-3 py-1 rounded-lg text-sm font-medium transition-all flex items-center gap-1 ${
                                        showTranslations
                                            ? 'bg-purple-600 text-white'
                                            : 'bg-slate-700 text-purple-300 hover:bg-slate-600'
                                    }`}
                                    title="Show translations"
                                >
                                    <Code2 className="w-4 h-4" />
                                    Translate
                                </button>
                                <button
                                    onClick={copyCode}
                                    className="p-2 hover:bg-slate-700 rounded-lg transition-colors"
                                    title="Copy code"
                                >
                                    {copied ? (
                                        <Check className="w-4 h-4 text-green-400" />
                                    ) : (
                                        <Copy className="w-4 h-4 text-purple-400" />
                                    )}
                                </button>
                            </div>
                        </div>

                        <textarea
                            value={code}
                            onChange={(e) => setCode(e.target.value)}
                            className="w-full h-[500px] bg-slate-900 text-gray-100 p-4 font-mono text-sm resize-none focus:outline-none"
                            style={{ tabSize: 4 }}
                            spellCheck={false}
                        />

                        <div className="bg-slate-900 px-4 py-3 border-t border-purple-500/30">
                            <button
                                onClick={runCode}
                                disabled={isRunning}
                                className="w-full bg-gradient-to-r from-purple-600 to-pink-600 hover:from-purple-700 hover:to-pink-700 text-white font-bold py-3 px-6 rounded-lg transition-all flex items-center justify-center gap-2 disabled:opacity-50 disabled:cursor-not-allowed shadow-lg"
                            >
                                <Play className="w-5 h-5" />
                                {isRunning ? 'Running...' : 'Run Code'}
                            </button>
                        </div>
                    </div>

                    {/* Output or Translation */}
                    {!showTranslations ? (
                        <div className="bg-slate-800 rounded-xl shadow-2xl overflow-hidden border border-purple-500/30">
                            <div className="bg-slate-900 px-4 py-3 flex items-center gap-2 border-b border-purple-500/30">
                                <AlertCircle className="w-5 h-5 text-green-400" />
                                <span className="text-white font-semibold">Simulation Output</span>
                            </div>

                            <div className="h-[500px] overflow-auto bg-slate-900">
                <pre className="p-4 text-sm text-gray-100 font-mono whitespace-pre-wrap">
                  {output || '👈 Select an example and click "Run Code" to see the simulation output'}
                </pre>
                            </div>

                            <div className="bg-slate-900 px-4 py-3 border-t border-purple-500/30">
                                <div className="text-xs text-purple-400">
                                    <p className="mb-1">💡 Features demonstrated:</p>
                                    <ul className="list-disc list-inside space-y-1">
                                        <li>Natural language syntax</li>
                                        <li>Physical units with automatic conversion</li>
                                        <li>Type-safe coordinate transformations</li>
                                        <li>Real-time execution guarantees</li>
                                    </ul>
                                </div>
                            </div>
                        </div>
                    ) : (
                        <div className="bg-slate-800 rounded-xl shadow-2xl overflow-hidden border border-purple-500/30">
                            <div className="bg-slate-900 px-4 py-3 flex items-center justify-between border-b border-purple-500/30">
                                <div className="flex items-center gap-2">
                                    <Code2 className="w-5 h-5 text-blue-400" />
                                    <span className="text-white font-semibold">Code Translation</span>
                                </div>
                                <div className="flex gap-2">
                                    <button
                                        onClick={() => setSelectedLang('cpp')}
                                        className={`px-3 py-1 rounded text-xs font-medium transition-all ${
                                            selectedLang === 'cpp'
                                                ? 'bg-blue-600 text-white'
                                                : 'bg-slate-700 text-gray-300 hover:bg-slate-600'
                                        }`}
                                    >
                                        C++
                                    </button>
                                    <button
                                        onClick={() => setSelectedLang('python')}
                                        className={`px-3 py-1 rounded text-xs font-medium transition-all ${
                                            selectedLang === 'python'
                                                ? 'bg-blue-600 text-white'
                                                : 'bg-slate-700 text-gray-300 hover:bg-slate-600'
                                        }`}
                                    >
                                        Python
                                    </button>
                                    <button
                                        onClick={() => setSelectedLang('rust')}
                                        className={`px-3 py-1 rounded text-xs font-medium transition-all ${
                                            selectedLang === 'rust'
                                                ? 'bg-blue-600 text-white'
                                                : 'bg-slate-700 text-gray-300 hover:bg-slate-600'
                                        }`}
                                    >
                                        Rust
                                    </button>
                                </div>
                            </div>

                            <div className="h-[500px] overflow-auto bg-slate-900">
                <pre className="p-4 text-sm text-gray-100 font-mono whitespace-pre-wrap">
                  {translateToLanguage(code, selectedLang)}
                </pre>
                            </div>

                            <div className="bg-slate-900 px-4 py-3 border-t border-purple-500/30">
                                <div className="text-xs text-purple-400">
                                    <p className="mb-1">📊 Comparison with {selectedLang.toUpperCase()}:</p>
                                    <ul className="list-disc list-inside space-y-1">
                                        <li>MANEUVER: {code.split('\n').length} lines (natural syntax)</li>
                                        <li>{selectedLang.toUpperCase()}: {translateToLanguage(code, selectedLang).split('\n').length} lines (boilerplate required)</li>
                                        <li>~{Math.round((1 - code.split('\n').length / translateToLanguage(code, selectedLang).split('\n').length) * 100)}% less code with MANEUVER</li>
                                    </ul>
                                </div>
                            </div>
                        </div>
                    )}
                </div>

                {/* Info Footer */}
                <div className="mt-6 bg-slate-800/50 rounded-xl p-6 border border-purple-500/30">
                    <h3 className="text-white font-bold text-lg mb-3">About MANEUVER</h3>
                    <div className="grid md:grid-cols-3 gap-4 text-sm">
                        <div>
                            <h4 className="text-purple-400 font-semibold mb-2">🎯 Design Goals</h4>
                            <ul className="text-gray-300 space-y-1">
                                <li>• Natural language precision</li>
                                <li>• Mathematical certainty</li>
                                <li>• 10-50x faster than C++</li>
                                <li>• Safety by default</li>
                            </ul>
                        </div>
                        <div>
                            <h4 className="text-purple-400 font-semibold mb-2">✨ Key Features</h4>
                            <ul className="text-gray-300 space-y-1">
                                <li>• Physical type system</li>
                                <li>• Coordinate frame safety</li>
                                <li>• Real-time guarantees</li>
                                <li>• Formal verification</li>
                            </ul>
                        </div>
                        <div>
                            <h4 className="text-purple-400 font-semibold mb-2">🤖 Applications</h4>
                            <ul className="text-gray-300 space-y-1">
                                <li>• Hobby robots</li>
                                <li>• Industrial automation</li>
                                <li>• Autonomous vehicles</li>
                                <li>• Research platforms</li>
                            </ul>
                        </div>
                    </div>
                </div>
            </div>
        </div>
    );
};

export default ManeuverPlayground;