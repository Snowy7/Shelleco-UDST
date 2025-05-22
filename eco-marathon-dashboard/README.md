# Eco-Marathon Dashboard

A comprehensive dashboard for monitoring and controlling the Shell Eco-marathon Autonomous Urban Concept vehicle.

## Features

- Real-time monitoring of vehicle state, sensors, and nodes
- Visualization of perception data (lane detection, obstacles, stop signs)
- Manual control via on-screen joystick
- Switching between manual and autonomous modes
- Emergency stop functionality
- Sensor and node health monitoring

## Installation

### ROS2 Dashboard Node

1. Copy the `dashboard_node.py` file to your ROS2 workspace:

```bash
cp dashboard_node.py ~/eco_marathon_ws/src/eco_planning/eco_planning/
```

2. Make it executable:

```bash
chmod +x ~/eco_marathon_ws/src/eco_planning/eco_planning/dashboard_node.py
```

3. Update the `setup.py` file in the `eco_planning` package to include the dashboard node:

```python
entry_points={
    'console_scripts': [
        'state_machine_node = eco_planning.state_machine_node:main',
        'section1_planner_node = eco_planning.section1_planner_node:main',
        'section2_planner_node = eco_planning.section2_planner_node:main',
        'dashboard_node = eco_planning.dashboard_node:main',
    ],
},
```

4. Build the workspace:

```bash
cd ~/eco_marathon_ws
colcon build
source install/setup.bash
```

### Dashboard Web Application

#### Option 1: Run as a web application

1. Install Node.js and npm if not already installed
2. Clone the dashboard repository
3. Install dependencies:

```bash
cd eco-marathon-dashboard
npm install
```

4. Start the development server:

```bash
npm start
```

5. Open a web browser and navigate to <http://localhost:3000>

#### Option 2: Run as a standalone application

1. Install dependencies:

```bash
cd eco-marathon-dashboard
npm install
```

2. Run the Electron development version:

```bash
npm run electron-dev
```

3. Build the standalone application:

```bash
npm run electron-pack
```

4. The built application will be in the `dist` folder

## Usage

1. Start the ROS2 dashboard node:

```bash
ros2 launch eco_planning dashboard.launch.py
```

2. Start the dashboard web application or standalone application
3. The dashboard will automatically connect to the WebSocket server running on the ROS2 node
4. Use the control panel to switch between manual and autonomous modes
5. In manual mode, use the on-screen joystick to control the vehicle
6. Monitor sensor status, node health, and perception data in real-time
7. Use the emergency stop button if needed

## Dependencies

- ROS2 Humble or later
- Node.js 14 or later
- npm 6 or later
- Python 3.8 or later
- websockets Python package
