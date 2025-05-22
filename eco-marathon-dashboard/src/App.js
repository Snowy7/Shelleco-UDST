import React, { useState, useEffect, useCallback } from "react";
import { Container, Row, Col, Navbar, Nav, Button, Alert } from "react-bootstrap";
import "bootstrap/dist/css/bootstrap.min.css";
import "./App.css";

// Import components
import SystemStatus from "./components/SystemStatus";
import SensorStatus from "./components/SensorStatus";
import NodeStatus from "./components/NodeStatus";
import ControlPanel from "./components/ControlPanel";
import PerceptionView from "./components/PerceptionView";
import SpeedDisplay from "./components/SpeedDisplay";
import SteeringDisplay from "./components/SteeringDisplay";
import ImageDisplay from "./components/ImageDisplay";
import JoystickControl from "./components/JoystickControl";

function App() {
  const [connected, setConnected] = useState(false);
  const [socket, setSocket] = useState(null);
  const [dashboardData, setDashboardData] = useState({
    system_state: { state: "DISCONNECTED", state_id: -1, timestamp: 0 },
    sensors: {
      camera: { connected: false, last_update: 0 },
      lidar: { connected: false, last_update: 0 },
      imu: { connected: false, last_update: 0 },
    },
    nodes: {
      lane_detection: { active: false, last_update: 0 },
      obstacle_detection: { active: false, last_update: 0 },
      stop_sign_detection: { active: false, last_update: 0 },
      state_machine: { active: false, last_update: 0 },
      section1_planner: { active: false, last_update: 0 },
      section2_planner: { active: false, last_update: 0 },
      longitudinal_controller: { active: false, last_update: 0 },
      lateral_controller: { active: false, last_update: 0 },
    },
    control: {
      mode: "MANUAL",
      emergency_stop: false,
      current_speed: 0.0,
      target_speed: 0.0,
      steering_angle: 0.0,
    },
    perception: {
      lane_center_offset: 0.0,
      lane_heading_error: 0.0,
      obstacles_count: 0,
      stop_sign_detected: false,
      stop_sign_distance: 0.0,
      parking_area_detected: false,
    },
    images: {
      lane_detection: null,
      debug_image: null,
    },
  });

  // Memoize the handleManualControl function so it doesn't change on every render
  const handleManualControl = useCallback((linearX, angularZ) => {
    if (socket && socket.readyState === WebSocket.OPEN) {
      socket.send(
        JSON.stringify({
          type: "manual_control",
          linear_x: linearX,
          angular_z: angularZ,
        })
      );
    }
  }, [socket]);

  // Connect to WebSocket server - fix the dependency issue
  useEffect(() => {
    const socketInstance = connectWebSocket();
    
    console.log("WebSocket connection established");
    // Cleanup function to close the WebSocket connection when the component unmounts
    return () => {
      if (socketInstance) {
        socketInstance.close();
      }
    };
  }, []); // Empty dependency array as connectWebSocket doesn't depend on any props/state

  const connectWebSocket = () => {
    // Get WebSocket URL from environment or use default
    const wsUrl = process.env.REACT_APP_WS_URL || "ws://localhost:8765";
    
    const newSocket = new WebSocket(wsUrl);
    
    newSocket.onopen = () => {
      console.log("WebSocket connected");
      setConnected(true);
      setSocket(newSocket);
    };
    
    newSocket.onmessage = (event) => {
      try {
        const message = JSON.parse(event.data);

        if (message.type === "dashboard_update") {
          setDashboardData(message.data);
        }
      } catch (error) {
        console.error("Error parsing WebSocket message:", error);
      }
    };

    newSocket.onclose = () => {
      console.log("WebSocket disconnected");
      setConnected(false);
      setSocket(null);

      // Try to reconnect after a delay
      setTimeout(connectWebSocket, 3000);
    };

    newSocket.onerror = (error) => {
      console.error("WebSocket error:", error);
      newSocket.close();
    };

    return newSocket;
  };

  const handleControlModeChange = (mode) => {
    if (socket && socket.readyState === WebSocket.OPEN) {
      socket.send(
        JSON.stringify({
          type: "control_mode",
          mode: mode,
        })
      );
    }
  };

  const handleEmergencyStop = () => {
    if (socket && socket.readyState === WebSocket.OPEN) {
      socket.send(
        JSON.stringify({
          type: "emergency_stop",
        })
      );
    }
  };

  const handleResetEmergency = () => {
    if (socket && socket.readyState === WebSocket.OPEN) {
      socket.send(
        JSON.stringify({
          type: "reset_emergency",
        })
      );
    }
  };

  return (
    <div className="App">
      <Navbar bg="dark" variant="dark" expand="lg">
        <Container>
          <Navbar.Brand>Wolves Racing</Navbar.Brand>
          <Navbar.Toggle aria-controls="basic-navbar-nav" />
          <Navbar.Collapse id="basic-navbar-nav" className="justify-content-end">
            <Navbar.Text>{connected ? <span className="text-success">Connected</span> : <span className="text-danger">Disconnected</span>}</Navbar.Text>
          </Navbar.Collapse>
        </Container>
      </Navbar>

      <Container fluid className="mt-3">
        {dashboardData.control.emergency_stop && (
          <Alert variant="danger" className="text-center">
            <h3>EMERGENCY STOP ACTIVATED</h3>
            <Button variant="warning" size="lg" onClick={handleResetEmergency}>
              Reset Emergency Stop
            </Button>
          </Alert>
        )}

        <Row>
          <Col md={3}>
            <SystemStatus state={dashboardData.system_state} />
            <SensorStatus sensors={dashboardData.sensors} />
            <NodeStatus nodes={dashboardData.nodes} />
          </Col>

          <Col md={6}>
            <Row>
              <Col md={12}>
                <ControlPanel controlMode={dashboardData.control.mode} onControlModeChange={handleControlModeChange} onEmergencyStop={handleEmergencyStop} />
              </Col>
            </Row>

            <Row className="mt-3">
              <Col md={6}>
                <ImageDisplay title="Lane Detection" imageData={dashboardData.images.lane_detection} />
              </Col>
              <Col md={6}>
                <ImageDisplay title="Debug View" imageData={dashboardData.images.debug_image} />
              </Col>
            </Row>

            <Row className="mt-3">
              <Col md={12}>
                <PerceptionView perception={dashboardData.perception} />
              </Col>
            </Row>
          </Col>

          <Col md={3}>
            <SpeedDisplay currentSpeed={dashboardData.control.current_speed} targetSpeed={dashboardData.control.target_speed} />
            <SteeringDisplay steeringAngle={dashboardData.control.steering_angle} />
            <JoystickControl onControl={handleManualControl} visible={dashboardData.control.mode === "MANUAL"} />
          </Col>
        </Row>
      </Container>
    </div>
  );
}

export default App;
