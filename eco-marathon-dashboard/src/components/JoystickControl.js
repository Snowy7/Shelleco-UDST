import React, { useState, useEffect, useRef } from "react";
import { Card } from "react-bootstrap";
import { Joystick } from "react-joystick-component";

const JoystickControl = ({ onControl, visible = true }) => {
  const [linearX, setLinearX] = useState(0);
  const [angularZ, setAngularZ] = useState(0);
  
  // Use refs to keep track of the latest values without triggering effect reruns
  const linearXRef = useRef(linearX);
  const angularZRef = useRef(angularZ);
  
  // Update refs when state changes
  useEffect(() => {
    linearXRef.current = linearX;
    angularZRef.current = angularZ;
  }, [linearX, angularZ]);

  // Set up the control interval only once
  useEffect(() => {
    if (!onControl) {
      console.error("onControl function is not provided");
      return;
    }
    
    const interval = setInterval(() => {
      // Use the ref values which will always be current
      onControl(linearXRef.current, angularZRef.current);
    }, 50);

    return () => {
      clearInterval(interval);
    };
  }, [onControl]); // Only depends on onControl

  const handleMove = (event) => {
    // Convert joystick position to control values
    // Y axis (forward/backward) controls linear velocity
    // X axis (left/right) controls angular velocity

    // Scale values: joystick returns -100 to 100, we want smaller ranges
    const maxLinearSpeed = 0.99; // m/s
    const maxAngularSpeed = 0.99; // rad/s

    // Get x and y from the event
    const x = event.x || 0;
    const y = event.y || 0;

    // Normalize values (the library returns values between -1 and 1)
    const normalizedX = x;
    const normalizedY = y;

    // Invert Y axis so pushing up is positive
    const newLinearX = normalizedY * maxLinearSpeed;
    const newAngularZ = -normalizedX * maxAngularSpeed;

    setLinearX(newLinearX);
    setAngularZ(newAngularZ);
  };

  const handleStop = () => {
    setLinearX(0);
    setAngularZ(0);
  };

  if (!visible) {
    return null; // Don't render anything if not visible
  }

  return (
    <Card className="mb-3">
      <Card.Header>Manual Control</Card.Header>
      <Card.Body className="text-center">
        <div className="mb-3">
          <p>Speed: {linearX.toFixed(2)} m/s</p>
          <p>Steering: {angularZ.toFixed(2)} rad/s</p>
        </div>

        <div className="d-flex justify-content-center">
          <Joystick size={150} baseColor="#f0f0f0" stickColor="#007bff" move={handleMove} stop={handleStop} />
        </div>
      </Card.Body>
    </Card>
  );
};

export default JoystickControl;
