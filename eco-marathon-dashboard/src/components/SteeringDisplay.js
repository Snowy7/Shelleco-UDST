import React from 'react';
import { Card } from 'react-bootstrap';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { faArrowsRotate } from '@fortawesome/free-solid-svg-icons';

const SteeringDisplay = ({ steeringAngle }) => {
  // Convert steering angle to degrees
  const steeringDegrees = (steeringAngle * 180 / Math.PI).toFixed(1);
  
  // Calculate rotation for the steering wheel icon
  const rotation = steeringAngle * 180 / Math.PI;
  
  return (
    <Card className="mb-3">
      <Card.Header>Steering</Card.Header>
      <Card.Body className="text-center">
        <div style={{ fontSize: '4rem', marginBottom: '1rem' }}>
          <FontAwesomeIcon 
            icon={faArrowsRotate} 
            style={{
              transform: `rotate(${rotation}deg)`,
              transition: 'transform 0.3s ease-out'
            }}
          />
        </div>
        <h3>{steeringDegrees}°</h3>
        <div className="d-flex justify-content-between">
          <span>Left</span>
          <span>Center</span>
          <span>Right</span>
        </div>
      </Card.Body>
    </Card>
  );
};

export default SteeringDisplay;
