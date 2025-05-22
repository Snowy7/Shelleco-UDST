import React from 'react';
import { Card, Badge } from 'react-bootstrap';

const SystemStatus = ({ state }) => {
  const getStateColor = (stateId) => {
    switch (stateId) {
      case 0: return 'secondary'; // IDLE
      case 1: case 4: return 'primary'; // LANE_FOLLOWING, OBSTACLE_AVOIDANCE
      case 2: case 5: return 'warning'; // APPROACHING_STOP_SIGN
      case 3: case 6: return 'info'; // STOPPED
      case 7: return 'primary'; // PARKING
      case 8: return 'success'; // PARKED
      case 9: return 'danger'; // EMERGENCY_STOP
      default: return 'secondary';
    }
  };

  return (
    <Card className="mb-3">
      <Card.Header>System Status</Card.Header>
      <Card.Body>
        <h3>
          <Badge bg={getStateColor(state.state_id)}>
            {state.state}
          </Badge>
        </h3>
        <p className="text-muted">
          Last updated: {new Date(state.timestamp * 1000).toLocaleTimeString()}
        </p>
      </Card.Body>
    </Card>
  );
};

export default SystemStatus;
