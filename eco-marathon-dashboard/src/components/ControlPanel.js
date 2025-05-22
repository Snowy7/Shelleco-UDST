import React from 'react';
import { Card, ButtonGroup, Button } from 'react-bootstrap';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { faRobot, faUser, faExclamationTriangle } from '@fortawesome/free-solid-svg-icons';

const ControlPanel = ({ controlMode, onControlModeChange, onEmergencyStop }) => {
  return (
    <Card>
      <Card.Header>Control Panel</Card.Header>
      <Card.Body>
        <div className="d-flex justify-content-between align-items-center">
          <div>
            <h5>Control Mode</h5>
            <ButtonGroup>
              <Button
                variant={controlMode === 'MANUAL' ? 'primary' : 'outline-primary'}
                onClick={() => onControlModeChange('MANUAL')}
              >
                <FontAwesomeIcon icon={faUser} className="me-2" />
                Manual
              </Button>
              <Button
                variant={controlMode === 'AUTONOMOUS' ? 'primary' : 'outline-primary'}
                onClick={() => onControlModeChange('AUTONOMOUS')}
              >
                <FontAwesomeIcon icon={faRobot} className="me-2" />
                Autonomous
              </Button>
            </ButtonGroup>
          </div>
          
          <div>
            <Button
              variant="danger"
              size="lg"
              onClick={onEmergencyStop}
            >
              <FontAwesomeIcon icon={faExclamationTriangle} className="me-2" />
              EMERGENCY STOP
            </Button>
          </div>
        </div>
      </Card.Body>
    </Card>
  );
};

export default ControlPanel;
