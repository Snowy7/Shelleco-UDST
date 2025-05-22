import React from 'react';
import { Card, ListGroup } from 'react-bootstrap';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { faCheck, faTimes } from '@fortawesome/free-solid-svg-icons';

const NodeStatus = ({ nodes }) => {
  const nodeDisplayNames = {
    lane_detection: 'Lane Detection',
    obstacle_detection: 'Obstacle Detection',
    stop_sign_detection: 'Stop Sign Detection',
    state_machine: 'State Machine',
    section1_planner: 'Section 1 Planner',
    section2_planner: 'Section 2 Planner',
    longitudinal_controller: 'Longitudinal Controller',
    lateral_controller: 'Lateral Controller'
  };

  return (
    <Card className="mb-3">
      <Card.Header>Node Status</Card.Header>
      <ListGroup variant="flush">
        {Object.entries(nodes).map(([nodeName, nodeData]) => (
          <ListGroup.Item 
            key={nodeName}
            className={nodeData.active ? 'text-success' : 'text-danger'}
          >
            {nodeDisplayNames[nodeName] || nodeName}
            {nodeData.active ? (
              <FontAwesomeIcon icon={faCheck} className="ms-2" />
            ) : (
              <FontAwesomeIcon icon={faTimes} className="ms-2" />
            )}
          </ListGroup.Item>
        ))}
      </ListGroup>
    </Card>
  );
};

export default NodeStatus;
