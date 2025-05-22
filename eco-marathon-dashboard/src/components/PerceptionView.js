import React from 'react';
import { Card, Row, Col, ProgressBar, Badge } from 'react-bootstrap';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { faRoad, faExclamationTriangle, faParking } from '@fortawesome/free-solid-svg-icons';

const PerceptionView = ({ perception }) => {
  // Convert lane center offset to a percentage for visualization
  const laneOffsetPercent = Math.min(100, Math.max(0, (perception.lane_center_offset + 1) * 50));
  
  // Convert heading error to degrees for display
  const headingErrorDegrees = (perception.lane_heading_error * 180 / Math.PI).toFixed(1);
  
  return (
    <Card>
      <Card.Header>Perception Data</Card.Header>
      <Card.Body>
        <Row>
          <Col md={6}>
            <h5>
              <FontAwesomeIcon icon={faRoad} className="me-2" />
              Lane Position
            </h5>
            <ProgressBar>
              <ProgressBar 
                variant="success" 
                now={laneOffsetPercent} 
                key={1} 
              />
            </ProgressBar>
            <div className="d-flex justify-content-between">
              <small>Left</small>
              <small>Center</small>
              <small>Right</small>
            </div>
            <p className="mt-2">
              Offset: {perception.lane_center_offset.toFixed(2)} m
              <br />
              Heading Error: {headingErrorDegrees}°
            </p>
          </Col>
          
          <Col md={6}>
            <h5>
              <FontAwesomeIcon icon={faExclamationTriangle} className="me-2" />
              Obstacles
            </h5>
            <p>Detected: {perception.obstacles_count}</p>
            
            <h5 className="mt-3">
              <FontAwesomeIcon icon={faExclamationTriangle} className="me-2" />
              Stop Sign
            </h5>
            <p>
              {perception.stop_sign_detected ? (
                <Badge bg="warning">Detected</Badge>
              ) : (
                <Badge bg="secondary">Not Detected</Badge>
              )}
              {perception.stop_sign_detected && (
                <span className="ms-2">
                  Distance: {perception.stop_sign_distance.toFixed(2)} m
                </span>
              )}
            </p>
            
            <h5 className="mt-3">
              <FontAwesomeIcon icon={faParking} className="me-2" />
              Parking Area
            </h5>
            <p>
              {perception.parking_area_detected ? (
                <Badge bg="info">Detected</Badge>
              ) : (
                <Badge bg="secondary">Not Detected</Badge>
              )}
            </p>
          </Col>
        </Row>
      </Card.Body>
    </Card>
  );
};

export default PerceptionView;
