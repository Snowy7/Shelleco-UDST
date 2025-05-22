import React from 'react';
import { Card, ListGroup } from 'react-bootstrap';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { faCheck, faTimes, faCamera, faBroadcastTower, faCompass } from '@fortawesome/free-solid-svg-icons';

const SensorStatus = ({ sensors }) => {
  return (
    <Card className="mb-3">
      <Card.Header>Sensor Status</Card.Header>
      <ListGroup variant="flush">
        <ListGroup.Item className={sensors.camera.connected ? 'text-success' : 'text-danger'}>
          <FontAwesomeIcon icon={faCamera} className="me-2" />
          Camera
          {sensors.camera.connected ? (
            <FontAwesomeIcon icon={faCheck} className="ms-2" />
          ) : (
            <FontAwesomeIcon icon={faTimes} className="ms-2" />
          )}
        </ListGroup.Item>
        
        <ListGroup.Item className={sensors.lidar.connected ? 'text-success' : 'text-danger'}>
          <FontAwesomeIcon icon={faBroadcastTower} className="me-2" />
          LiDAR
          {sensors.lidar.connected ? (
            <FontAwesomeIcon icon={faCheck} className="ms-2" />
          ) : (
            <FontAwesomeIcon icon={faTimes} className="ms-2" />
          )}
        </ListGroup.Item>
        
        <ListGroup.Item className={sensors.imu.connected ? 'text-success' : 'text-danger'}>
          <FontAwesomeIcon icon={faCompass} className="me-2" />
          IMU
          {sensors.imu.connected ? (
            <FontAwesomeIcon icon={faCheck} className="ms-2" />
          ) : (
            <FontAwesomeIcon icon={faTimes} className="ms-2" />
          )}
        </ListGroup.Item>
      </ListGroup>
    </Card>
  );
};

export default SensorStatus;
