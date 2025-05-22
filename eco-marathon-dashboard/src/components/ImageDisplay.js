import React from 'react';
import { Card } from 'react-bootstrap';

const ImageDisplay = ({ title, imageData }) => {
  return (
    <Card>
      <Card.Header>{title}</Card.Header>
      <Card.Body className="text-center p-0">
        {imageData ? (
          <img 
            src={`data:image/jpeg;base64,${imageData}`} 
            alt={title}
            className="img-fluid"
            style={{ maxHeight: '240px' }}
          />
        ) : (
          <div 
            className="d-flex justify-content-center align-items-center bg-light"
            style={{ height: '240px' }}
          >
            <p className="text-muted">No image data available</p>
          </div>
        )}
      </Card.Body>
    </Card>
  );
};

export default ImageDisplay;
