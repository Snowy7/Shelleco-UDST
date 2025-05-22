import React from 'react';
import { Card } from 'react-bootstrap';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, ResponsiveContainer } from 'recharts';

const SpeedDisplay = ({ currentSpeed, targetSpeed }) => {
  // Create data for the speed chart (last 20 seconds)
  const [speedData, setSpeedData] = React.useState(
    Array(20).fill().map((_, i) => ({
      time: i - 19,
      current: 0,
      target: 0
    }))
  );
  
  // Update speed data
  React.useEffect(() => {
    setSpeedData(prevData => {
      const newData = [...prevData.slice(1), {
        time: 0,
        current: currentSpeed,
        target: targetSpeed
      }];
      
      // Update time values
      return newData.map((point, i) => ({
        ...point,
        time: i - (newData.length - 1)
      }));
    });
  }, [currentSpeed, targetSpeed]);
  
  return (
    <Card className="mb-3">
      <Card.Header>Speed</Card.Header>
      <Card.Body>
        <div className="text-center mb-3">
          <h2>{(currentSpeed * 3.6).toFixed(1)} km/h</h2>
          <p className="text-muted">
            Target: {(targetSpeed * 3.6).toFixed(1)} km/h
          </p>
        </div>
        
        <ResponsiveContainer width="100%" height={200}>
          <LineChart data={speedData}>
            <CartesianGrid strokeDasharray="3 3" />
            <XAxis 
              dataKey="time" 
              domain={[-19, 0]}
              label={{ value: 'Time (s)', position: 'insideBottom', offset: -5 }}
            />
            <YAxis 
              label={{ value: 'Speed (m/s)', angle: -90, position: 'insideLeft' }}
            />
            <Tooltip />
            <Line 
              type="monotone" 
              dataKey="current" 
              stroke="#8884d8" 
              name="Current Speed"
            />
            <Line 
              type="monotone" 
              dataKey="target" 
              stroke="#82ca9d" 
              strokeDasharray="5 5"
              name="Target Speed"
            />
          </LineChart>
        </ResponsiveContainer>
      </Card.Body>
    </Card>
  );
};

export default SpeedDisplay;
