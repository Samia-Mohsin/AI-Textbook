import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import Button from '../Button';
import styles from './HumanoidRobotViewer.module.css';

interface JointAngle {
  hip_pitch: number;
  knee_pitch: number;
  ankle_pitch: number;
  hip_roll: number;
  ankle_roll: number;
  shoulder_pitch: number;
  shoulder_roll: number;
  elbow_yaw: number;
  elbow_roll: number;
}

const HumanoidRobotViewer: React.FC = () => {
  const [isSimulating, setIsSimulating] = useState(false);
  const [selectedView, setSelectedView] = useState<'3d' | 'schematic' | 'controls'>('3d');
  const [jointAngles, setJointAngles] = useState<JointAngle>({
    hip_pitch: 0,
    knee_pitch: 0,
    ankle_pitch: 0,
    hip_roll: 0,
    ankle_roll: 0,
    shoulder_pitch: 0,
    shoulder_roll: 0,
    elbow_yaw: 0,
    elbow_roll: 0
  });
  const [robotState, setRobotState] = useState({
    battery: 85,
    status: 'ready',
    position: { x: 0, y: 0, z: 0 },
    orientation: { roll: 0, pitch: 0, yaw: 0 }
  });

  useEffect(() => {
    let interval: NodeJS.Timeout;

    if (isSimulating) {
      interval = setInterval(() => {
        // Simulate robot state changes
        setRobotState(prev => ({
          ...prev,
          position: {
            x: prev.position.x + (Math.random() - 0.5) * 0.01,
            y: prev.position.y + (Math.random() - 0.5) * 0.01,
            z: prev.position.z + (Math.random() - 0.5) * 0.01
          },
          orientation: {
            roll: prev.orientation.roll + (Math.random() - 0.5) * 0.02,
            pitch: prev.orientation.pitch + (Math.random() - 0.5) * 0.02,
            yaw: prev.orientation.yaw + (Math.random() - 0.5) * 0.02
          },
          battery: Math.max(0, prev.battery - 0.01)
        }));

        // Simulate subtle joint movements
        setJointAngles(prev => ({
          ...prev,
          hip_pitch: prev.hip_pitch + (Math.random() - 0.5) * 0.02,
          shoulder_pitch: prev.shoulder_pitch + (Math.random() - 0.5) * 0.02
        }));
      }, 100);
    }

    return () => {
      if (interval) clearInterval(interval);
    };
  }, [isSimulating]);

  const handleJointChange = (joint: keyof JointAngle, value: number) => {
    setJointAngles(prev => ({
      ...prev,
      [joint]: value
    }));
  };

  const handleSimulateToggle = () => {
    setIsSimulating(!isSimulating);
  };

  const handleReset = () => {
    setJointAngles({
      hip_pitch: 0,
      knee_pitch: 0,
      ankle_pitch: 0,
      hip_roll: 0,
      ankle_roll: 0,
      shoulder_pitch: 0,
      shoulder_roll: 0,
      elbow_yaw: 0,
      elbow_roll: 0
    });
    setRobotState({
      battery: 85,
      status: 'ready',
      position: { x: 0, y: 0, z: 0 },
      orientation: { roll: 0, pitch: 0, yaw: 0 }
    });
    setIsSimulating(false);
  };

  const render3DView = () => (
    <div className={styles.viewer3d}>
      <div className={styles.robotContainer}>
        <div className={styles.robotHead}></div>
        <div className={styles.robotTorso}></div>
        <div className={styles.robotArms}>
          <div className={clsx(styles.robotArm, styles.leftArm)} style={{ transform: `rotate(${jointAngles.shoulder_pitch}rad)` }}></div>
          <div className={clsx(styles.robotArm, styles.rightArm)} style={{ transform: `rotate(${-jointAngles.shoulder_pitch}rad)` }}></div>
        </div>
        <div className={styles.robotLegs}>
          <div className={clsx(styles.robotLeg, styles.leftLeg)} style={{ transform: `rotate(${jointAngles.hip_pitch}rad)` }}></div>
          <div className={clsx(styles.robotLeg, styles.rightLeg)} style={{ transform: `rotate(${jointAngles.hip_pitch}rad)` }}></div>
        </div>
      </div>
      <div className={styles.infoPanel}>
        <div className={styles.statusIndicator}>
          <span className={clsx(styles.statusLight, styles[robotState.status])}></span>
          <span>Status: {robotState.status.toUpperCase()}</span>
        </div>
        <div className={styles.batteryIndicator}>
          <div className={styles.batteryBar}>
            <div
              className={styles.batteryFill}
              style={{ width: `${robotState.battery}%` }}
            ></div>
          </div>
          <span>Battery: {Math.round(robotState.battery)}%</span>
        </div>
      </div>
    </div>
  );

  const renderSchematicView = () => (
    <div className={styles.schematicView}>
      <svg viewBox="0 0 400 300" className={styles.schematicSvg}>
        {/* Robot schematic representation */}
        <circle cx="200" cy="80" r="20" fill="#25c2a0" stroke="#1a1a1a" strokeWidth="2" />

        {/* Body */}
        <rect x="180" y="100" width="40" height="80" fill="#3598db" stroke="#1a1a1a" strokeWidth="2" />

        {/* Arms */}
        <line x1="180" y1="120" x2="120" y2="140" stroke="#1a1a1a" strokeWidth="4" />
        <line x1="220" y1="120" x2="280" y2="140" stroke="#1a1a1a" strokeWidth="4" />

        {/* Legs */}
        <line x1="190" y1="180" x2="170" y2="240" stroke="#1a1a1a" strokeWidth="4" />
        <line x1="210" y1="180" x2="230" y2="240" stroke="#1a1a1a" strokeWidth="4" />

        {/* Joints visualization */}
        <circle cx="180" cy="120" r="5" fill="#ff6b6b" />
        <circle cx="220" cy="120" r="5" fill="#ff6b6b" />
        <circle cx="190" cy="180" r="5" fill="#ff6b6b" />
        <circle cx="210" cy="180" r="5" fill="#ff6b6b" />
      </svg>

      <div className={styles.jointValues}>
        <h4>Joint Values (Radians)</h4>
        {Object.entries(jointAngles).map(([joint, value]) => (
          <div key={joint} className={styles.jointValue}>
            <span>{joint}:</span>
            <span>{value.toFixed(2)}</span>
          </div>
        ))}
      </div>
    </div>
  );

  const renderControlsView = () => (
    <div className={styles.controlsView}>
      <h4>Joint Controls</h4>
      <div className={styles.jointControls}>
        {Object.entries(jointAngles).map(([joint, value]) => (
          <div key={joint} className={styles.jointControl}>
            <label>{joint}</label>
            <input
              type="range"
              min="-1.57"
              max="1.57"
              step="0.01"
              value={value}
              onChange={(e) => handleJointChange(joint as keyof JointAngle, parseFloat(e.target.value))}
              className={styles.jointSlider}
            />
            <span className={styles.jointValue}>{value.toFixed(2)}</span>
          </div>
        ))}
      </div>
    </div>
  );

  return (
    <div className={styles.container}>
      <div className={styles.header}>
        <h3>Humanoid Robot Viewer</h3>
        <div className={styles.viewTabs}>
          <button
            className={clsx(styles.tab, { [styles.active]: selectedView === '3d' })}
            onClick={() => setSelectedView('3d')}
          >
            3D View
          </button>
          <button
            className={clsx(styles.tab, { [styles.active]: selectedView === 'schematic' })}
            onClick={() => setSelectedView('schematic')}
          >
            Schematic
          </button>
          <button
            className={clsx(styles.tab, { [styles.active]: selectedView === 'controls' })}
            onClick={() => setSelectedView('controls')}
          >
            Controls
          </button>
        </div>
      </div>

      <div className={styles.viewer}>
        {selectedView === '3d' && render3DView()}
        {selectedView === 'schematic' && renderSchematicView()}
        {selectedView === 'controls' && renderControlsView()}
      </div>

      <div className={styles.actions}>
        <Button
          variant={isSimulating ? 'secondary' : 'primary'}
          onClick={handleSimulateToggle}
        >
          {isSimulating ? 'Stop Simulation' : 'Start Simulation'}
        </Button>
        <Button variant="outline" onClick={handleReset}>
          Reset Position
        </Button>
      </div>

      <div className={styles.robotStats}>
        <div className={styles.stat}>
          <h5>Position</h5>
          <p>X: {robotState.position.x.toFixed(2)} Y: {robotState.position.y.toFixed(2)} Z: {robotState.position.z.toFixed(2)}</p>
        </div>
        <div className={styles.stat}>
          <h5>Orientation</h5>
          <p>Roll: {robotState.orientation.roll.toFixed(2)} Pitch: {robotState.orientation.pitch.toFixed(2)} Yaw: {robotState.orientation.yaw.toFixed(2)}</p>
        </div>
      </div>
    </div>
  );
};

export default HumanoidRobotViewer;