import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import Button from '../Button';
import Card from '../Card';
import styles from './InteractiveDemo.module.css';

interface InteractiveDemoProps {
  title: string;
  description: string;
  demoType: 'simulation' | 'control' | 'perception' | 'navigation' | 'vla';
  initialParams?: Record<string, any>;
  className?: string;
}

const InteractiveDemo: React.FC<InteractiveDemoProps> = ({
  title,
  description,
  demoType,
  initialParams = {},
  className,
}) => {
  const [params, setParams] = useState(initialParams);
  const [isRunning, setIsRunning] = useState(false);
  const [demoState, setDemoState] = useState<any>(null);
  const [logs, setLogs] = useState<string[]>([]);

  useEffect(() => {
    setParams(initialParams);
  }, [initialParams]);

  const handleParamChange = (key: string, value: any) => {
    setParams(prev => ({
      ...prev,
      [key]: value
    }));
  };

  const handleRunDemo = () => {
    setIsRunning(true);
    addToLog(`Starting ${title} demo...`);

    // Simulate demo execution
    setTimeout(() => {
      const newState = simulateDemo(demoType, params);
      setDemoState(newState);
      addToLog(`Demo completed. State: ${JSON.stringify(newState)}`);
      setIsRunning(false);
    }, 2000);
  };

  const handleReset = () => {
    setDemoState(null);
    setLogs([]);
    setParams(initialParams);
    setIsRunning(false);
    addToLog('Demo reset to initial state.');
  };

  const addToLog = (message: string) => {
    const timestamp = new Date().toLocaleTimeString();
    setLogs(prev => [...prev, `[${timestamp}] ${message}`]);
  };

  const simulateDemo = (type: string, params: any) => {
    switch(type) {
      case 'simulation':
        return {
          physics_accuracy: params.accuracy || 0.95,
          frame_rate: params.fps || 60,
          stability: 'stable'
        };
      case 'control':
        return {
          joint_angles: params.joint_angles || [0, 0, 0, 0, 0, 0],
          control_effort: params.effort || 0.8,
          response_time: 0.05
        };
      case 'perception':
        return {
          objects_detected: params.object_count || 3,
          detection_accuracy: params.accuracy || 0.92,
          processing_time: 0.02
        };
      case 'navigation':
        return {
          path_found: true,
          distance: params.distance || 5.0,
          obstacles: params.obstacles || 2
        };
      case 'vla':
        return {
          command_understood: true,
          action_executed: params.action || 'move_forward',
          confidence: 0.98
        };
      default:
        return { status: 'initialized' };
    }
  };

  const renderControls = () => {
    switch(demoType) {
      case 'simulation':
        return (
          <div className={styles.controlsGrid}>
            <div className={styles.controlGroup}>
              <label>Physics Accuracy</label>
              <input
                type="range"
                min="0.1"
                max="1.0"
                step="0.05"
                value={params.accuracy || 0.95}
                onChange={(e) => handleParamChange('accuracy', parseFloat(e.target.value))}
                className={styles.slider}
              />
              <span>{(params.accuracy || 0.95).toFixed(2)}</span>
            </div>
            <div className={styles.controlGroup}>
              <label>Frame Rate (FPS)</label>
              <select
                value={params.fps || 60}
                onChange={(e) => handleParamChange('fps', parseInt(e.target.value))}
                className={styles.select}
              >
                <option value={30}>30 FPS</option>
                <option value={60}>60 FPS</option>
                <option value={120}>120 FPS</option>
              </select>
            </div>
          </div>
        );

      case 'control':
        return (
          <div className={styles.controlsGrid}>
            <div className={styles.controlGroup}>
              <label>Joint Angles</label>
              {[0, 1, 2, 3, 4, 5].map(i => (
                <div key={i} className={styles.jointControl}>
                  <span>Joint {i + 1}:</span>
                  <input
                    type="range"
                    min="-3.14"
                    max="3.14"
                    step="0.1"
                    value={params.joint_angles?.[i] || 0}
                    onChange={(e) => {
                      const newAngles = [...(params.joint_angles || [0, 0, 0, 0, 0, 0])];
                      newAngles[i] = parseFloat(e.target.value);
                      handleParamChange('joint_angles', newAngles);
                    }}
                    className={styles.slider}
                  />
                  <span>{(params.joint_angles?.[i] || 0).toFixed(2)}</span>
                </div>
              ))}
            </div>
          </div>
        );

      default:
        return null;
    }
  };

  return (
    <Card
      title={title}
      description={description}
      className={clsx(styles.demoCard, className)}
    >
      <div className={styles.demoContainer}>
        <div className={styles.visualization}>
          <div className={styles.visualArea}>
            {demoState ? (
              <div className={styles.stateDisplay}>
                <h4>Demo State:</h4>
                <pre>{JSON.stringify(demoState, null, 2)}</pre>
              </div>
            ) : (
              <div className={styles.placeholder}>
                <div className={styles.robotIcon}>🤖</div>
                <p>Configure parameters and run the demo</p>
              </div>
            )}
          </div>
        </div>

        <div className={styles.controls}>
          <h4>Parameters:</h4>
          {renderControls()}

          <div className={styles.buttonGroup}>
            <Button
              variant="primary"
              onClick={handleRunDemo}
              disabled={isRunning}
              className={styles.runButton}
            >
              {isRunning ? 'Running...' : 'Run Demo'}
            </Button>
            <Button
              variant="secondary"
              onClick={handleReset}
              className={styles.resetButton}
            >
              Reset
            </Button>
          </div>
        </div>

        <div className={styles.logs}>
          <h4>Logs:</h4>
          <div className={styles.logContainer}>
            {logs.length === 0 ? (
              <p className={styles.emptyLog}>No logs yet. Run the demo to see output.</p>
            ) : (
              logs.map((log, index) => (
                <div key={index} className={styles.logEntry}>{log}</div>
              ))
            )}
          </div>
        </div>
      </div>
    </Card>
  );
};

export default InteractiveDemo;