import React from 'react';
import styles from './progressIndicator.module.css';

type ProgressIndicatorProps = {
  progress: number; // 0-100 percentage
  size?: 'small' | 'medium' | 'large';
  showPercentage?: boolean;
  label?: string;
};

function ProgressIndicator({
  progress,
  size = 'medium',
  showPercentage = true,
  label
}: ProgressIndicatorProps): JSX.Element {
  // Ensure progress is within valid range
  const clampedProgress = Math.min(100, Math.max(0, progress));

  return (
    <div className={styles.progressContainer}>
      {label && <div className={styles.label}>{label}</div>}
      <div className={`${styles.progressBar} ${styles[size]}`}>
        <div
          className={styles.progressBarFill}
          style={{ width: `${clampedProgress}%` }}
        ></div>
      </div>
      {showPercentage && (
        <div className={styles.percentageText}>{Math.round(clampedProgress)}%</div>
      )}
    </div>
  );
}

export default ProgressIndicator;