import React, { useState, useEffect } from 'react';
import { useAuth } from '../../contexts/AuthContext';

const ProgressCircle = ({ chapterId }) => {
  const [progress, setProgress] = useState(0);

  // Safely get user from context, defaulting to null if context is not available
  let user;
  try {
    const auth = useAuth();
    user = auth?.user || null;
  } catch {
    user = null;
  }

  // Simulate progress based on chapter ID (in a real app, this would come from the API)
  useEffect(() => {
    if (user && chapterId) {
      // In a real implementation, fetch progress from the backend
      // For now, we'll simulate based on chapter ID
      const simulatedProgress = (chapterId.length * 7) % 100;
      setProgress(simulatedProgress);
    }
  }, [user, chapterId]);

  // Calculate stroke properties for the circle
  const radius = 20;
  const circumference = 2 * Math.PI * radius;
  const strokeDashoffset = circumference - (progress / 100) * circumference;

  return (
    <div className="relative w-10 h-10">
      <svg className="w-10 h-10 transform -rotate-90" viewBox="0 0 44 44">
        {/* Background circle */}
        <circle
          cx="22"
          cy="22"
          r={radius}
          stroke="currentColor"
          strokeWidth="3"
          fill="transparent"
          className="text-gray-200 dark:text-gray-600"
        />
        {/* Progress circle */}
        <circle
          cx="22"
          cy="22"
          r={radius}
          stroke="currentColor"
          strokeWidth="3"
          fill="transparent"
          strokeDasharray={circumference}
          strokeDashoffset={strokeDashoffset}
          className={`text-blue-600 transition-all duration-300 ${
            progress > 70 ? 'text-green-500' : progress > 30 ? 'text-yellow-500' : 'text-red-500'
          }`}
          strokeLinecap="round"
        />
      </svg>
      <div className="absolute inset-0 flex items-center justify-center">
        <span className="text-xs font-medium text-gray-700 dark:text-gray-300">
          {progress}%
        </span>
      </div>
    </div>
  );
};

export default ProgressCircle;