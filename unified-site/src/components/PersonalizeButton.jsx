import React, { useState, useContext } from 'react';
import { usePersonalization } from '../contexts/PersonalizationContext';
import { useAuth } from '../contexts/AuthContext';

const PersonalizeButton = ({ content, onPersonalize }) => {
  const [isPersonalizing, setIsPersonalizing] = useState(false);
  const [personalizationApplied, setPersonalizationApplied] = useState(false);

  // Safely get context values, defaulting to safe values if context is not available
  let userPreferences, updateUserPreferences;
  let user;

  try {
    const personalization = usePersonalization();
    userPreferences = personalization?.userPreferences || { experienceLevel: 'intermediate' };
    updateUserPreferences = personalization?.updatePreferences || (() => {});
  } catch {
    userPreferences = { experienceLevel: 'intermediate' };
    updateUserPreferences = () => {};
  }

  try {
    const auth = useAuth();
    user = auth?.user || null;
  } catch {
    user = null;
  }

  const handlePersonalize = async () => {
    if (!user) {
      alert('Please sign in to use personalization feature');
      return;
    }

    setIsPersonalizing(true);
    try {
      const response = await fetch('/api/content/personalize', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${localStorage.getItem('token')}`
        },
        body: JSON.stringify({
          content: content,
          context: userPreferences
        })
      });

      if (!response.ok) {
        throw new Error('Personalization failed');
      }

      const data = await response.json();

      if (onPersonalize) {
        onPersonalize(data.personalized_content);
      }

      setPersonalizationApplied(true);
    } catch (error) {
      console.error('Personalization error:', error);
      alert('Personalization failed. Please try again.');
    } finally {
      setIsPersonalizing(false);
    }
  };

  const handleReset = () => {
    if (onPersonalize) {
      onPersonalize(content); // Reset to original content
    }
    setPersonalizationApplied(false);
  };

  if (!user) {
    return (
      <div className="personalize-notice">
        <p>Please sign in to personalize this content based on your background.</p>
      </div>
    );
  }

  return (
    <div className="personalize-component">
      {personalizationApplied ? (
        <div className="personalization-controls">
          <button
            onClick={handleReset}
            className="button button--secondary button--sm"
            disabled={isPersonalizing}
          >
            {isPersonalizing ? 'Resetting...' : 'Reset to Original'}
          </button>
          <p className="personalization-notice">
            Content personalized for your background (Level: {userPreferences.experienceLevel})
          </p>
        </div>
      ) : (
        <button
          onClick={handlePersonalize}
          className="button button--primary button--sm personalize-button"
          disabled={isPersonalizing}
        >
          {isPersonalizing ? 'Personalizing...' : 'Personalize Content'}
        </button>
      )}
    </div>
  );
};

export default PersonalizeButton;