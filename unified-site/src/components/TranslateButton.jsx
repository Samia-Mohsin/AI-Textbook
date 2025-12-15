import React, { useState } from 'react';
import { usePersonalization } from '../contexts/PersonalizationContext';
import { useAuth } from '../contexts/AuthContext';

const TranslateButton = ({ content, onTranslate, language = 'ur' }) => {
  const [isTranslating, setIsTranslating] = useState(false);
  const [translatedContent, setTranslatedContent] = useState(null);

  // Safely get context values, defaulting to safe values if context is not available
  let userPreferences;
  let user;

  try {
    const personalization = usePersonalization();
    userPreferences = personalization?.userPreferences || {};
  } catch {
    userPreferences = {};
  }

  try {
    const auth = useAuth();
    user = auth?.user || null;
  } catch {
    user = null;
  }

  const handleTranslate = async () => {
    if (!user) {
      alert('Please sign in to use translation feature');
      return;
    }

    setIsTranslating(true);
    try {
      const response = await fetch('/api/translate', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${localStorage.getItem('token')}`
        },
        body: JSON.stringify({
          text: content,
          target_lang: language
        })
      });

      if (!response.ok) {
        throw new Error('Translation failed');
      }

      const data = await response.json();
      setTranslatedContent(data.translated_text);

      if (onTranslate) {
        onTranslate(data.translated_text);
      }
    } catch (error) {
      console.error('Translation error:', error);
      alert('Translation failed. Please try again.');
    } finally {
      setIsTranslating(false);
    }
  };

  const handleReset = () => {
    setTranslatedContent(null);
    if (onTranslate) {
      onTranslate(content); // Reset to original content
    }
  };

  if (!user) {
    return (
      <div className="translate-notice">
        <p>Please sign in to translate this content to {language.toUpperCase()}.</p>
      </div>
    );
  }

  return (
    <div className="translate-component">
      {translatedContent ? (
        <div className="translation-controls">
          <button
            onClick={handleReset}
            className="button button--secondary button--sm"
            disabled={isTranslating}
          >
            {isTranslating ? 'Resetting...' : 'Show Original'}
          </button>
          <p className="translation-notice">
            Content translated to {language.toUpperCase()} using AI
          </p>
        </div>
      ) : (
        <button
          onClick={handleTranslate}
          className="button button--primary button--sm translate-button"
          disabled={isTranslating}
        >
          {isTranslating ? 'Translating...' : `Translate to ${language.toUpperCase()}`}
        </button>
      )}
    </div>
  );
};

export default TranslateButton;