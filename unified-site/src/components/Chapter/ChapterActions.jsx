import React, { useState } from 'react';
import { useAuth } from '../../contexts/AuthContext';
import { usePersonalization } from '../../contexts/PersonalizationContext';
import PersonalizeButton from './PersonalizeButton';
import TranslateButton from './TranslateButton';
import ProgressCircle from './ProgressCircle';
import { FaSlidersH, FaGlobeAsia, FaCircle } from 'react-icons/fa';

const ChapterActions = ({ chapterId, chapterTitle = 'Current Chapter' }) => {
  // Safely get context values, defaulting to safe values if context is not available
  let user, userPreferences;

  try {
    const auth = useAuth();
    user = auth?.user || null;
  } catch {
    user = null;
  }

  try {
    const personalization = usePersonalization();
    userPreferences = personalization?.userPreferences || { experienceLevel: 'intermediate', programmingLanguages: [] };
  } catch {
    userPreferences = { experienceLevel: 'intermediate', programmingLanguages: [] };
  }

  const [showActions, setShowActions] = useState(false);

  if (!user) {
    return null; // Don't show buttons if user is not logged in
  }

  return (
    <div className="chapter-actions-container mb-6">
      <div className="flex items-center justify-between bg-gray-50 dark:bg-gray-800 p-4 rounded-lg border border-gray-200 dark:border-gray-700">
        <div className="flex items-center space-x-4">
          <h2 className="text-xl font-semibold text-gray-800 dark:text-gray-200">
            {chapterTitle}
          </h2>
        </div>

        <div className="flex items-center space-x-3">
          {/* Progress Circle */}
          <div className="flex items-center">
            <ProgressCircle chapterId={chapterId} />
          </div>

          {/* Action Buttons - Only show when clicked */}
          <div className="relative">
            <button
              onClick={() => setShowActions(!showActions)}
              className="bg-gray-200 dark:bg-gray-700 hover:bg-gray-300 dark:hover:bg-gray-600 text-gray-800 dark:text-gray-200 p-2 rounded-full transition-colors duration-200"
              aria-label="Chapter actions"
            >
              <FaSlidersH />
            </button>

            {showActions && (
              <div className="absolute right-0 mt-2 w-64 bg-white dark:bg-gray-800 rounded-lg shadow-xl border border-gray-200 dark:border-gray-700 z-10 py-2">
                <div className="px-4 py-2 border-b border-gray-200 dark:border-gray-700">
                  <h3 className="font-medium text-gray-800 dark:text-gray-200">Chapter Actions</h3>
                </div>

                <div className="p-2 space-y-2">
                  <div>
                    <label className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-1">
                      Personalize Content
                    </label>
                    <PersonalizeButton
                      content={chapterTitle}
                      variant="compact"
                    />
                  </div>

                  <div>
                    <label className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-1">
                      Translate to Urdu
                    </label>
                    <TranslateButton
                      content={chapterTitle}
                      variant="compact"
                    />
                  </div>

                  <div className="pt-2 border-t border-gray-200 dark:border-gray-700">
                    <div className="text-xs text-gray-500 dark:text-gray-400 mb-1">
                      Your preferences:
                    </div>
                    <div className="text-xs text-gray-600 dark:text-gray-300">
                      Experience: {userPreferences?.experienceLevel || 'Not set'}<br />
                      Languages: {userPreferences?.programmingLanguages?.join(', ') || 'None specified'}
                    </div>
                  </div>
                </div>
              </div>
            )}
          </div>
        </div>
      </div>
    </div>
  );
};

export default ChapterActions;