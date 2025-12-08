import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';

interface User {
  id: string;
  email: string;
  name: string;
  preferences: {
    language: string;
    learningPath: string;
    experienceLevel: string;
  };
  progress: {
    [moduleId: string]: {
      [chapterId: string]: {
        completed: boolean;
        timestamp: Date;
        score?: number;
      };
    };
  };
}

interface UserContextType {
  user: User | null;
  loading: boolean;
  login: (email: string, password: string) => Promise<void>;
  logout: () => void;
  updateUserPreferences: (preferences: Partial<User['preferences']>) => void;
  updateChapterProgress: (moduleId: string, chapterId: string, completed: boolean, score?: number) => void;
  getChapterProgress: (moduleId: string, chapterId: string) => any;
  getModuleProgress: (moduleId: string) => number;
}

const UserContext = createContext<UserContextType | undefined>(undefined);

export const UserProvider: React.FC<{ children: ReactNode }> = ({ children }) => {
  const [user, setUser] = useState<User | null>(null);
  const [loading, setLoading] = useState(true);

  // Load user from localStorage on initial load
  useEffect(() => {
    const savedUser = localStorage.getItem('userProfile');
    if (savedUser) {
      try {
        const parsedUser = JSON.parse(savedUser);
        // Convert timestamp strings back to Date objects
        if (parsedUser.progress) {
          Object.keys(parsedUser.progress).forEach(moduleId => {
            Object.keys(parsedUser.progress[moduleId]).forEach(chapterId => {
              if (parsedUser.progress[moduleId][chapterId].timestamp) {
                parsedUser.progress[moduleId][chapterId].timestamp = new Date(parsedUser.progress[moduleId][chapterId].timestamp);
              }
            });
          });
        }
        setUser(parsedUser);
      } catch (e) {
        console.error('Error parsing user data', e);
      }
    }
    setLoading(false);
  }, []);

  const login = async (email: string, password: string) => {
    // Simulate API call
    setLoading(true);

    // In a real implementation, this would call an auth API
    // For now, we'll create a mock user or retrieve from storage
    const existingUser = localStorage.getItem('userProfile');

    if (existingUser) {
      const parsedUser = JSON.parse(existingUser);
      setUser(parsedUser);
    } else {
      const newUser: User = {
        id: `user_${Date.now()}`,
        email,
        name: email.split('@')[0],
        preferences: {
          language: 'en',
          learningPath: 'beginner',
          experienceLevel: 'beginner'
        },
        progress: {}
      };
      setUser(newUser);
      localStorage.setItem('userProfile', JSON.stringify(newUser));
    }

    setLoading(false);
  };

  const logout = () => {
    setUser(null);
    localStorage.removeItem('userProfile');
  };

  const updateUserPreferences = (preferences: Partial<User['preferences']>) => {
    if (user) {
      const updatedUser = {
        ...user,
        preferences: {
          ...user.preferences,
          ...preferences
        }
      };
      setUser(updatedUser);
      localStorage.setItem('userProfile', JSON.stringify(updatedUser));
    }
  };

  const updateChapterProgress = (moduleId: string, chapterId: string, completed: boolean, score?: number) => {
    if (user) {
      const updatedProgress = {
        ...user.progress,
        [moduleId]: {
          ...(user.progress[moduleId] || {}),
          [chapterId]: {
            completed,
            timestamp: new Date(),
            score
          }
        }
      };

      const updatedUser = {
        ...user,
        progress: updatedProgress
      };

      setUser(updatedUser);
      localStorage.setItem('userProfile', JSON.stringify(updatedUser));
    }
  };

  const getChapterProgress = (moduleId: string, chapterId: string) => {
    if (user && user.progress[moduleId] && user.progress[moduleId][chapterId]) {
      return user.progress[moduleId][chapterId];
    }
    return null;
  };

  const getModuleProgress = (moduleId: string): number => {
    if (!user || !user.progress[moduleId]) return 0;

    const moduleProgress = user.progress[moduleId];
    const chapters = Object.keys(moduleProgress);
    if (chapters.length === 0) return 0;

    const completedChapters = chapters.filter(chapterId => moduleProgress[chapterId].completed).length;
    return Math.round((completedChapters / chapters.length) * 100);
  };

  const value = {
    user,
    loading,
    login,
    logout,
    updateUserPreferences,
    updateChapterProgress,
    getChapterProgress,
    getModuleProgress
  };

  return (
    <UserContext.Provider value={value}>
      {children}
    </UserContext.Provider>
  );
};

export const useUser = () => {
  const context = useContext(UserContext);
  if (context === undefined) {
    throw new Error('useUser must be used within a UserProvider');
  }
  return context;
};