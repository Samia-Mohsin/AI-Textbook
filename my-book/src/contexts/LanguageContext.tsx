import React, { createContext, useContext, useState, ReactNode } from 'react';

interface LanguageContextType {
  currentLanguage: string;
  availableLanguages: string[];
  changeLanguage: (lang: string) => void;
  t: (key: string) => string;
}

const LanguageContext = createContext<LanguageContextType | undefined>(undefined);

// Basic translation dictionary
const translations: Record<string, Record<string, string>> = {
  en: {
    // English translations
    'welcome': 'Welcome',
    'login': 'Login',
    'logout': 'Logout',
    'dashboard': 'Dashboard',
    'modules': 'Modules',
    'profile': 'Profile',
    'settings': 'Settings',
    'language': 'Language',
    'english': 'English',
    'urdu': 'Urdu',
    'select_language': 'Select Language',
    'personalized_learning': 'Personalized Learning',
    'chapter_progress': 'Chapter Progress',
    'complete': 'Complete',
    'incomplete': 'Incomplete',
    'continue_learning': 'Continue Learning',
    'start_learning': 'Start Learning',
    'save_preferences': 'Save Preferences',
    'learning_preferences': 'Learning Preferences',
    'experience_level': 'Experience Level',
    'beginner': 'Beginner',
    'intermediate': 'Intermediate',
    'advanced': 'Advanced',
    'learning_path': 'Learning Path',
    'self_paced': 'Self-Paced',
    'guided': 'Guided',
    'intensive': 'Intensive',
    'module_1_title': 'Module 1: The Robotic Nervous System (ROS 2)',
    'module_2_title': 'Module 2: The Digital Twin (Gazebo & Unity)',
    'module_3_title': 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
    'module_4_title': 'Module 4: Vision-Language-Action (VLA)',
    'capstone_project': 'Capstone Project',
    'autonomous_humanoid': 'Autonomous Humanoid Robot',
    'voice_control': 'Voice Control',
    'navigation': 'Navigation',
    'manipulation': 'Manipulation',
    'safety_behaviors': 'Safety & Fallback Behaviors',
    'progress_tracking': 'Progress Tracking',
    'analytics': 'Analytics',
    'personalized_recommendations': 'Personalized Recommendations',
    'email': 'Email',
    'password': 'Password',
    'sign_up': 'Sign Up',
    'sign_in': 'Sign In',
    'create_account': 'Create Account',
    'user_login': 'User Login',
    'account_created_successfully': 'Account created successfully!',
    'login_successful': 'Login successful!',
    'authentication_failed': 'Authentication failed. Please try again.',
    'already_logged_in': 'Already Logged In',
    'welcome_back': 'Welcome back',
    'already_have_account': 'Already have an account? ',
    'no_account': 'Don\'t have an account? ',
    'user_dashboard': 'User Dashboard',
    'user_profile': 'User Profile',
    'please_login_to_view_profile': 'Please login to view your profile',
    'user_info': 'User Information',
    'name': 'Name',
    'learning_preferences': 'Learning Preferences',
    'ai_assistant': 'AI Assistant',
    'ask_robotics_question': 'Ask a robotics question...',
    'send': 'Send'
  },
  ur: {
    // Urdu translations
    'welcome': 'خوش آمدید',
    'login': 'لاگ ان کریں',
    'logout': 'لاگ آوٹ',
    'dashboard': 'ڈیش بورڈ',
    'modules': 'ماڈیولز',
    'profile': 'پروفائل',
    'settings': 'ترتیبات',
    'language': 'زبان',
    'english': 'انگریزی',
    'urdu': 'اردو',
    'select_language': 'زبان منتخب کریں',
    'personalized_learning': 'ذاتی نوعیت کی سیکھنے کی',
    'chapter_progress': 'چیپٹر کی پیشرفت',
    'complete': 'مکمل',
    'incomplete': 'نامکمل',
    'continue_learning': 'سیکھنا جاری رکھیں',
    'start_learning': 'سیکھنا شروع کریں',
    'save_preferences': 'ترجیحات محفوظ کریں',
    'learning_preferences': 'سیکھنے کی ترجیحات',
    'experience_level': 'تجربے کی سطح',
    'beginner': 'شروع کرنے والا',
    'intermediate': 'درمیانی',
    'advanced': 'اعلی درجے کا',
    'learning_path': 'سیکھنے کا راستہ',
    'self_paced': 'خود کی رفتار',
    'guided': 'راہ نما',
    'intensive': 'گہرا',
    'module_1_title': 'ماڈیول 1: روبوٹ کا نروس سسٹم (ROS 2)',
    'module_2_title': 'ماڈیول 2: ڈیجیٹل ٹوئن (گیزبو اور یونٹی)',
    'module_3_title': 'ماڈیول 3: AI روبوٹ براہین (NVIDIA Isaac)',
    'module_4_title': 'ماڈیول 4: ویژن-لینگویج-ایکشن (VLA)',
    'capstone_project': 'کیپ اسٹون پروجیکٹ',
    'autonomous_humanoid': 'خود مختار ہیومنوائڈ روبوٹ',
    'voice_control': 'صوتی کنٹرول',
    'navigation': 'نیویگیشن',
    'manipulation': 'مینوپولیشن',
    'safety_behaviors': 'حفاظت اور واپسی کے رویے',
    'progress_tracking': 'پیشرفت کا جائزہ',
    'analytics': 'تجزیات',
    'personalized_recommendations': 'ذاتی نوعیت کی سفارشات',
    'email': 'ای میل',
    'password': 'پاس ورڈ',
    'sign_up': 'سائن اپ کریں',
    'sign_in': 'سائن ان کریں',
    'create_account': 'اکاؤنٹ بنائیں',
    'user_login': 'صارف لاگ ان',
    'account_created_successfully': 'اکاؤنٹ کامیابی سے بن گیا!',
    'login_successful': 'لاگ ان کامیاب ہوا!',
    'authentication_failed': 'توثیق ناکام ہو گئی۔ براہ کرم دوبارہ کوشش کریں۔',
    'already_logged_in': 'پہلے سے لاگ ان ہیں',
    'welcome_back': 'واپس مبارک ہو',
    'already_have_account': 'پہلے سے اکاؤنٹ موجود ہے؟ ',
    'no_account': 'اکاؤنٹ نہیں ہے؟ ',
    'user_dashboard': 'صارف ڈیش بورڈ',
    'user_profile': 'صارف پروفائل',
    'please_login_to_view_profile': 'اپنا پروفائل دیکھنے کے لیے براہ کرم لاگ ان کریں',
    'user_info': 'صارف کی معلومات',
    'name': 'نام',
    'learning_preferences': 'سیکھنے کی ترجیحات',
    'ai_assistant': 'AI اسسٹنٹ',
    'ask_robotics_question': 'روبوٹس کے بارے میں سوال پوچھیں...',
    'send': 'بھیجیں'
  }
};

export const LanguageProvider: React.FC<{ children: ReactNode }> = ({ children }) => {
  const [currentLanguage, setCurrentLanguage] = useState<string>(() => {
    // Get language from localStorage or default to 'en'
    const savedLanguage = localStorage.getItem('language');
    return savedLanguage || 'en';
  });

  const availableLanguages = ['en', 'ur'];

  const changeLanguage = (lang: string) => {
    if (availableLanguages.includes(lang)) {
      setCurrentLanguage(lang);
      localStorage.setItem('language', lang);
    }
  };

  const t = (key: string): string => {
    // Return translation if available, otherwise return key
    return translations[currentLanguage]?.[key] || key;
  };

  const value = {
    currentLanguage,
    availableLanguages,
    changeLanguage,
    t
  };

  return (
    <LanguageContext.Provider value={value}>
      {children}
    </LanguageContext.Provider>
  );
};

export const useLanguage = () => {
  const context = useContext(LanguageContext);
  if (context === undefined) {
    throw new Error('useLanguage must be used within a LanguageProvider');
  }
  return context;
};