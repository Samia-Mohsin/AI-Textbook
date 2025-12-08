import React from 'react';
import { useLanguage } from '../contexts/LanguageContext';
import styles from './languageSelector.module.css';

const LanguageSelector: React.FC = () => {
  const { currentLanguage, availableLanguages, changeLanguage, t } = useLanguage();

  return (
    <div className={styles.languageSelector}>
      <label htmlFor="language-select" className={styles.label}>
        {t('language')}:
      </label>
      <select
        id="language-select"
        value={currentLanguage}
        onChange={(e) => changeLanguage(e.target.value)}
        className={styles.select}
      >
        {availableLanguages.map((lang) => (
          <option key={lang} value={lang}>
            {lang === 'en' ? t('english') : t('urdu')}
          </option>
        ))}
      </select>
      <button
        className={`${styles.flagButton} ${currentLanguage === 'ur' ? styles.active : ''}`}
        onClick={() => changeLanguage('ur')}
        title={t('urdu')}
        aria-label={t('urdu')}
      >
        🇵🇰 {/* Pakistan flag as Urdu is primarily spoken there */}
      </button>
      <button
        className={`${styles.flagButton} ${currentLanguage === 'en' ? styles.active : ''}`}
        onClick={() => changeLanguage('en')}
        title={t('english')}
        aria-label={t('english')}
      >
        🇺🇸 {/* US flag for English */}
      </button>
    </div>
  );
};

export default LanguageSelector;