import React from 'react';
import { useUser } from '../contexts/UserContext';
import { useLanguage } from '../contexts/LanguageContext';
import Layout from '@theme/Layout';
import styles from './profile.module.css';

const ProfilePage = () => {
  const { user, updateUserPreferences } = useUser();
  const { t } = useLanguage();

  if (!user) {
    return (
      <Layout title={t('profile')} description={t('user_profile')}>
        <div className={styles.profileContainer}>
          <h1>{t('profile')}</h1>
          <p>{t('please_login_to_view_profile')}</p>
        </div>
      </Layout>
    );
  }

  const handlePreferenceChange = (preferenceType: string, value: string) => {
    updateUserPreferences({ [preferenceType]: value });
  };

  return (
    <Layout title={t('profile')} description={t('user_profile')}>
      <div className={styles.profileContainer}>
        <div className={styles.profileCard}>
          <h1>{t('profile')}</h1>

          <div className={styles.userInfo}>
            <h2>{t('user_info')}</h2>
            <p><strong>{t('name')}:</strong> {user.name}</p>
            <p><strong>{t('email')}:</strong> {user.email}</p>
          </div>

          <div className={styles.userPreferences}>
            <h2>{t('learning_preferences')}</h2>

            <div className={styles.preferenceItem}>
              <label>{t('experience_level')}:</label>
              <select
                value={user.preferences.experienceLevel}
                onChange={(e) => handlePreferenceChange('experienceLevel', e.target.value)}
                className={styles.preferenceSelect}
              >
                <option value="beginner">{t('beginner')}</option>
                <option value="intermediate">{t('intermediate')}</option>
                <option value="advanced">{t('advanced')}</option>
              </select>
            </div>

            <div className={styles.preferenceItem}>
              <label>{t('learning_path')}:</label>
              <select
                value={user.preferences.learningPath}
                onChange={(e) => handlePreferenceChange('learningPath', e.target.value)}
                className={styles.preferenceSelect}
              >
                <option value="self_paced">{t('self_paced')}</option>
                <option value="guided">{t('guided')}</option>
                <option value="intensive">{t('intensive')}</option>
              </select>
            </div>

            <div className={styles.preferenceItem}>
              <label>{t('language')}:</label>
              <select
                value={user.preferences.language}
                onChange={(e) => handlePreferenceChange('language', e.target.value)}
                className={styles.preferenceSelect}
              >
                <option value="en">{t('english')}</option>
                <option value="ur">{t('urdu')}</option>
              </select>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
};

export default ProfilePage;