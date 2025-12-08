import React, { useState } from 'react';
import { useUser } from '../contexts/UserContext';
import { useLanguage } from '../contexts/LanguageContext';
import Layout from '@theme/Layout';
import styles from './login.module.css';

const LoginPage = () => {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [isSignUp, setIsSignUp] = useState(false);
  const { login, user } = useUser();
  const { t } = useLanguage();

  const handleSubmit = async (e) => {
    e.preventDefault();
    try {
      await login(email, password);
      // Redirect or show success message
      alert(isSignUp ? t('account_created_successfully') : t('login_successful'));
    } catch (error) {
      console.error('Authentication error:', error);
      alert(t('authentication_failed'));
    }
  };

  if (user) {
    return (
      <Layout title={t('dashboard')} description={t('user_dashboard')}>
        <div className={styles.loginContainer}>
          <h1>{t('already_logged_in')}</h1>
          <p>{t('welcome_back')}, {user.name}!</p>
        </div>
      </Layout>
    );
  }

  return (
    <Layout title={isSignUp ? t('sign_up') : t('login')} description={isSignUp ? t('create_account') : t('user_login')}>
      <div className={styles.loginContainer}>
        <div className={styles.loginForm}>
          <h1>{isSignUp ? t('sign_up') : t('login')}</h1>

          <form onSubmit={handleSubmit}>
            <div className={styles.formGroup}>
              <label htmlFor="email">{t('email')}:</label>
              <input
                type="email"
                id="email"
                value={email}
                onChange={(e) => setEmail(e.target.value)}
                required
                className={styles.inputField}
              />
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="password">{t('password')}:</label>
              <input
                type="password"
                id="password"
                value={password}
                onChange={(e) => setPassword(e.target.value)}
                required
                className={styles.inputField}
              />
            </div>

            <button type="submit" className={styles.submitButton}>
              {isSignUp ? t('create_account') : t('login')}
            </button>
          </form>

          <div className={styles.switchMode}>
            <p>
              {isSignUp ? t('already_have_account') : t('no_account')}
              <button
                onClick={() => setIsSignUp(!isSignUp)}
                className={styles.switchButton}
              >
                {isSignUp ? t('sign_in') : t('sign_up')}
              </button>
            </p>
          </div>
        </div>
      </div>
    </Layout>
  );
};

export default LoginPage;