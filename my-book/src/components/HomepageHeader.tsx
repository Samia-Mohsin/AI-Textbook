import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './HomepageHeader.module.css';

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className={styles.logoContainer}>
          <div className={styles.robotIcon}>🤖</div>
          <h1 className={clsx('hero__title', styles.mainTitle)}>{siteConfig.title}</h1>
        </div>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/modules">
            Start Learning
          </Link>
          <Link
            className="button button--primary button--lg margin-left--md"
            to="/dashboard">
            View Progress
          </Link>
        </div>
      </div>
    </header>
  );
}

export default HomepageHeader;