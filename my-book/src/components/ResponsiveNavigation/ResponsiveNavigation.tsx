import React, { useState, useEffect } from 'react';
import Link from '@docusaurus/Link';
import useBaseUrl from '@docusaurus/useBaseUrl';
import { useLocation } from '@docusaurus/router';
import clsx from 'clsx';
import styles from './ResponsiveNavigation.module.css';

interface NavItem {
  label: string;
  to: string;
  type?: string;
  position?: string;
}

const ResponsiveNavigation: React.FC = () => {
  const [isMenuOpen, setIsMenuOpen] = useState(false);
  const [scrolled, setScrolled] = useState(false);
  const location = useLocation();

  const navItems: NavItem[] = [
    { label: 'Home', to: '/' },
    { label: 'Modules', to: '/docs/intro' },
    { label: 'Capstone', to: '/docs/capstone-project/intro' },
    { label: 'Blog', to: '/blog' },
    { label: 'GitHub', to: 'https://github.com/humanoid-robotics-book/humanoid-robotics-book' },
  ];

  useEffect(() => {
    const handleScroll = () => {
      setScrolled(window.scrollY > 10);
    };

    window.addEventListener('scroll', handleScroll);
    return () => window.removeEventListener('scroll', handleScroll);
  }, []);

  useEffect(() => {
    setIsMenuOpen(false);
  }, [location]);

  const toggleMenu = () => {
    setIsMenuOpen(!isMenuOpen);
  };

  return (
    <nav className={clsx(styles.navbar, { [styles.scrolled]: scrolled })}>
      <div className={styles.navContainer}>
        <Link to="/" className={styles.navLogo}>
          <img
            src={useBaseUrl('/img/logo.svg')}
            alt="Humanoid Robotics Book"
            className={styles.logoImg}
          />
          <span className={styles.logoText}>Humanoid Robotics</span>
        </Link>

        <div className={clsx(styles.navMenu, { [styles.navActive]: isMenuOpen })}>
          {navItems.map((item, index) => (
            <Link
              key={index}
              to={item.to}
              className={clsx(
                styles.navLink,
                location.pathname === item.to && styles.navLinkActive
              )}
              target={item.to.startsWith('http') ? '_blank' : undefined}
              rel={item.to.startsWith('http') ? 'noopener noreferrer' : undefined}
            >
              {item.label}
            </Link>
          ))}
        </div>

        <div className={styles.navBtn} onClick={toggleMenu}>
          <div className={clsx(styles.hamburger, { [styles.hamburgerActive]: isMenuOpen })}>
            <span className={styles.bar}></span>
            <span className={styles.bar}></span>
            <span className={styles.bar}></span>
          </div>
        </div>
      </div>
    </nav>
  );
};

export default ResponsiveNavigation;