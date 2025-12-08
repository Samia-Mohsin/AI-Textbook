import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import styles from './Button.module.css';

interface ButtonProps {
  children: React.ReactNode;
  variant?: 'primary' | 'secondary' | 'outline' | 'ghost';
  size?: 'small' | 'medium' | 'large';
  href?: string;
  onClick?: () => void;
  disabled?: boolean;
  className?: string;
  type?: 'button' | 'submit' | 'reset';
  target?: string;
  rel?: string;
}

const Button: React.FC<ButtonProps> = ({
  children,
  variant = 'primary',
  size = 'medium',
  href,
  onClick,
  disabled = false,
  className,
  type = 'button',
  target,
  rel,
}) => {
  const buttonClasses = clsx(
    styles.button,
    styles[variant],
    styles[size],
    { [styles.disabled]: disabled },
    className
  );

  const handleClick = (e: React.MouseEvent) => {
    if (disabled) {
      e.preventDefault();
      return;
    }
    if (onClick) {
      onClick();
    }
  };

  if (href) {
    return (
      <Link
        to={href}
        className={buttonClasses}
        onClick={handleClick}
        target={target}
        rel={rel}
        aria-disabled={disabled}
      >
        {children}
      </Link>
    );
  }

  return (
    <button
      className={buttonClasses}
      onClick={handleClick}
      disabled={disabled}
      type={type}
      aria-disabled={disabled}
    >
      {children}
    </button>
  );
};

export default Button;