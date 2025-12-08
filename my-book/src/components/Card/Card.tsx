import React from 'react';
import clsx from 'clsx';
import styles from './Card.module.css';

interface CardProps {
  title?: string;
  description?: string;
  children?: React.ReactNode;
  className?: string;
  imageUrl?: string;
  linkUrl?: string;
  variant?: 'default' | 'primary' | 'secondary' | 'feature';
  onClick?: () => void;
}

const Card: React.FC<CardProps> = ({
  title,
  description,
  children,
  className,
  imageUrl,
  linkUrl,
  variant = 'default',
  onClick,
}) => {
  const cardClasses = clsx(
    styles.card,
    styles[variant],
    className
  );

  const handleClick = () => {
    if (onClick) {
      onClick();
    }
  };

  const CardContent = (
    <div className={styles.cardContent} onClick={handleClick}>
      {imageUrl && (
        <div className={styles.cardImage}>
          <img src={imageUrl} alt={title} />
        </div>
      )}
      <div className={styles.cardBody}>
        {title && <h3 className={styles.cardTitle}>{title}</h3>}
        {description && <p className={styles.cardDescription}>{description}</p>}
        {children}
      </div>
    </div>
  );

  if (linkUrl) {
    return (
      <a href={linkUrl} className={cardClasses}>
        {CardContent}
      </a>
    );
  }

  return (
    <div className={cardClasses}>
      {CardContent}
    </div>
  );
};

export default Card;