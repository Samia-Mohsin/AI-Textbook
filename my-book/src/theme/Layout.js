import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import { UserProvider } from '../contexts/UserContext';
import { LanguageProvider } from '../contexts/LanguageContext';

// This component wraps the Docusaurus Layout with our contexts
export default function LayoutWrapper(props) {
  return (
    <UserProvider>
      <LanguageProvider>
        <OriginalLayout {...props} />
      </LanguageProvider>
    </UserProvider>
  );
}