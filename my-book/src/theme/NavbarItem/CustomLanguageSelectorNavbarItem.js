import React from 'react';
import {useNavbarItems} from '@docusaurus/theme-common';
import NavbarItem from '@theme/NavbarItem';
import LanguageSelector from '@site/src/components/LanguageSelector';

// This component renders the language selector in the navbar
export default function CustomLanguageSelectorNavbarItem() {
  return <LanguageSelector />;
}