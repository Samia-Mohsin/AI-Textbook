// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Getting Started',
      items: ['getting-started/installation', 'getting-started/configuration'],
    },
    {
      type: 'category',
      label: 'Authentication',
      items: ['authentication/setup', 'authentication/user-profiles'],
    },
    {
      type: 'category',
      label: 'Personalization',
      items: ['personalization/user-preferences', 'personalization/adaptation'],
    },
    {
      type: 'category',
      label: 'AI Automation',
      items: ['ai-automation/subagents', 'ai-automation/workflows'],
    },
    {
      type: 'category',
      label: 'Localization',
      items: ['localization/multilingual', 'localization/urdu-support'],
    },
    {
      type: 'category',
      label: 'Deployment',
      items: ['deployment/github-pages', 'deployment/ci-cd'],
    },
  ],
};

export default sidebars;