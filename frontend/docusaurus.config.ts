import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI & Humanoid Robotics Educational Platform',
  tagline: 'An interactive educational platform for learning Physical AI and Humanoid Robotics with multilingual support',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    // v4: true, // Commented out due to scroll controller issues
  },

  // Set the production url of your site here
  url: 'https://humanoid-robotics-book.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'humanoid-robotics-book', // Usually your GitHub org/user name.
  projectName: 'humanoid-robotics-book.github.io', // Usually your repo name.

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'], // Adding Urdu locale for localization support
  },

  // Configuration to handle broken links during build
  onBrokenLinks: 'ignore',
  onBrokenMarkdownLinks: 'warn',

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/humanoid-robotics-book/humanoid-robotics-book',
          // Use default routeBasePath to avoid conflicts with homepage
          showLastUpdateTime: true,
          showLastUpdateAuthor: true,
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/humanoid-robotics-book/humanoid-robotics-book',
          // Useful options to enforce blogging best practices
          onInlineTags: 'warn',
          onInlineAuthors: 'warn',
          onUntruncatedBlogPosts: 'warn',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themes: [
    // Add additional themes here if needed
    // For example, @docusaurus/theme-live-codeblock for live code blocks
  ],

  plugins: [
  ],


  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      defaultMode: 'light',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },
    scripts: [
      {
        src: '/js/language-switcher.js',
        async: true,
      },
      // Add script for the chatbot widget
      {
        src: '/js/chatbot.js',
        async: true,
      },
    ],
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Physical AI & Humanoid Robotics Logo',
        src: 'img/logo.jpg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'bookSidebar',
          position: 'left',
          label: 'Book',
        },
        {
          type: 'localeDropdown',
          position: 'right',
        },
        {
          type: 'dropdown',
          position: 'right',
          label: '🌐 Translate',
          items: [
            {
              label: 'English',
              to: '/?lang=en',
              className: 'language-switcher-item',
            },
            {
              label: 'اردو',
              to: '/?lang=ur',
              className: 'language-switcher-item',
            }
          ],
        },
        {
          href: 'https://github.com/humanoid-robotics-book/humanoid-robotics-book',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Documentation',
          items: [
            {
              label: 'Getting Started',
              to: '/docs/intro',
            },
            {
              label: 'Chapter 1',
              to: '/docs/chapter1',
            },
            {
              label: 'Module 1: ROS2',
              to: '/docs/module-1-ros2/chapter-1-architecture',
            },
            {
              label: 'Module 2: Simulation',
              to: '/docs/module-2-simulation/chapter-1-gazebo-setup',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/humanoid-robotics-book/humanoid-robotics-book',
            },
            {
              label: 'Discord',
              href: 'https://discord.gg/humanoid-robotics',
            },
            {
              label: 'Research Papers',
              href: 'https://humanoid-robotics-book.github.io/research',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'Capstone Project',
              to: '/docs/capstone-project/intro',
            },
            {
              label: 'Module 3: AI Brain',
              to: '/docs/module-3-ai-brain/chapter-1-isaac-sim-setup',
            },
            {
              label: 'Module 4: VLA Systems',
              to: '/docs/module-4-vla/chapter-1-vla-overview',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Educational Platform. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['python', 'bash', 'json', 'yaml', 'docker', 'powershell'],
    },
    algolia: {
      // The application ID provided by Algolia
      appId: 'YOUR_ALGOLIA_APP_ID',
      // Public API key: it is safe to commit it
      apiKey: 'YOUR_ALGOLIA_API_KEY',
      indexName: 'humanoid-robotics-book',
      contextualSearch: true,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
