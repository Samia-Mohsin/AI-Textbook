import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI & Humanoid Robotics Book',
  tagline: 'An interactive educational platform for learning Physical AI and Humanoid Robotics',
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

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'], // Adding Urdu locale for localization support
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
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
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
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


  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Humanoid Robotics Book',
      logo: {
        alt: 'Humanoid Robotics Book Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'bookSidebar',
          position: 'left',
          label: 'Book',
        },
        {to: '/blog', label: 'Blog', position: 'left'},
        {
          type: 'dropdown',
          position: 'right',
          label: '🌐 EN/UR',
          items: [
            {
              label: 'English',
              href: '#',
              className: 'language-switcher-item',
              'data-lang': 'en'
            },
            {
              label: 'اردو',
              href: '#',
              className: 'language-switcher-item',
              'data-lang': 'ur'
            }
          ],
        },
        {
          to: '/login',
          label: 'Login',
          position: 'right',
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
          title: 'Learning Modules',
          items: [
            {
              label: 'Module 1: ROS2 Fundamentals',
              to: '/docs/module-1-ros2/chapter-1-architecture',
            },
            {
              label: 'Module 2: Simulation',
              to: '/docs/module-2-simulation/chapter-1-gazebo-setup',
            },
            {
              label: 'Module 3: AI & Brain',
              to: '/docs/module-3-ai-brain/chapter-1-isaac-sim-setup',
            },
            {
              label: 'Module 4: VLA Systems',
              to: '/docs/module-4-vla/chapter-1-vla-overview',
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
              label: 'Contributing',
              to: '/docs/contributing',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Project. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
