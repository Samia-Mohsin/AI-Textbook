// @ts-check
import { themes as prismThemes } from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics Book',
  tagline:
    'An interactive educational platform for learning Physical AI and Humanoid Robotics with multilingual support',
  favicon: 'img/favicon.ico',

  url: 'https://physical-ai-humanoid-robotics-platform.vercel.app',
  baseUrl: '/', // ✅ changed for Vercel

  organizationName: 'Samia-Mohsin',
  projectName: 'AI-Textbook',

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  markdown: {
    mermaid: true,
    mdx1Compat: { comments: true, admonitions: true, headingIds: true },
  },

  i18n: { defaultLocale: 'en', locales: ['en', 'ur'] },

  presets: [
    [
      'classic',
      {
        docs: { sidebarPath: './sidebars.js', editUrl: 'https://github.com/Samia-Mohsin/AI-Textbook/tree/main/' },
        blog: { showReadingTime: true, editUrl: 'https://github.com/Samia-Mohsin/AI-Textbook/tree/main/' },
        theme: { customCss: './src/css/custom.css' },
      },
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    colorMode: { defaultMode: 'light', disableSwitch: false, respectPrefersColorScheme: true },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['python', 'bash', 'json', 'yaml', 'docker', 'powershell'],
    },
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: { alt: 'Physical AI & Humanoid Robotics Logo', src: 'img/logo.png' },
      items: [
        { type: 'docSidebar', sidebarId: 'bookSidebar', position: 'left', label: 'Book' },
        { type: 'localeDropdown', position: 'right' },
        { href: 'https://github.com/Samia-Mohsin/AI-Textbook', label: 'GitHub', position: 'right' },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Documentation',
          items: [
            { label: 'Getting Started', to: '/docs/intro' },
            { label: 'Chapter 1', to: '/docs/chapter1' },
            { label: 'Module 1: ROS2', to: '/docs/module-1-ros2/chapter-1-architecture' },
            { label: 'Module 2: Simulation', to: '/docs/module-2-simulation/chapter-1-gazebo-setup' },
          ],
        },
        {
          title: 'Community',
          items: [
            { label: 'GitHub', href: 'https://github.com/Samia-Mohsin/AI-Textbook' },
            { label: 'Discord', href: 'https://discord.gg/humanoid-robotics' },
          ],
        },
        {
          title: 'More',
          items: [
            { label: 'Capstone Project', to: '/docs/capstone-project/intro' },
            { label: 'Module 3: AI Brain', to: '/docs/module-3-ai-brain/chapter-1-isaac-sim-setup' },
            { label: 'Module 4: VLA Systems', to: '/docs/module-4-vla/chapter-1-vla-overview' },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Educational Platform. Built with Docusaurus.`,
    },
  },
};

export default config;
