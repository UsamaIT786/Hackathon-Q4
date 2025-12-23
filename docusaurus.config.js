// @ts-check
const lightCodeTheme = require('prism-react-renderer').themes.github;
const darkCodeTheme = require('prism-react-renderer').themes.dracula;

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'From Digital Intelligence to Embodied Autonomous Systems',
  favicon: 'img/favicon.ico',

  // ===============================
  // REQUIRED SITE CONFIG (FIXED)
  // ===============================
  url: 'http://localhost:3000',
  baseUrl: '/',

  onBrokenLinks: 'warn',

  // ===============================
  // MARKDOWN CONFIG
  // ===============================
  markdown: {
    hooks: {
      onBrokenMarkdownLinks: 'warn',
    },
  },

  // ===============================
  // INTERNATIONALIZATION
  // ===============================
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  // ===============================
  // PRESETS
  // ===============================
  presets: [
    [
      'classic',
      {
        docs: {
          routeBasePath: '/', // ⭐ IMPORTANT: docs served at root
          sidebarPath: require.resolve('./docusaurus_sidebar.js'),
          editUrl: undefined,

          // ⭐ MDX SAFE MODE (IMPORTANT FOR YOUR ERRORS)
          remarkPlugins: [require('remark-gfm')],
          rehypePlugins: [],
        },
        blog: false,
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      },
    ],
  ],

  // ===============================
  // THEME CONFIG
  // ===============================
  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      defaultMode: 'light',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },

    navbar: {
      title: 'Physical AI Textbook',
      logo: {
        alt: 'Physical AI Logo',
        src: 'img/logo.svg',
      },
      hideOnScroll: false,
      style: 'primary',
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'textbookSidebar', // ⭐ MUST MATCH sidebars.js
          position: 'left',
          label: '📚 Chapters',
        },
        {
          href: 'https://github.com',
          label: '💻 GitHub',
          position: 'right',
        },
        {
          type: 'search',
          position: 'right',
        },
      ],
    },

    footer: {
      style: 'dark',
      links: [
        {
          title: 'Content',
          items: [
            {
              label: 'Introduction',
              to: '/',
            },
            {
              label: 'Part 1: Foundations',
              to: '/part-1/PART_1_overview',
            },
            {
              label: 'Part 2: ROS 2',
              to: '/part-2/PART_2_overview',
            },
          ],
        },
        {
          title: 'References',
          items: [
            {
              label: 'Glossary',
              to: '/glossary',
            },
            {
              label: 'RAG Index',
              to: '/rag_index',
            },
            {
              label: 'Resources',
              to: '/resources',
            },
          ],
        },
      ],
      copyright:
        `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics. All rights reserved.`,
    },

    prism: {
      theme: lightCodeTheme,
      darkTheme: darkCodeTheme,
      additionalLanguages: ['bash', 'diff', 'json', 'python', 'cpp', 'yaml', 'latex', 'markup'],
      magicComments: [
        {
          className: 'theme-code-block-highlighted-line',
          line: 'highlight-next-line',
          block: { start: 'highlight-start', end: 'highlight-end' },
        },
      ],
    },
  },
};

module.exports = config;
