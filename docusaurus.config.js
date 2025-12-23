// @ts-check
const lightCodeTheme = require('prism-react-renderer').themes.github;
const darkCodeTheme = require('prism-react-renderer').themes.dracula;

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'From Digital Intelligence to Embodied Autonomous Systems',
  favicon: 'img/favicon.ico',

  // ===============================
  // SITE CONFIG (VERCEL SAFE)
  // ===============================
  url: 'https://physical-ai-robotics-textbook.vercel.app',
  baseUrl: '/',

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  // ===============================
  // INTERNATIONALIZATION
  // ===============================
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  // ===============================
  // PRESETS (FIXED – NO EMPTY PRESET)
  // ===============================
  presets: [
    [
      'classic',
      {
        docs: {
          routeBasePath: '/', // docs at root
          sidebarPath: require.resolve('./docusaurus_sidebar.js'),
          editUrl: undefined,
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
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'textbookSidebar', // MUST match sidebar file
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
            { label: 'Introduction', to: '/' },
            { label: 'Part 1: Foundations', to: '/part-1/PART_1_overview' },
            { label: 'Part 2: ROS 2', to: '/part-2/PART_2_overview' },
          ],
        },
        {
          title: 'References',
          items: [
            { label: 'Glossary', to: '/glossary' },
            { label: 'RAG Index', to: '/rag_index' },
            { label: 'Resources', to: '/resources' },
          ],
        },
      ],
      copyright:
        `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics.`,
    },

    prism: {
      theme: lightCodeTheme,
      darkTheme: darkCodeTheme,
      additionalLanguages: [
        'bash',
        'diff',
        'json',
        'python',
        'cpp',
        'yaml',
        'latex',
        'markup',
      ],
    },
  },
};

module.exports = config;
