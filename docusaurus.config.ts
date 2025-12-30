import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Complete Educational Resource for Advanced Robotics Systems',
  favicon: 'img/robotics-logo.svg',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://kashankamboh.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/Physical-AI-Humanoid-Robotics-Book/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'KashanKamboh', // Usually your GitHub org/user name.
  projectName: 'Physical-AI-Humanoid-Robotics-Book', // Usually your repo name.
  trailingSlash: false,

  onBrokenLinks: 'throw',
  markdown: {
    format: 'mdx',
    mermaid: false,
    hooks: {
      onBrokenMarkdownLinks: 'warn',
    },
  },

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'], // Adding Urdu for multi-language support
    localeConfigs: {
      ur: {
        direction: 'rtl', // Right-to-left layout for Urdu
        htmlLang: 'ur',
      },
      en: {
        direction: 'ltr', // Left-to-right layout for English
        htmlLang: 'en',
      },
    },
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl: 'https://github.com/KashanKamboh/Physical-AI-Humanoid-Robotics-Book.git/edit/main/',
          showLastUpdateAuthor: false,
          showLastUpdateTime: false,
          lastVersion: 'current',
          admonitions: {},
          remarkPlugins: [
            [require('@docusaurus/remark-plugin-npm2yarn'), {sync: true}],
          ],
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl: 'https://github.com/KashanKamboh/Physical-AI-Humanoid-Robotics-Book/edit/main/',
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

  plugins: [
    // RAG Chatbot Integration Plugin
    [
      '@docusaurus/plugin-client-redirects',
      {
        redirects: [
          {
            to: '/docs/intro',
            from: ['/docs', '/home', '/welcome'],
          },
        ],
      },
    ],
    // Add the chatbot to all pages using a custom plugin
    async function myPlugin(context, options) {
      return {
        name: 'docusaurus-plugin-rag-chatbot',
        getClientModules() {
          return [
            require.resolve('./src/clientModules/chatbotInjector.js'),
          ];
        },
      };
    },
  ],


  themeConfig: {
    // Replace with your project's social card
    image: 'img/robotics-social-card.jpg',
    colorMode: {
      defaultMode: 'dark',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Physical AI & Humanoid Robotics Logo',
        src: 'img/robotics-logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Modules',
        },
        {to: '/blog', label: 'Blog', position: 'left'},
        {
          type: 'localeDropdown',
          position: 'right',
        },
        {
          href: 'https://github.com/KashanKamboh/Physical-AI-Humanoid-Robotics-Book.git',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Modules',
          items: [
            {
              label: 'Module 1: Robotic Nervous System',
              to: '/docs/module-1/intro',
            },
            {
              label: 'Module 2: Digital Twin',
              to: '/docs/module-2/intro',
            },
            {
              label: 'Module 3: AI-Robot Brain',
              to: '/docs/module-3/intro',
            },
            {
              label: 'Module 4: Vision-Language-Action',
              to: '/docs/module-4/intro',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'GitHub Repository',
              href: 'https://github.com/KashanKamboh/Physical-AI-Humanoid-Robotics-Book.git',
            },
            {
              label: 'ROS Documentation',
              href: 'https://docs.ros.org/',
            },
            {
              label: 'NVIDIA Isaac',
              href: 'https://developer.nvidia.com/isaac',
            },
            {
              label: 'Unity Robotics',
              href: 'https://unity.com/solutions/industries/robotics',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'Constitution',
              to: '/docs/constitution',
            },
            {
              label: 'Course Roadmap',
              to: '/docs/course-roadmap',
            },
            {
              label: 'Reference Materials',
              to: '/docs/reference/glossary',
            },
            {
              label: 'Capstone Project',
              to: '/docs/capstone/intro',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Book. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['python', 'bash', 'json', 'yaml', 'cpp', 'csharp', 'java', 'docker'],
    },
    algolia: {
      // The application ID provided by Algolia
      appId: 'YOUR_APP_ID',
      // Public API key: it is safe to commit it
      apiKey: 'YOUR_SEARCH_API_KEY',
      indexName: 'homanoid-robotics-book',
      // Optional: see doc section below
      contextualSearch: true,
      // Optional: Specify domains where the navigation should occur through window.location instead on history.push. Useful when our Algolia config crawls multiple documentation sites and we want to navigate with window.location.href to them.
      externalUrlRegex: 'external\\.com|domain\\.com',
      // Optional: Replace parts of the item URLs from Algolia. Useful when using the same search index for multiple deployments using a different baseUrl. You can use regexp or string in the `from` param. For example: localhost:3000 vs myCompany.com/docs
      replaceSearchResultPathname: {
        from: '/docs/', // or as RegExp: /\/docs\//
        to: '/docs/',
      },
      // Optional: Algolia search parameters
      searchParameters: {},
      // Optional: path for search page that enabled by default (`false` to disable it)
      searchPagePath: 'search',
    },
  } satisfies Preset.ThemeConfig,
};

export default config;