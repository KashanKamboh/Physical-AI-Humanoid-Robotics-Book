import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Preface',
      items: ['intro', 'constitution', 'instructor-guide', 'course-roadmap'],
    },
    {
      type: 'category',
      label: 'Module 1: Robotic Nervous System (ROS 2)',
      items: [
        'module-1/intro',
        'module-1/chapter-1',
        'module-1/chapter-2',
        'module-1/chapter-3',
        'module-1/chapter-4',
        'module-1/ai-notes',
        'module-1/research-source'
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin (Gazebo & Unity)',
      items: [
        'module-2/intro',
        'module-2/chapter-5',
        'module-2/chapter-6',
        'module-2/chapter-7',
        'module-2/ai-notes',
        'module-2/research-source'
      ],
    },
    {
      type: 'category',
      label: 'Module 3: AI-Robot Brain (NVIDIA Isaac)',
      items: [
        'module-3/intro',
        'module-3/chapter-8',
        'module-3/chapter-9',
        'module-3/chapter-10',
        'module-3/ai-notes',
        'module-3/research-source'
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-4/intro',
        'module-4/chapter-11',
        'module-4/chapter-12',
        'module-4/chapter-13',
        'module-4/ai-notes',
        'module-4/research-source'
      ],
    },
    {
      type: 'category',
      label: 'Capstone Project',
      items: ['capstone/intro', 'capstone/guide'],
    },
    {
      type: 'category',
      label: 'Reference Materials',
      items: ['reference/glossary', 'reference/technical-standards', 'reference/bibliography'],
    }
  ],
};

export default sidebars;
