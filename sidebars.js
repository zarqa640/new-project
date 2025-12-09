// @ts-check

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.

 @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  // Custom sidebar for the Physical AI Humanoid Robotics project
  tutorialSidebar: [
    'intro',  // Main introduction page
    {
      type: 'category',
      label: 'Project Overview',
      items: [
        'overview',
        'getting-started',
      ],
    },
    {
      type: 'category',
      label: 'System Components',
      items: [
        'components',
        'ros2-integration',
      ],
    },
    {
      type: 'category',
      label: 'Development & Maintenance',
      items: [
        'development-maintenance',
      ],
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1/chapter-1',
        'module-1/chapter-2',
        'module-1/chapter-3',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module-2/chapter-1',
        'module-2/chapter-2',
        'module-2/chapter-3',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      items: [
        'module-3/chapter-1',
        'module-3/chapter-2',
        'module-3/chapter-3',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-4/chapter-1',
        'module-4/chapter-2',
        'module-4/chapter-3',
      ],
    },
    // Include tutorial content if needed, but organized properly
    {
      type: 'category',
      label: 'Tutorials',
      items: [
        'tutorial-basics/create-a-document',
        'tutorial-basics/create-a-page',
        'tutorial-basics/create-a-blog-post',
        'tutorial-basics/markdown-features',
        'tutorial-basics/congratulations',
      ],
    },
    {
      type: 'category',
      label: 'Advanced Topics',
      items: [
        'tutorial-extras/manage-docs-versions',
        'tutorial-extras/translate-your-site',
      ],
    },
  ],
};

export default sidebars;