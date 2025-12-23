// docusaurus_sidebar.js
/**
 * AI-Native Physical AI & Humanoid Robotics Textbook
 * Docusaurus Sidebar Configuration
 * 
 * This file defines the navigation structure for the textbook
 * Compatible with Docusaurus 2.x
 * 
 * Generated: 2025-12-21
 */

module.exports = {
  textbookSidebar: [
    {
      type: 'doc',
      id: '00_introduction',
      label: 'Book Introduction',
    },
    {
      type: 'category',
      label: 'PART 1: Foundations of Physical AI',
      collapsed: false,
      items: [
        {
          type: 'doc',
          id: 'part-1/PART_1_overview',
          label: 'Part 1 Overview',
        },
        {
          type: 'doc',
          id: 'part-1/01_physical_ai_fundamentals',
          label: 'Chapter 1: Physical AI Fundamentals',
        },
        {
          type: 'doc',
          id: 'part-1/02_robotics_systems',
          label: 'Chapter 2: Robotics Systems & Embodied Intelligence',
        },
        {
          type: 'doc',
          id: 'part-1/03_sim_to_reality',
          label: 'Chapter 3: From Simulation to Reality (Sim2Real)',
        },
      ],
    },
    {
      type: 'category',
      label: 'PART 2: ROS 2 – The Robotic Nervous System',
      collapsed: false,
      items: [
        {
          type: 'doc',
          id: 'part-2/PART_2_overview',
          label: 'Part 2 Overview',
        },
        {
          type: 'doc',
          id: 'part-2/04_ros2_architecture',
          label: 'Chapter 4: ROS 2 Architecture & Middleware',
        },
        {
          type: 'doc',
          id: 'part-2/05_nodes_topics_services_actions',
          label: 'Chapter 5: Nodes, Topics, Services, and Actions',
        },
        {
          type: 'doc',
          id: 'part-2/06_python_ros2_development',
          label: 'Chapter 6: Python-Based ROS 2 Development with rclpy',
        },
        {
          type: 'doc',
          id: 'part-2/07_agent_ros_communication',
          label: 'Chapter 7: Agent-to-ROS Communication Patterns',
        },
        {
          type: 'doc',
          id: 'part-2/08_urdf_robot_description',
          label: 'Chapter 8: URDF and Robot Description for Humanoids',
        },
      ],
    },
    {
      type: 'category',
      label: 'PART 3: Digital Twin & Simulation',
      collapsed: true,
      items: [
        {
          type: 'doc',
          id: 'part-3/PART_3_overview',
          label: 'Part 3 Overview',
        },
      ],
    },
    {
      type: 'category',
      label: 'PART 4: NVIDIA Isaac – The AI Robot Brain',
      collapsed: true,
      items: [
        {
          type: 'doc',
          id: 'part-4/PART_4_overview',
          label: 'Part 4 Overview',
        },
      ],
    },
    {
      type: 'category',
      label: 'PART 5: Vision-Language-Action (VLA)',
      collapsed: true,
      items: [
        {
          type: 'doc',
          id: 'part-5/PART_5_overview',
          label: 'Part 5 Overview',
        },
      ],
    },
    {
      type: 'category',
      label: 'PART 6: Conversational Humanoid Robots',
      collapsed: true,
      items: [
        {
          type: 'doc',
          id: 'part-6/PART_6_overview',
          label: 'Part 6 Overview',
        },
      ],
    },
    {
      type: 'category',
      label: 'PART 7: Capstone – Autonomous Humanoid System',
      collapsed: true,
      items: [
        {
          type: 'doc',
          id: 'part-7/PART_7_overview',
          label: 'Part 7 Overview',
        },
      ],
    },
    {
      type: 'category',
      label: 'Reference Materials',
      collapsed: false,
      items: [
        {
          type: 'doc',
          id: 'glossary',
          label: 'Technical Glossary',
        },
        {
          type: 'doc',
          id: 'rag_index',
          label: 'RAG Index & Semantic Map',
        },
        {
          type: 'doc',
          id: 'resources',
          label: 'Additional Resources',
        },
      ],
    },
  ],
};
