import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '5ff'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', '5ba'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', 'a2b'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', 'c3c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '156'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '88c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', '000'),
    exact: true
  },
  {
    path: '/',
    component: ComponentCreator('/', '03c'),
    routes: [
      {
        path: '/',
        component: ComponentCreator('/', '46e'),
        routes: [
          {
            path: '/',
            component: ComponentCreator('/', '388'),
            routes: [
              {
                path: '/404',
                component: ComponentCreator('/404', 'adb'),
                exact: true
              },
              {
                path: '/glossary',
                component: ComponentCreator('/glossary', '27d'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/introduction',
                component: ComponentCreator('/introduction', '77b'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/part-1/PART_1_overview',
                component: ComponentCreator('/part-1/PART_1_overview', '0b9'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/part-1/physical_ai_fundamentals',
                component: ComponentCreator('/part-1/physical_ai_fundamentals', '6b8'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/part-1/robotics_systems',
                component: ComponentCreator('/part-1/robotics_systems', '07c'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/part-1/sim_to_reality',
                component: ComponentCreator('/part-1/sim_to_reality', 'bfc'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/part-2/agent_ros_communication',
                component: ComponentCreator('/part-2/agent_ros_communication', '9d7'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/part-2/nodes_topics_services_actions',
                component: ComponentCreator('/part-2/nodes_topics_services_actions', 'cdc'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/part-2/PART_2_overview',
                component: ComponentCreator('/part-2/PART_2_overview', '187'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/part-2/python_ros2_development',
                component: ComponentCreator('/part-2/python_ros2_development', '9f3'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/part-2/ros2_architecture',
                component: ComponentCreator('/part-2/ros2_architecture', '060'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/part-2/urdf_robot_description',
                component: ComponentCreator('/part-2/urdf_robot_description', '1c4'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/part-3/PART_3_overview',
                component: ComponentCreator('/part-3/PART_3_overview', '525'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/part-4/PART_4_overview',
                component: ComponentCreator('/part-4/PART_4_overview', '636'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/part-5/PART_5_overview',
                component: ComponentCreator('/part-5/PART_5_overview', 'f2b'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/part-6/PART_6_overview',
                component: ComponentCreator('/part-6/PART_6_overview', 'bf4'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/part-7/PART_7_overview',
                component: ComponentCreator('/part-7/PART_7_overview', '7b7'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/rag_index',
                component: ComponentCreator('/rag_index', '9c1'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/resources',
                component: ComponentCreator('/resources', '8c8'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/UI_COMPONENTS_SHOWCASE',
                component: ComponentCreator('/UI_COMPONENTS_SHOWCASE', 'f66'),
                exact: true
              },
              {
                path: '/',
                component: ComponentCreator('/', 'c48'),
                exact: true
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
