---
id: 3
title: Book Content Spec
stage: spec
date: 2025-12-07
surface: agent
model: claude-sonnet-4-5-20250929
feature: book-content
branch: 002-book-content
user: user
command: /sp.specify
labels: [book, content, textbook, robotics, specification]
links:
  spec: specs/002-book-content/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/002-book-content/spec.md
 - specs/002-book-content/checklists/requirements.md
 - history/prompts/002-book-content/
tests:
 - null
---

## Prompt

/sp.specs.details — Book Content & Writing Specifications
Project Title

Physical AI & Humanoid Robotics — Teaching Textbook

Purpose

This specification defines the detailed structure, content requirements, and constraints for the development of a full AI-native technical textbook that teaches students how to design, simulate, program, and deploy humanoid robots using ROS 2, Gazebo, Unity, NVIDIA Isaac, and LLM-based VLA systems.

SECTION 1 — BOOK STRUCTURE REQUIREMENTS
1. Book Flow

Book must follow this strict layout:

Preface (Written for students and parents)

Instructor Guide

Course Roadmap & Timeline

Module-Based Learning Units

Capstone Guide

Glossary

Reference Section

Appendix Materials

RAG-Indexed Data Folder

SECTION 2 — MODULE-LEVEL SPECIFICATION

Your book will contain 4 modules, matching the course.

Module 1 Specification
Module Name

The Robotic Nervous System (ROS 2)

Module Goals

Students gain working knowledge of controlling robots via ROS 2 architecture.

Required Learning Outcomes

After module completion, student must be able to:

Build ROS packages

Create nodes, publish & subscribe

Connect Python programs to ROS controllers

Understand robot URDF structures

Required Chapters
Chapter 1 — Introduction to ROS 2 Architecture

Must cover:

Nodes, Topics, Services, Actions

ROS graph

Explanation diagrams

Real industrial use cases

Output artifact:
✔ ROS graph visual diagram

Chapter 2 — Writing Nodes with rclpy

Must include:

Minimal publisher

Minimal subscriber

Launch files

Code explanation

Output artifact:
✔ Repo folder /code/ros-basic/

Chapter 3 — URDF for Humanoids

Include mandatory elements:

Links & joints

Inertial configuration

Transmission elements

Sensors & actuators

Output artifact:
✔ humanoid.urdf.xacro

Chapter 4 — Integrating Python Agents

Must demonstrate:

Agent → ROS bridge

GPT-generated task → executable ROS code

Output artifact:
✔ Completed project output demonstration

Module 2 Specification
Module Name

Digital Twin Simulation

Tools:
🔹 Gazebo
🔹 Unity

Learning Goals

Students learn physics-based robot simulation.

Required Chapters
Chapter 5 — Setting Up Gazebo

Topics must include:

Setting world

Robot spawning

Collision systems

Output:
✔ Sample file /simulation/basic.world

Chapter 6 — Sensors Simulation

Must teach:

Voxels

Depth maps

IMU drifting models

Real-time collision handling

Output:
✔ Gazebo plugin demonstrations

Chapter 7 — Unity for Robotics

Mandatory coverage:

Unity humanoid rigs

Motion blending

Ray-interaction events

Output:
✔ Unity sample project folder

Module 3 Specification
Module Name

The AI-Robot Brain

Tools must include:

NVIDIA Isaac Sim

Isaac ROS

Navigation (Nav2)

Required Chapters
Chapter 8 — Isaac Simulation Pipeline

Must include:

USD scene format

Real-world texture rendering

Camera feeds from Isaac

Asset deliverable:
✔ /assets/environment.usd

Chapter 9 — Autonomous Navigation

Required concepts:
✔ mapping
✔ localization
✔ path execution

Output:
✔ Nav2 tutorial example

Chapter 10 — Perception Pipeline

Include:

Detection pipeline

Segmentation pipeline

Pose estimation

Output:
✔ Jupyter notebook

Module 4 Specification
Module Name

Vision-Language-Action Systems

Required Chapters
Chapter 11 — Voice-to-Action Agents

Must integrate:

Whisper

GPT planning

ROS commands

Output:
✔ CLI demonstration video script

Chapter 12 — Cognitive Task Planning

Examples must include:
User command → Task breakdown → Robot execution

Case studies:

"Clean the desk"

"Pick the book"

"Navigate to room"

Chapter 13 — Capstone Execution

Final pipeline must be presented visually:

VOICE ⟶ PLAN ⟶ NAVIGATE ⟶ RECOGNIZE OBJECT ⟶ MANIPULATE

Output:
✔ Capstone submission checklist
✔ Demo recording instructions

SECTION 3 — TECHNICAL REQUIREMENTS
Every Chapter Must Contain
Component	Description
Concepts	Illustrated diagrams
Code Samples	Real executable code
Real-world mapping	Industry relevance
Quiz	7-10 questions
Summary sheet	For revision
Data for RAG	Marked blocks
Source Formatting

All content written in Markdown (.md)

Code fenced with language detection

APA citation formatting

Diagrams named using diagram-xx.png

AI-generated text must be marked via :::ai-block

SECTION 4 — RAG SPECIFICATION

You must prepare data for indexing.

Each chapter must produce:

chapter-title.rag.txt


Containing:

Definitions

Key formulae

Command usage

Safety notes

Troubleshooting steps

RAG files stored under:

/rag/chapters/


Chatbot must answer from user-selected blocks only.

SECTION 5 — EVALUATION REQUIREMENTS

Your content must map to expected evaluation rubric.

Component	Score
Base Book	100 pts
Agent-based Intelligence	+50 pts
Personalized Experience	+50 pts
Auto-Translate URDU	+50 pts

Alignment Points:
✔ Teaching content satisfies capstone
✔ Implements industry-grade robotics pipeline
✔ Demonstrates physical vs simulation comparison
✔ User outcomes measurable

SECTION 6 — Final Book Deliverables Checklist

📌 When book is complete, it must include:

Content Deliverables

✔ 13 chapters
✔ Instructor guide
✔ Glossary of at least 150 terms
✔ Hardware evaluation guide
✔ Full ROS simulation
✔ Full Isaac simulation

Software Deliverables

✔ Example code folders
✔ Workspace setup
✔ RAG dataset

Presentation Deliverables

✔ Demo recording script
✔ Screenshots
✔ Published GitHub site

FINAL STATEMENT

This specification defines exact structure, content, learning outcomes, and production criteria for your AI-native textbook.

## Response snapshot

Created comprehensive specification for the Physical AI & Humanoid Robotics Textbook content with:

- 3 detailed user stories covering students, instructors, and evaluators
- 10 functional requirements covering book structure, modules, chapters, technical artifacts, and deliverables
- 6 measurable success criteria
- 4 key entities defined
- Quality checklist created and validated

Specification addresses all requirements from user input including 4 modules with 13 chapters, RAG indexing, technical artifacts, and evaluation criteria.

## Outcome

- ✅ Impact: Created complete specification for book content with detailed module structure
- 🧪 Tests: null
- 📁 Files: specs/002-book-content/spec.md, specs/002-book-content/checklists/requirements.md, history/prompts/002-book-content/
- 🔁 Next prompts: Ready for /sp.plan or /sp.clarify
- 🧠 Reflection: Specification comprehensively covers all aspects of book content with clear structure and measurable outcomes

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null