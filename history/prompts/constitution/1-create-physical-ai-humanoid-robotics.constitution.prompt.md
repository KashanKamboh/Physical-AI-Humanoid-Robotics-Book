---
id: 1
title: Create Physical AI Humanoid Robotics
stage: constitution
date: 2025-12-07
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: user
command: /sp.constitution
labels: [constitution, robotics, textbook]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
 - .specify/templates/plan-template.md
 - .specify/templates/spec-template.md
 - .specify/templates/tasks-template.md
tests:
 - null
---

## Prompt

/sp.constitution — Physical AI & Humanoid Robotics AI-Native Book Project

# Project Title
Textbook for Teaching Physical AI & Humanoid Robotics (AI-Native Edition)

---

# Core Philosophy

This project believes that learning robotics must bridge:

Digital Intelligence → Physical Execution

Meaning:

Students should not only think in AI terms, they must be able to control physical or simulated robots.

Learners must develop mastery over:

✓ Digital twin-based world understanding
✓ Humanoid systems
✓ Real-world motion execution
✓ AI-based perception and reasoning

---

# Constitutional Principles

## 1. Accuracy
All definitions, algorithms, technologies, and robotics concepts must be:

✓ fact-checked
✓ sourced from authoritative documents

Accepted source types include:

- ROS Official documentation
- Nvidia Isaac documentation
- Unity & Gazebo engineering docs
- Peer-reviewed university publications
- IEEE robotics conferences
- ACM publications

Content must never be fabricated or estimated incorrectly.

---

## 2. Clarity
Book writing style must match:

→ 3rd-year CS-Engineering level

Clear and stepwise explanations are mandatory.

Every concept must contain:

✓ context
✓ definitions
✓ real examples
✓ minimum one hands-on activity

---

## 3. Reproducibility

Every technical chapter must ensure that students can reproduce the learning experience.

This means:

✓ environment instructions
✓ exact library versions
✓ simulation assets
✓ deployment commands
✓ working code

Repository must contain:

/code
/notebooks
/models
/simulation-worlds
/specs

yaml
Copy code

---

## 4. Rigor

The book MUST contain accurate mathematical, robotics-level reasoning and verifiable technical explanations.

Examples:

🔹 Forward kinematics — correct vectors
🔹 IMU data extraction — correct pipeline
🔹 Perception blocks — accurate architectural boundaries

Weak, generic, inaccurate content = NOT allowed.

---

# Execution Standards

Final book MUST exist in:

✓ Live website (Docusaurus build)
✓ Downloadable PDF with title page
✓ Structured table of contents
✓ Automated references

Submission MUST pass:

✓ 0% plagiarism score
✓ Factual audit
✓ Correct citation formatting

---

# Accepted Citation Format
APA Style Example:

Siddiqi, A., & Fox, D. (2024). ROS2 Real-Time Control Systems. IEEE Robotics Journal, 14(2), 218–229.

Inline:

(Fox & Siddiqi, 2024)

---

# Book Production Rules

Book MUST be generated using specification-driven workflow.

Structure MUST follow:

specs/main.yaml
specs/modules/
specs/chapters/

diff
Copy code

Spec files will contain:

- Title
- Learning outcomes
- Dependencies
- Code plan
- Assessment requirements

Generated content MUST go into:

/docs
/static

yaml
Copy code

---

# System-Level Requirements

The textbook MUST teach:

### Module 1 — ROS Control Layer
Students must:

✓ Understand ROS graph
✓ Write publishers/subscribers
✓ Create sensor subscribers
✓ Work with launch configurations

Deliverable: Working ROS Node controlling something in simulation

---

### Module 2 — Digital Twin
Student must:

✓ Create simulation environment
✓ Spawn humanoid model
✓ Apply kinematics
✓ Visualize motion

Deliverable: Running simulation world file

---

### Module 3 — Nvidia-Isaac Stack
Student must learn:

✓ Isaac Sim
✓ Material assets
✓ Simulation fidelity
✓ Real-time processing

Deliverable: Isaac environment scene

---

### Module 4 — Vision-Language-Action System

Students must:

✓ Give natural language command
✓ Convert it into plan
✓ Convert plan to motion command
✓ Validate execution in simulation

Example accepted command:
"Walk ahead and pick the object"

---

# Minimum Content Requirements Per Chapter

Each chapter must contain:

✓ Learning Outcomes
✓ Key Concepts
✓ Diagram or system block
✓ Minimum 2 runnable code blocks
✓ Lab-style execution steps
✓ Verification section (how you know it worked)

Example verification:

- If system publishes IMU data → screenshot of UI or terminal
- If robot moves → proof in simulation

---

# Required Deliverables to Judge

## Judge Must Receive:

1. Public GitHub Repo
2. Working Live Website
3. Downloadable PDF version
4. Video demo under 90 seconds
5. RAG-enabled chatbot integrated

---

# Bonus Features and Marks

If implemented, these increase scoring:

✓ Agent-Based Generation (+50 marks)
✓ Personalized content mode (+50 marks)
✓ Press-to-translate (Urdu mode) (+50 marks)
✓ Authentication + profile tracking (+50 marks)
✓ Sub-agents for content generation (+50 marks)

---

# Ownership & Academic Honesty Declaration

Submitting this project declares:

→ All material is fact-supported
→ All external sources are referenced
→ No content was fabricated
→ Code and models execute as documented
→ Book meets reproducibility standards

---

This constitution document governs the full execution of this project.
Any deviation must still satisfy accuracy, clarity, reproducibility, and validation requirements.

This is now your official foundational constitution document.
⭐⭐ COPY-PASTE DOCUMENT END ⭐⭐
Aap ise bilkul seedha copy karke paste kar sakti hain — koi formatting issue nahi aayega.

Agar aap chahein to main:

✨ PDF version bhi bana doon
✨ GitHub-style markdown version format kar doon
✨ Sidebar auto-structured layout bhi generate kar doon

Just say:
"PDF bana do"
or
"GitHub format de do"

## Response snapshot

I'll help you update the project constitution based on the user input. Let me first check if there's an existing constitution template to work with.

[Read .specify/memory/constitution.md file content shown]

Now I'll create a todo list to track the steps needed to update the constitution based on the user's input.

[Detailed process of updating the constitution with principles: Accuracy, Clarity, Reproducibility, Rigor, Execution Standards, Module Requirements, and updating dependent templates]

## Outcome

- ✅ Impact: Created comprehensive constitution for Physical AI & Humanoid Robotics textbook project with 6 core principles
- 🧪 Tests: null
- 📁 Files: Updated .specify/memory/constitution.md and templates for plan, spec, and tasks
- 🔁 Next prompts: null
- 🧠 Reflection: Constitution now properly defines project requirements for robotics textbook with focus on accuracy, clarity, reproducibility and technical rigor

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null