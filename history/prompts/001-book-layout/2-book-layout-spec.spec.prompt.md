---
id: 2
title: Book Layout Spec
stage: spec
date: 2025-12-07
surface: agent
model: claude-sonnet-4-5-20250929
feature: book-layout
branch: 001-book-layout
user: user
command: /sp.specify
labels: [book, layout, textbook, robotics, specification]
links:
  spec: specs/001-book-layout/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/001-book-layout/spec.md
 - specs/001-book-layout/checklists/requirements.md
 - history/prompts/001-book-layout/
tests:
 - null
---

## Prompt

/sp.specify Complete Book Layout for Physical AI & Humanoid Robotics Textbook

Audience:
AI, Robotics, CS students + instructors evaluating Physical AI curriculum
Also usable by reviewers, hackathon judges, and Panaversity administrators

Focus:
Standardized layout rules that make every chapter reproducible, teachable, and AI-processable.

Success criteria:
- Every chapter follows an identical structural template
- Students can run at least one runnable example in each chapter
- Reader understands theory + practical execution sequence
- Content can be referenced by Claude Code / Agents without ambiguity
- Each chapter yields measurable learning outcomes

Constraints:
- Format: Markdown-first
- Word density per chapter: 1,500–2,200 words
- Layout must support Docusaurus deployment
- Chapter content must be chunkable for RAG
- All diagrams must be exportable as SVG or PNG
- Code must be runnable on ROS 2 Humble environment

Not building:
- Printed physical textbook design
- Instructor guide for faculty
- Full-featured pedagogy outcomes dataset

===========================================================
BOOK STRUCTURE SPECIFICATION
===========================================================

### PART A — PRE-CHAPTER SECTIONS

1. Title Page
   - Course Title
   - Cohort Year
   - Book Release Version
   - Authors + contributors
   - Copyright notice

2. Book Summary Page
   One short page answering:
   - Why Physical AI?
   - Who is this book for?
   - What skills will be gained?

3. Software & Hardware Requirements Page
   MUST list minimum viable hardware:
   - Workstation specs
   - Jetson setup
   - Camera options
   - OS versioning

4. How to Use This Book Page
   Must include:
   - How to navigate chapters
   - Expected time per module
   - Evaluation milestones

5. Table of Contents
   Auto-generated

===========================================================
PART B — MODULE HEADERS
===========================================================

Each Module must begin with a header page containing:

Required format:

Module Title
Why This Module Exists
What You Will Be Able to Do at the End
Tools Used (with versions)
Expected Completion Time
lua
Copy code

Metrics:
- Must state number of chapters inside module
- Must describe inputs → outputs for module

Example format:

Module Objective:
"Go from ROS-zero to autonomous humanoid control in simulation"

markdown
Copy code

===========================================================
PART C — CHAPTER SPECIFICATION (MANDATORY TEMPLATE)
===========================================================

Each chapter must contain the following sequential structure:

1. Chapter Title
2. Learning Outcomes
Clear measurable outcomes

Must start with "You will be able to…"

3. Prerequisites Checklist
Required software installed

Required module completion

Files needed

4. Core Concept Explanation
Text explaining topic

Maximum 3 sub-sections

5. One Diagram or Pipeline
Type: PNG/SVG
Naming convention: diagrams/module-chapter-title.png

6. Runnable Code Example A
Must run on ROS 2 Humble (or Isaac)

Includes verification output

7. Runnable Code Example B
(variation or extension of example A)

8. "Try Yourself" Mini Task
Example:
"Modify publisher node to control walking speed"

9. Verification Procedure
Must answer:

What appears in terminal?

What changes in simulation?

10. Checklist for Completion
11. Summary
12. References
Exact citation format required

markdown
Copy code

Structural constraints:
- No paragraph longer than 7 lines
- No section left empty
- Code blocks tagged with language identifier

===========================================================
PART D — CAPSTONE SPEC SECTION
===========================================================

Must include:

Capstone Title:
"The Autonomous Humanoid"

Capstone Outcomes:
User speaks → Robot interprets → Plan → Navigate → Detect object → Execute motion

Required artifacts delivered:

Simulation export

Voice pipeline implementation

Video demonstration (<=90 seconds)

GitHub repository

markdown
Copy code

Capstone Documentation Must Contain:
- Hardware connection architecture
- ROS topics list
- Models asset references

===========================================================
PART E — BACK MATTER STRUCTURE
===========================================================

Pages required:

1. Glossary of Physical AI terms
 (~80–150 definitions)

2. Technical Standards Page
 MUST reference:
- ROS REP (Robot Enhancement Proposals)
- Isaac API docs
- Gazebo physics defaults

3. Appendix (Optional)
Examples:
- Extra commands
- Dataset download links

4. Bibliography
APA references
Minimum:
- 2 sources per chapter
- 1 open-standards reference per module

===========================================================

Completion Definition:
The Book Layout Spec is satisfied when:

✓ All chapters adhere to template
✓ All runnable code prints visible output
✓ Diagrams exist where required
✓ Table of contents auto-generates without missing routing links
✓ No section left unstructured

## Response snapshot

Created comprehensive specification for the Physical AI & Humanoid Robotics Textbook layout with:

- 3 detailed user stories covering students, instructors, and reviewers
- 10 functional requirements covering chapter template, word count, code examples, diagrams, and deployment
- 6 measurable success criteria
- 4 key entities defined
- Quality checklist created and validated

Specification addresses all requirements from user input including standardized chapter template, module structure, capstone project, and technical constraints.

## Outcome

- ✅ Impact: Created complete specification for book layout with standardized template
- 🧪 Tests: null
- 📁 Files: specs/001-book-layout/spec.md, specs/001-book-layout/checklists/requirements.md, history/prompts/001-book-layout/
- 🔁 Next prompts: Ready for /sp.plan or /sp.clarify
- 🧠 Reflection: Specification comprehensively covers all aspects of book layout with clear templates and measurable outcomes

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null