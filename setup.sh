set -euo pipefail

# 0) Sanity check
test -d .git || { echo "Run this from the root of your enme480.github.io repo"; exit 1; }

# 1) Create structure
mkdir -p .github/workflows docs/{assets,blog,javascripts,labs}

# 2) mkdocs.yml
cat > mkdocs.yml <<'YAML'
site_name: ENME480 - Introduction to Robotics (Fall 2025)
site_description: University of Maryland — ENME480 course website (lectures, labs, resources, policies)
site_url: https://enme480.github.io/
repo_url: https://github.com/ENME480/enme480.github.io

theme:
  name: material
  features:
    - navigation.instant
    - navigation.tabs
    - navigation.sections
    - toc.integrate
    - content.code.copy
    - content.tabs.link
  palette:
    - media: "(prefers-color-scheme)"
      toggle:
        icon: material/brightness-auto
        name: Switch to light mode
    - media: "(prefers-color-scheme: light)"
      scheme: default
      toggle:
        icon: material/brightness-7
        name: Switch to dark mode
    - media: "(prefers-color-scheme: dark)"
      scheme: slate
      toggle:
        icon: material/brightness-4
        name: Switch to system preference

extra:
  version:
    provider: mike

plugins:
  - search
  - blog

markdown_extensions:
  - admonition
  - pymdownx.details
  - pymdownx.superfences
  - pymdownx.tabbed:
      alternate_style: true
  - pymdownx.highlight:
      anchor_linenums: true
      line_spans: __span
      pygments_lang_class: true
  - pymdownx.snippets
  - pymdownx.inlinehilite
  - pymdownx.arithmatex:
      generic: true

extra_javascript:
  - https://cdn.jsdelivr.net/npm/katex@0.16.9/dist/katex.min.js
  - https://cdn.jsdelivr.net/npm/katex@0.16.9/dist/contrib/auto-render.min.js
  - javascripts/math.js

extra_css:
  - https://cdn.jsdelivr.net/npm/katex@0.16.9/dist/katex.min.css

nav:
  - Home: index.md
  - Syllabus: syllabus.md
  - Schedule: schedule.md
  - Labs:
      - Overview: labs/index.md
      - Week 01: labs/week-01.md
      - Week 02: labs/week-02.md
      - Week 03: labs/week-03.md
      - Week 04: labs/week-04.md
      - Week 05: labs/week-05.md
      - Week 06: labs/week-06.md
      - Week 07: labs/week-07.md
      - Week 08: labs/week-08.md
      - Week 09: labs/week-09.md
      - Week 10: labs/week-10.md
      - Week 11: labs/week-11.md
      - Week 12: labs/week-12.md
      - Week 13: labs/week-13.md
      - Week 14: labs/week-14.md
      - Week 15: labs/week-15.md
      - Final Project: labs/final-project.md
  - Resources: resources.md
  - Policies: policies.md
  - Help: help.md
  - Blog: blog/index.md
YAML

# 3) Workflow (deploy to gh-pages using mike)
mkdir -p .github/workflows
cat > .github/workflows/deploy.yml <<'YAML'
name: build-and-deploy
on:
  push:
    branches: [ main ]
  workflow_dispatch:

permissions:
  contents: write

env:
  VERSION: fa2025

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - name: Checkout (with submodules)
        uses: actions/checkout@v4
        with:
          submodules: recursive
          fetch-depth: 0  # mike needs history for gh-pages ops
      - name: Set up Python
        uses: actions/setup-python@v5
        with:
          python-version: '3.11'
      - name: Install dependencies
        run: pip install -r requirements.txt
      - name: Configure git identity
        run: |
          git config user.name "github-actions[bot]"
          git config user.email "41898282+github-actions[bot]@users.noreply.github.com"
      - name: Build & deploy version with mike
        run: |
          mike deploy --push --update-aliases ${VERSION} latest
          mike set-default --push latest
YAML

# 4) Python requirements
cat > requirements.txt <<'TXT'
mkdocs>=1.5
mkdocs-material>=9.5
mike>=2.1.0
pymdown-extensions>=10.8
TXT

# 5) Docs pages
cat > docs/index.md <<'MD'
# ENME480 — Introduction to Robotics (Fall 2025)

Welcome to the course hub. Use the sidebar to find **Labs**, **Schedule**, **Resources**, and more.

> **Quick links**
>
> - [Syllabus](./syllabus.md)
> - [Schedule](./schedule.md)
> - [Labs](./labs/index.md)
> - [Blog / Announcements](./blog/index.md)

!!! tip "Dark mode"
    Use the sun/moon toggle in the header to switch themes.
MD

cat > docs/syllabus.md <<'MD'
# Syllabus — Fall 2025

**Course:** ENME480 — Intro to Robotics  
**Credits:** 3  
**Dates:** Sep 2, 2025 – Dec 12, 2025  
**Professor:** Dr. Nikhil Chopra — <nchopra@umd.edu>  
**Office Hours:** Wed 10–11:30, 2149 Martin Hall — [Zoom link](https://umd.zoom.us/j/99088503503?pwd=pQKi2zBOOaUWaaRqNaEESbRxLlDzqh.1)  
**Teaching Assistants:** Alex Beyer (<abeyer@umd.edu>), **Kaustubh Joshi** (<kjoshi@umd.edu>)  
**TA Office Hours:** **TBD**  
**Canvas (ELMS):** <http://www.elms.umd.edu/>  
**Piazza (official Q&A):** <http://piazza.com/umd/fall2025/enme480>  
**Lab GitHub org:** <https://github.com/ENME480>

---

## Course Description
This course introduces elementary concepts in robotics with integrated theory and lab components. Labs emphasize interdisciplinary teamwork, developing and testing code on **UR3e** robotic arms across programming studios and hands-on lab sections.

## Learning Outcomes
- Apply mathematics, science, and engineering to robotics problems  
- Analyze and interpret experimental data  
- Use robot geometry for kinematics analysis  
- Apply robot dynamics for planning and control

## Required Resources
- **Course website:** ELMS-Canvas  
- **Textbook:** *Robot Modeling and Control* (2e), Spong, Hutchinson, Vidyasagar, 2020, ISBN 978-1119523994  
- **Hardware/Software:** Laptop capable of running **ROS 2** (setup guided in labs)

## Course Structure
- **Lectures & in-class assignments:** Short comprehension questions (extra credit) may be assigned after lectures.  
- **Studios & Labs:** Run by TAs in **KEB 2111** (programming) and **EAF 3119** (Robotics & Autonomy Lab). Safety seminar + online training required before using robots.  
- **Homework:** Posted Fridays 11:59 pm; due one week later via Canvas. Extensions via Piazza only (solutions release soon after deadlines).  
- **Exams:** Two midterms (see Schedule). One page of notes (front/back) permitted.  
- **Final Project:** Vision-enabled pick-and-place with UR3e: locate blocks, grasp, and build a tower. Group project with write-up and video.

## Major Assignments & Weighting
| Component | % |
|---|---:|
| Homework | 20% |
| Studio/Lab Assignments | 20% |
| Midterm 1 | 20% |
| Midterm 2 | 20% |
| Final Project | 20% |
| **Extra Credit:** In-class assignments | **Up to 5%** |

**Final grade cutoffs:** A+: 97, A: 94, A-: 90; B+: 87, B: 84, B-: 80; C+: 77, C: 74, C-: 70; D+: 67, D: 64, D-: 60; F: < 60.

## Communication & Participation
- **Piazza is official** for course questions; Canvas hosts materials/announcements.  
- Professional, inclusive discussion is expected in all channels and sessions.  
- Attendance and on-time arrival for studios/labs are essential; coordinate via Piazza if you must miss your assigned session.

## Policies (Summary)
- **Academic Integrity:** The University’s Code applies; collaboration on graded work is prohibited unless stated. Unauthorized use of course-assistance sites or AI-generated solutions is not permitted. Pledge required on each assignment/exam.  
- **AI Usage (Course-specific):** Brainstorming/review OK; final work must be your own. **Do not run AI-generated code on physical robots.**  
- **Accessibility & Accommodations:** See ADS; contact instructor promptly for arrangements.  
- **Campus resources:** Emergency Preparedness, Basic Needs, Veteran Resources, Title IX, Course Evaluation—see Policies page.
MD

cat > docs/schedule.md <<'MD'
# Schedule — Fall 2025

> **Note:** Locations may change; monitor **Piazza** for updates. Studios meet in **KEB 2111** (programming) and **EAF 3119** (robot lab) unless noted.

| Week | Dates | Mon (Lecture) | Tue (Lab/Studio) | Wed (Lecture) | Thu (Lab/Studio) | Fri (Lab/Studio) |
|---:|---|---|---|---|---|---|
| 1 | 9/1–9/5 | **No Lecture (Mon)** | **Lab Intro** | Intro & Linear Algebra Primer | **Lab Intro** | **Lab Intro** |
| 2 | 9/8–9/12 | Linear Algebra Primer | RAL Intro, Setup | Linear Algebra Primer | RAL Intro, Setup | RAL Intro, Setup |
| 3 | 9/15–9/19 | Rigid Motions | Python Intro, ROS Intro, Studio 1 | Rigid Motions | Python Intro, ROS Intro, Studio 1 | Python Intro, ROS Intro, Studio 1 |
| 4 | 9/22–9/26 | Rigid Motions | Gazebo Demo, Studio 2 | Rigid Motions | Gazebo Demo, Studio 2 | Gazebo Demo, Studio 2 |
| 5 | 9/29–10/3 | Forward Kinematics | **FK Lab 1.1** | Forward Kinematics | **FK Lab 1.1** | **FK Lab 1.1** |
| 6 | 10/6–10/10 | Velocity Kinematics | **FK Lab 1.2** | Velocity Kinematics | **FK Lab 1.2** | **FK Lab 1.2** |
| 7 | 10/13–10/17 | **No Lecture (Mon/Tue)** | **No Lecture** | Velocity Kinematics | **No Lab (Make-up/Office Hours)** | **No Lab (Make-up/Office Hours)** |
| 8 | 10/20–10/24 | **Exam** | **IK Studio** | Inverse Kinematics | **IK Studio** | **IK Studio** |
| 9 | 10/27–10/31 | Inverse Kinematics | **IK Lab** | Inverse Kinematics | **IK Lab** | **IK Lab** |
| 10 | 11/3–11/7 | Inverse Kinematics | **IK Lab** | Dynamics | **IK Lab** | **IK Lab** |
| 11 | 11/10–11/14 | Dynamics | Intro to Cameras | Dynamics | Intro to Cameras | Intro to Cameras |
| 12 | 11/17–11/21 | Path & Trajectory Planning | **Camera Lab** | **Exam 2** | **Camera Lab** | **Camera Lab** |
| 13 | 11/24–11/29 | Path & Trajectory Planning | **No Lab (Make-up/Office Hours)** | **No Lecture (Wed–Fri)** | **No Lecture** | **No Lecture** |
| 14 | 12/1–12/5 | Path & Trajectory Planning | **Final Project** | Independent Joint Control | **Final Project** | **Final Project** |
| 15 | 12/8–12/12 | Independent Joint Control | **Final Project** | Independent Joint Control | **Final Project** | **Final Project** |
MD

cat > docs/labs/index.md <<'MD'
# Labs

Studios/Labs are group-based and alternate between programming studios (**KEB 2111**) and robot time (**EAF 3119**). Safety training is required before robot access. Follow **Piazza** for location updates.

Use the week pages on the left for objectives, setup, and deliverables. Lab code is tracked in the **Lab-Code** submodule.

> **Submodule path:** `labs/Lab-Code/`  
> **Upstream repo:** https://github.com/ENME480/Lab-Code
MD

# Lab week pages (link into submodule)
cat > docs/labs/week-01.md <<'MD'
# Week 01 — Lab Intro

**Focus:** course/lab onboarding, environments, safety overview, space orientation (KEB 2111 & EAF 3119).

- 📁 Lab materials (if present): [`labs/Lab-Code/Week 1 Materials/`](../Lab-Code/Week%201%20Materials/)
- ✅ Deliverables: confirm environment + accounts; complete safety training checkpoint (details in Canvas/Piazza).
- 🧭 Rooms: programming in **KEB 2111**; robot lab in **EAF 3119**.

<!-- Inline README later:
--8<-- "labs/Lab-Code/Week 1 Materials/README.md"
-->
MD

cat > docs/labs/week-02.md <<'MD'
# Week 02 — RAL Intro & Setup / Ubuntu + Python Intro

- 📁 [`labs/Lab-Code/Week 2 - Ubuntu & Python Intro/`](../Lab-Code/Week%202%20-%20Ubuntu%20%26%20Python%20Intro/)
<!--
--8<-- "labs/Lab-Code/Week 2 - Ubuntu & Python Intro/README.md"
-->
MD

cat > docs/labs/week-03.md <<'MD'
# Week 03 — Python + ROS Intro (Studio 1)

- 📁 [`labs/Lab-Code/Week 3 - ROS/`](../Lab-Code/Week%203%20-%20ROS/)
<!--
--8<-- "labs/Lab-Code/Week 3 - ROS/README.md"
-->
MD

cat > docs/labs/week-04.md <<'MD'
# Week 04 — Gazebo Demo (Studio 2)

- 📁 [`labs/Lab-Code/Week 4 - Gazebo & Python/`](../Lab-Code/Week%204%20-%20Gazebo%20%26%20Python/)
<!--
--8<-- "labs/Lab-Code/Week 4 - Gazebo & Python/README.md"
-->
MD

cat > docs/labs/week-05.md <<'MD'
# Week 05 — Forward Kinematics Lab 1.1

- 📁 Possibly relevant: [`labs/Lab-Code/Week 5 - UR3e Intro/`](../Lab-Code/Week%205%20-%20UR3e%20Intro/) and [`labs/Lab-Code/Week 6 - Forward Kinematics/`](../Lab-Code/Week%206%20-%20Forward%20Kinematics/)
<!--
--8<-- "labs/Lab-Code/Week 5 - UR3e Intro/README.md"
-->
MD

cat > docs/labs/week-06.md <<'MD'
# Week 06 — Forward Kinematics Lab 1.2

- 📁 [`labs/Lab-Code/Week 6 - Forward Kinematics/`](../Lab-Code/Week%206%20-%20Forward%20Kinematics/)
MD

cat > docs/labs/week-07.md <<'MD'
# Week 07 — Make-up / Office Hours

No scheduled lab; use time for catch-up and TA help.
MD

cat > docs/labs/week-08.md <<'MD'
# Week 08 — IK Studio

- 📁 [`labs/Lab-Code/Week 8 - Inverse Kinematics/`](../Lab-Code/Week%208%20-%20Inverse%20Kinematics/)
MD

cat > docs/labs/week-09.md <<'MD'
# Week 09 — Inverse Kinematics Lab

- 📁 [`labs/Lab-Code/Week 11 - Inverse Kinematics Lab/`](../Lab-Code/Week%2011%20-%20Inverse%20Kinematics%20Lab/)
- Note: An `IK Lab Solution.pdf` exists in repo root for staff reference.
MD

cat > docs/labs/week-10.md <<'MD'
# Week 10 — IK Lab / Dynamics intro

- 📁 [`labs/Lab-Code/Week 10 - Forward Kinematics Lab/`](../Lab-Code/Week%2010%20-%20Forward%20Kinematics%20Lab/)
MD

cat > docs/labs/week-11.md <<'MD'
# Week 11 — Dynamics + Intro to Cameras

- 📄 Camera perspective PDF in repo: [`labs/Lab-Code/PerspectiveTransformEstimation (1).pdf`](../Lab-Code/PerspectiveTransformEstimation%20(1).pdf)
MD

cat > docs/labs/week-12.md <<'MD'
# Week 12 — Camera Lab (+ Exam 2 week)

- 📁 Camera Lab materials: use camera PDF above and any posted updates.
MD

cat > docs/labs/week-13.md <<'MD'
# Week 13 — Make-up / Office Hours

No lab meetings (holiday week). Use time for project prep.
MD

cat > docs/labs/week-14.md <<'MD'
# Week 14 — Final Project (starts)

- 📁 Final Project folder: [`labs/Lab-Code/Final Project/`](../Lab-Code/Final%20Project/)
MD

cat > docs/labs/week-15.md <<'MD'
# Week 15 — Final Project (wrap-up)

- 📁 Continue work in `labs/Lab-Code/Final Project/`
- Deliverables: write-up + video demo (see syllabus).
MD

cat > docs/labs/final-project.md <<'MD'
# Final Project — Vision-Enabled Pick & Place

Build a pipeline to detect blocks on a table, move UR3e to them, grasp, and stack into a tower. Team-based; submit write-up + demo video.

- 📁 Repo folder: [`labs/Lab-Code/Final Project/`](../Lab-Code/Final%20Project/)
MD

cat > docs/resources.md <<'MD'
# Resources

- **Lab code (submodule):** `labs/Lab-Code/` → upstream: https://github.com/ENME480/Lab-Code
- **ROS 2 (Humble) tutorials & docs:** installation, nodes, topics, tf2, URDF, Gazebo, etc.
- **Textbook:** Spong, Hutchinson, Vidyasagar — *Robot Modeling and Control* (2e, 2020).

> **TODO:** Add department/proctoring links, UR3e datasheets, safety docs, camera calibration notes, lab-specific setup scripts.
MD

cat > docs/policies.md <<'MD'
# Policies & Campus Resources

This course follows University policies (integrity, conduct, accessibility, attendance, grades, IP). See campus policy pages and the syllabus for full details.

## Academic Integrity (Course-specific)
- Unauthorized collaboration or AI-generated solutions are not permitted unless explicitly allowed.
- Pledge on each assessment: *“I pledge on my honor that I have not given or received any unauthorized assistance on this exam/assignment.”*
- When in doubt about collaboration boundaries, ask the course staff.

## AI Usage (Course-specific)
- You may use AI tools for **brainstorming/review**; your **final submissions must be your own**.
- **Never run AI-generated code on physical robots.**

## Accessibility & Accommodations
UMD ADS provides accommodations; contact the instructor promptly to arrange.

## Title IX & Mandatory Reporting
The instructor is a Responsible University Employee. Confidential resources include CARE to Stop Violence and the Counseling Center.

## Participation & Attendance
Attendance and on-time arrival are essential, especially for group studios/labs. If you must miss your assigned lab time, message TAs via Piazza in advance and coordinate with your team.

## Course Evaluation
Please complete Student Feedback on Course Experiences at semester end.
MD

cat > docs/help.md <<'MD'
# Help & Contact

**Instructor:** Dr. Nikhil Chopra — <nchopra@umd.edu>  
**OH:** Wed 10–11:30 (2149 Martin Hall) — [Zoom](https://umd.zoom.us/j/99088503503?pwd=pQKi2zBOOaUWaaRqNaEESbRxLlDzqh.1)

**TAs:**  
- Alex Beyer — <abeyer@umd.edu>  
- Kaustubh Joshi — <kjoshi@umd.edu>  
**TA OH:** **TBD**

**Where things live**  
- **Canvas (ELMS):** class materials, announcements, homework submissions  
- **Piazza:** official Q&A and extension requests  
- **Lab Code:** `labs/Lab-Code/` (this repo’s submodule)
MD

cat > docs/blog/index.md <<'MD'
# Announcements

Course updates will appear here (and on Canvas/Piazza). Create new posts by adding dated Markdown files to `docs/blog/`.
MD

cat > docs/blog/2025-09-01-welcome.md <<'MD'
---
date: 2025-09-01
categories:
  - announcement
---

# Welcome to ENME480 (FA 2025)

Check the **Schedule** for weekly topics and labs. The **Labs** section links to the code submodule. Good luck & have fun!
MD

# 6) Math auto-render
cat > docs/javascripts/math.js <<'JS'
document.addEventListener("DOMContentLoaded", function() {
  if (typeof renderMathInElement !== "undefined") {
    renderMathInElement(document.body, {
      delimiters: [
        {left: "$$", right: "$$", display: true},
        {left: "$", right: "$", display: false},
        {left: "\\[", right: "\\]", display: true},
        {left: "\\(", right: "\\)", display: false}
      ],
      throwOnError: false
    });
  }
});
JS

# 7) Syllabus PDF placeholder + auto-copy if you already have one nearby
mkdir -p docs/assets
touch docs/assets/.keep
for CAND in "./ENME480 -Syllabus"*.pdf "../ENME480 -Syllabus"*.pdf; do
  if [ -f "$CAND" ]; then
    cp -f "$CAND" "docs/assets/syllabus.pdf"
    echo "Copied syllabus PDF from: $CAND"
    break
  fi
done

# 8) Add Lab-Code as submodule if missing
if [ ! -d "labs/Lab-Code/.git" ] && [ ! -d "labs/Lab-Code" ]; then
  mkdir -p labs
  git submodule add https://github.com/ENME480/Lab-Code labs/Lab-Code || true
fi

# 9) Commit & push
git add .github docs mkdocs.yml requirements.txt
git add labs/Lab-Code || true
git commit -m "ENME480 site: MkDocs Material + labs submodule + CI (mike versioning)"
git push origin main

echo "Done! Next: enable Pages → Branch: gh-pages (root)."
