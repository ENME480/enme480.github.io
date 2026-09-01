---
title: ENME480 — Introduction to Robotics
description: Kinematics, dynamics, and ROS 2 on UR3e arms — University of Maryland, Fall 2026
---

<div class="hero" markdown>

<p class="eyebrow">University of Maryland · Mechanical Engineering · Fall 2026</p>

# Introduction to Robotics

<p class="lede">Rigid motions, kinematics and control worked out on paper, then run on a six-axis UR3e in the lab.</p>

[Syllabus (PDF)](assets/docs/syllabus-fall2026.pdf){ .md-button .md-button--primary }
[Labs](labs/index.md){ .md-button }
[Site Map](site-map.md){ .md-button }

</div>

## The course

ENME480 is the first course in the [Robotics and Autonomous Systems Minor](https://robotics.umd.edu/minor). Lectures build the math: rigid-body motions, forward and inverse kinematics, velocity kinematics, dynamics, trajectory planning and joint control. Studios and labs turn each of those into working ROS 2 code, first in simulation and then on a UR3e arm.

The final project ties the semester together. An overhead camera finds ArUco-tagged blocks on the table, your inverse kinematics solves for the joint angles that reach them, and the arm stacks a tower.

<dl class="spec" markdown>
<dt>Term</dt>
<dd markdown="span">31 August – 9 December 2026 · 3 credits</dd>
<dt>Lectures</dt>
<dd markdown="span">Mon and Wed, 2:00–2:50 pm · CHE 2110</dd>
<dt>Studios</dt>
<dd markdown="span">See the section table below</dd>
<dt>Instructor</dt>
<dd markdown="span">Dr. Nikhil Chopra · <a href="mailto:nchopra@umd.edu">nchopra@umd.edu</a></dd>
<dt>Office hours</dt>
<dd markdown="span">Thu 10:00–11:30 am on <a href="https://umd.zoom.us/my/nikhilchopra">Zoom</a></dd>
<dt>Teaching assistants</dt>
<dd markdown="span">Alex Beyer (<a href="mailto:abeyer@umd.edu">abeyer@umd.edu</a>), Kaustubh Joshi (<a href="mailto:kjoshi@umd.edu">kjoshi@umd.edu</a>) · hours TBD</dd>
<dt>Textbook</dt>
<dd markdown="span">Spong, Hutchinson and Vidyasagar, <em>Robot Modeling and Control</em>, 2nd ed. (2020) · ISBN 978-1119523994</dd>
<dt>Questions</dt>
<dd markdown="span"><a href="https://piazza.com/umd/fall2026/enme480">Piazza</a> is the official channel</dd>
<dt>Materials and grades</dt>
<dd markdown="span"><a href="https://elms.umd.edu">Canvas / ELMS</a></dd>
<dt>Lab code</dt>
<dd markdown="span"><a href="https://github.com/ENME480">github.com/ENME480</a></dd>
</dl>

## Sections

| Section | Day | Time | Rooms |
|---|---|---|---|
| Lecture | Mon, Wed | 2:00–2:50 pm | CHE 2110 |
| Studio 0101 | Thu | 12:00–2:00 pm | KEB 2111 / EAF 3119 |
| Studio 0102 | Fri | 8:00–10:00 am | KEB 2111 / EAF 3119 |
| Studio 0103 | Tue | 12:00–2:00 pm | KEB 2107 / EAF 3119 |

Programming studios run in the Kim Engineering Building. Robot sessions run in the IDEA Factory. Rooms can change during the semester, so watch Piazza.

## This week

<!-- THIS_WEEK:auto:start -->
_(Auto-filled from `data/this_week.yml` by GitHub Actions on each push)_
<!-- THIS_WEEK:auto:end -->

## Recently changed

<!-- WHATS_NEW:auto:start -->
_(Short announcements from `data/this_week.yml`)_
<!-- WHATS_NEW:auto:end -->

## Grading

| Component | Weight |
|---|---|
| Midterm 1 | 25% |
| Midterm 2 | 25% |
| Studio and lab assignments | 20% |
| Final project | 20% |
| Homework | 7% |
| In-class assignments | 3% |
| Extra credit | 1% |

Exam 1 falls on Monday 19 October, Exam 2 on Wednesday 18 November. One page of notes, front and back, is allowed in each.

Cutoffs: A+ 97, A 94, A− 90 · B+ 87, B 84, B− 80 · C+ 77, C 74, C− 70 · D+ 67, D 64, D− 60 · F below 60.

## Start here

1. Read the [syllabus](assets/docs/syllabus-fall2026.pdf). It is the authority on grading, policies and the week-by-week schedule.
2. Complete the [UR3e safety training](https://academy.universal-robots.com/free-e-learning/e-series-e-learning/e-series-core-track/). Robot lab access depends on it.
3. Install Ubuntu and ROS 2 — see [Ubuntu Setup](ubuntu-setup.md) and [ROS Setup](ros-setup.md).
4. Work through [Week 1](labs/week-01.md) before your first session.

<p class="note-quiet">Not sure where something lives? The <a href="site-map.md">site map</a> lists every page on this site.</p>
