---
title: Labs
description: Lab structure, weekly progression, locations, and software requirements for ENME480
---

<p class="eyebrow">ENME480 · Lab track</p>

# Labs

<p class="lede">Every lab builds on the one before it, and the final project uses all of them at once.</p>

## How lab sessions work

Labs alternate between two settings. **Programming studios** open each assignment and run in the Kim Engineering Building, so you have time to write, test and debug code with a TA in the room. **Robot sessions** run in the Robotics and Autonomy Lab in the IDEA Factory, where that code goes on the physical arms.

Most work is done in instructor-assigned groups. Studio attendance is mandatory; if you have to miss a session, arrange it with the TAs on Piazza beforehand.

## Progression

Weeks and rooms follow the course outline in the [syllabus](../assets/docs/syllabus-fall2026.pdf), which is the authority if the two ever disagree.

| Week | Dates | Lab topic | Room |
|---|---|---|---|
| 1 | Aug 31 – Sep 4 | [Lab intro](week-01.md), safety, accounts | KEB 2111 |
| 2 | Sep 7 – 11 | [RAL intro and setup](week-02.md) | EAF 3119 |
| 3 | Sep 14 – 18 | [Python intro, ROS intro, Studio 1](week-03.md) | KEB 2111 |
| 4 | Sep 21 – 25 | [Gazebo demo, Studio 2](week-04.md) | KEB 2111 |
| 5 | Sep 28 – Oct 2 | [FK Lab 1.1](week-05.md) | EAF 3119 |
| 6 | Oct 5 – 9 | [FK Lab 1.2](week-06.md) | EAF 3119 |
| 7 | Oct 12 – 16 | [No lab](week-07.md) — makeup and office hours | — |
| 8 | Oct 19 – 23 | [IK Studio](week-08.md) | KEB 2111 |
| 9 | Oct 26 – 30 | [IK Lab](week-09.md) | KEB 2111 |
| 10 | Nov 2 – 6 | IK Lab | KEB 2111 |
| 11 | Nov 9 – 13 | IK Lab testing | EAF 3119 |
| 12 | Nov 16 – 20 | [Camera lab](week-12.md) | EAF 3119 |
| 13 | Nov 23 – 27 | No lab — Thanksgiving | — |
| 14–15 | Nov 30 – Dec 11 | [Final project](week-13.md) | EAF 3119 |

Exam 1 is Monday 19 October and Exam 2 is Wednesday 18 November, both during lecture.

## Locations

<dl class="spec" markdown>
<dt>Programming studio</dt>
<dd markdown="span">Kim Engineering Building — KEB 2111 (sections 0101 and 0102), KEB 2107 (section 0103). Workstations and simulation tools.</dd>
<dt>Robot lab</dt>
<dd markdown="span">E.A. Fernandez IDEA Factory — EAF 3119, the Robotics and Autonomy Lab. UR3e arms, suction grippers, overhead cameras.</dd>
</dl>

## What you need installed

Set this up before Week 2. [Ubuntu Setup](../ubuntu-setup.md) and [ROS Setup](../ros-setup.md) walk through it, and a small number of loaner machines are available if you cannot get a stable install.

| Requirement | Version | Notes |
|---|---|---|
| Ubuntu | 22.04 LTS | Native install preferred; a VM works |
| ROS 2 | Humble Hawksbill | Matches the lab machines |
| Python | 3.10+ | Ships with Ubuntu 22.04 |
| Gazebo | Bundled with ROS 2 | Installed by the desktop ROS package |
| Git | Any recent version | For cloning lab code |

!!! danger "Before you touch a robot"
    The safety seminar and online training are prerequisites for entering the robot lab. Running AI-generated code on a physical arm is prohibited and can cost you robot access for the rest of the semester.
