---
title: Help
description: Who to contact, how to ask, safety procedures, and first-line troubleshooting
---

<p class="eyebrow">ENME480 · Support</p>

# Help

<p class="lede">Piazza first for anything the whole class might hit. Email only for things specific to you.</p>

## Who to ask

| Channel | Contact | Use it for |
|---|---|---|
| Piazza | [piazza.com/umd/fall2026/enme480](https://piazza.com/umd/fall2026/enme480) | Course and lab questions, extension requests, announcements |
| Instructor | Dr. Nikhil Chopra · <nchopra@umd.edu> | Course content, grades, personal circumstances |
| Office hours | Thu 10:00–11:30 am on [Zoom](https://umd.zoom.us/my/nikhilchopra) | Anything above, in person |
| Teaching assistants | Alex Beyer <abeyer@umd.edu> · Kaustubh Joshi <kjoshi@umd.edu> | Labs, setup, debugging |
| Canvas / ELMS | [elms.umd.edu](https://elms.umd.edu) | Materials, submissions, grades |
| GitHub | [github.com/ENME480](https://github.com/ENME480) | Lab code and updates |

TA office hours are TBD and will be posted on Piazza. Email is likely to get buried, and ELMS messages are not monitored closely, so Piazza really is the fastest route.

## Safety

!!! danger "Robot lab"
    Safety training is a prerequisite for entry. Never bypass an interlock or remove a guard. Stay outside the arm's reach while it is powered. If you are unsure what the robot is about to do, hit the emergency stop first and ask afterwards.

Running AI-generated code on a physical system is unsafe for you, your classmates, the TAs and the robots. Anyone caught doing it risks losing robot access for the rest of the semester.

For a medical or fire emergency call 911 or the University Police, then follow the posted evacuation procedure. Report equipment damage to a TA immediately and log it on Piazza so the next group knows.

## Lab expectations

- **Before:** finish the pre-lab reading and have your environment working. Studio time is for debugging your code, not your install.
- **During:** ask early. A TA can unblock in two minutes what will otherwise cost you the session.
- **Attendance:** studios are mandatory. If you have a conflict, post on Piazza in advance and coordinate with your group.
- **Homework:** posted Fridays at 11:59 pm, due one week later on Canvas. Extensions go through Piazza and are given sparingly, since solutions are released soon after the deadline.

## First-line troubleshooting

Try these before posting. Most lab problems are one of them.

```bash
# Which Ubuntu am I actually on?
lsb_release -a

# ROS 2 environment sourced?
printenv | grep ROS_DISTRO      # expect: humble
source /opt/ros/humble/setup.bash

# Workspace overlay sourced? (needed again after every build)
source ~/ros2_ws/install/setup.bash

# Is the node running and publishing?
ros2 node list
ros2 topic list
ros2 topic hz /joint_states

# Out of disk? A full partition breaks colcon in confusing ways.
df -h
```

## Posting a question that gets answered fast

Search Piazza first, then include four things: what you ran, what you expected, the full error text, and what you already tried.

> `ros2 run enme480_labs fk_node` exits with `ImportError: cannot import name 'kinematic_functions'`.
> I re-sourced `install/setup.bash`, confirmed the file is listed in `setup.py` under `packages`, and rebuilt with `--symlink-install`.
> Full traceback: ...

A screenshot of a terminal is harder to help with than pasted text. Paste the text.
