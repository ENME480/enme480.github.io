# Lab 1 — Setup & First Motion

## Objectives
- Verify environment setup
- Run a minimal control loop

## Setup
!!! note "Pre-reqs"
    - Clone the lab submodule (automatically checked out by CI)
    - Install Python 3.10+ and required packages

## Steps
=== "Python"
    ```py title="hello_control.py"
    import time
    print("Starting control loop...")
    for i in range(3):
        print(f"tick {i}")
        time.sleep(0.5)
    ```

=== "CLI"
    ```bash
    python hello_control.py
    ```

## Math check
Inline: $\dot{x} = J(q)\,\dot{q}$

Block:

$$
\tau = J^T(q)\,F
$$

## Deliverables
- Short report with results and questions
