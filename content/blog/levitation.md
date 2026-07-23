+++
title = "FPGA-Controlled Acoustic Levitation"
description = "FPGA-controlled phased ultrasonic array that levitates and moves lightweight particles in midair using acoustic standing waves"
date = "2026-05-15"

[taxonomies]
tags = ["ece5760", "DE1-SoC FPGA", "C Programming", "Ultrasonics", "Verilog"]
+++

<iframe width="560" height="315" src="https://www.youtube.com/embed/-T1ODJWkBr0?si=Bfx2a5PL1oo1SjG1" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

Our project is an FPGA-controlled phased ultrasonic array that levitates and moves lightweight particles in midair using acoustic standing waves.

For this project, we designed and implemented an acoustic levitation system using opposing top and bottom 4x4 arrays of 40 kHz ultrasonic transducers controlled by an FPGA. Each transducer was driven using PWM-generated square waves, and by carefully controlling the phase relationship between signals, the arrays created standing wave pressure nodes capable of suspending lightweight styrofoam particles in midair. We developed mathematical models and Python simulations to calculate the phase offsets required to focus and steer the acoustic field. Distance-based phase steering across the transducer arrays enabled lateral particle movement, while phase-shifting one array relative to the other allowed vertical movement between the two arrays. Through this project, we also pushed the boundaries of how much external hardware could be simultaneously interfaced and synchronized with a single FPGA. The project explored how phased-array ultrasound and digital waveform control can achieve contactless particle manipulation with potential applications in micro-assembly, contamination-free manufacturing, and scientific research.

[Check out the project website here for details!](https://people.ece.cornell.edu/land/courses/ece5760/FinalProjects/s2026/aj477_drh253_lag289/aj477_drh253_lag289/aj477_drh253_lag289/index.html)