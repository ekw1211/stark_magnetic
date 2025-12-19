# Reference Implementation of "Strongly Coupled Simulation of Magnetic Rigid Bodies" in Stark

This repository provides a reference implementation of the publication "Strongly Coupled Simulation of Magnetic Rigid Bodies" by Westhofen et al.:

> L. Westhofen, J. A. Fernández-Fernández, S. R. Jeske, and J. Bender. 2024. Strongly Coupled Simulation of Magnetic Rigid Bodies. In Proceedings of the ACM SIGGRAPH/Eurographics Symposium on Computer Animation (SCA '24). Eurographics Association, Goslar, DEU, 1–11. https://doi.org/10.1111/cgf.15185

with my additions for the final course project in CSC2521.

# CSC2521 Final Project Work
My final report summary can be found on OneDrive [here](https://utoronto-my.sharepoint.com/:b:/g/personal/ella_walsh_mail_utoronto_ca/IQDPZKwkZomWQYwHGJjlqDOtAYsIPpIYuLgXAUuDXyHjeQg?e=CQMiBr). My reimplementations/contributions can be found in:
- EnergyRigidBodyMagnetic.cpp (in stark_magnetic/stark/src/models/rigidbodies)

    - Lines 14–124: reimplementation of the magnetic energy computation, which is looped over for each magnetic dipole sample pair. 

    - Lines 293–376: identifies each pair of interacting dipoles and performed the hybrid solver analysis to determine if they are far enough for weak coupling/explicit solve with directly-applied forces and torques outside of energy minimization optimization step, or close enough for strong coupling/implicit solve within optimization step.

- Main.cpp (in stark_magnetic/examples)

    - Lines 51–176: Two demo functions magnetic_attraction_ekw() and magnetic_repulsion_ekw()

- [Experiment Results (OneDrive link)](https://utoronto-my.sharepoint.com/:f:/g/personal/ella_walsh_mail_utoronto_ca/IgAZvy6kuDlUSbtXF1gHH6LQAWNSwb9Zqkd_WVRb6vMwbFE?e=6Ni8WV)

    - Excel spreadsheet with experimental data, and two videos of the two demo cases generated from PNG exports of the VTK scenes
    - To run the experiments: Comment out line 258 in Main.cpp to run only the Attraction demo. For experiment 1, vary the Size and Mass parameters in lines 54–55. For experiment 2, vary the max_time_step_size parameter in line 67.

# Original Project Notes
Please refer to the [publication page](https://animation.rwth-aachen.de/publication/0590/) for more details. 
The implementation relies mainly on [Stark](https://github.com/InteractiveComputerGraphics/stark).
Please also check the upstream repository for further details on Stark.

# Building & Running

To build the repository, `cmake` is being used. 
The project builds in source using the following commands:

```bash
cmake -DCMAKE_BUILD_TYPE=Release .
make -j 12
```

To run my demo experiments, execute the binary created in the `examples` folder

```bash
cd examples
./examples
```

The .vtk files for each demonstration can be found in respectively-named folders within the `output` folder. They can be viewed using a VTK visualizer such as [ParaView](https://www.paraview.org/download/).

# Compatibility

The code has been tested and compiled with:

- Debian 12 64-bit, CMake 3.25.1, GCC 12.2.0

(I tried and failed to get it running on my Windows machine)

# License 

As a fork of Stark, the main library entailing all folders except `models` are licensed under the Apache-2.0 license in accordance with Stark itself.
See the `LICENSE` file for more details.

The `Armadillo.obj`, `Dragon.obj` and `Bunny.obj` in `models` are variations of the Stanford Armadillo, Dragon & Bunny, courtesy of the [Stanford Computer Graphics Laboratory](https://graphics.stanford.edu/data/3Dscanrep/).
