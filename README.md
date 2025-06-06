# Rendering using BVH
Rendering project
Based on https://github.com/SammaelA/SdfTaskTemplate.git

## Build

Install SDF (Ubuntu 22)

    sudo apt-get install libsdl2-2.0-0

- For installations on other platforms see https://wiki.libsdl.org/

Build the executable:

    cmake -B build && cmake --build build

## Execute

    ./render

## Contents

This repository contains several things useful for working on the task.

- Small maths library with vertices, matrices and a set of useful functions (LiteMath/LiteMath.h)
- Functions for saving and loading images (LiteMath/Image2d.h)
- SimpleMesh, data structure to store triangle mesh, and functions to load it from .obj files (mesh.h)
- A template for your application: creating window with SDF, handling keyboard input, rendering to the window (main.cpp)
- A set of test meshes (cube.obj, as1-oc-214.obj, MotorcycleCylinderHead.obj, spot.obj, stanford-bunny.obj)
  All these models are watertight and can be converted to SDF without issues