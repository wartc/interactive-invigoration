# Interactive Invigoration: Volumetric Modeling of Trees with Strands

This repository contains an implementation of the paper **"Interactive Invigoration: Volumetric Modeling of Trees with Strands"** by Li et al. (2024). Specifically, the project focuses on the core idea of strand-based volumetric modeling for trees, including strand calculation, Position-Based Dynamics (PBD) for strand packing, and mesh generation from strands.

While not all features of the paper are fully implemented, the core functionality for strand propagation, collision resolution, and basic mesh generation is present.

## Implementation Status

This implementation focuses on the core components of the paper:

- **Implemented**:
  - Strand propagation and merging.
  - PBD for strand packing.
  - Basic mesh generation from strands.
- **Not yet implemented**:
  - Advanced operators (e.g., branch twisting).
  - Full Rhizomorph-based skeleton generation.
  - Complete boundary strand connection and mesh refinement.

## How to Run the Code

### Dependencies

The following packages must be installed on your system:

- **GLFW 3.3+**
- **GLM**
- **CGAL** (Computational Geometry Algorithms Library) - for Delaunay triangulation and least-squares fitting

**\*GLAD** is already included in `external/glad` - no installation needed\*

### Build Instructions

To build the project, you must have **CMake** (3.10+) installed.

1. Clone the repository:

```bash
git clone https://github.com/wartc/interactive-invigoration.git
cd interactive-invigoration
```

2. Create a build directory and compile the project:

```bash
cmake -B build -S .
cmake --build build
```

3. Run the executable

```bash
./interactive-invigoration
```

### Controls

- **H**: Show help message in the terminal.
- **WASD**: Move the camera.
- **Middle Mouse Drag**: Rotate the camera.
- **Right Mouse Drag**: Pan the camera.
- **Scroll Wheel**: Zoom in/out.
- **M**: Toggle mesh visualization.
- **T**: Toggle strand visualization.
- **F**: Toggle wireframe mode.
- **ESC**: Exit the program.

### Acknowledgments

This project is based on the paper "Interactive Invigoration: Volumetric Modeling of Trees with Strands" by Li et al. (2024). Special thanks to the authors for their great work in volumetric tree modeling.
