# physicSim

Simulation of Feedback Mechanisms Arising From Diversely Seeded Star and Black Hole Formation

<br>

**Branches:**
- Main (SPH, most updated)
- Parallel (Gravity, Parallelized tree)
- Tree (Gravity, Basic quadtree)
- Standard (Gravity, Pairwise exact computation)

<br>

**Build & Run Instructions (linux only)**
1. Clone the git repo
2. Make sure to install the correct packages for:
    - Eigen
    - GSL
    - OpenGL
    - GLFW
    - OpenMP
    - PNG
3. Create a 'build/' directory in project root
4. Run 'cmake -B build' to initialize cmake
5. Run 'cmake --build build' to compile the project
6. Run './build/runSim' to execute the file
7. Only step 6 is needed to run and only step 5 is needed if changes are made
