# Project 2 Beta Write-up

## Design Overview
The beta design introduces the following improvements based on the starter code:

### Render() function redesign
In the starter code, `render()` is implemented by applying a ray-tracing algorithm. To determine the color of a pixel, a ray is created from the eye to the pixel on the 2D plane, then the program checks if the ray intersects with one of the spheres. If it does, then the pixel will be rendered the color of the sphere. This approach is highly inefficient due to its high volume of redundant ray-sphere intersection checks.

`Render()` is redesigned in the beta submission. Before checking for ray-sphere intersection, the program will create 3D bounding boxes for each sphere and project the 3D bounding box onto the 2D plane to get an approximate range of projection. Then, only the pixels that fall into the 2D projection range will be applied the ray-sphere intersection check, which greatly reduces the number of checks that are performed and improves the efficiency of the program.

### Simulate() collision check optimization
In the starter code, `simulate()` checks for sphere collision using an accurate but costly function. The beta submission optimizes this by first applying a cheaper and less accurate collision check that compares bounding box coordinates and filters out the non-colliding sphere pairs, then apply the costly collision check only if two spheres pass the cheap collision check. This approach filters out many sphere pairs that don't collide with the cheap collision check, and reduces the frequency of calling the costly collision check function.

### Sweep&prune algorithm
Sweep&prune algorithm is also implemented to further optimize `simulate()`. Before applying any collision check, the spheres are sorted on the x-axis, so that if a sphere `i` doesn't collide with a sphere `j`, it also won't collide with sphere `j+1` as `j+1` is further away on the x-axis. Similiar checks on y and z-axis are also added before the collision check functions. This algorithm further reduces the number of collision check function calls, and the program reaches tier 40 on `telerun` at this point.

### Quicksort for render() sphere sorting
The beta submission implements a quicksort function to replace the original sorting function in `render()`, which brings the program to tier 41.

### Color precomputation for render()
The beta submission implements color precomputation, which precomputes and reuses the constant part in pixel coloring instead of computing for each pixel at each time. The program is still tier 41 at this point.

### Parallelism
The beta submission uses `cilk_for` to parallelise the following functions: the bounding box ray-tracing part in `render()` (tier 48), `update_accelerations()`, `update_velocities()` and `update_positions()` in `simulate()` (tier 62) running in 8 cores on `telerun`.

## State of Completeness and Expected Performance
Currently, the beta submission passes the correctness check and reaches tier 62 in performance check when running on `telerun` in 8 cores. 

## Additional Information
Some effort was spent on modifying `update_accel_sphere()` to exploit symmetry in gravititional force calculation and reuse some computation results, but this attempt has failed due to floating point precision issue. After several attempts to improve precision, the changes were reverted eventually to ensure more stable performance.

## Next step
* Function inlining
* Built-in intrinsics
* Vectorization
* Race condition check
* Further parallelism according to the handout