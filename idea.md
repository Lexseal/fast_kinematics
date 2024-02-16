Forward kinematics can be calculated with $q_1 v_1 q_1^{-1} + q_1 q_2 v_2 q_2^{-1} q_1^{-1} + \ldots$. Now it's just a matter of finding the best implementation. Whether it's just like above or using dual quaternion.

But for inverse kinematics, we don't

## Geometric Jacobian vs Analytical Jacobian

They are the same for the translational part, but different for the rotational part. The geometric Jacobian represents the rotation as the rotational axis. i.e. rotation around the x, y, z axis. The analytical Jacobian represents the rotation as the derivative of whatever representation of rotation you are using.

## Parallel Inverse Kinematics

1. https://dl.acm.org/doi/abs/10.1145/2887740
2. https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9126798
3. https://github.com/CarletonABL/QuIK/blob/main/SLloydEtAl2022_QuIK_preprint.pdf
4. https://arxiv.org/pdf/2311.05938.pdf
5. https://cseweb.ucsd.edu/classes/sp16/cse169-a/slides/CSE169_08.pdf
6. https://sim2real.github.io/assets/papers/kaspar.pdf
7. https://github.com/UM-ARM-Lab/pytorch_kinematics

## First, do everything in CPU and compare the results to existing libraries.
1. ~~Set up the build system and include urdfdom and eigen properly.~~
2. ~~Parese the panda urdf file and print out all the joint names, limits, origins, and axes.~~
3. ~~We will store all the information inside a kinematics tree. The tree will contain the joint names, limits, origins, types, and axis of roation or translation. It will also contain the parent and children.~~
4. ~~Then it is time to set up the forward kinematics using homogeneous transformation.~~
5. ~~After that we will set up the forward kinematics using quaternion. We will need to treat the rotation and translation separately.~~
6. ~~Jacobian calculation~~
7. ~~Make implementation parallel~~
8. Transfer to GPU

Memory layout:
data consists of [translation (3 floats), rotation (4 floats), type (1 float), axis (3 floats), ...]
control consists of [angle for first joint, angle for second joint, ...]
two other arrays to specify cumulative number of joints for each robot and cumulative number of active joints for each robot

## Benchmarks

The time it takes to calculate 10000 Jacobians in parallel (including the memory copy time but not including the data prepreation time) is ~2ms on m4000. The same calculation takes ~3.2ms on an M1 and ~9ms on E5-2623 single core.

10000 J
==18797== NVPROF is profiling process 18797, command: ./test_client
==18797== Profiling application: ./test_client
==18797== Profiling result:
   Start  Duration            Grid Size      Block Size     Regs*    SSMem*    DSMem*      Size  Throughput  SrcMemType  DstMemType           Device   Context    Stream  Name
352.26ms  619.91us                    -               -         -         -         -  4.1962MB  6.6103GB/s    Pageable      Device  Quadro M4000 (0         1         7  [CUDA memcpy HtoD]
352.93ms  33.407us                    -               -         -         -         -  312.50KB  8.9210GB/s    Pageable      Device  Quadro M4000 (0         1         7  [CUDA memcpy HtoD]
352.98ms  9.6000us                    -               -         -         -         -  78.125KB  7.7610GB/s    Pageable      Device  Quadro M4000 (0         1         7  [CUDA memcpy HtoD]
353.01ms  9.4080us                    -               -         -         -         -  78.125KB  7.9194GB/s    Pageable      Device  Quadro M4000 (0         1         7  [CUDA memcpy HtoD]
353.15ms  202.78us                    -               -         -         -         -  1.8311MB  8.8183GB/s    Pageable      Device  Quadro M4000 (0         1         7  [CUDA memcpy HtoD]
353.36ms  296.95us             (40 1 1)       (256 1 1)        48        0B        0B         -           -           -           -  Quadro M4000 (0         1         7  jacobian(float*, float*, unsigned long*, unsigned long*, float*, unsigned long) [122]
353.66ms  194.49us                    -               -         -         -         -  1.8311MB  9.1940GB/s      Device    Pageable  Quadro M4000 (0         1         7  [CUDA memcpy DtoH]

Regs: Number of registers used per CUDA thread. This number includes registers used internally by the CUDA driver and/or tools and can be more than what the compiler shows.
SSMem: Static shared memory allocated per CUDA block.
DSMem: Dynamic shared memory allocated per CUDA block.
SrcMemType: The type of source memory accessed by memory operation/copy
DstMemType: The type of destination memory accessed by memory operation/copy
==18797== Generated result file: /home/paperspace/fast_kinematics/build/results.nvprof

100000 J
==19104== NVPROF is profiling process 19104, command: ./test_client
==19104== Profiling application: ./test_client
==19104== Profiling result:
   Start  Duration            Grid Size      Block Size     Regs*    SSMem*    DSMem*      Size  Throughput  SrcMemType  DstMemType           Device   Context    Stream  Name
311.65ms  5.8044ms                    -               -         -         -         -  41.962MB  7.0598GB/s    Pageable      Device  Quadro M4000 (0         1         7  [CUDA memcpy HtoD]
317.59ms  363.26us                    -               -         -         -         -  3.0518MB  8.2042GB/s    Pageable      Device  Quadro M4000 (0         1         7  [CUDA memcpy HtoD]
318.04ms  82.046us                    -               -         -         -         -  781.25KB  9.0810GB/s    Pageable      Device  Quadro M4000 (0         1         7  [CUDA memcpy HtoD]
318.22ms  81.438us                    -               -         -         -         -  781.25KB  9.1488GB/s    Pageable      Device  Quadro M4000 (0         1         7  [CUDA memcpy HtoD]
318.42ms  2.5035ms                    -               -         -         -         -  18.311MB  7.1424GB/s    Pageable      Device  Quadro M4000 (0         1         7  [CUDA memcpy HtoD]
320.93ms  3.9205ms            (391 1 1)       (256 1 1)        48        0B        0B         -           -           -           -  Quadro M4000 (0         1         7  jacobian(float*, float*, unsigned long*, unsigned long*, float*, unsigned long) [122]
324.86ms  2.0000ms                    -               -         -         -         -  18.311MB  8.9406GB/s      Device    Pageable  Quadro M4000 (0         1         7  [CUDA memcpy DtoH]

Regs: Number of registers used per CUDA thread. This number includes registers used internally by the CUDA driver and/or tools and can be more than what the compiler shows.
SSMem: Static shared memory allocated per CUDA block.
DSMem: Dynamic shared memory allocated per CUDA block.
SrcMemType: The type of source memory accessed by memory operation/copy
DstMemType: The type of destination memory accessed by memory operation/copy
==19104== Generated result file: /home/paperspace/fast_kinematics/build/results.nvprof
