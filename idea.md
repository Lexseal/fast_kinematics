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
