# Fast Kinematics

This is a cuda enabled library for calculating forward kinematics and Jacobian of a kinematics chain. It can be used with numpy arrays or pytorch tensors. Compared to `pytorch_kinematics`, this is about 1000 times faster at Jacobian calculation with 4096 parallel 7dof robot arms.

## Installation

### From PyPi

```bash
pip install fast_kinematics
```

### From Source

1. Get the docker image
   ```bash
   docker pull lexseal/fast_kinematics:latest
   ``` 
2. Run the docker image
   ```bash
   docker run -it lexseal/fast_kinematics:latest
   ```
3. Update the repo
   ```bash
   cd /workspace/fast_kinematics && git pull
   ```
4. Run `build.sh` to build all wheels. The built wheels will showup in the `wheelhouse` dir
   ```bash
   ./build.sh
   ```
5. Take the wheel out of the docker container. From your host machine, run
   ```bash
   docker cp <container_id>:/workspace/fast_kinematics/wheelhouse .
   ```

## Usage

> [!WARNING]
> Please use dtype `np.float32` or `torch.float32` for the joint configuration. The library is optimized for `float32` and will not work with `float64`.

> [!NOTE]
> You might need to import torch before using this library as torch will load the shared libraries. Your mileage may vary.

Minimal numpy example:
  
  ```python
  import numpy as np
  from fast_kinematics import FastKinematics

  N = 1024  # number of parallel calculations

  # need the urdf file, number of parallel calculations and end effector link
  model = FastKinematics("kuka_iiwa.urdf", N, "lbr_iiwa_link_7")

  # this robot has 7 joints, so the joint configuration has size (N*7,) (1d array)
  # the first 7 values are for the first robot... Note we need to use float32!
  joint_config = np.random.rand(N*7).astype(np.float32)

  # get the forward kinematics size (N*7,) (1d array)
  # the first 7 values are the first robot's pose [x,y,z,qw,qx,qy,qz]
  ee_pose = model.forward_kinematics(joint_config)

  # get the jacobian size (N*6*7,) (1d array) The 7 here is the number of joints
  jac = model.jacobian_mixed_frame(joint_config)
  # we can reshape the jacobian to (N,7,6) and then transpose to (N,6,7)
  jac = jac.reshape(-1, 6, 7).transpose(0, 2, 1)
  ```

Minimal pytorch example:

  ```python
  import torch
  from fast_kinematics import FastKinematics

  N = 1024
  model = FastKinematics("kuka_iiwa.urdf", N, "lbr_iiwa_link_7")

  joint_config = torch.rand(N*7, dtype=torch.float32, device="cuda")

  ee_pose = model.forward_kinematics_pytorch(joint_config)
  ee_pose = ee_pose.view(-1,7,6).permute(0,2,1)
  ```

There is also a jupyter notebook in the `tests` folder that contains usage as well as some benchmarks against `pytorch_kinematics`.

## APIs

The docs are embedded in the library itself, you can either print out the doc string or use the code suggestions in your IDE to see the API documentation.
Alternatively, please see `pybind11/pybind_fast_kinematics.cpp` for the API documentation.
