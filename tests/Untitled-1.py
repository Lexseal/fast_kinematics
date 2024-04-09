# %%
# # relative import ../build/fast_kinematics
# import sys
# sys.path.append('../build/')
import numpy as np
import time
import torch
import fast_kinematics
# import pytorch_kinematics as pk

# # %%
N = 4096
print(torch.__version__)

# # %%
# # fask kinematics model
model = fast_kinematics.FastKinematics("../fetch.urdf", N, "gripper_link")

# # %%
# # pk chain
# chain = pk.build_serial_chain_from_urdf(open("../kuka_iiwa.urdf").read(), "lbr_iiwa_link_7")
d = "cuda"
dtype = torch.float32

# chain = chain.to(dtype=dtype, device=d)

# # %%
th = torch.rand(N, 7, dtype=dtype, device=d, requires_grad=True)
# all_times = []
# J = None
# for _ in range(100):
#   start_time = time.time()
#   J = chain.jacobian(th)
#   end_time = time.time()
#   all_times.append(end_time - start_time)
# print(f"Time taken for jacobian_mixed_frame: {np.mean(all_times)} with std: {np.std(all_times)}")

# # %%
# # the output of fast_kinematics is column major 1d array, so we need to reshape it
# # experimenting to make sure we got it right
# flat_array = torch.arange(1,43)
# print(flat_array.view(-1,7,6).permute(0,2,1))

# # %%
# # flatten th
th = th.view(N*7)
# all_times = []
# J2 = None
# for _ in range(1000):
#   start_time = time.time()
#   J2 = model.jacobian_mixed_frame_pytorch(th).view(-1,7,6).permute(0,2,1)
#   end_time = time.time()
#   all_times.append(end_time - start_time)
# print(f"Time taken for fk: {np.mean(all_times)} with std: {np.std(all_times)}")

# # %%
# assert torch.allclose(J, J2, atol=1e-5)

# # %%
# # test out pinv vs lstsq

# b = torch.rand(6, 1, dtype=dtype, device=d, requires_grad=False)
# x_pinv = torch.zeros(N, 7, dtype=dtype, device=d)
# all_times = []
# for i in range(100):
#   start_time = time.time()
#   # J2 is a collection of matrices but pinv seems to work on the entire batch
#   J2_pinv = torch.pinverse(J2)
#   if i == 0: x_pinv = torch.matmul(J2_pinv, b)
#   end_time = time.time()
#   all_times.append(end_time - start_time)
# print(f"Time taken for pinv: {np.mean(all_times)} with std: {np.std(all_times)}")

# # unit test correctness of pinv for an entire batch
# first_pinv = J2[0].pinverse()
# first_x = first_pinv @ b
# assert torch.allclose(first_x, x_pinv[0], atol=1e-5)

# # %%
# all_times = []
# x_lstsq = None
# new_b = b.view(1,6,1).repeat(N,1,1)  # repeat the same b for all N
# for i in range(100):
#   start_time = time.time()
#   x_lstsq = torch.linalg.lstsq(J2, new_b).solution
#   end_time = time.time()
#   all_times.append(end_time - start_time)
# print(f"Time taken for lstsq: {np.mean(all_times)} with std: {np.std(all_times)}")

# # %%
# print(x.shape)
# print(J2_lstsq.shape)
# for i in range(N):
#   assert torch.allclose(x[i], J2_lstsq[i], atol=10), f"i: {i}, x: {x[i]}, lstsq: {J2_lstsq[i]}"

# # %%
# # checking to make sure the results are the same
# print(J.shape)
# print(J2.shape)
# assert torch.allclose(J, J2, atol=1e-5)

# # %%
# # just for fun numpy version still faster
# qpos = np.random.rand(7*N)
# qpos = qpos.astype(np.float32)

# all_times = []
# for _ in range(1000):
#   start_time = time.time()
#   poses = model.jacobian_mixed_frame(qpos)
#   poses = poses.reshape(N,-1,6).transpose(0,2,1)
#   end_time = time.time()
#   all_times.append(end_time - start_time)
# print(f"Time taken for jacobian_mixed_frame: {np.mean(all_times)} with std: {np.std(all_times)}")


