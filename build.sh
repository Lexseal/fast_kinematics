#!/bin/bash

PYTHON_VERSIONS="3.8 3.9 3.10 3.11 3.12"

for version in $PYTHON_VERSIONS; do
    echo "Building wheel for Python $version"
    python"$version" -m pip wheel . -w dist
done

# exclude all the cuda 11 and 12 libaries
auditwheel repair dist/*.whl --exclude libcudart.so.12 --exclude libcusparse.so.12 --exclude libcublasLt.so.12 --exclude libcufft.so.11 --exclude libcublas.so.12 --exclude libcudnn.so.8 --exclude libcupti.so.12 --exclude libnvJitLink.so.12 --exclude libnvToolsExt.so.1 --exclude libtorch.so --exclude libtorch_cpu.so --exclude libtorch_python.so --exclude libtorch_cuda.so --exclude libcurand.so.10 --exclude libc10_cuda.so --exclude libcusparseLt.so.0 --exclude libc10.so --exclude libc10.so --exclude libc10_cuda.so --exclude libcublas.so.11 --exclude libcublasLt.so.11 --exclude libcudart.so.11.0 --exclude libcudart.so.12.4.99 --exclude libcudnn.so.8 --exclude libcufft.so.10 --exclude libcupti.so.11.8 --exclude libcurand.so.10.3.5.119 --exclude libcusparse.so.11 --exclude libcusparseLt.so.0 --exclude libgomp.so.1 --exclude libnccl.so.2 --exclude libnvToolsExt.so.1.0.0 --exclude libshm.so --exclude libtorch.so --exclude libtorch_cpu.so --exclude libtorch_cuda.so --exclude libtorch_python.so --exclude libnvrtc.so.12 --exclude libnvrtc.so.11 --exclude libcuda.so.1 -w wheelhouse
