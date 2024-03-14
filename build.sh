#!/bin/bash

PYTHON_VERSIONS="3.8 3.9 3.10 3.11 3.12"

for version in $PYTHON_VERSIONS; do
    echo "Building wheel for Python $version"
    python"$version" -m pip wheel . -w dist
done

auditwheel repair dist/*.whl --exclude libcudart.so.12 --exclude libcusparse.so.12 --exclude libcublasLt.so.12 --exclude libcufft.so.11 --exclude libcublas.so.12 --exclude libcudnn.so.8 --exclude libcupti.so.12 --exclude libnvJitLink.so.12 --exclude libnvToolsExt.so.1 --exclude libtorch.so --exclude libtorch_cpu.so --exclude libtorch_python.so --exclude libtorch_cuda.so --exclude libcurand.so.10 --exclude libc10_cuda.so --exclude libcusparseLt-f8b4a9fb.so.0 --exclude libc10-e7eb1568.so -w wheelhouse
