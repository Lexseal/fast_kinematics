import os
import subprocess
import sys
from pathlib import Path

from setuptools import Extension, setup
from setuptools.command.build_ext import build_ext

# A CMakeExtension needs a sourcedir instead of a file list.
# The name must be the _single_ output extension from the CMake build.
# If you need multiple extensions, see scikit-build.
class CMakeExtension(Extension):
  def __init__(self, name: str, sourcedir: str = "") -> None:
    super().__init__(name, sources=[])
    self.sourcedir = os.fspath(Path(sourcedir).resolve())


class CMakeBuild(build_ext):
  def build_extension(self, ext: CMakeExtension) -> None:

    # install numpy and pytorch for the CMake build
    subprocess.run(["python3.10", "-m", "pip", "install", "numpy"])
    subprocess.run(["python3.10", "-m", "pip", "install", "torch"])

    # Must be in this form due to bug in .resolve() only fixed in Python 3.10+
    ext_fullpath = Path.cwd() / self.get_ext_fullpath(ext.name)
    extdir = ext_fullpath.parent.resolve()

    # Using this requires trailing slash for auto-detection & inclusion of
    # auxiliary "native" libs

    debug = int(os.environ.get("DEBUG", 0)) if self.debug is None else self.debug
    cfg = "Debug" if debug else "Release"

    # Set Python_EXECUTABLE instead if you use PYBIND11_FINDPYTHON
    # EXAMPLE_VERSION_INFO shows you how to pass a value into the C++ code
    # from Python.
    cmake_args = [
      f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={extdir}{os.sep}",
      f"-DPYTHON_EXECUTABLE={sys.executable}",
      f"-DCMAKE_BUILD_TYPE={cfg}",  # not used on MSVC, but no harm
    ]
    build_args = []
    # Adding CMake arguments set as environment variable
    # (needed e.g. to build for ARM OSx on conda-forge)
    if "CMAKE_ARGS" in os.environ:
      cmake_args += [item for item in os.environ["CMAKE_ARGS"].split(" ") if item]

    # In this example, we pass in the version to C++. You might not need to.
    cmake_args += [f"-DEXAMPLE_VERSION_INFO={self.distribution.get_version()}"]

    build_args += ["-j"]  # parallel build

    build_temp = Path(self.build_temp) / ext.name
    if not build_temp.exists():
      build_temp.mkdir(parents=True)

    subprocess.run(
      ["cmake", ext.sourcedir, *cmake_args], cwd=build_temp, check=True
    )
    subprocess.run(
      ["cmake", "--build", ".", *build_args], cwd=build_temp, check=True
    )

this_directory = Path(__file__).parent
long_description = (this_directory / "readme.md").read_text()

# The information here can also be placed in setup.cfg - better separation of
# logic and declaration, and simpler if you include description/version in a file.
setup(
  name="fast_kinematics",
  version="0.2.0",
  author="Xinsong Lin",
  author_email="x8lin@ucsd.edu",
  description="A fast kinematics library for robotics",
  long_description=long_description,
  long_description_content_type="text/markdown",
  ext_modules=[CMakeExtension("fast_kinematics")],
  cmdclass={"build_ext": CMakeBuild},
  zip_safe=False,
  requires=["pytorch", "numpy"],
  python_requires=">=3.8",
  home_page="https://github.com/Lexseal/fast_kinematics",
  license="MIT",
)