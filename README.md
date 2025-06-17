# exoskeleton_C++Mujoco

Minimal yet fully-interactive **MuJoCo C++** demo that

* loads an **exoskeleton** model (`model/scene.xml`)
* opens a GLFW window with smooth mouse / keyboard controls
* lets you **set individual joint targets (°)** at runtime via a tiny C++ helper

Designed for easy relocation: all resources are referenced **relative to the project root**, so the code works no matter where you build or launch it.

---

## 1  Directory layout

```text
exoskeleton_C++Mujoco/
├─ build/                 ← out-of-source CMake build directory (created later)
├─ model/
│   ├─ meshes/ …          ← any STL/OBJ meshes used by the MJCF
│   └─ scene.xml          ← main MuJoCo model
├─ mujoco/
│   ├─ include/ …         ← full MuJoCo headers  (mujoco.h …)
│   └─ lib/
│       ├─ libmujoco.so   ← symlink → libmujoco.so.3.x.y
│       └─ libmujoco.so.3.x.y
├─ main.c++               ← demo source (open-source C++17)
└─ CMakeLists.txt
```
---

## 2  Prerequisites
| Component          | Version / Notes                                                                                                                                                                                                                      |
| ------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| **MuJoCo 3.x**     | Download the Linux tarball from the [official releases](https://github.com/google-deepmind/mujoco/releases), extract, then copy *both* `include/` and `lib/` here. Make sure `lib/libmujoco.so → libmujoco.so.3.x.y` symlink exists. |
| **GLFW 3**         | `sudo apt install libglfw3-dev` (Ubuntu)                                                                                                                                                                                             |
| **OpenGL / Mesa**  | Desktop GPUs already have it; on headless servers install `libgl1-mesa-dev`.                                                                                                                                                         |
| **CMake ≥ 3.10**   | Any modern distro works.                                                                                                                                                                                                             |
| **C++17 compiler** | gcc 9+/clang 10+ tested.                                                                                                                                                                                                             |
## 3  Build

```bash
# 1) From project root
mkdir -p build && cd build

# 2) Point the linker to our packaged MuJoCo
export LD_LIBRARY_PATH="$(pwd)/../mujoco/lib:${LD_LIBRARY_PATH}"

# 3) Configure & compile
cmake ..
make -j$(nproc)

# Executable appears as ./main
```
## 4  Run

```bash
# Still inside build/ (or anywhere)
./main          # window pops up with the exoskeleton scene
# Press 'R' to reset physics
# Press 'Esc' to exit
```
