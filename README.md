# kinova_control_experiments

Simple impedance controller for Kinova Gen 3 arm which was tested on a real robot. This requries ROS to work because of
`kdl_parser` for URDF files. The ABAG implementation is generated using Domain-Specific languages realized in
[Jetbrains MPS](https://www.jetbrains.com/mps/), which can be found at the
[`rosym-project/controller-dsl`](https://github.com/rosym-project/controller-dsl) and
[`rosym-project/algorithm-dsl`](https://github.com/rosym-project/algorithm-dsl) repositories.

## Contents

- [kinova_control_experiments](#kinova_control_experiments)
  - [Contents](#contents)
  - [Installation](#installation)
  - [Visualization](#visualization)
  - [Maintainers:](#maintainers)

## Installation

The kinova zip file is added using `git-lfs`, see [project page](https://git-lfs.github.com) for setup instruction.
All necessary headers, libraries and URDF model are expected under `kinova_libs/extracted` directory. After installing
`git-lfs` (and maybe system package `zip`), the necessary files can be extracted to the right repository by
running the following commands at the repository's top level:

```
$ git lfs pull
$ cd kinova_libs && unzip kinova_libs.zip
```

The included [configuration file](config/kinova.cfg) is handled using
[`libconfig`](https://hyperrealm.github.io/libconfig). On Ubuntu all necessary libraries and headers can be installed
with `sudo apt install libconfig++-dev`.

## Visualization

Script [`plot_abag_data.py`](executables/plot_abag_data.py) can be used to plot the generated control data.
Run `plot_abag_data.py -h` for full usage documentation. A sample execution:

```
./executables/plot_abag_data.py ./build/ctrl_data_pos_z.csv ./build/desired_values.csv pos_z
```

## Maintainers:

* Minh Nguyen
  - email: `minh.nguyen at h-brs dot de`
* Djordje Vukcevic
  - email: `djordje.vukcevic at h-brs dot de`
  - GitHub: `djolemne`
