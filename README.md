# kinova_control_experiments

Simple impedance controller for Kinova Gen 3 arm which was tested on a real robot. This requries ROS to work because of `kdl_parser` for URDF files.

The kinova zip file is added using `git-lfs`, see [project page](https://git-lfs.github.com)
for setup instruction. All necessary headers, libraries and URDF model are expected under `kinova_libs/extracted` directory.

Script [`plot_abag_data.py`](executables/plot_abag_data.py) can be used to plot the generated control data. Run `plot_abag_data.py -h` for full usage.
A sample execution:
```
./executables/plot_abag_data.py ./build/ctrl_data_pos_z.csv ./build/desired_values.csv pos_z
```
