# SLAMesh
## SLAMesh: Real-time LiDAR Simultaneous Localization and Meshing

This is the code of SLAMesh.

**You are welcome to visit my [personal repository](https://github.com/RuanJY/SLAMesh) and raise issues there** so that I can receive immediate notifications. I may also update it more frequently but sometimes the result may slightly differ from the paper. 

### Update ###

17/Aug/2023, Code released. Feel free to contact me with any questions. We are confident that this work introduced a novel approach to LiDAR SLAM, and we welcome everyone to explore opportunities in this approach. :two_men_holding_hands:

10/Mar/2023, Preprint of our paper can be found on: [**paper**](https://arxiv.org/pdf/2303.05252.pdf 'title text'), [**slides**](https://drive.google.com/file/d/15xyX93LAsZ775Ywg8gNO_gJGThD4Hv3N/view?usp=sharing).

16/Jan/2023, The paper has been accepted for presentation on **ICRA 2023**.

15/Sep/2022, The paper has been submitted to ICRA 2023.

## 1. Introduction

This work designs a **S**imultaneously **L**ocalization **A**nd **Mesh**ing system (SLAMesh). Mesh is a lightweight 3-D dense model. It can model complex structures and has the feasibility for rendering. We bridge localization with meshing at the same time to benefit each other.

### 1.1 Main features

- Build, register, and update the **mesh** maps directly in **real-time** with CPU resources, around 40 Hz.
- Provide accurate **odometry**. Kitti odometry benchmark: 0.6763%.
- Established a **new** approach to LiDAR SLAM different from point-cloud, NDT, and surfel map SLAM.
- **Continuous** mapping with **uncertainty** via a reconstruction. Fast meshing, matching **without kd-tree**.

<div style="text-align: center;">
  <img src="https://github.com/RuanJY/SLAMesh/blob/master/fig/mesh_macity.jpg" alt="cover" width="60%" />
</div>

<div style="display: flex;">

[//]: # (  <img src="https://github.com/RuanJY/SLAMesh/blob/master/fig/slamesh_real_world_test_1.gif" width="50%" alt="slamesh_gif_real_world1">)
  <img src="https://github.com/RuanJY/SLAMesh/blob/master/fig/slamesh_real_world_test_2.gif" width="48%" alt="slamesh_gif_real_world2">
  <img src="https://github.com/RuanJY/SLAMesh/blob/master/fig/slamesh_kitti07.gif" width="48%" alt="slamesh_gif_kitti07">
</div>

### 1.2 Demo video

On public dataset:

<div align=center>
<a href="https://www.youtube.com/watch?v=bm9u0-d4giw" target="_blank"><img src="https://github.com/RuanJY/SLAMesh/blob/master/fig/cover.png" alt="video1" width="85%" /></a>
</div>

(or watch it on [bilibili](https://www.bilibili.com/video/BV1HB4y1J7k3/?vd_source=a7075e8cce0b5d3273610c2b2539377d))

On self-collected dataset:

<div align=center>
<a href="https://www.youtube.com/watch?v=-zMNndGmUho" target="_blank"><img src="https://github.com/RuanJY/SLAMesh/blob/master/fig/real_word_cover2.png" alt="video2" width="85%" /></a>
</div>

(or watch it on [bilibili](https://www.bilibili.com/video/BV1u84y1h7g3/?vd_source=a7075e8cce0b5d3273610c2b2539377d))

### 1.3 Find more detail

If you find our research helpful to your work, please cite our paper:

[1] Jianyuan Ruan, Bo Li, Yibo Wang, and Yuxiang Sun, "SLAMesh: Real-time LiDAR Simultaneous Localization and Meshing" ICRA 2023 ([**pdf**](https://arxiv.org/pdf/2303.05252.pdf 'title text'), [**IEEE**](https://ieeexplore.ieee.org/abstract/document/10161425 'title text'), [**slides**](https://drive.google.com/file/d/15xyX93LAsZ775Ywg8gNO_gJGThD4Hv3N/view?usp=sharing).).

```
@INPROCEEDINGS{10161425,
  author={Ruan, Jianyuan and Li, Bo and Wang, Yibo and Sun, Yuxiang},
  booktitle={2023 IEEE International Conference on Robotics and Automation (ICRA)}, 
  title={SLAMesh: Real-time LiDAR Simultaneous Localization and Meshing}, 
  year={2023},
  volume={},
  number={},
  pages={3546-3552},
  doi={10.1109/ICRA48891.2023.10161425}}
```

Other related papers:

[2] Jianyuan Ruan, Bo Li, Yinqiang Wang and Zhou Fang, "GP-SLAM+: real-time 3D lidar SLAM based on improved regionalized Gaussian process map reconstruction," IROS 2020. [link](https://ieeexplore.ieee.org/abstract/document/9341028).

[3] Bo Li, Yinqiang Wang, Yu Zhang. Wenjie Zhao, Jianyuan Ruan, and Pin Li, "GP-SLAM: laser-based SLAM approach based on regionalized Gaussian process map reconstruction". Autonomous Robot 2020.[link](https://link.springer.com/article/10.1007/s10514-020-09906-z)


If you understand Chinese, you can also refer to my [Master's thesis](https://connectpolyu-my.sharepoint.com/:b:/g/personal/21041552r_connect_polyu_hk/ESjrlb1oNbVMr4tFeG4bhY0BO0jmY-hlC61a3y67whp-Ww?e=pqOtjT), an article on the WeChat platform: [SLAMesh: 实时LiDAR定位与网格化模型构建
](https://mp.weixin.qq.com/s/zYORZ1sOVkh-UnPkzzfh_g), and a talk in [自动驾驶之心and计算机视觉life](https://www.bilibili.com/video/BV1j84y1Z7Fb/?vd_source=a7075e8cce0b5d3273610c2b2539377d).


## 2. Build

### 2.1 Prerequisite

We tested our code in *Ubuntu18.04* with *ROS melodic* and *Ubuntu20.04* with *ROS neotic*.

**ROS**

Install ros following [ROS Installation](http://wiki.ros.org/melodic/Installation/Ubuntu). We use the PCL and Eigen library in ROS.

**Ceres**

Install Ceres Solver, version 2.0 or 2.1. follow [Ceres Installation](http://ceres-solver.org/installation.html).

Notice that version > 2.1 may have some compatibility issues with our code. So you can use following command to install ceres:
```
apt-get install cmake libgoogle-glog-dev libgflags-dev libatlas-base-dev libsuitesparse-dev -y
git clone https://github.com/ceres-solver/ceres-solver.git -b 2.1.0
mkdir ceres-bin && cd ceres-bin && cmake .. && make -j$(($(nproc)-2))
make install
```

**mesh_tools**

We use mesh_tools to visualize the mesh map with the `mesh_msgs::MeshGeometryStamped` ROS message. mesh_tools also incorporates navigation functions upon mesh map. [Mesh tool introduction](https://github.com/naturerobots/mesh_tools)

Install mesh_tools by:

1. Install [lvr2](https://github.com/uos/lvr2):
```
sudo apt-get install build-essential \
     cmake cmake-curses-gui libflann-dev \
     libgsl-dev libeigen3-dev libopenmpi-dev \
     openmpi-bin opencl-c-headers ocl-icd-opencl-dev \
     libboost-all-dev \
     freeglut3-dev libhdf5-dev qtbase5-dev \
     qt5-default libqt5opengl5-dev liblz4-dev \
     libopencv-dev libyaml-cpp-dev
```
If in Ubuntu18.04, use `libvtk6` because `libvtk7` will conflict with `pcl-ros` in melodic.
```
sudo apt-get install  libvtk6-dev libvtk6-qt-dev
```
Else if, in Ubuntu 20.04,
```
sudo apt-get install  libvtk7-dev libvtk7-qt-dev
```
End of IF

then:
```
cd a_non_ros_dir
```
build:
```
git clone https://github.com/uos/lvr2.git
cd lvr2 
mkdir build && cd build
cmake .. && make
sudo make install
```
It may take you some time.

2. Install mesh_tools, (I can not install it from official ROS repos now, so I build it from source)

```
cd slamesh_ws/src
git clone https://github.com/naturerobots/mesh_tools.git
cd ..
rosdep update
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

### 2.2 SLAMesh

Clone this repository and build:
```
cd slamesh_ws/src
git clone https://github.com/RuanJY/SLAMesh.git
cd .. && catkin_make
mkdir slamesh_result
source ~/slamesh_ws/src/devel/setup.bash
```
### 2.3 Docker support

If you encounter some troubles when building slamesh, the problem may lay down on the prerequisite; we advise you to use the docker image:

**Option 1**, Build the docker image use Dockerfile:

```
cd slamesh_ws/src/SLAMesh/docker/
chmod +x run_docker.sh
./run_docker.sh -b
```

Run the docker image:
```
docker run -it --rm     --gpus=all     --runtime=nvidia     -e NVIDIA_DRIVER_CAPABILITIES=all     --env="DISPLAY=$DISPLAY" \
-e "QT_X11_NO_MITSHM=1"     --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"     --env="XAUTHORITY=$XAUTH" --volume="$XAUTH:$XAUTH" \
--net=host     --privileged \
--volume=path_of_dataset_in_your_PC:/home/ruanjy/Dataset/ \
--volume=/home/$USER/slamesh_result:/home/slamesh/slamesh_ws/slamesh_result \
--name=test_slamesh    slamesh     bash
```
change the `path_of_dataset_in_your_PC` to the folder path of the dataset in your PC, like `--volume=/home/ruanjy/kitti_odometry/data_odometry_velodyne/dataset/:/home/ruanjy/Dataset/ \`

In this way, SLAMesh are built already, so source the workspace and run the launch file. You can also play rosbag in your host PC.

**Option 2**, otherwise, you can also pull the pre-built docker image with VNC directly:

```
docker pull pleaserun/rjy_slam_work:slamesh_18.04
```

Run the docker image:
```
docker run -it -p 5900:5900 -p 2222:22 -e RESOLUTION=1920x1080  \
--volume=path_of_dataset_in_your_PC:/home/ruanjy/Dataset/ \
--volume=/home/$USER/slamesh_result:/home/slamesh/slamesh_ws/slamesh_result \
--name test_slamesh pleaserun/rjy_slam_work:slamesh_18.04
```

change the `path_of_dataset_in_your_PC` to the folder path of the dataset in your PC, like `--volume=/home/ruanjy/kitti_odometry/data_odometry_velodyne/dataset/:/home/ruanjy/Dataset/ \`

In this way you can use VNC to enter a graphical interface via port 5900, or use ssh to connect container via port 2222.

Move to the dictionary `slamesh_ws/src`, and complete step from 2.2.

## 3. Usage

### 3.1 Run kitti dataset:

The dataset is available at [KITTI dataset](https://www.cvlibs.net/datasets/kitti/eval_odometry.php).

Set the parameter `data_path` in `slamesh_kitti.launch` to your folder of kitti dataset path

The file tree should be like this:

```
file_loc_dataset
    ├── 00
    |   └──velodyne
    |       ├── 000000.bin
    |       └── ...
    └──01   

```
For example, if you want to run the 07 sequence:
```
roslaunch slamesh slamesh_kitti_meshing.launch seq:=/07
```
You should get:

<div align=center>
<img src="https://github.com/RuanJY/SLAMesh/blob/master/fig/kitti07_mesh.png" alt="kitti07_mesh" width="60%" />
</div>

If you can not see the mesh, check that the Rviz plugin is sourced correctly. When `mesh_visualization` is disabled, only vertices are published as a point cloud.


### 3.2 Run Mai City dataset:

The dataset is available at [Mai City Dataset](https://www.ipb.uni-bonn.de/data/mai-city-dataset/). Sequence 01 can be fed into SLAM and sequence 02 can be accumulated into a dense ground truth point cloud map.

Similarly, set the parameter `data_path` in `slamesh_maicity.launch` to your folder of the kitti dataset path.

```
roslaunch slamesh slamesh_maicity.launch seq:=/01
```
You should get:

<div align=center>
<img src="https://github.com/RuanJY/SLAMesh/blob/master/fig/maicity_mesh.png" alt="maicity_mesh" width="60%" />
</div>

### 3.3 Run online or ros bag
```
roslaunch slamesh slamesh_online.launch
```
And play your bag, in launch file, remap the topic "/velodyne_points" to your LiDAR topic like "/os_cloud_node/points".
```
rosbag play your_bag.bag 
```
The number of LiDAR channels does not matter because our algorithm does not extract features.

You can use our sample data recorded with an Ouster OS1-32 LiDAR: [SLAMesh dataset](https://connectpolyu-my.sharepoint.com/:f:/g/personal/21041552r_connect_polyu_hk/EjhEKl8-1GBLseA_2F7TOvEB3w7OyAJ_kS7DAaWoLay9ng?e=GK1fsd).

### 3.4 About visualization

Because mesh-tools rviz plugin do not support incremental mesh intersection, we provide three visualization modes controlled by `visualisation_type` in the param.yaml file:

visualisation_type =

lighter &#x2191;

- 0, publish registered raw points in world frame, just like fast-lio, lio sam
- 1, + publish the vertices of mesh as point cloud, each scan + (1/n) map global
- 2, + visualize local updated mesh, each scan
- 3, + visualize global mesh, (1/n) frame

heavy &#x2193;

after finish whole process, the global mesh map will be visualized in any mode

## 4. Evaluation

SLAMesh saves all its report to the path `result_path` given in each **launch** file. If you find ros warning: ` Can not open Report file`, create the folder of `result_path` first.

### 4.1 Kitti odometry accuracy

Run SLAMesh by:
```
roslaunch slamesh slamesh_kitti_odometry.launch seq:=/07
```

Then SLAMesh produce a file named `0*_pred.txt` in the KITTI path format. I use [KITTI odometry evaluation tool
](https://github.com/LeoQLi/KITTI_odometry_evaluation_tool) for evaluation:

```
cd slamesh_ws/slamesh_result
git clone https://github.com/LeoQLi/KITTI_odometry_evaluation_tool
cd KITTI_odometry_evaluation_tool/
python evaluation.py --result_dir=.. --eva_seqs=07.pred
```

The result on the KITTI odometry benchmark is:

[//]: # ()
[//]: # (| Sequence         | 00     | 01     | 02     | 03     | 04     | 05     | 06     | 07     | 08     | 09     | 10     | Average |)

[//]: # (| ---------------- | ------ | ------ | ------ | ------ | ------ | ------ | ------ | ------ | ------ | ------ | ------ | ------- |)

[//]: # (| Translation &#40;%&#41;  | 0.771  | 1.2519 | 0.7742 | 0.6366 | 0.5044 | 0.5182 | 0.5294 | 0.3607 | 0.8745 | 0.573  | 0.6455 | 0.6763  |)

[//]: # (| Rotation &#40;deg/m&#41; | 0.0035 | 0.003  | 0.003  | 0.0043 | 0.0013 | 0.003  | 0.0022 | 0.0023 | 0.0027 | 0.0025 | 0.0042 | 0.0029  |)

<div align=center>
<img src="https://github.com/RuanJY/SLAMesh/blob/master/fig/table_odometry_accuracy.png" alt="table_odometry_accuracy" width="100%" />
</div>


<div align=center>
<img src="https://github.com/RuanJY/SLAMesh/blob/master/fig/slamesh_kitti_path.png" alt="slamesh_kitti_path" width="40%" />
</div>

Why use the `slamesh_kitti_odometry.launch` ? To achieve better KITTI odometry performance, the parameter in `slamesh_kitti_odometry.launch` are set as followed:
```
full_cover: false # due to discontinuity phenomenon between cells, shirnk the test locations can improve accuracy.
num_margin_old_cell: 500 # margin old cells, because KITTI evaluate odometry accuracy rather than consistency.
```

While in launch `slamesh_kitti_meshing.launch`, to have better meshing result, they aree:
```
full_cover: true # so that there is no gap between cells.
num_margin_old_cell: -1  # do not margin old cells, the cell-based map will have implicit loop closure effect.
```

### 4.2 Mesh quality

<div align=center>
<img src="https://github.com/RuanJY/SLAMesh/blob/master/fig/kitti_mesh_sample.png" alt="kitti_mesh_sample" width="100%" />
</div>

To save the mesh map, set parameter `save_mesh_map` in yaml file to `true`. A `***.ply` file should be saved in `slamesh_ws/slamesh_result`.

I use the `TanksAndTemples/evaluation` tool to evaluate the mesh. I slightly modify it (remove trajectory). You can find it here: [TanksAndTemples/evaluation_rjy](https://github.com/RuanJY/TanksAndTemples)

Then compare the mesh with the ground-truth point cloud map:
```
cd TanksAndTemples_direct_rjy/python_toolbox/evaluation/
python run.py \
--dataset-dir ./data/ground_truth_point_cloud.ply \
--traj-path ./ \
--ply-path ./data/your_mesh.ply
```

### 4.3 Time cost

The direct meshing method enables SLAMesh 180x faster than Poisson reconstruction based method, Puma, in the Maicity dataset. The time cost of each step is recorded in the `***_report.txt` file.

<div align=center>
<img src="https://github.com/RuanJY/SLAMesh/blob/master/fig/timecost.png" alt="timecost" width="60%" />
</div>

## 5. Help you to read the code

TODO

## Contact

Author: Jianyuan Ruan, Bo Li, Yibo Wang, Yuxiang Sun.

Email: jianyuan.ruan@connect.polyu.hk, 120052@zust.edu.cn, me-yibo.wang@connect.polyu.hk, yx.sun@polyu.edu.hk

## Acknowledgement

Most of the code are build from scratch , but we also want to acknowledge the following open-source projects:

[TanksAndTemples/evaluation](https://github.com/isl-org/TanksAndTemples/tree/master/python_toolbox/evaluation): evaluation

[A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM): kitti dataset loader and tic-toc

[F-LOAM](https://github.com/wh200720041/floam): help me to write the ceres residuals

[VGICP](https://github.com/SMRT-AIST/fast_gicp): multi-thread




