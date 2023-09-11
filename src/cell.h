#pragma once
#ifndef MAP_CELL_H_
#define MAP_CELL_H_
//std
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <cmath>
#include <unordered_map>
#include <queue>
#include <ctime>
#include <chrono>
//eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>
//ros
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"
#include <tf/transform_broadcaster.h>
//pcl
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree_search.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>
//ceres
#include <ceres/ceres.h>
//meshing_tsdf
#include <mesh_msgs_conversions/conversions.h>
#include <mesh_msgs/MeshTexture.h>
//ours
#include "utility/pointMatrix.h"
#include "utility/tool.h"
#include "factor/lidarFactor.hpp"

#include "omp.h"


typedef  Eigen::Matrix <double, 1, Eigen::Dynamic> Matrix1xd;
enum Direction{
    X = 0, Y, Z, Unknown
};
enum MapState{
    INIT_ED, KD_TREE_ED, GP_ED
};
enum WhosPath{
    Slam, Odom, Grt
};
class Region{
    //define a voxel region, which is cells' borders
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double x_min{0}, y_min{0}, z_min{0}, x_max{0}, y_max{0}, z_max{0};
    Region(){}
    Region(double x_min_, double y_min_, double z_min_, double x_max_, double y_max_, double z_max_)
    : x_min(x_min_), y_min(y_min_), x_max(x_max_), y_max(y_max_), z_max(z_max_), z_min(z_min_){
    }
    double getDirMin(Direction dir){
        if(dir == X){
            return x_min;
        }
        else if (dir == Y){
            return y_min;
        }
        else if (dir == Z){
            return z_min;
        }
    }
    double getDirMax(Direction dir){
        if(dir == X){
            return x_max;
        }
        else if (dir == Y){
            return y_max;
        }
        else if (dir == Z){
            return z_max;
        }
    }
};
class Cell{
    // define a basic data structure of the map, a voxel cell containing points, several layers of mesh, and so on.
private:
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    //state
    bool empety = true, not_surface = true;
    int time_stamp = 0;
    Region region;
    double hash_position = 0;
    Point center {0,0,0};
    int revisited = 0;//if the sensor scan the same region after a certain time, revisited ++
    Point viewed_location {0, 0, 0};//sensor's location when this cell is observed
    int viewed_dir_count = 0;
    double average_viewed_distance = 0;//distance from sensor
    //data
    PointMatrix cell_raw_points;
    PointMatrix ary_cell_vertices[3];//direction X, Y, Z
    size_t updated_times[3];//if object is only observed in several frames, it may be dynamic object and will not shown in the final map
    //slamesh can also build tsdf map, it is optional


    void gaussianProcess(enum Direction gp_direction);
    void reconstructSurfaces(bool glb_cell_not_surface);
    void updateVertices(Cell & cell_new, enum Direction update_direction);
    void clearPointsQuick(){
        //may be memorial comsuming
        cell_raw_points.clear_quick();
        for(auto & i_vertices : ary_cell_vertices){
            i_vertices.clear_quick();
        }
        empety = true;
    }
    void updateViewedLocation(const Transf & transf_viewed);
    void initCell(PointMatrix & raw_points, int time_stamp_,
                  double posi_, Region region_,
                  bool reconstruct, bool not_surface_or_unknown_map_glb){
        //try to use it to reduce copy of cells in dividePointsIntoCells()
        cell_raw_points = raw_points;
        region = region_;
        hash_position = posi_;
        time_stamp = time_stamp_;
        not_surface = true;
        revisited = 0;
        viewed_location << 0,0,0;
        viewed_dir_count = 0;
        average_viewed_distance = 0;//distance from sensor
        memset(updated_times, 0, sizeof(updated_times));
        center << (region.x_min + region.x_max) / 2.0, (region.y_min + region.y_max) / 2.0,(region.z_min + region.z_max) / 2.0;
        if(reconstruct){
            reconstructSurfaces(not_surface_or_unknown_map_glb);// if no map_glb info, this value is true
        }
        empety = false;
    };


    Cell(PointMatrix & raw_points, int time_stamp_,
         double posi_, Region region_,
         bool reconstruct, bool not_surface_or_unknown_map_glb) :
            cell_raw_points(raw_points), region(region_), hash_position(posi_), time_stamp(time_stamp_){
        //used in dividePointsIntoCell, and dividePointsIntoCellInitMap.
        center << (region.x_min+region.x_max)/2.0, (region.y_min+region.y_max)/2.0,(region.z_min+region.z_max)/2.0;
        if(reconstruct){
            reconstructSurfaces(not_surface_or_unknown_map_glb);// if no map_glb info, this value is true
        }
        empety = false;
    };
    Cell(PointMatrix & raw_points, PointMatrix ary_cell_vertices_[], int time_stamp_,
         double posi_, Region region_) :
            cell_raw_points(raw_points), region(region_), hash_position(posi_), time_stamp(time_stamp_){
        // used in map_update add new cell, just copy all cell_vertices , not reconstruct again
        center << (region.x_min+region.x_max)/2.0, (region.y_min+region.y_max)/2.0,(region.z_min+region.z_max)/2.0;
        for(int i = 0; i < 3; i++){
            ary_cell_vertices[i] = ary_cell_vertices_[i];
        }
        empety = false;
    };
    Cell(){
    };
};

void VoxelFilter2D(int num_input, int & num_output, double grid,
                   Matrix1xd & f, Matrix1xd & train_x, Matrix1xd & train_y);
#endif