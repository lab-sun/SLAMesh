#pragma once
#ifndef TOOL_H_
#define TOOL_H_
#include "pointMatrix.h"
#include <iostream>
#include <chrono>
//eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <utility>
//ros
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include <visualization_msgs/Marker.h>
//pcl
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/common/distances.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/kdtree/kdtree_flann.h>

//format transform
Transf state2trans3(State state);
Transf state2quat2trans3(State state);
Transf state2quat2trans3_down(State state);
State trans32state(Transf & transf);
sensor_msgs::PointCloud matrix2DtoPclMsg(Eigen::MatrixXd matrix, int num_points);
sensor_msgs::PointCloud matrix3DtoPclMsg(Eigen::MatrixXd matrix, int num_points);
sensor_msgs::PointCloud matrix3DtoPclMsg(Eigen::MatrixXd matrix, int num_points, int skip_step);
Eigen::Matrix<double, 1, 6> txt2matrixSeattleOdom(int line, std::ifstream &file_odom);
pcl::PointCloud<pcl::PointXYZ> matrix3D2pcl(const PointMatrix & points);
void eigen2pcl();
geometry_msgs::PoseWithCovariance transf2PoseWithCovariance(Transf transf);
Transf PoseWithCovariance2transf(geometry_msgs::PoseWithCovariance pose);
bool readKitti(const std::string & file_dataset, const std::string& seq, int line_num, int dataset,
               pcl::PointCloud<pcl::PointXYZ> & laser_cloud);

//transformation
Point trans3Dpoint(int x, int y, int z, const Transf& transf);
Eigen::MatrixXd trans3Dpoints(const Eigen::MatrixXd &bef_rotate, Eigen::Isometry3d &T);
Transf createTrans(double x, double y, double z, double roll, double pitch, double yaw);

//filter
sensor_msgs::PointCloud2 pclMsg2VoxelFilter(const sensor_msgs::PointCloud2::ConstPtr& pcl_msg, double voxel_size) ;
pcl::PointCloud<pcl::PointXYZ> pclVoxelFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr & pcl, double voxel_size) ;

//visualize
void scanPrint2D(ros::Publisher& cloud_pub, PointMatrix& scan);
//void scanPrint3D(ros::Publisher& cloud_pub, PointMatrix& scan);
void scanPrint3D(ros::Publisher& cloud_pub, PointMatrix& scan, int skip_step);
void posePrint(ros::Publisher& cloud_pub, Eigen::Matrix<double, 3, Eigen::Dynamic> & pose, int pose_num);
bool visualizeArrow(ros::Publisher& lines_pub, PointMatrix start_points, PointMatrix end_points);

//others
inline double exp_quick(double x){
    x = 1.0 + x/64;
    x *= x;       x *= x;       x *= x;       x *= x;
    x *= x;       x *= x;
    return x;
}
double getPosi(double x_min, double y_min, double z_min, double grid);
double getPosiWithTime(double x_min, double y_min, double z_min, double grid, int visited_times);
class TicToc
{//from A-LOAM, thanks
public:
    TicToc()
    {
        tic();
    }
    explicit TicToc(std::string  _name):name(std::move(_name))
    {
        tic();
    }
    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    double toc()
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }
    void toc_print_ms(){
        std::cout<<name<<":"<< toc() << "ms"<< std::endl;
    }
    void toc_print_us(){
        std::cout<<name<<":"<< toc()*1000 << "us"<< std::endl;
    }

private:
    std::string name{"not named timer"};
    std::chrono::time_point<std::chrono::system_clock> start, end;
};
float computeMSE(const PointMatrix & points_source, const PointMatrix & points_target);
inline void evenSetLinSpaced(Eigen::Matrix<double, 1, Eigen::Dynamic> & test, int num_test, double min, double max, bool fullcover){
    //true false, cover all cell length
    //full_cover: test point location start from 0.5*step, like 0.5, 1.5, ..., 9.5
    //if full_cover == false: test point location start from 0*step, like 0, 1, ..., 10
    if(fullcover) {
        test = Eigen::MatrixXd::Zero(1, num_test);
        if(num_test!=1){
            double interval = (max - min) / (num_test*1.0-1);
            test(0,0) = min;
            for(int i=1; i< num_test-1; i++){
                test(0,i) = min + (i)*interval;
            }
            test(0,num_test-1) = max;
        }
    }
    else{
        test = Eigen::MatrixXd::Zero(1, num_test);
        double interval = (max - min) / (num_test*1.0);
        test(0,0) = min + 0.5 * interval;
        for(int i=1; i< num_test-1; i++){
            test(0,i) = min + (i+0.5)*interval;
        }
        test(0,num_test-1) = max - 0.5 * interval;
    }
}
bool extendEigen1dVector(Eigen::Matrix<double, 1, Eigen::Dynamic> & ori_vector, int extend_length);
#endif

