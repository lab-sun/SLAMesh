//This file is for the class SLAMesh, it is the main class of the project, and it is the node of ROS.
#ifndef SLAMESH_SLAMESHER_NODE_H
#define SLAMESH_SLAMESHER_NODE_H

#endif //SLAMESH_SLAMESHER_NODE_H
#include "cell.h"
#include "map.h"
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/registration/icp.h>
class Parameter{
    //algorithm parameter
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int    max_steps{10000}, register_times, num_test, min_points_num_to_gp, num_thread, cross_cell_overlap_length, dataset;
    double range_max, range_min, range_unit;
    double variance_register, variance_map_update, variance_map_show, variance_min, variance_sensor;
    double grid, voxel_size, converge_thr;

    double correction_x{0}, correction_y{0}, correction_z{0},
    correction_roll_degree{0}, correction_pitch_degree{0}, correction_yaw_degree{0};
    double test_param;
    double eigen_1, eigen_2, eigen_3;//PCA

    std::string file_loc_report, file_loc_dataset, seq;
    bool three_dir;//features fixed
    bool odom_available, read_offline_pcd, cross_overlap, grt_available, imu_feedback,
            meshing_tsdf, full_cover, save_raw_point_clouds, point2mesh{true},
            residual_combination{true};
    int visualisation_type;
    int num_margin_old_cell;
    double bias_acc_x, bias_acc_y;

    Parameter(){
    };
    void initParameter(ros::NodeHandle& nh);
};
class Log{
    //global variable, for recording and runtime
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    //record
    std::ofstream file_loc_report_wrt, file_loc_path_wrt, file_loc_path_odom_wrt, file_loc_path_grt_wrt;
    std::string log_file_path_GP_map_points, log_file_path_raw_pcl;
    double t_gp{0}, t_compute_rt{0};
    int log_length{0};
    Eigen::Matrix<double, 1, Eigen::Dynamic> num_cells_now, num_cells_glb, num_cells_new ,
            time_cost, time_get_pcl, time_gp, time_compute_rt, time_update,
            time_cost_draw_map, time_find_overlap, time_find_overlap_points, time_pub_odom,
            time_down_sample, time_devide_point,
            not_a_surface_cell_num, overlap_point_num, gp_times, rg_times;
    Eigen::Matrix<double, 3, Eigen::Dynamic> pose;
    double trajectory_length = 0;

    nav_msgs::Path path;
    nav_msgs::Path path_odom;
    nav_msgs::Path path_grt;
    PointMatrix pose_imu, pose_grt;

    pcl::PointCloud<pcl::PointXYZ> pcl_raw_accumulated;
    //for debug MSE error
    std::queue<PointMatrix> ary_last_raw_points;
    PointMatrix last_raw_points;

    //runtime
    //init
    Transf grt_first_transf;
    bool receive_first_imu = false, if_first_alt = false, imu_init = false, if_first_trans_init = false;
    int step = 0;
    //transformPoints
    std::vector<Transf> T_seq;

    Transf transf_odom_last  = Eigen::MatrixXd::Identity(4, 4),
           transf_odom_now   = Eigen::MatrixXd::Identity(4, 4),//used to calculate incremental transformation between two odometry frames
           transf_slam       = Eigen::MatrixXd::Identity(4, 4);
    std::vector<State> odom_offline;//not used
    std::queue<nav_msgs::OdometryConstPtr> odometry_msg_buf;
    //imu
    std::queue<sensor_msgs::ImuConstPtr> imu_msg_buf;
    Eigen::Vector3d imu_pos, imu_vel, imu_Ba, imu_Bg, acc_0, gyr_0;
    Eigen::Matrix3d imu_rot;
    Eigen::Vector3d g;
    //lidar
    std::deque<sensor_msgs::PointCloud2> pcl_msg_buff_deque;
    sensor_msgs::PointCloud2 pcl_msg_buff;

    Log();
    void initLog(std::string & log_file_path);
    void extendLog();
    Transf initFirstTransf();
    void updatePose(Transf & now_slam_transf);
    inline void recordPoseToPath(enum WhosPath whos_path, const Transf & transf){
        //save transformation to path msg
        //pose
        geometry_msgs::PoseStamped_<std::allocator<void>> path_tmp;
        path_tmp.pose.position.x = transf(0, 3);
        path_tmp.pose.position.y = transf(1, 3);
        path_tmp.pose.position.z = transf(2, 3);

        //orientation
        //transformPoints->tf::matrix->roll yaw pitch->tf::Q, there is no direct way from tf::matrix to tf::Q
        tf::Matrix3x3 tmp_m(transf(0, 0), transf(0, 1), transf(0, 2),
                            transf(1, 0), transf(1, 1), transf(1, 2),
                            transf(2, 0), transf(2, 1), transf(2, 2));
        double roll, yaw, pitch;
        tmp_m.getEulerYPR(yaw, pitch, roll);
        tf::Quaternion tmp_q;
        tmp_q.setRPY(roll, pitch, yaw);
        path_tmp.pose.orientation.x = tmp_q.x();
        path_tmp.pose.orientation.y = tmp_q.y();
        path_tmp.pose.orientation.z = tmp_q.z();
        path_tmp.pose.orientation.w = tmp_q.w();

        //save
        switch (whos_path){
            case Slam:{
                path.poses.push_back(path_tmp); break;
            }
            case Odom:{
                path_odom.poses.push_back(path_tmp); break;
            }
            case Grt:{
                path_grt.poses.push_back(path_tmp); break;
            }
            default: ROS_INFO("wrong usage of recordPoseToPath");
        }
    }
    void savePath2TxtKitti(std::ofstream & file_out, nav_msgs::Path & path_msg);
    void savePathTxt(std::ofstream & file_out, nav_msgs::Path & path_msg);
    void saveResult(double code_whole_time, const PointMatrix & map_glb_point_filtered);
    void pose_print(ros::Publisher& cloud_pub) const;
    void accumulateRawPoint(pcl::PointCloud<pcl::PointXYZ> pcl_raw, Transf& transf_this_step);
};
class SLAMesher{
    // node of this project
public:
    Parameter & param;//initialize ros parameters
    Log & g_data;//initialize global variables
    ros::NodeHandle & nh;

    ros::Publisher odom_pub,
    map_vertices_glb_pub, map_vertices_now_pub, raw_points_in_world_pub, overlap_point_glb, overlap_point_now,
    pose_pub, pose_imu_pub, path_pub, path_odom_pub, path_grt_pub,
    mesh_pub , mesh_pub_local, mesh_voxblox_pub, arrow_pub;

    ros::Subscriber pointcloud_sub, laser_sub, odom_sub, imu_sub, imu_raw_sub, alt_sub, ground_truth_sub, ground_truth_uav_sub;

    void groundTruthCallback(const geometry_msgs::PoseStamped::ConstPtr & ground_truth_msg);
    void groundTruthUavCallback(const nav_msgs::Odometry::ConstPtr & odom_msg);
    void altitudeCallback(const std_msgs::Float64 & alt_msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr & imu_msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr & odom_msg);
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr & pcl_msg);

    Transf getOdom();
    void imuIntegration(const sensor_msgs::ImuConstPtr & imu_msg);
    bool visualize(Map & map_glb, Map & map_now, int option);
    void pubTf();
    SLAMesher(ros::NodeHandle & nh_, Parameter & param_, Log & g_data_);
    void process();
};