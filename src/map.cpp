#include "map.h"
#include "slamesher_node.h"

extern Parameter param;//in SLAMesh node
extern Log g_data;//in SLAMesh node

bool getPointCloud(PointMatrix & points_result, pcl::PointCloud<pcl::PointXYZ> & pcl_got, double & get_laser_time,
                   double voxel_filter_size){
    // get the new point cloud, from pcd file or from ros message
    ROS_DEBUG("getPointCloud");
    TicToc t_get_pcl;
    double range_max = param.range_max, range_min = param.range_min;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_raw_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_filtered_ptr(new pcl::PointCloud<pcl::PointXYZ>);//downsampled
    PointMatrix points_raw, points_filted;
    int data_set = param.dataset;//1  kitti, 2 maicity,
    //read
    if(param.read_offline_pcd) {
        if(!readKitti(param.file_loc_dataset, param.seq, g_data.step, data_set,  *pcl_raw_ptr)){
            std::cout << "No more PCD file!" << std::endl;
            return false;
        }
    }
    else{
        ros::spinOnce();
        while(ros::ok() && g_data.pcl_msg_buff_deque.empty()){
            //ROS_INFO("NO pcl scan, Wait 2 second...");
            usleep(100);
            ros::spinOnce();
        }
        bool first = true;
        while(ros::ok() && g_data.pcl_msg_buff_deque.empty()){
            if(first) ROS_INFO("Waiting for new pcl scan in get point cloud...");
            usleep(1);
            ros::spinOnce();
            first = false;
        }
        if(!g_data.pcl_msg_buff_deque.empty()){
            //get the message
            pcl::fromROSMsg(g_data.pcl_msg_buff_deque.front(), *pcl_raw_ptr);
            get_laser_time = g_data.pcl_msg_buff_deque.front().header.stamp.toSec();
            g_data.pcl_msg_buff = g_data.pcl_msg_buff_deque.front();
            g_data.pcl_msg_buff_deque.pop_front();
        }
        else{
            return false;
        }
    }

    //voxel_filter_size filter
    TicToc t_down_sample;
    if(voxel_filter_size > 0){
        *pcl_filtered_ptr = pclVoxelFilter(pcl_raw_ptr, voxel_filter_size);
    }
    else{
        pcl_filtered_ptr = pcl_raw_ptr;
    }
    //pcl::transformPointCloud(*pcl_filtered_ptr, *pcl_filtered_ptr, g_data.lidar_install_transf.cast<float>());
    points_raw = pcl_filtered_ptr->getMatrixXfMap().cast<double>();
    //range filter
    int filter_point_step = 1;//simple down-sample
    double range_max_square = range_max * range_max,
           range_min_sqaure = range_min * range_min;
    if(range_max <= 100){
        points_result.clear_quick();
        for(int i = 0; i < points_raw.num_point; i += filter_point_step){
            double range_sqaure = points_raw.point(0, i) * points_raw.point(0, i) +
                                  points_raw.point(1, i) * points_raw.point(1, i) +
                                  points_raw.point(2, i) * points_raw.point(2, i);
            if(range_sqaure < range_max_square && range_sqaure > range_min_sqaure ){
                points_result.addPoint(points_raw.point.col(i));
            }
        }
    }
    else{
        points_result.clear_quick();
        points_result = points_raw;
    }
    std::cout << "after filter:" << points_result.num_point << " points. time: " << t_get_pcl.toc() << std::endl;
    if(points_result.num_point <= 0){
        std::cout << "No valid points in the scan, please check your data or parameters, range_max and range_min." << std::endl;
        return false;
    }
    pcl_got = *pcl_filtered_ptr;
    g_data.time_down_sample(0, g_data.step) = t_down_sample.toc();
    g_data.time_get_pcl(0, g_data.step) = t_get_pcl.toc();
    return true;
}

Map::Map(Transf & Tguess): step(0){
    //constructor of the first map glb
    ROS_DEBUG("Map");
    name = "MAP";
    //TicToc t_get_pcl, t_turn_point, t_gp;
    getPointCloud(points_turned, pcl_raw, timestamp, param.voxel_size);
    //std::cout<<"t_get_pcl:"<<t_get_pcl.toc() << "ms"<< std::endl;
    points_turned.transformPoints(Tguess);
    dividePointsIntoCellInitMap(points_turned);
    cells_glb.reserve(5000);
}
Map::Map(){
    //constructor of the map now
    ROS_DEBUG("Map");
    name = "CURRENT_SCAN";
    cells_now.resize(pow(2 * (param.range_max / param.grid) + 2, 3) / 5);
}

void  Map::dividePointsIntoCellInitMap(PointMatrix & points_raw){
    //Divide raw points into cells and conduct gaussian process reconstrution if there are enough points,
    // cells are adjacent voxels in the world frame
    // this function initialze the global map
    ROS_DEBUG("dividePointsIntoCellInitMap");
    TicToc t_gp;
    double range_max = (param.range_max>100) ? 100 : param.range_max;
    double grid = param.grid, min_num_to_gp = param.min_points_num_to_gp;
    cells_glb.clear();
    //point division
    //find a bounding box of current raw points in world frame
    double max_x = points_raw.point.block(0, 0, 1, points_raw.num_point).maxCoeff();
    double max_y = points_raw.point.block(1, 0, 1, points_raw.num_point).maxCoeff();
    double max_z = points_raw.point.block(2, 0, 1, points_raw.num_point).maxCoeff();
    double min_x = points_raw.point.block(0, 0, 1, points_raw.num_point).minCoeff();
    double min_y = points_raw.point.block(1, 0, 1, points_raw.num_point).minCoeff();
    double min_z = points_raw.point.block(2, 0, 1, points_raw.num_point).minCoeff();
    double  bot_grid_x = grid*floor(min_x/grid),
            bot_grid_y = grid*floor(min_y/grid),
            bot_grid_z = grid*floor(min_z/grid);
    int num_grid_x = int(ceil(max_x/grid)-floor(min_x/grid)),
        num_grid_y = int(ceil(max_y/grid)-floor(min_y/grid)),
        num_grid_z = int(ceil(max_z/grid)-floor(min_z/grid));
    int num_grid = num_grid_x * num_grid_y * num_grid_z;
    //allocate memory
    PointMatrix init_points_bucket;
    Cell init_cell;
    //int num_bucket = pow(2 * (range_max / grid) + 2, 3) / 1.3;//vertical FOV<90 will be enough, memory comsuming here
    int num_bucket = num_grid_x * num_grid_y * num_grid_z;//vertical FOV<90 will be enough, memory comsuming here

    std::vector<PointMatrix> ary_points_bucket(num_bucket, init_points_bucket);
    std::vector<Cell> ary_bucket_cell(num_bucket, init_cell);

    if(num_grid > ary_points_bucket.size()){
        //std::cout << "num_grid " << num_grid << " " << "ary_points_bucket.size " ;
        ROS_ERROR("Num grid > reserved in ary_points_bucket!");
        while(num_grid > ary_points_bucket.size()){
            ary_points_bucket.push_back(init_points_bucket);
            ary_bucket_cell.push_back(init_cell);
            //cells_now.push_back(std::pair<double, Cell> (0, init_cell));
            num_bucket++;
            std::cout << ary_points_bucket.size() << " ";
        }
        std::cout<<std::endl;
    }

    //clear
    for(int i = 0; i < num_grid; i++){
        ary_points_bucket[i].clear_quick();//clear buckets
        ary_bucket_cell[i].clearPointsQuick();
    }
    index_bucket_enough_point.clear();

    //push points into bucket
    int i_grid;
    for(int i = 0; i < points_raw.num_point; i ++){
        i_grid = 1 *                       floor((points_raw.point(0, i) - bot_grid_x) / grid) +
                 num_grid_x *              floor((points_raw.point(1, i) - bot_grid_y) / grid) +
                 num_grid_x * num_grid_y * floor((points_raw.point(2, i) - bot_grid_z) / grid);
        ary_points_bucket[i_grid].addPoint(points_raw.point.col(i));
    }
    //remember which buckets have points
    for(int i_bucket = 0; i_bucket < num_grid; i_bucket ++){
        if(ary_points_bucket[i_bucket].num_point > min_num_to_gp){
            index_bucket_enough_point.push_back(i_bucket);
        }
    }

    //reconstruction
    #pragma omp parallel for num_threads(param.num_thread) default(shared)
    for(int i_bucket = 0; i_bucket < index_bucket_enough_point.size(); i_bucket ++){
        //for all buckets have enough raw points
        PointMatrix & points_raw_cell = ary_points_bucket[index_bucket_enough_point[i_bucket]];

        double  iz = ( index_bucket_enough_point[i_bucket] / (num_grid_x * num_grid_y)) * grid + bot_grid_z,
                iy = ((index_bucket_enough_point[i_bucket] % (num_grid_x * num_grid_y)) / num_grid_x) * grid + bot_grid_y,
                ix = ((index_bucket_enough_point[i_bucket] % (num_grid_x * num_grid_y)) % num_grid_x) * grid + bot_grid_x;
        double posi_tmp;
        posi_tmp = getPosiWithTime(ix, iy, iz, grid, 0);
        Region region_tmp(ix, iy, iz, ix + grid, iy + grid,iz + grid);

        Cell tmp_cell(points_raw_cell, g_data.step,
                      posi_tmp, region_tmp,
                      true, true);
        ary_bucket_cell[index_bucket_enough_point[i_bucket]] = tmp_cell;//usage?
    }
    //because emplace are not suitable to be multi-threaded, so use ary_bucket_cell to store cells first
    for(int i_bucket = 0; i_bucket < index_bucket_enough_point.size(); i_bucket++){
        cells_glb.emplace(ary_bucket_cell[index_bucket_enough_point[i_bucket]].hash_position,
                          ary_bucket_cell[index_bucket_enough_point[i_bucket]]);
    }
    std::cout<<"t_gp:"<<t_gp.toc() << "ms"<< std::endl;
    g_data.time_gp(0, g_data.step) += t_gp.toc();
}
void  Map::dividePointsIntoCell(PointMatrix & points_raw, const Map & map_glb, bool conduct_gp){
    // map_glb and map_now use different data structure to store cells. map_glb use unordered map for fast query,
    // map_now use vector for parallelization, so there are two dividePointsIntoCell functions.
    // when deciding whether to conduct GP reconstruction, we use the information from map_glb
    ROS_DEBUG("dividePointsIntoCell");
    TicToc t_gp;
    double range_max = (param.range_max>100) ? 100 : param.range_max;
    double grid = param.grid, min_num_to_gp = param.min_points_num_to_gp;
    //point division
    //find a bounding box of current raw points in world frame
    double max_x = points_raw.point.block(0, 0, 1, points_raw.num_point).maxCoeff();
    double max_y = points_raw.point.block(1, 0, 1, points_raw.num_point).maxCoeff();
    double max_z = points_raw.point.block(2, 0, 1, points_raw.num_point).maxCoeff();
    double min_x = points_raw.point.block(0, 0, 1, points_raw.num_point).minCoeff();
    double min_y = points_raw.point.block(1, 0, 1, points_raw.num_point).minCoeff();
    double min_z = points_raw.point.block(2, 0, 1, points_raw.num_point).minCoeff();
    double  bot_grid_x = grid * floor(min_x / grid),
            bot_grid_y = grid * floor(min_y / grid),
            bot_grid_z = grid * floor(min_z / grid);
    int num_grid_x = int(ceil(max_x / grid) - floor(min_x / grid)),
        num_grid_y = int(ceil(max_y / grid) - floor(min_y / grid)),
        num_grid_z = int(ceil(max_z / grid) - floor(min_z / grid));
    int num_grid = num_grid_x * num_grid_y * num_grid_z;
    //allocate memory
    PointMatrix init_points_bucket;
    Cell init_cell;
    //int num_bucket = pow(2 * (range_max / grid) + 2, 3) / 5;//vertical FOV<90 will be enough, you can decrease this
    int num_bucket = num_grid_x * num_grid_y * num_grid_z;//vertical FOV<90 will be enough, memory comsuming here
    if(g_data.step == 1){
        cells_now.resize(num_bucket);
    }
    // initial size of pointMatrix to reduce memory comsumption here
    static std::vector<PointMatrix> ary_points_bucket(num_bucket, init_points_bucket);
    static std::vector<int> index_bucket_not_empty;
    if(num_grid > ary_points_bucket.size()){
        TicToc t_pushing_new_bucket;
        while(num_grid > ary_points_bucket.size()){
            ary_points_bucket.push_back(init_points_bucket);
            cells_now.push_back(std::pair<double, Cell> (0, init_cell));
            num_bucket++;
        }
        ROS_WARN("In dividePointsIntoCell, Num grid %d > reserved size in ary_points_bucket %d, pushing..., cost time %d  ms",
                 num_grid, int(ary_points_bucket.size()), int(t_pushing_new_bucket.toc()));
    }

    //clear
    for(int i_bucket = 0; i_bucket < index_bucket_not_empty.size(); i_bucket++){
        ary_points_bucket[index_bucket_not_empty[i_bucket]].clear_quick();
    }//clear bucket, index_bucket_not_empty is used to go through unempty buckets only and save time
    for(int i_bucket = 0; i_bucket < index_bucket_enough_point.size(); i_bucket++){
        cells_now[index_bucket_enough_point[i_bucket]].second.clearPointsQuick();
    }//clear cells
    index_bucket_enough_point.clear();
    index_bucket_not_empty.clear();

    //push points into bucket
    TicToc t_push_points;
    int i_grid;
    for(int i = 0; i < points_raw.num_point; i++){
        i_grid = 1 *                       floor((points_raw.point(0, i) - bot_grid_x) / grid) +
                 num_grid_x *              floor((points_raw.point(1, i) - bot_grid_y) / grid) +
                 num_grid_x * num_grid_y * floor((points_raw.point(2, i) - bot_grid_z) / grid);
        ary_points_bucket[i_grid].addPoint(points_raw.point.col(i));
    }
    //remember which buckets have points
    for(int i_bucket = 0; i_bucket < num_grid; i_bucket++){
        if(ary_points_bucket[i_bucket].num_point > min_num_to_gp){
            index_bucket_enough_point.push_back(i_bucket);
        }
        if(ary_points_bucket[i_bucket].num_point > 0){
            index_bucket_not_empty.push_back(i_bucket);
        }
    }

    //reconstruction
    #pragma omp parallel for num_threads(param.num_thread)
    for(int i_bucket_not_empty = 0; i_bucket_not_empty < index_bucket_enough_point.size(); i_bucket_not_empty++){
        //for all buckets have enough raw points
        PointMatrix & points_raw_cell = ary_points_bucket[index_bucket_enough_point[i_bucket_not_empty]];
        //TicToc t_gp_pre_info;
        double  iz = ( index_bucket_enough_point[i_bucket_not_empty] / (num_grid_x * num_grid_y)) * grid + bot_grid_z,
                iy = ((index_bucket_enough_point[i_bucket_not_empty] % (num_grid_x * num_grid_y)) / num_grid_x) * grid + bot_grid_y,
                ix = ((index_bucket_enough_point[i_bucket_not_empty] % (num_grid_x * num_grid_y)) % num_grid_x) * grid + bot_grid_x;
        double posi_tmp;
        posi_tmp = getPosiWithTime(ix, iy, iz, grid, 0);
        Region region_tmp(ix, iy, iz, ix + grid, iy + grid,iz + grid);

        bool map_glb_not_surface;
        if(step != 0){//check wheather this cell can form a surfece in map_glb
            auto cell_glb = map_glb.cells_glb.find(posi_tmp);
            if(cell_glb != map_glb.cells_glb.end()){
                map_glb_not_surface = cell_glb->second.not_surface;
            }
        }
       std::pair<double, Cell> tmp_cell (posi_tmp, Cell(points_raw_cell, g_data.step,
                                                        posi_tmp, region_tmp,
                                                        conduct_gp, map_glb_not_surface));
        tmp_cell.second.time_stamp = g_data.step;
        cells_now[index_bucket_enough_point[i_bucket_not_empty]] = tmp_cell;
    }
    std::cout<<"t_gp:"<<t_gp.toc() << "ms"<< std::endl;
    g_data.time_gp(0, g_data.step) += t_gp.toc();
}
bool  Map::processNewScan(Transf & Tguess, int step_, const Map & map_glb){
    //CURRENT_SCAN procee new scan
    ROS_DEBUG("processNewScan");
    step = step_;
    points_turned.clear_quick();
    vertices_filted.clear_quick();
    for(int i = 0; i < 3; i ++){
        ary_vertices_all_show[i].clear_quick();
        ary_overlap_vertices[i].clear_quick();
    }
    name = "CURRENT_SCAN";
    TicToc t_get_pcl, t_turn_point, t_gp;
    if(!getPointCloud(points_turned, pcl_raw, timestamp, param.voxel_size)){
        return false;
    }
    g_data.time_get_pcl(0, g_data.step) = t_get_pcl.toc();
    std::cout<<"t_get_pcl:"<<t_get_pcl.toc() << "ms"<< std::endl;
    points_turned.transformPoints(Tguess);
    dividePointsIntoCell(points_turned, map_glb, true);
    return true;
}

OverlapCellsRelation Map::overlapCells(Map & map_glb){
    // find overlapped cells between map_glb and map_now
    ROS_DEBUG("overlapCells");
    TicToc t_overlap_region;
    OverlapCellsRelation overlap_ship;
    //g_data.un_bounded_count = 0;

    for(int i = 0; i < index_bucket_enough_point.size(); i++){
        std::pair<double, Cell> & i_cell_now = cells_now[index_bucket_enough_point[i]];
        //don't consider revisit
        auto i_cell_glb = map_glb.cells_glb.find(i_cell_now.second.hash_position);
        if(i_cell_glb != map_glb.cells_glb.end() && (param.num_margin_old_cell < 0 ||
            std::abs(g_data.step - i_cell_glb->second.time_stamp) < param.num_margin_old_cell)){
            //if(i_cell_glb != map_glb.cells_glb.end() && (!param.margin_old_cell || g_data.step - i_cell_glb->second.time_stamp < 500)){
            Cell *ptr_cell_glb = &(i_cell_glb->second);
            Cell *ptr_cell_now = &(i_cell_now.second);
            overlap_ship.cells_glb.push_back(ptr_cell_glb);
            overlap_ship.cells_now.push_back(ptr_cell_now);
        }
        else{
            Cell* ptr_cell_new = &(i_cell_now.second);
            overlap_ship.cells_now_new.push_back(ptr_cell_new);
        }
    }
    g_data.time_find_overlap(0, g_data.step) += t_overlap_region.toc();
    return overlap_ship;
}
OverlapCellsRelation Map::overlapCellsCrossCell(Map& map_glb, int overlap_length){
    ROS_DEBUG("overlapCellsCrossCell");
    TicToc t_overlap_region;
    double grid = param.grid;
    OverlapCellsRelation overlap_ship;
    //g_data.un_bounded_count = 0;
    //for(auto& i_cmap_now : cells_glb){
    for(int i = 0; i < index_bucket_enough_point.size(); i++){
        std::pair<double, Cell> & i_cell_now = cells_now[index_bucket_enough_point[i]];
        std::vector<Cell*> multi_overlap_cell_glb(6 * overlap_length + 1);
        std::vector<double> overlap_index(6 * overlap_length + 1);//just for test
        //don't consider revisit
        auto i_cell_glb = map_glb.cells_glb.find(i_cell_now.second.hash_position);
        //std::cout<<"ori hash_position:"<<i_cell_now.second.hash_position<<" "<<std::endl;
        if(i_cell_glb != map_glb.cells_glb.end() &&
           (param.num_margin_old_cell < 0 ||
            std::abs(g_data.step - i_cell_glb->second.time_stamp) < param.num_margin_old_cell)){
            //if(i_cell_glb != map_glb.cells_glb.end() && (!param.margin_old_cell || g_data.step - i_cell_glb->second.time_stamp < 300)){
            //if(i_cell_glb->second.isBounded(0.5) && i_cell_now.second.isBounded(0.5)){
            Cell* ptr_cell_glb = &(i_cell_glb->second);
            Cell* ptr_cell_now = &(i_cell_now.second);
            overlap_ship.cells_glb.push_back(ptr_cell_glb);
            overlap_ship.cells_now.push_back(ptr_cell_now);
            multi_overlap_cell_glb[0]=(ptr_cell_glb);
            overlap_index[0] = i_cell_now.second.hash_position;
        }
        else{
            Cell* ptr_cell_now = &(i_cell_now.second);
            overlap_ship.cells_now_new.push_back(ptr_cell_now);
            multi_overlap_cell_glb[0]=(nullptr);
            overlap_ship.cells_now.push_back(ptr_cell_now);
            overlap_index[0] = i_cell_now.second.hash_position;
        }
        //cross overlap
        for(int iz = -1 * overlap_length; iz <= overlap_length; iz++){
            for(int iy = -1 * overlap_length; iy <= overlap_length; iy++){
                for(int ix = -1 * overlap_length; ix <= overlap_length; ix++){
                    if((ix * iy == 0) + (ix * iz == 0) + (iz * iy == 0) >=3){
                        //don't consider revisit
                        int index = 1 + (ix + iy + iz + overlap_length / (1 + (ix + iy + iz > 0))) +
                                  (iy != 0) * overlap_length * 2 +
                                  (iz != 0) * overlap_length * 4;
                        //0,x-2,x-1,x+1,x+1,y-2,y-1,y+1,y+2,z-2,z-1,z+1,z+2
                        double tmp_posi = getPosiWithTime(i_cell_now.second.region.x_min + ix * grid,
                                                          i_cell_now.second.region.y_min + iy * grid,
                                                          i_cell_now.second.region.z_min + iz * grid, grid, 0);
                        //std::cout<<std::endl<<"changed hash_position:"<<tmp_posi<<" ";
                        auto neighboor_cell_glb = map_glb.cells_glb.find(tmp_posi);
                        overlap_index[index] = tmp_posi;
                        if(neighboor_cell_glb != map_glb.cells_glb.end() && (param.num_margin_old_cell < 0 ||
                                                                             std::abs(g_data.step - neighboor_cell_glb->second.time_stamp) < param.num_margin_old_cell)){
                            //if(neighboor_cell_glb != map_glb.cells_glb.end() && (!param.margin_old_cell || g_data.step - neighboor_cell_glb->second.time_stamp < 500)){
                            Cell *ptr_cell_glb = &(neighboor_cell_glb->second);
                            multi_overlap_cell_glb[index] = (ptr_cell_glb);
                        }
                        else{
                            multi_overlap_cell_glb[index]=(nullptr);
                        }
                    }
                }
            }
        }
        overlap_ship.multi_cells_glb.push_back(multi_overlap_cell_glb);
    }
    g_data.time_find_overlap(0, g_data.step) += t_overlap_region.toc();
    return overlap_ship;
}

void  Map::findMatchPoints(OverlapCellsRelation & overlap_ship, Map & map_glb, double variance_thr,
                           bool residual_combination){
    //find Match Point for registration. 3Dir, cross cell
    ROS_DEBUG("findMatchPoints");
    bool weighted_avg = true;
    int num_test_square = param.num_test * param.num_test;
    //clear
    TicToc t_overlap_point;
    for(int i = 0; i < 3; i++){
        map_glb.ary_overlap_vertices[i].clear_quick();
        ary_overlap_vertices[i].clear_quick();//map_now
    }
    int overlap_length = (overlap_ship.multi_cells_glb[0].size() - 1) / 6; //num of outside cells for corresponding
    //collect all valid correspondings
    // can use multithread to speed up this step, find match points time may decrease from 2 ms to 0.5 ms,
    // but the odometry will have slightly unstable

    //omp_lock_t mylock;
    //omp_init_lock( & mylock);
    //#pragma omp parallel for num_threads(param.num_thread)
    for(size_t i_cell_now = 0; i_cell_now < overlap_ship.cells_now.size(); i_cell_now ++) {
        Eigen::Matrix<double, 5, Eigen::Dynamic> closest_point_in_glb = Eigen::MatrixXd::Zero(5, num_test_square);//storer

        for(int dir = 0; dir < 3; dir ++){
            closest_point_in_glb = Eigen::MatrixXd::Zero(5, num_test_square);
            closest_point_in_glb.row(4).fill(-1);
            PointMatrix & points_now = (*overlap_ship.cells_now[i_cell_now]).ary_cell_vertices[dir];
            if(points_now.num_point == 0){
                continue;
            }
            //new: stop when find valid point, no dist calculate, faild
            //old calculate dist and choose the minimal point
            for(int i = -1 * overlap_length; i <= overlap_length; i++) {
                int index;
                if (i == 0) {
                    index = 0;
                } else if (i < 0) {
                    index = 1 + 2 * overlap_length * dir + i + overlap_length;
                } else {
                    index = 1 + 2 * overlap_length * dir + i + overlap_length / 2;
                }
                //go through follow: x-2,x-1,0,x+1,x+2 y-2, ...
                if (overlap_ship.multi_cells_glb[i_cell_now][index] == nullptr) {
                    continue;
                }
                PointMatrix & points_glb = (*overlap_ship.multi_cells_glb[i_cell_now][index]).ary_cell_vertices[dir];
                if (points_glb.num_point == 0 || points_now.num_point == 0) {
                    continue;
                }
                for (int i_point = 0; i_point < points_now.num_point; i_point++) {
                    if ((points_glb.variance(0, i_point) < variance_thr) &&
                        (points_now.variance(0, i_point) < variance_thr)) {
                        double distance_sqrt = (points_glb.point(dir, i_point) - points_now.point(dir, i_point)) *
                                               (points_glb.point(dir, i_point) - points_now.point(dir, i_point));
                        if (closest_point_in_glb(4, i_point) < 0) {
                            closest_point_in_glb.col(i_point).topRows(3) = points_glb.point.col(i_point);
                            closest_point_in_glb.col(i_point).row(3) = points_glb.variance.col(i_point);
                            closest_point_in_glb(4, i_point) = distance_sqrt;
                        } else if (distance_sqrt < closest_point_in_glb(4, i_point)) {
                            closest_point_in_glb.col(i_point).topRows(3) = points_glb.point.col(i_point);
                            closest_point_in_glb.col(i_point).row(3) = points_glb.variance.col(i_point);
                            closest_point_in_glb(4, i_point) = distance_sqrt;
                        }
                    }
                }
            }
            if(residual_combination){
                //combine residuals in each cell, speed up the solver time, less accuracy
                //TicToc t_avg_point;
                Eigen::Matrix<double, 4, 1> avged_point_glb, avged_point_now;
                avged_point_glb.fill(0);
                avged_point_now.fill(0);
                int num_valid_points = 0;
                double sum_variance_glb(0), sum_variance_now(0);
                for(int i_result=0; i_result < closest_point_in_glb.cols(); i_result++){
                    if(closest_point_in_glb(4, i_result) > 0) {
                        num_valid_points++;
                        if(weighted_avg){
                            avged_point_glb += closest_point_in_glb.col(i_result).topRows(4) * 1.0;
                            sum_variance_glb += 1.0;
                            avged_point_now += points_now.getPointWithVariance(i_result) * 1.0;//points_now.variance(i_result);
                            sum_variance_now += 1.0;
                            //give weight accoring to variance
                            /*avged_point_glb += closest_point_in_glb.col(i_result).topRows(4)*closest_point_in_glb(3,i_result);//closest_point_in_glb(3,i_result); 1.0
                            sum_variance_glb += closest_point_in_glb(3,i_result);
                            avged_point_now += points_now.getPointWithVariance(i_result)*points_now.variance(i_result);
                            sum_variance_now += points_now.variance(i_result);*/
                            /*avged_point_glb += closest_point_in_glb.col(i_result).topRows(4)*1.0/closest_point_in_glb(3,i_result);
                            sum_variance_glb += 1.0/closest_point_in_glb(3,i_result);
                            avged_point_now += points_now.getPointWithVariance(i_result)*1.0/points_now.variance(i_result);
                            sum_variance_now += 1.0/points_now.variance(i_result);*/
                        }
                        else{
                            avged_point_glb += closest_point_in_glb.col(i_result).topRows(4);
                            avged_point_now += points_now.getPointWithVariance(i_result);
                        }
                    }
                }
                if(num_valid_points > 0){
                    if(weighted_avg){
                        for(int i = 0; i < 4; i++){
                            avged_point_glb(i,0) /= sum_variance_glb;
                            avged_point_now(i,0) /= sum_variance_now;
                        }
                    }
                    else{
                        for(int i = 0; i < 4; i++){
                            avged_point_glb(i,0) /= num_valid_points;
                            avged_point_now(i,0) /= num_valid_points;
                        }
                    }
                    //std::cout<<"avged_point \n"<<avged_point_glb<<std::endl;
                    //time_avg_point += t_avg_point.toc();
                    //TicToc t_add_point;
                    //omp_set_lock( & mylock);
                    map_glb.ary_overlap_vertices[dir].addPointWithVariance(avged_point_glb);
                    ary_overlap_vertices[dir].addPointWithVariance(avged_point_now);
                    //omp_unset_lock( & mylock);
                    //time_add_point += t_add_point.toc();
                }
            }
            else{
                //ori
                for(int i_result = 0; i_result < closest_point_in_glb.cols(); i_result++){
                    if(closest_point_in_glb(4, i_result) > 0){
                        //omp_set_lock( & mylock);
                        map_glb.ary_overlap_vertices[dir].addPointWithVariance(
                                closest_point_in_glb.col(i_result).topRows(4));
                        ary_overlap_vertices[dir].addPointWithVariance(points_now.getPointWithVariance(i_result));
                        //omp_unset_lock( & mylock);
                    }
                }
            }
        }
    }
    //omp_destroy_lock(&mylock);
    g_data.time_find_overlap_points(0, g_data.step) += t_overlap_point.toc();
}
void  Map::findMatchPointToMesh(OverlapCellsRelation & overlap_ship, Map & map_glb, double variance_thr,
                                bool residual_combination){
    //find Match Point for registration. 3Dir, cross cell, point to mesh
    ROS_DEBUG("findMatchPointToMesh");
    int num_test_square = param.num_test * param.num_test;
    int n_row =  param.num_test;
    //clear
    TicToc t_overlap_point;
    for(int i = 0; i < 3; i ++){
        map_glb.ary_overlap_vertices[i].clear_quick();
        map_glb.ary_normal[i].clear_quick();
        ary_overlap_vertices[i].clear_quick();
    }
    //int cross_cell_overlap_length = param.cross_cell_overlap_length; //num of outside cells for corresponding
    int overlap_lace = (overlap_ship.multi_cells_glb[0].size() - 1) / 6; //num of outside cells for corresponding
    double time_sort_point_0(0), time_sort_point_1(0), time_sort_point_2(0), time_avg_point(0), time_add_point(0);
    //omp_lock_t mylock;
    //omp_init_lock(&mylock);
    //#pragma omp parallel for num_threads(param.num_thread)
    for(size_t i_cell_now = 0; i_cell_now < overlap_ship.cells_now.size(); i_cell_now++) {
        //for each cell in map_now overlaped
        for(int dir = 0; dir < 3; dir++){
            Eigen::Matrix<double, 5, Eigen::Dynamic> closest_point_glb = Eigen::MatrixXd::Zero(5, num_test_square);//storer
            closest_point_glb = Eigen::MatrixXd::Zero(5, num_test_square);
            closest_point_glb.row(4).fill(-1);//distance, -1: not found
            PointMatrix & point_now = (*overlap_ship.cells_now[i_cell_now]).ary_cell_vertices[dir];
            if(point_now.num_point == 0){
                continue;
            }
            //find closest
            //old
            for(int i = -1 * overlap_lace; i <= overlap_lace; i++) {
                int index;
                if (i == 0) {
                    index = 0;
                } else if (i < 0) {
                    index = 1 + 2 * overlap_lace * dir + i + overlap_lace;
                } else {
                    index = 1 + 2 * overlap_lace * dir + i + overlap_lace / 2;
                }
                if (overlap_ship.multi_cells_glb[i_cell_now][index] == nullptr) {
                    continue;
                }
                PointMatrix & points_glb = (*overlap_ship.multi_cells_glb[i_cell_now][index]).ary_cell_vertices[dir];
                if (points_glb.num_point == 0) {
                    continue;
                }
                if(index == 0){
                    for (int i_point = 0; i_point < point_now.num_point; i_point++) {
                        double distance_sqrt = (points_glb.point(dir, i_point) - point_now.point(dir, i_point)) *
                                               (points_glb.point(dir, i_point) - point_now.point(dir, i_point));
                        if (closest_point_glb(4, i_point) < 0) {
                            closest_point_glb.col(i_point).topRows(3) = points_glb.point.col(i_point);
                            closest_point_glb.col(i_point).row(3) = points_glb.variance.col(i_point);
                            closest_point_glb(4, i_point) = distance_sqrt;
                        } else if (distance_sqrt < closest_point_glb(4, i_point)) {
                            closest_point_glb.col(i_point).topRows(3) = points_glb.point.col(i_point);
                            closest_point_glb.col(i_point).row(3) = points_glb.variance.col(i_point);
                            closest_point_glb(4, i_point) = distance_sqrt;
                        }
                    }
                }

                for (int i_point = 0; i_point < point_now.num_point; i_point++) {
                    if ((points_glb.variance(0, i_point) < variance_thr)) {
                        double dis_square = (points_glb.point(dir, i_point) - point_now.point(dir, i_point)) *
                                            (points_glb.point(dir, i_point) - point_now.point(dir, i_point));
                        if (closest_point_glb(4, i_point) < 0) {
                            closest_point_glb.col(i_point).topRows(3) = points_glb.point.col(i_point);
                            closest_point_glb.col(i_point).row(3) = points_glb.variance.col(i_point);
                            closest_point_glb(4, i_point) = dis_square;
                        } else if (dis_square < closest_point_glb(4, i_point) || closest_point_glb(3, i_point) > variance_thr) {
                            closest_point_glb.col(i_point).topRows(3) = points_glb.point.col(i_point);
                            closest_point_glb.col(i_point).row(3) = points_glb.variance.col(i_point);
                            closest_point_glb(4, i_point) = dis_square;
                        }
                    }
                }
            }

            //compute normal
            PointMatrix normal_cell(num_test_square);
            Eigen::Matrix<int, 2, 4> offset;
            offset << 1, 0,-1, 0,
                      0,-1, 0, 1;
            for(int i_point = 0; i_point < num_test_square; i_point ++) {
                //Point curr_point = point_now.point.col(i_point);
                Eigen::Vector3d normal_at_i;
                normal_at_i.setZero();
                if(closest_point_glb(4, i_point) < 0) {
                    continue;
                }
                int num_valid_neighboor = 0;
                int ix_cell = i_point % (n_row);
                int iy_cell = i_point / (n_row);
                for(int i_offset=0; i_offset<4; i_offset++) {
                    if ((ix_cell + offset(0, i_offset) < n_row &&
                         ix_cell + offset(0, i_offset) >= 0 &&
                         iy_cell + offset(1, i_offset) < n_row &&
                         iy_cell + offset(1, i_offset) >= 0)
                        &&
                        (ix_cell + offset(0, (i_offset + 1) % 4) < n_row &&
                         ix_cell + offset(0, (i_offset + 1) % 4) >= 0 &&
                         iy_cell + offset(1, (i_offset + 1) % 4) < n_row &&
                         iy_cell + offset(1, (i_offset + 1) % 4) >= 0)){

                        int idx_1 = i_point + offset(0, i_offset) +           n_row * offset(1, i_offset);
                        int idx_2 = i_point + offset(0, (i_offset + 1) % 4) + n_row * offset(1, (i_offset + 1) % 4);
                        if(closest_point_glb(4, idx_1) > 0 && closest_point_glb(4, idx_2) > 0){
                            //delta_face_i += closest_point_glb(3, idx_1);
                            //delta_face_i += closest_point_glb(3, idx_2);
                            Point curr_index_1 = closest_point_glb.col(i_point).topRows(3) - closest_point_glb.col(idx_1).topRows(3);
                            Point curr_index_2 = closest_point_glb.col(i_point).topRows(3) - closest_point_glb.col(idx_2).topRows(3);
                            if(curr_index_1.norm() > 1.0 * param.grid || curr_index_2.norm() > 1.0 * param.grid){
                                continue;
                            }
                            Eigen::Vector3d normal_i_cross = curr_index_1.cross(curr_index_2);
                            normal_i_cross.normalize();
                            normal_at_i += normal_i_cross;
                            num_valid_neighboor++;
                        }
                    }
                }
                if(num_valid_neighboor == 0){
                    normal_at_i = point_now.point.col(i_point) - closest_point_glb.col(i_point).topRows(3);
                    normal_at_i.normalize();
                }
                else {
                    //std::cout<<"num_valid_neighboor "<<num_valid_neighboor<<std::endl;
                    //delta_face_i = delta_face_i/num_valid_neighboor/2.0;
                    //closest_point_glb(3, i_point) = delta_face_i;
                    normal_at_i.normalize();
                }
                normal_cell.point.col(i_point) = normal_at_i;//zero : no normal
            }
            //push
            bool weighted_avg = false;
            if(residual_combination){
                PointWithVariance avged_point_glb, avged_point_now;
                Point normal_avg;
                avged_point_glb.fill(0);
                avged_point_now.fill(0);
                double weight_sum = 0;
                normal_avg.setZero();
                int num_valid_points = 0;
                double all_variance_glb(0), all_variance_now(0);
                for(int i_result=0; i_result < closest_point_glb.cols(); i_result++){
                    if(closest_point_glb(4, i_result) > 0 && point_now.variance(i_result) < variance_thr
                       && closest_point_glb(3, i_result) < variance_thr && normal_cell.point.col(i_result).norm() > 0.1) {
                        num_valid_points++;
                        if(weighted_avg){
                            //double weight = 1;
                            //Eigen::Vector3d glb_currect_normalized = closest_point_glb.col(i_result).topRows(3) - point_now.point.col(i_result);
                            //glb_currect_normalized.normalize();
                            //double weight = glb_currect_normalized.dot(normal_cell.point.col(i_result));
                            double weight = 1/(closest_point_glb(3, i_result) + point_now.variance(i_result));
                            avged_point_glb += closest_point_glb.col(i_result).topRows(4)*weight;//closest_point_glb(3,i_result); 1.0
                            avged_point_now += point_now.getPointWithVariance(i_result) * weight;
                            normal_avg += normal_cell.point.col(i_result)*weight;
                            weight_sum += weight;
                        }
                        else{
                            avged_point_glb += closest_point_glb.col(i_result).topRows(4);
                            avged_point_now += point_now.getPointWithVariance(i_result);
                            normal_avg += normal_cell.point.col(i_result);
                        }
                    }
                }
                if(num_valid_points>0){
                    if(weighted_avg){
                        avged_point_glb /= weight_sum;
                        avged_point_now /= weight_sum;
                    }
                    else{
                        avged_point_glb /= num_valid_points;
                        avged_point_now /= num_valid_points;
                    }
                    normal_avg.normalize();
                    //omp_set_lock(&mylock);
                    map_glb.ary_overlap_vertices[dir].addPointWithVariance(avged_point_glb);
                    ary_overlap_vertices[dir].addPointWithVariance(avged_point_now);
                    map_glb.ary_normal[dir] .addPoint(normal_avg);
                    //omp_unset_lock(&mylock);
                }
            }
            else{
                for(int i_result = 0; i_result < closest_point_glb.cols(); i_result++){
                    if(closest_point_glb(4, i_result) > 0 && point_now.variance(i_result) < variance_thr
                       && closest_point_glb(3, i_result) < variance_thr && normal_cell.point.col(i_result).norm() > 0.1) {
                        //omp_set_lock(&mylock);
                        map_glb.ary_overlap_vertices[dir].addPointWithVariance(
                                closest_point_glb.col(i_result).topRows(4));
                        ary_overlap_vertices[dir].addPointWithVariance(point_now.getPointWithVariance(i_result));
                        map_glb.ary_normal[dir].addPoint(normal_cell.point.col(i_result));
                        //omp_unset_lock(&mylock);
                    }
                }
            }
            //compute norm what different?
            /* PointMatrix normal_cell;
             Eigen::Matrix<int, 2, 4> offset;
             offset << 1, 0,-1, 0,
                       0,-1, 0, 1;
             for(int i_point=0; i_point < closest_point_glb.cols(); i_point++) {
                 Point curr_point = point_now.point.col(i_point);
                 if(closest_point_glb(4, i_point) < 0) {
                     continue;
                 }
                 int ix_cell = i_point % (n_row);
                 int iy_cell = i_point / (n_row);
                 int num_valid_neighboor = 0;
                 double delta_face_i = 0;
                 Eigen::Vector3d normal_i;
                 normal_i.setZero();
                 for(int i_offset=0; i_offset<4; i_offset++) {
                     if ((ix_cell + offset(0, i_offset) < n_row &&
                          ix_cell + offset(0, i_offset) >= 0 &&
                          iy_cell + offset(1, i_offset) < n_row &&
                          iy_cell + offset(1, i_offset) >= 0)
                          &&
                         (ix_cell + offset(0, (i_offset + 1) % 4) < n_row &&
                          ix_cell + offset(0, (i_offset + 1) % 4) >= 0 &&
                          iy_cell + offset(1, (i_offset + 1) % 4) < n_row &&
                          iy_cell + offset(1, (i_offset + 1) % 4) >= 0)){

                         int idx_1 = i_point + offset(0, i_offset) +           n_row * offset(1, i_offset);
                         int idx_2 = i_point + offset(0, (i_offset + 1) % 4) + n_row * offset(1, (i_offset + 1) % 4);
                         if(closest_point_glb(4, idx_1) > 0 && closest_point_glb(4, idx_2) > 0){
                             delta_face_i += closest_point_glb(3, idx_1);
                             delta_face_i += closest_point_glb(3, idx_2);
                             Eigen::Vector3d normal_i_cross = (curr_point - closest_point_glb.col(idx_1).topRows(3)).cross(
                                                               curr_point - closest_point_glb.col(idx_2).topRows(3));
                             normal_i_cross.normalize();
                             normal_i += normal_i_cross;
                             num_valid_neighboor++;
                         }
                     }
                 }
                 if(num_valid_neighboor == 0){
                     normal_i = point_now.point.col(i_point) - closest_point_glb.col(i_point).topRows(3);
                     normal_i.normalize();
                 }
                 else {
                     //std::cout<<"num_valid_neighboor "<<num_valid_neighboor<<std::endl;
                     delta_face_i = delta_face_i/num_valid_neighboor/2.0;
                     //closest_point_glb(3, i_point) = delta_face_i;
                     normal_i.normalize();
                 }
                 normal_cell.addPoint(normal_i);
                 if(!residual_combination){
                     map_glb.ary_overlap_vertices[dir] .addPointWithDelta(closest_point_glb.col(i_point).topRows(4));
                     ary_overlap_vertices[dir].addPointWithDelta(point_now.getPointWithDelta(i_point));
                     map_glb.ary_normal[dir].addPoint(normal_i);
                 }
             }
             if(residual_combination){
                 Eigen::Matrix<double, 4, 1> avged_point_glb, avged_point_now;
                 Point normal_avg;
                 avged_point_glb.fill(0);
                 avged_point_now.fill(0);
                 normal_avg.setZero();
                 int num_valid_points = 0;
                 double all_delta_glb(0), all_delta_now(0);
                 for(int i_result=0; i_result < closest_point_glb.cols(); i_result++){
                     if(closest_point_glb(4, i_result) > 0) {
                         num_valid_points++;
                         avged_point_glb += closest_point_glb.col(i_result).topRows(4);
                         avged_point_now += point_now.getPointWithVariance(i_result);
                         normal_avg += normal_cell.point.col(i_result);
                     }
                 }
                 if(num_valid_points>0){
                     avged_point_glb /= num_valid_points;
                     avged_point_now /= num_valid_points;
                     map_glb.ary_overlap_vertices[dir] .addPointWithDelta(avged_point_glb);
                     ary_overlap_vertices[dir].addPointWithVariance(avged_point_now);
                     normal_avg.normalize();
                     map_glb.ary_normal[dir] .addPoint(normal_avg);
                 }
             }*/
        }
    }
    //omp_destroy_lock(&mylock);
    g_data.time_find_overlap_points(0, g_data.step) += t_overlap_point.toc();
}

void  Map::filterVerticesByVariance(double variance_thr){
    //used to draw map
    ROS_DEBUG("filterVerticesByVariance");
    //clear
    vertices_filted.clear_quick();
    for(int dir = 0; dir < 3; dir++){
        ary_vertices_all_show[dir].clear_quick();
    }
    //for(int dir = 0; dir < 3; dir++){
    //    ary_overlap_vertices[dir].clear_quick();
    //}
    bool showxyzmap = false;

    //collect point
    if(name == "CURRENT_SCAN"){
        for(int i_cell=0; i_cell < index_bucket_enough_point.size(); i_cell++){
            std::pair<double, Cell> & cell_now = cells_now[index_bucket_enough_point[i_cell]];
            for(int dir = 0; dir < 3; dir ++){
                PointMatrix & points_filt = cell_now.second.ary_cell_vertices[dir];
                for(int i_point=0; i_point < points_filt.num_point; i_point++){
                    if(((points_filt.variance(0, i_point)) < variance_thr)){
                        vertices_filted.addPointWithVariance(points_filt.getPointWithVariance(i_point));
                    }
                }
            }
        }
    }
    else{//map glb
        for(auto& cell_glb : cells_glb){
            for(int dir = 0; dir < 3; dir++){
                PointMatrix & point_filt = cell_glb.second.ary_cell_vertices[dir];
                for(int i = 0; i < point_filt.num_point; i++){
                    if(((point_filt.variance(0,i)) < variance_thr)){
                        vertices_filted.addPointWithVariance(point_filt.getPointWithVariance(i));
                        if(showxyzmap && name == "MAP_GLB"){
                            ary_vertices_all_show[dir].addPointWithVariance(point_filt.getPointWithVariance(i));
                        }
                    }
                }
            }
        }
    }
}
void  Map::filterVerticesByVarianceOverlap(double variance_thr, Map & map_now){
    //used to draw map only overlap region, in order to save time
    //filterVerticesByVariance3DirOverlap
    ROS_DEBUG("filterVerticesByVarianceOverlap");
    //clear
    vertices_filted.clear();
    for(int dir = 0; dir < 3; dir++){
        ary_vertices_all_show[dir].clear();
    }
    bool only_pub_overlap_region = true;
    bool showxyzmap = true;

    OverlapCellsRelation overlap_ship = map_now.overlapCells(*this);
    for(auto& i_cell_glb : overlap_ship.cells_glb){
        for(int dir = 0; dir < 3; dir++){
            PointMatrix & points_filt = (*i_cell_glb).ary_cell_vertices[dir];
            for(int i = 0; i < points_filt.num_point; i++){
                if(((points_filt.variance(0, i)) < variance_thr)){
                    vertices_filted.addPointWithVariance(points_filt.getPointWithVariance(i));
                    if(showxyzmap && name == "MAP_GLB"){
                        ary_vertices_all_show[dir].addPointWithVariance(points_filt.getPointWithVariance(i));
                    }
                }
            }
        }
    }
}

Transf Map::computeT(Map & map_glb, Map & map_now){
    // compute the transformation (T) between two scans, given residuals, use ceres instead of svd, consider 3dir
    //name computeT
    ROS_DEBUG("computeT");
    //TicToc t_rt_pre;
    //parm to optimize
    Transf transf_last_curr = Eigen::MatrixXd::Identity(4, 4);

    //build problem
    //given J
    double parameters[7] = {0, 0, 0, 1, 0, 0, 0};
    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
    ceres::Problem::Options problem_options;
    ceres::Problem problem(problem_options);
    problem.AddParameterBlock(parameters,7,new PoseSE3Parameterization());

    for(int dir = 0; dir < 3; dir ++){
        PointMatrix & scan_glb = map_glb.ary_overlap_vertices[dir];
        PointMatrix & scan_now = map_now.ary_overlap_vertices[dir];
        //Eigen::MatrixXd delta_glb, delta_now, weight;
        //delta_glb = scan_glb.variance;
        //delta_now = scan_now.variance;
        //weight = Eigen::MatrixXd::Zero(1, scan_glb.num_point);
        //valid check
        if((scan_glb.num_point != scan_now.num_point)){
            ROS_ERROR("Different num of points in computeT");
        }

        //add laser gp error
        for(int i = 0; i < scan_glb.num_point; i++){
            Eigen::Matrix<double, 3, 1> curr_point = scan_now.point.col(i).topRows(3),
                                        last_point = scan_glb.point.col(i).topRows(3);
            double tmp_min_variance = param.variance_min;
            double s = 1 / (scan_now.variance(0, i) + scan_glb.variance(0, i) + tmp_min_variance);//0.01 in order to not inf
            //double s = (scan_now.variance(0,i) + scan_glb.variance(0,i))/(scan_now.variance(0,i)*scan_glb.variance(0,i));//0.01 in order to not inf
            //weight(0, i) = s;
            //double s = 1.0;
            if(s > 0){
                if(dir == X){
                    ceres::CostFunction *cost_function = new LidarPointFactorDirX(curr_point, last_point, s);
                    problem.AddResidualBlock(cost_function, loss_function, parameters);
                } else if (dir == Y){
                    ceres::CostFunction *cost_function = new LidarPointFactorDirY(curr_point, last_point, s);
                    problem.AddResidualBlock(cost_function, loss_function, parameters);
                } else{
                    ceres::CostFunction *cost_function = new LidarPointFactorDirZ(curr_point, last_point, s);
                    problem.AddResidualBlock(cost_function, loss_function, parameters);
                }
            }
        }
    }

    //solve problem
    TicToc t_rt_solve;
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.dense_linear_algebra_library_type = ceres::LAPACK;
    options.max_num_iterations = 10;
    options.minimizer_progress_to_stdout = false;
    options.num_threads = 1;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    //std::cout << summary.BriefReport() << "\n";
    //std::cout << summary.FullReport() << "\n";
    //std::cout<<"t_rt_solve:"<<t_rt_solve.toc() << "ms" << std::endl;

    //output result
    Eigen::Map< Eigen::Quaterniond> q_last_curr(parameters);
    Eigen::Map< Eigen::Vector3d> t_last_curr(parameters+4);
    Eigen::Matrix3d rot = q_last_curr.toRotationMatrix();
    transf_last_curr << rot, t_last_curr, 0, 0, 0, 1;

    return transf_last_curr;
}
Transf Map::computeTPointToMesh(Map & map_glb, Map & map_now){
    // compute the transformation (T) between two scans, given residuals, use ceres, consider 3dir, use point to mesh
    // computeTPointToMesh
    ROS_DEBUG("computeTPointToMesh");
    double tmp_min_variance = param.variance_min;
    //TicToc t_rt_pre;
    //parm to optimize
    Transf transf_last_curr = Eigen::MatrixXd::Identity(4, 4);
    int n_row = param.num_test;
    double variance_face = param.variance_register;
    //build problem
    //given J
    double parameters[7] = {0, 0, 0, 1, 0, 0, 0};
    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
    ceres::Problem::Options problem_options;
    ceres::Problem problem(problem_options);
    problem.AddParameterBlock(parameters,7,new PoseSE3Parameterization());

    for(int dir = 0; dir < 3; dir ++){
        PointMatrix & scan_glb = map_glb.ary_overlap_vertices[dir];
        PointMatrix & scan_now = map_now.ary_overlap_vertices[dir];
        PointMatrix & normals   = map_glb.ary_normal[dir];
        //valid check
        if((scan_glb.num_point != scan_now.num_point)){
            ROS_ERROR("Different num of points in computeT");
        }
        //point 2 plane simple
        for(int i = 0; i < scan_glb.num_point; i ++){
            Eigen::Matrix<double, 3, 1> curr_point = scan_now.point.col(i).topRows(3);
            Eigen::Matrix<double, 3, 1> last_point = scan_glb.point.col(i).topRows(3);
            Eigen::Matrix<double, 3, 1> norm = normals.point.col(i).topRows(3);
            if(norm.isZero()){
                //it is possible that we can not obtain valid normal, in case the problem degenerate, here add point to point residuals
/*                if(dir==X){
                    ceres::CostFunction *cost_function = new LidarPointFactorDirX(curr_point, last_point, 1);
                    problem.AddResidualBlock(cost_function, loss_function, parameters);
                }
                else if (dir==Y){
                    ceres::CostFunction *cost_function = new LidarPointFactorDirY(curr_point, last_point, 1);
                    problem.AddResidualBlock(cost_function, loss_function, parameters);
                }
                else{
                    ceres::CostFunction *cost_function = new LidarPointFactorDirZ(curr_point, last_point, 1);
                    problem.AddResidualBlock(cost_function, loss_function, parameters);
                }*/
            }
            else{
                double residual_weight = 1;
                ceres::CostFunction *cost_function =
                        new SurfNormAnalyticCostFunction(curr_point, last_point, norm, residual_weight);
                problem.AddResidualBlock(cost_function, loss_function, parameters);
            }
        }
    }
    //solve problem
    TicToc t_rt_solve;
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.dense_linear_algebra_library_type = ceres::LAPACK;
    options.max_num_iterations = 10;
    options.minimizer_progress_to_stdout = false;
    options.num_threads = 1;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    //std::cout << summary.BriefReport() << "\n";
    //std::cout << summary.FullReport() << "\n";
    //std::cout<<"t_rt_solve:"<<t_rt_solve.toc() << "ms" << std::endl;

    //output result
    Eigen::Map< Eigen::Quaterniond> q_last_curr(parameters);
    Eigen::Map< Eigen::Vector3d> t_last_curr(parameters+4);
    Eigen::Matrix3d rot = q_last_curr.toRotationMatrix();
    transf_last_curr << rot, t_last_curr, 0, 0, 0, 1;

    return transf_last_curr;
}

void  Map::registerToMap(Map & map_glb, Transf & Tguess, double max_time){
    ROS_DEBUG("registerToMap");
    //scan to map registration, iterativel conduct overlapCellsCrossCell, findMatchPointToMesh, computeTPointToMesh and dividePointsIntoCell,
    // 3Dir cross cell
    TicToc t_rg;//rg: registration
    int rg_max_times = param.register_times, num_test = param.num_test;
    double variance_regist = param.variance_register, converge_thr = param.converge_thr, gird = param.grid;
    int min_step = 10;
    Transf transf_delta = Eigen::MatrixXd::Identity(4, 4), transf_total = Tguess, transf_this_step = Eigen::MatrixXd::Identity(4, 4);
    //some report
    std::vector<int> ary_num_overlap_point;
    std::vector<double> ary_delta_transf_scale;
    bool precise = false;
    int i_rg;
    double delta_transf_scale = 100;

    for(i_rg = 0; i_rg < rg_max_times && !precise; i_rg++){
        //TicToc t_rg_overlap_region;
        if(i_rg < rg_max_times - 1){
            OverlapCellsRelation overlap_ship = overlapCellsCrossCell(map_glb, param.cross_cell_overlap_length);
            if(param.point2mesh){
                findMatchPointToMesh(overlap_ship, map_glb, variance_regist, param.residual_combination);
            }
            else{
                findMatchPoints(overlap_ship, map_glb, variance_regist, param.residual_combination);
            }
        }
        else {
            //in the final step, do not need to search too far
            OverlapCellsRelation overlap_ship = overlapCellsCrossCell(map_glb, 0);
            findMatchPointToMesh(overlap_ship, map_glb, variance_regist, param.residual_combination);
        }
        TicToc t_rg_computert;
        if(param.point2mesh){
            transf_delta = computeTPointToMesh(map_glb, *this);
        }
        else{
            transf_delta = computeT(map_glb, *this);
        }
        g_data.time_compute_rt(0,g_data.step) += t_rg_computert.toc();
        transf_total = transf_delta * transf_total;
        transf_this_step = transf_delta * transf_this_step;
        points_turned.transformPoints(transf_delta);

        ary_num_overlap_point.push_back(ary_overlap_vertices[0].num_point +
                                        ary_overlap_vertices[1].num_point +
                                        ary_overlap_vertices[2].num_point);
        delta_transf_scale = 5 * (transf_delta.block(0, 0, 3, 3) - Eigen::MatrixXd::Identity(3, 3)).norm() +
                transf_delta.block(0, 3, 3, 1).norm();
        ary_delta_transf_scale.push_back(delta_transf_scale);
        if((delta_transf_scale < converge_thr && i_rg > 0) || i_rg == rg_max_times - 1 || t_rg.toc() > 0.9 * max_time){
            precise = true;
        }
        dividePointsIntoCell(points_turned, map_glb, true);//reconstruct the point again before next iteration
    }
    g_data.updatePose(transf_total);
    g_data.overlap_point_num(0, g_data.step) = ary_num_overlap_point.back();
    g_data.not_a_surface_cell_num(0, g_data.step) /= i_rg;
    g_data.rg_times(0, g_data.step) = i_rg;
    if(param.save_raw_point_clouds){
        g_data.accumulateRawPoint(pcl_raw, transf_total);
    }
    //print report
    bool print_rg_report = false;
    if(print_rg_report){
        std::cout << "register_times: " << i_rg << "\n";
        //g_data.time_find_overlap(0, g_data.step) = t_overlap_region_sum;
        std::cout << "overlap_point: ";
        for(auto &it : ary_num_overlap_point) std::cout << it << " ";
        std::cout << "\n";
        std::cout << "delta_transf_scale: ";
        for(auto & it : ary_delta_transf_scale){
            std::cout << it << " ";
        }
        std::cout << "\n";
        std::cout << "Pose SLAM: " << "x: " << transf_total(0, 3) << "  y: " << transf_total(1, 3) << "  z " << transf_total(2, 3) << "\n";// std::endl;
    }
}

void  Map::updateMap(Map & map_now){
    //update map_glb with map_now, update all 3 directions
    ROS_DEBUG("updateMap");

    //find overlapped regions
    OverlapCellsRelation overlap_ship = map_now.overlapCells(*this);
    newly_updated_cells.clear();
    //update overlapped region
    //#pragma omp parallel for num_threads(param.num_thread)
    for(int i = 0; i < overlap_ship.cells_now.size(); i++){
        if((overlap_ship.cells_glb[i] != nullptr) &&
           (overlap_ship.cells_now[i] != nullptr)){
            Cell & cell_glb = *overlap_ship.cells_glb[i];
            Cell & cell_now = *overlap_ship.cells_now[i];
            if(cell_glb.not_surface && !cell_now.not_surface){//&& (overlap_ship.cells_now[i])->not_surface
                cell_glb.cell_raw_points += cell_now.cell_raw_points;
                cell_glb.reconstructSurfaces(true);
            }
            else{
                for(int i_dir = 0; i_dir < 3; i_dir ++){
                    if(cell_glb.ary_cell_vertices[i_dir].num_point != 0 &&
                       cell_now.ary_cell_vertices[i_dir].num_point != 0 ) {
                        cell_glb.updateVertices(cell_now, Direction(i_dir));
                    }
                    else if(cell_glb.ary_cell_vertices[i_dir].num_point == 0 &&
                            cell_now.ary_cell_vertices[i_dir].num_point != 0){
                        //cells_glb.ary_cell_vertices[i_dir] = cells_now.ary_cell_vertices[i_dir];
                    }
                }
            }
            cell_glb.time_stamp = g_data.step;
            cell_glb.updateViewedLocation(g_data.T_seq[g_data.step]);

            newly_updated_cells.push_back(& cell_glb);
        }
    }

    //add new cell
    //std::cout << "size of cells_now_new: " << overlap_ship.cells_now_new.size() << std::endl;
    if( !overlap_ship.cells_now_new.empty()){
        for(auto & cell_now_new : overlap_ship.cells_now_new){
            Point current_pose, current_viewed_dir;
            current_pose = trans3Dpoint(0, 0, 0, g_data.T_seq[g_data.step]);
            current_viewed_dir = (current_pose - cell_now_new->center) / (current_pose - cell_now_new->center).norm();
            double tmp_posi = getPosiWithTime(cell_now_new->region.x_min,
                                              cell_now_new->region.y_min,
                                              cell_now_new->region.z_min, param.grid, cell_now_new->revisited);

            //cells_glb.emplace((*cells_now_new).hash_position, Cell((*cells_now_new).cell_raw_points,
            //        (*cells_now_new).ary_cell_vertices, tmp_posi, (*cells_now_new).direction, (*cells_now_new).region, g_data.step));
            //cells_glb.emplace(tmp_posi, Cell((*cells_now_new).cell_raw_points, (*cells_now_new).ary_cell_vertices, tmp_posi,
            //                           (*cells_now_new).direction, (*cells_now_new).region, g_data.step, current_viewed_dir)); //TO DO: test correct margin old
            //cells_now_new->time_stamp = g_data.step;
            //cells_glb.emplace(tmp_posi, *cells_now_new);
            Cell tmp_cell ((*cell_now_new).cell_raw_points, (*cell_now_new).ary_cell_vertices, g_data.step,
                           tmp_posi, (*cell_now_new).region);
            tmp_cell.updateViewedLocation(g_data.T_seq[g_data.step]);

            //cells_glb.emplace(tmp_posi, tmp_cell);
            pair<std::unordered_map<double, Cell>::iterator, bool> inserted_cell = cells_glb.emplace(tmp_posi, tmp_cell);
            newly_updated_cells.push_back(& (inserted_cell.first->second));
        }
    }
    std::cout << " num_cells: now:" << map_now.cells_glb.size();
    std::cout << " new:" << overlap_ship.cells_now_new.size();
    std::cout << " glb:" << cells_glb.size() << "\n";

    g_data.num_cells_glb(0, g_data.step) = cells_glb.size();
    g_data.num_cells_now(0, g_data.step) = map_now.cells_now.size();
    g_data.num_cells_new(0, g_data.step) = overlap_ship.cells_now_new.size();
}

void  Map::filterMeshLocal(){
    //mesh_msg current scan by direct connectin, visualized by mesh_tool
    ROS_DEBUG("filterMeshLocal");
    double variance_point = -1;
    double variance_face = param.variance_map_show;
    int n_row(param.num_test), n_col(param.num_test);
    double grid = param.grid;
    //init msg
    mesh_msgs::MeshGeometryStamped tmp_mesh;
    tmp_mesh.header.frame_id = "map";
    tmp_mesh.header.stamp = ros::Time::now();

    for(int i = 0; i < index_bucket_enough_point.size(); i++){
        std::pair<double, Cell> & i_cell = cells_now[index_bucket_enough_point[i]];

        // in each cell, dir, filter mesh_msg under vatiance threshold
        for(int dir = 0; dir < 3; dir ++){
            PointMatrix & vertices =  i_cell.second.ary_cell_vertices[dir];
            if(vertices.num_point != 0){
                //inside mesh_msg
                int start_vertex_i = tmp_mesh.mesh_geometry.vertices.size();
                for(int vertex_i = 0; vertex_i < vertices.num_point ; vertex_i++){
                    geometry_msgs::Point tmp_point;
                    tmp_point.x =  vertices.point(0, vertex_i);
                    tmp_point.y =  vertices.point(1, vertex_i);
                    tmp_point.z =  vertices.point(2, vertex_i);
                    tmp_mesh.mesh_geometry.vertices.push_back(tmp_point);
                }
                for(int vertex_i = 0; vertex_i < vertices.num_point; vertex_i++){
                    int ix_cell = vertex_i % (n_row);
                    int iy_cell = vertex_i / (n_row);
                    mesh_msgs::MeshTriangleIndices tmp_face;
                    if(ix_cell + 1 < n_row && iy_cell - 1 >= 0){
                        double variance_face_i = (vertices.variance(0, vertex_i) +
                                                  vertices.variance(0, vertex_i + 1) +
                                                  vertices.variance(0, vertex_i + 1 - n_row)) / 3.0;
                        if(variance_face_i < variance_face){
                            tmp_face.vertex_indices[0] = start_vertex_i + vertex_i;
                            tmp_face.vertex_indices[1] = start_vertex_i + vertex_i + 1;
                            tmp_face.vertex_indices[2] = start_vertex_i + vertex_i + 1 - n_row;
                            tmp_mesh.mesh_geometry.faces.push_back(tmp_face);
                        }
                    }
                    if(ix_cell + 1 < n_row && iy_cell + 1 < n_row ){
                        double variance_face_i = (vertices.variance(0, vertex_i) +
                                                  vertices.variance(0, vertex_i + 1) +
                                                  vertices.variance(0, vertex_i + n_row)) / 3.0;
                        if(variance_face_i < variance_face){
                            tmp_face.vertex_indices[0] = start_vertex_i + vertex_i;
                            tmp_face.vertex_indices[1] = start_vertex_i + vertex_i + 1;
                            tmp_face.vertex_indices[2] = start_vertex_i + vertex_i + n_row;
                            tmp_mesh.mesh_geometry.faces.push_back(tmp_face);
                        }
                    }
                }
            }
        }
    }
    std::cout<<"mesh_msg: point " << tmp_mesh.mesh_geometry.vertices.size() << " faces: " << tmp_mesh.mesh_geometry.faces.size() <<std::endl;
    mesh_msg = tmp_mesh;
}
void  Map::filterMeshGlb(){
    //mesh_msg global scan by direct connectin, visualized by mesh_tool, need to traverse all cell in map
    ROS_DEBUG("filterMeshGlb");
    double variance_point = -1;
    double variance_face_show = param.variance_map_show;
    int n_row(param.num_test), n_col(param.num_test);
    double grid = param.grid;
    int min_show_updated_times = 1;
    double max_show_distance = 70;
    //init msg
    mesh_msgs::MeshGeometryStamped tmp_mesh;
    tmp_mesh.header.frame_id = "map";
    tmp_mesh.header.stamp = ros::Time::now();

    TicToc t_connect_push("t_connect_push");
    for (auto & i_cell : cells_glb){
        // in each cell, dir, build mesh_msg
        if(i_cell.second.average_viewed_distance > max_show_distance ){
            continue;
        }
        for(int dir = 0; dir < 3; dir ++){
            if(i_cell.second.updated_times[dir] < min_show_updated_times){
                continue;
            }
            PointMatrix & vertices = i_cell.second.ary_cell_vertices[dir];
            if(vertices.num_point == 0){
                continue;
            }
            //inside mesh_msg
            int start_vertex_i = tmp_mesh.mesh_geometry.vertices.size();
            for(int vertex_i = 0; vertex_i < vertices.num_point ; vertex_i ++){
                geometry_msgs::Point tmp_point;
                tmp_point.x =  vertices.point(0, vertex_i);
                tmp_point.y =  vertices.point(1, vertex_i);
                tmp_point.z =  vertices.point(2, vertex_i);
                tmp_mesh.mesh_geometry.vertices.push_back(tmp_point);
            }
            for(int vertex_i = 0; vertex_i < vertices.num_point; vertex_i++){
                int ix_cell = vertex_i % (n_row);
                int iy_cell = vertex_i / (n_row);
                mesh_msgs::MeshTriangleIndices tmp_face;
                if(ix_cell + 1 < n_row && iy_cell - 1 >= 0){
                    double variance_face_i = (vertices.variance(0, vertex_i) +
                                              vertices.variance(0, vertex_i + 1) +
                                              vertices.variance(0, vertex_i + 1 - n_row)) / 3.0;
                    if(variance_face_i < variance_face_show){
                        tmp_face.vertex_indices[0] = start_vertex_i + vertex_i;
                        tmp_face.vertex_indices[1] = start_vertex_i + vertex_i + 1;
                        tmp_face.vertex_indices[2] = start_vertex_i + vertex_i + 1 - n_row;
                        tmp_mesh.mesh_geometry.faces.push_back(tmp_face);
                    }
                }
                if(ix_cell + 1 < n_row && iy_cell + 1 < n_row ){
                    double variance_face_i = (vertices.variance(0, vertex_i) +
                                              vertices.variance(0, vertex_i + 1) +
                                              vertices.variance(0, vertex_i + n_row)) / 3.0;
                    if(variance_face_i < variance_face_show){
                        tmp_face.vertex_indices[0] = start_vertex_i + vertex_i;
                        tmp_face.vertex_indices[1] = start_vertex_i + vertex_i + 1;
                        tmp_face.vertex_indices[2] = start_vertex_i + vertex_i + n_row;
                        tmp_mesh.mesh_geometry.faces.push_back(tmp_face);
                    }
                }
            }
        }
    }
    std::cout<<"mesh_msg: point " << tmp_mesh.mesh_geometry.vertices.size() << " faces: " << tmp_mesh.mesh_geometry.faces.size() <<std::endl;
    mesh_msg = tmp_mesh;
}


bool  Map::outputMeshAsPly(const std::string& filename, const mesh_msgs::MeshGeometryStamped & mesh_msg_to_save) {
    //save the mesh in mesh_tool msg
    ROS_DEBUG("outputMeshAsPly");
    //std::ofstream stream(filename.c_str());
    std::ofstream stream;
    stream.open(filename, std::ios::out);
    if (!stream) {
        ROS_WARN("Can not open outputMeshAsPly file");
        std::cout <<"wrong file location of mesh_msg_to_save to save" <<std::endl;
        return false;
    }

    size_t num_points = 3 * mesh_msg_to_save.mesh_geometry.faces.size();
    if(num_points == 0) {
        ROS_WARN("Empty mesh msg");
        return false;
    }
    stream << "ply" << std::endl;
    stream << "format ascii 1.0" << std::endl;
    stream << "element vertex " << num_points << std::endl;
    stream << "property float x" << std::endl;
    stream << "property float y" << std::endl;
    stream << "property float z" << std::endl;
/*
    if (mesh_msg_to_save.hasNormals()) {
        stream << "property float normal_x" << std::endl;
        stream << "property float normal_y" << std::endl;
        stream << "property float normal_z" << std::endl;
    }
    if (mesh_msg_to_save.hasColors()) {
        stream << "property uchar red" << std::endl;
        stream << "property uchar green" << std::endl;
        stream << "property uchar blue" << std::endl;
        stream << "property uchar alpha" << std::endl;
    }
*/

    //if (mesh_msg_to_save.hasTriangles()) {
    stream << "element face " << mesh_msg_to_save.mesh_geometry.faces.size() << std::endl;
    stream << "property list uchar int vertex_indices"
           << std::endl;  // pcl-1.7(ros::kinetic) breaks ply convention by not
    // using "vertex_index"
    //}
    stream << "end_header" << std::endl;

    size_t vertices_index = 0;
    for (const mesh_msgs::MeshTriangleIndices& vert : mesh_msg_to_save.mesh_geometry.faces) {
        stream << mesh_msg_to_save.mesh_geometry.vertices[vert.vertex_indices[0]].x << " "
               << mesh_msg_to_save.mesh_geometry.vertices[vert.vertex_indices[0]].y << " "
               << mesh_msg_to_save.mesh_geometry.vertices[vert.vertex_indices[0]].z << std::endl;
        stream << mesh_msg_to_save.mesh_geometry.vertices[vert.vertex_indices[1]].x << " "
               << mesh_msg_to_save.mesh_geometry.vertices[vert.vertex_indices[1]].y << " "
               << mesh_msg_to_save.mesh_geometry.vertices[vert.vertex_indices[1]].z << std::endl;
        stream << mesh_msg_to_save.mesh_geometry.vertices[vert.vertex_indices[2]].x << " "
               << mesh_msg_to_save.mesh_geometry.vertices[vert.vertex_indices[2]].y << " "
               << mesh_msg_to_save.mesh_geometry.vertices[vert.vertex_indices[2]].z << std::endl;

/*        if (mesh_msg_to_save.hasNormals()) {
            const Point& normal = mesh_msg_to_save.normals[vertices_index];
            stream << " " << normal.x() << " " << normal.y() << " " << normal.z();
        }
        if (mesh_msg_to_save.hasColors()) {
            const Color& color = mesh_msg_to_save.colors[vertices_index];
            int r = static_cast<int>(color.r);
            int g = static_cast<int>(color.g);
            int b = static_cast<int>(color.b);
            int a = static_cast<int>(color.a);
            // Uint8 prints as character otherwise. :(
            stream << " " << r << " " << g << " " << b << " " << a;
        }*/

        vertices_index ++;
    }
    //if (mesh_msg_to_save.hasTriangles()) {
    for (size_t i = 0; i < (3 * mesh_msg_to_save.mesh_geometry.faces.size()); i += 3) {
        stream << "3 ";

        for (int j = 0; j < 3; j++) {
            stream << i + j << " ";
        }
        stream << std::endl;
    }
    //}
    stream.close();
    return true;
}