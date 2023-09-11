#include "cell.h"
#include "slamesher_node.h"
extern Parameter param;//in SLAMesh node
extern Log g_data;//in SLAMesh node

void VoxelFilter2D(int num_input, int & num_output, double grid,
                   Matrix1xd & f, Matrix1xd & train_x, Matrix1xd & train_y){
    //principled and fast 2D voxel filter to downsample the training points in GP process. Faster than kdtree.
    int    num_test = param.num_test;
    double interval = grid / num_test;

    double bot_x = floor(train_x(0,0) / grid) * grid;
    double bot_y = floor(train_y(0,0) / grid) * grid;

    Eigen::MatrixXd point_result(4, num_test * num_test);
    point_result.topRows(3).fill(0);
    point_result.row(3).fill(-1);// -1 means not filled yet

    //downsample mode chose
    bool just_first_point = false;//true false
    bool even_closest_one_point = true;
    for(int i = 0; i < num_input; i++){
        int index = 1 * int((train_x(0, i) - bot_x) / interval) +
                  num_test * int((train_y(0,i)-bot_y) / interval);
        if(just_first_point){
            if(point_result(3, index) < 1){
                point_result(3, index) = 1;
                point_result(0, index) = train_x(0, i);
                point_result(1, index) = train_y(0, i);
                point_result(2, index) = f(0, i);
            }
        }
        else if(even_closest_one_point){
            int index_x = int((train_x(0, i) - bot_x) / interval);//0 offset
            int index_y = int((train_y(0, i) - bot_y) / interval);
            int index_linear = 1 * index_x + num_test * index_y;

            double  iy = (index_linear / num_test) * interval + bot_y + 0.5 * interval,
                    ix = (index_linear % num_test) * interval + bot_x + 0.5 * interval;
            double dis_now = pow(train_x(0,i) - ix, 2) + pow(train_y(0,i) - iy, 2);
            if(point_result(3, index_linear) < 0){
                point_result(3, index_linear) = dis_now;
                point_result(0, index_linear) = train_x(0, i);
                point_result(1, index_linear) = train_y(0, i);
                point_result(2, index_linear) = f(0, i);
            }
            else if(dis_now < point_result(3, index_linear)){
                point_result(3, index_linear) = dis_now;
                point_result(0, index_linear) = train_x(0, i);
                point_result(1, index_linear) = train_y(0, i);
                point_result(2, index_linear) = f(0, i);
            }
        }
    }

    int col = 0;
    for(size_t i = 0; i < num_test*num_test; i++){
        if(point_result(3,i) > 0){
            train_x(0, col) = point_result(0, i);
            train_y(0, col) = point_result(1, i);
            f      (0, col) = point_result(2, i);
            col ++;
        }
    }
    num_output = col;
}

void Cell::gaussianProcess(enum Direction gp_direction){
    //standard gaussianProcess
    bool full_cover = param.full_cover; //whether the first and final test points lie on the border or not.
    //full_cover: test point location start from 0.5*step, like 0.5, 1.5, ..., 9.5
    //if full_cover == false: test point location start from 0*step, like 0, 1, ..., 10
    int num_test_side = param.num_test;
    double grid = param.grid;
    int num_test_square = param.num_test * param.num_test;
    double interval = grid / num_test_side;
    double variance_sensor = param.variance_sensor;
    double kernel_length = 1.2;// gp kernel
    //create test point
    Eigen::MatrixXd I_test =  Eigen::MatrixXd::Identity(num_test_square, num_test_square);
    Eigen::Matrix<double, 1, Eigen::Dynamic>  test_x, test_y;
    Eigen::MatrixXd points_testm(num_test_square, 2);

    Direction prediction_dir = Direction((gp_direction + 0) % 3);
    Direction location_dir_x = Direction((gp_direction + 1) % 3);
    Direction location_dir_y = Direction((gp_direction + 2) % 3);

    //set test locations
    if(gp_direction == X){
        evenSetLinSpaced(test_x, num_test_side, region.y_min, region.y_max, full_cover);
        evenSetLinSpaced(test_y, num_test_side, region.z_min, region.z_max, full_cover);
    }
    else if(gp_direction == Y){
        evenSetLinSpaced(test_x, num_test_side, region.z_min, region.z_max, full_cover);
        evenSetLinSpaced(test_y, num_test_side, region.x_min, region.x_max, full_cover);
    }
    else if(gp_direction == Z){
        evenSetLinSpaced(test_x, num_test_side, region.x_min, region.x_max, full_cover);
        evenSetLinSpaced(test_y, num_test_side, region.y_min, region.y_max, full_cover);
    }

    for(int row=0; row < num_test_square; row++){
        points_testm(row, 0) = test_x(0, row / num_test_side);
        points_testm(row, 1) = test_y(0, row % num_test_side);
    }

    //creat train point
    int num_train = -1;
    Eigen::Matrix<double, 1, Eigen::Dynamic> f_raw, train_x_raw, train_y_raw;
    Eigen::MatrixXd f(1, num_test_square), train_x(1, num_test_square), train_y(1, num_test_square);
    double train_x_min, train_x_max, train_y_min, train_y_max;
    //voxelfilter to downsample train point
    int num_train_raw = cell_raw_points.num_point;
    if(num_train_raw == 0){
        ROS_ERROR("No trian pointin num in gp! num_train_raw");
        while(ros::ok());
    }

    f_raw       = cell_raw_points.point.row(prediction_dir).leftCols(num_train_raw);
    train_x_raw = cell_raw_points.point.row(location_dir_x).leftCols(num_train_raw);//train_x---points_testm.col(0)---gp_direction + 1
    train_y_raw = cell_raw_points.point.row(location_dir_y).leftCols(num_train_raw);
    train_x_min = train_x_raw.minCoeff();
    train_x_max = train_x_raw.maxCoeff();
    train_y_min = train_y_raw.minCoeff();
    train_y_max = train_y_raw.maxCoeff();

    VoxelFilter2D(num_train_raw, num_train, grid, f_raw, train_x_raw, train_y_raw);
    if(num_train > num_test_square ){
        ROS_ERROR("Too many trian pointin num in gp! num_train_after_voxel_filter");
        while(ros::ok());
    }
    if(num_train == 0 ){
        ROS_ERROR("0 trian pointin num in gp! num_train_after_voxel_filter");
        while(ros::ok());
    }
    f.leftCols      (num_train) = f_raw.leftCols      (num_train);
    train_x.leftCols(num_train) = train_x_raw.leftCols(num_train);
    train_y.leftCols(num_train) = train_y_raw.leftCols(num_train);

    Eigen::MatrixXd I_train = Eigen::MatrixXd::Identity(num_train, num_train);

    //gp
    double mean = f.leftCols(num_train).mean();
    f.leftCols(num_train).array() -= mean;

    Eigen::MatrixXd f_starm(num_test_square, 1),
             variance_starm(num_test_square, 1);
    Eigen::MatrixXd       k(num_test_square, num_test_square),
                         ky(num_test_square, num_test_square),
                     ky_inv(num_test_square, num_test_square),
                    k_starm(num_test_square, num_test_square),
                        kky(num_test_square, num_test_square);

    for(int row = 0; row < num_train; row++){
        for(int col = 0; col < num_train; col++){
            k(row, col) = exp_quick(-kernel_length * sqrt(pow((train_x(0, col) - train_x(0, row)), 2) +
                                                          pow((train_y(0, col) - train_y(0, row)), 2)));

        }
    }
    for(int row = 0; row < num_test_square; row++){
        for(int col = 0; col < num_train; col++){
            k_starm(row, col) = exp_quick(-kernel_length * sqrt(pow((train_x(0, col) - points_testm(row, 0)), 2) +
                                                                pow((train_y(0, col) - points_testm(row, 1)), 2)));

        }
    }
    ky.block(0, 0, num_train, num_train) = k.block(0, 0, num_train, num_train);
    for(int row = 0; row < num_train; row++){
        ky(row, row) = k(row, row) + variance_sensor * variance_sensor;
    }
    //choose the method to calculate the inverse of ky
    //kky = k_starm.adjoint()*(ky.inverse());//PartialPivLU
    //kky = k_starm.adjoint()*(ky.ldlt().solve(I_train));//LDLT
    kky.block(0, 0, num_test_square, num_train) = k_starm.block(0, 0, num_test_square, num_train) *
                                                  (ky.block(0, 0, num_train, num_train).llt().solve(I_train));//LLT fastest
    f_starm = (kky.block(0, 0, num_test_square, num_train) * ((f.leftCols(num_train)).transpose())).array() + mean;

    variance_starm = (I_test - kky.block(0, 0, num_test_square, num_train) *
                               k_starm.block(0, 0, num_test_square, num_train).transpose()) .diagonal();

    ary_cell_vertices[gp_direction] = Eigen::MatrixXd::Zero(3, num_test_square);
    ary_cell_vertices[gp_direction].point.row(prediction_dir).leftCols(num_test_square) = f_starm.transpose();
    ary_cell_vertices[gp_direction].point.row(location_dir_x).leftCols(num_test_square) = points_testm.col(0).transpose();
    ary_cell_vertices[gp_direction].point.row(location_dir_y).leftCols(num_test_square) = points_testm.col(1).transpose();
    ary_cell_vertices[gp_direction].num_point = num_test_square;
    ary_cell_vertices[gp_direction].variance.leftCols(num_test_square) = variance_starm.transpose();
    ary_cell_vertices[gp_direction].variance_sum = ary_cell_vertices[gp_direction].variance.leftCols(num_test_square).sum();
    //limit the prediction inside the cell
    bool bound_point = true;//false true
    if(bound_point){
        for(int i_point = 0; i_point < num_test_square; i_point++){
            //prediction should not exceed the region
            if(ary_cell_vertices[gp_direction].point(prediction_dir, i_point) > region.getDirMax(gp_direction)){
                ary_cell_vertices[gp_direction].point(prediction_dir, i_point) = region.getDirMax(gp_direction);
            }
            if(ary_cell_vertices[gp_direction].point(prediction_dir, i_point) < region.getDirMin(gp_direction)){
                ary_cell_vertices[gp_direction].point(prediction_dir, i_point) = region.getDirMin(gp_direction);
            }
            //if the locations of test points exceed min/max locations of training points, the variance are punished.
            if(ary_cell_vertices[gp_direction].point(location_dir_x, i_point) > train_x_max ||
               ary_cell_vertices[gp_direction].point(location_dir_x, i_point) < train_x_min ||
               ary_cell_vertices[gp_direction].point(location_dir_y, i_point) > train_y_max ||
               ary_cell_vertices[gp_direction].point(location_dir_y, i_point) < train_y_min){
               ary_cell_vertices[gp_direction].variance(0, i_point) *= 2;
            }
        }
    }
    empety = false;
}
void Cell::reconstructSurfaces(bool glb_cell_not_surface){
    //use gaussian process to reconstruct the local surfaces inside a cell, one cell can have 3 surfaces in 3 directions
    ROS_DEBUG("reconstructSurfaces");
    //TicToc t_mc_gp_init;
    double num_test = param.num_test;
    double eig_3ratio2_not_surface = param.eigen_1,
           eig_2minus1_distinctive_angle = param.eigen_2,
           eig_3minus2_obscure_angle = param.eigen_3;
    for(auto & i_vertices : ary_cell_vertices){
        i_vertices.clear();
    }
    //decide direction of GP, based on PCA (Principal Component Analysis)
    bool direction_list[3];
    memset(direction_list, false, sizeof(direction_list));
    memset(updated_times, 0, sizeof(updated_times));
    Eigen::Matrix<double, 1, 3> angle_a, angle_b;
    std::map<double, int> sorted_angle_a, sorted_angle_b;

    cell_raw_points.eigenDecomposition();
    angle_a = (cell_raw_points.eig_sorted.begin() ->second.transpose() * Eigen::MatrixXd::Identity(3, 3)).array().acos();
    angle_b = (cell_raw_points.eig_sorted.rbegin()->second.transpose() * Eigen::MatrixXd::Identity(3, 3)).array().acos();
    //sort
    for(int i = 0; i < 3; i++){
        angle_a(0, i) = (angle_a(0, i) > M_PI / 2) ? M_PI - angle_a(0, i) : angle_a(0, i);
        angle_b(0, i) = (angle_b(0, i) > M_PI / 2) ? M_PI - angle_b(0, i) : angle_b(0, i);
        sorted_angle_a.emplace(angle_a(0, i), i);
        sorted_angle_b.emplace(angle_b(0, i), i);
    }

    if(cell_raw_points.eig_sorted.rbegin()->first / (++cell_raw_points.eig_sorted.begin())->first > eig_3ratio2_not_surface){//30
        //3/2 > threshold, not surface
        not_surface = true;
        g_data.not_a_surface_cell_num(0, g_data.step) ++;
        //unless map_glb say its cell on this region is a surface, no reconstruction will be conducted
        if(!glb_cell_not_surface){
            direction_list[(sorted_angle_b.begin()->second + 1) % 3] = true;//do not predict the max beta angle direction
            direction_list[(sorted_angle_b.begin()->second + 2) % 3] = true;
        }
    }
    else{
        //can form surface(s)
        not_surface = false;
        if((++sorted_angle_a.begin()) ->first - sorted_angle_a.begin()->first > eig_2minus1_distinctive_angle){//0.2
            //2-1 > threshold, distinctive angle, only one gp function is enough
            direction_list[sorted_angle_a.begin()->second] = true;
        }
        else {
            //<0.2
            if(sorted_angle_a.rbegin()->first - (++sorted_angle_a.begin())->first > eig_3minus2_obscure_angle){//0.2
                //3-2 obscure angle(OA)
                direction_list[   sorted_angle_a.begin() ->second] = true;
                direction_list[(++sorted_angle_a.begin())->second] = true;
            }
            else{
                //2-1<0.2 && 3-2<0.2, use 3 gp function, or there are 3 layers inside this cell
                for(auto & i : direction_list) i = true;//all is true
            }
        }
    }
    //carry out reconstruction
    for(int i = 0; i < 3; i++){
        if(direction_list[i]){
            gaussianProcess(Direction(i));
            g_data.gp_times(0, g_data.step) ++;
            updated_times[i] = 1;
        }
        else{
            updated_times[i] = 0;
        }
    }
}
 void Cell::updateVertices(Cell & cell_new, enum Direction update_direction){
    //update the vertices of a local surface (with certain prediction direction) inside a cell
    ROS_DEBUG("updateVertices");
    //init
    //double dyna_thr_grid_ratio = 0.5;
    int num_test_square = param.num_test * param.num_test;
    double variance_map_update = param.variance_map_update;
    Eigen::Matrix<double, 1, Eigen::Dynamic> update, observe, variance_update ,variance_observe;
     update = observe = variance_update = variance_observe = Eigen::MatrixXd::Zero(1, num_test_square);

    PointMatrix & points_old = ary_cell_vertices[update_direction];
    PointMatrix & points_new = cell_new.ary_cell_vertices[update_direction];

     update = points_old.point.leftCols(num_test_square).row(update_direction);
     observe = points_new.point.leftCols(num_test_square).row(update_direction);
     variance_update = points_old.variance.leftCols(num_test_square);
     variance_observe = points_new.variance.leftCols(num_test_square);

    //iteratively least square update
    for(int i = 0; i < num_test_square; i++){
        if((variance_update(0, i) <= variance_map_update) && (variance_observe(0, i) <= variance_map_update)){
            update(0, i)   = (update(0, i) * variance_observe(0, i) + observe(0, i) * variance_update(0, i))
                             / (variance_update(0, i) + variance_observe(0, i));
            variance_update(0, i) = variance_update(0, i) * variance_observe(0, i) / (variance_update(0, i) + variance_observe(0, i));
        }
        else if((variance_update(0, i) > variance_map_update) || (variance_observe(0, i) > variance_map_update)){
            update(0, i)   = (variance_update(0, i) <= variance_observe(0, i)) ? (update  (0, i)) : (observe (0, i));
            variance_update(0, i) = (variance_update(0, i) <= variance_observe(0, i)) ? (variance_update(0, i)) : (variance_observe(0, i));
        }
    }
    updated_times[update_direction] ++;

     points_old.point.leftCols(num_test_square).row(update_direction) = update;
     points_old.variance.leftCols(num_test_square) = variance_update;

     points_old.variance_sum = points_old.variance.leftCols(num_test_square).sum();
}
void Cell::updateViewedLocation(const Transf & transf_viewed) {
    //iteratively update the view direction from sensor to certain surface inside a cell
    ROS_DEBUG("updateViewedLocation");
    Point viewed_pose = trans3Dpoint(0, 0, 0, transf_viewed);
    Point viewed_arrow = viewed_pose - center;
    Point current_viewed_dir = viewed_arrow / viewed_arrow.norm();

    viewed_location = (viewed_location * viewed_dir_count + current_viewed_dir) /
                      (viewed_location * viewed_dir_count + current_viewed_dir).norm();
    average_viewed_distance = (average_viewed_distance * viewed_dir_count + viewed_arrow.norm()) / (1 + viewed_dir_count);
    viewed_dir_count ++;
}

