#ifndef SLAMESH_MAP_H
#define SLAMESH_MAP_H

#include "cell.h"

class OverlapCellsRelation{
    //store the overlaping correspondences of map cells between current scan and global map
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    //for each cell in current scan, it can be overlapped with global map:
    std::vector<Cell*> cells_now;//array of ptr to map cell in map_now
    //the correspondent cell in the global map are store in one-to-one correspondence vector:
    std::vector<Cell*> cells_glb;//array of ptr to map cell in map_glb
    //when cross-cell overlapping is allowed, we look for neighbour cells too, so 1-multi correspondence exist,
    std::vector<std::vector<Cell*>> multi_cells_glb;
    //inside the array, the map cells are store in order: 0,x-1,x+1,y-1,y+1,z-1,z+1 (when overlap legnth is 1)
    // cell in current scan can also have no correspondent cell in the global map, in this way, it is new map cells:
    std::vector<Cell*> cells_now_new;
};

class Map{
    // Map class. The current scan and the global map share similar behaviour, so both of them use the "Map" class,
    // eventhough traditionally you may think only the global map is "Map".
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::string name;// map_glb or map_now
    int step;// frame count
    double timestamp{-1};//timestamp of point cloud

    std::deque<pcl::PointCloud<pcl::PointXYZ>> pcl_raw_buff;//store new pcl data from callback
    pcl::PointCloud<pcl::PointXYZ> pcl_raw;//current new scan
    pcl::PointCloud<pcl::PointXYZ> pcl_voxel_filted;//downsampled
    PointMatrix points_turned;//raw points, store in point matrix, applied T on it after registration

    PointMatrix vertices_filted;//for visiualization
    PointMatrix ary_vertices_all_show[3];//for visiualization
    PointMatrix ary_overlap_vertices[3];//for registration
    PointMatrix ary_normal[3];//for point2planeregistration

    mesh_msgs::MeshGeometryStamped mesh_msg;//for mesh visualization

    std::unordered_map<double, Cell> cells_glb;//store the gped point in map glb
    std::vector<std::pair<double, Cell>> cells_now;//map now, why vector: easy for multi-thread
    std::vector<int> index_bucket_enough_point;//only cells have enough raw points will be reconstructed
    std::vector<Cell*> newly_updated_cells;//for visualization

    Map(Transf & Tguess);
    Map();
    void dividePointsIntoCell(PointMatrix & points_raw, const Map & map_glb, bool conduct_gp);
    void dividePointsIntoCellInitMap(PointMatrix & points_raw);
    OverlapCellsRelation overlapCells(Map & map_glb);
    OverlapCellsRelation overlapCellsCrossCell(Map & map_glb, int overlap_length);
    #ifdef VOXBLOX
    OverlapCellsRelation overlappedCellsForTsdf(Map &map_glb);
    #endif
    void filterVerticesByVariance(double variance_thr);
    void filterVerticesByVarianceOverlap(double variance_thr, Map & map_now);
    void findMatchPoints        (OverlapCellsRelation & overlap_ship, Map & map_glb, double variance_thr, bool residual_combination);
    void findMatchPointToMesh   (OverlapCellsRelation & overlap_ship, Map & map_glb, double variance_thr, bool residual_combination);
    void updateMap(Map & map_now);
    void registerToMap(Map & map_glb, Transf & Tguess, double max_time);
    Transf computeT(Map & map_glb, Map & map_now);
    Transf computeTPointToMesh(Map & map_glb, Map & map_now);
    bool processNewScan(Transf & Tguess, int step_, const Map & map_glb);
    void filterMeshLocal();
    void filterMeshGlb();
    #ifdef VOXBLOX
    void meshingConnectVoxblox(Map & map_now);
    void meshingTsdfGlb(Map &map_now);
    #endif
    bool outputMeshAsPly(const std::string & filename, const mesh_msgs::MeshGeometryStamped & mesh_msg_to_save);
};
#endif //SLAMESH_MAP_H
