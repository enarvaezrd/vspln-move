#ifndef MAP_GEN
#define MAP_GEN

#include "uav_ugv_commands.hpp"

class ObstacleMapGen
{
public:
    ObstacleMapGen(int map_size_, float scale_, int image_size_, float rad_int_, float rad_ext_) : MapSize(map_size_ + 1),
                                                                                                   max_dimm(scale_),
                                                                                                   x_offset(0.23),
                                                                                                   k(20),
                                                                                                   image_size(image_size_),
                                                                                                   rad_ext(rad_ext_),
                                                                                                   rad_int(rad_int_)
    {
        toDegrees = 180.0 / 3.141593;
        HalfMapSize = (MapSize - 1) / 2;
        MapResolution = (MapSize - 1) / (max_dimm * 2.0);
        ObstacleMap.resize(MapSize);

        for (int i = 0; i < MapSize; i++)
            ObstacleMap[i].resize(MapSize);

        ObstacleMapV = ObstacleMap; //resize
        for (int i = 0; i < MapSize; i++)
            for (int j = 0; j < MapSize; j++)
                ObstacleMapV[i][j] = 0;
        ObstacleMap = ObstacleMapV;
        //  /opt/ros/kinetic/share/robotnik_sensors/urdf/hokuyo_ust10lx.urdf.xacro  //To modify frecuency
        sub_Laser = nh_map_gen.subscribe("/scan", 1, &ObstacleMapGen::Laser_Handler, this); ///robot1/front_laser/scan. REAL

        //sub_Laser = nh_map_gen.subscribe("/robot1/front_laser/scan", 1, &ObstacleMapGen::Laser_Handler, this);   ///robot1/front_laser/scan SIMM
        map_img_factor = (double)(image_size) / (float)(MapSize);
    }

    void Thicken_Map();
    void CreateMap();
    void Laser_Handler(const sensor_msgs::LaserScan &ls);
    int R_to_Cells(double real_point, bool limit);
    double Cells_to_Real(float cells_point);
    std::vector<VectorInt> get_Map()
    {
        Map_mtx.lock();
        std::vector<VectorInt> MapT = ObstacleMap;
        Map_mtx.unlock();
        return MapT;
    }
    std::vector<Position> get_Obs_Points()
    {
        Pts_mtx.lock();
        std::vector<Position> Pts = Obstacle_Points;
        Pts_mtx.unlock();
        return Pts;
    }
    std::vector<Position> get_Obs_Points_Thick()
    {
        Pts_mtx.lock();
        std::vector<Position> Pts = Obstacle_Points_Thick;
        Pts_mtx.unlock();
        return Pts;
    }
    std::vector<VectorInt> Thicken_Map(std::vector<VectorInt> Obs_Map, std::vector<Position> &obs_positions);
    std::vector<VectorInt> Manhattan(std::vector<VectorInt> Obs_Map);

    std::vector<VectorInt> Thicken_Map_Manhattan(std::vector<VectorInt> Obs_Map, std::vector<Position> &obs_positions);

    std::vector<VectorInt> Thicken_Map_from_Image(cv::Mat Image, std::vector<Position> &obs_positions);
    void Get_Obstacle_Points(std::vector<VectorInt> Obs_Map, std::vector<Position> &obs_positions);

    void ExpandObstacle_Polar(std::vector<VectorInt> &ObstacleMapT, std::vector<Position> &obs_pos);

    void Rect_to_Polar(int x, int y, double &radius, double &angle);

    void Expand_Obstacle(double radius, double angle, std::vector<VectorInt> &ObstacleMapT);

    Printer Print;
    std::vector<VectorInt> ObstacleMap, ObstacleMapV;
    float max_dimm;
    ros::NodeHandle nh_map_gen;
    ros::Subscriber sub_Laser; //Marker pose
    LaserDataS LaserData;
    float x_offset;
    std::mutex Map_mtx;
    std::mutex Pts_mtx;
    int MapSize;
    int HalfMapSize;
    double MapResolution;
    int k;
    double map_img_factor;
    int image_size;
    std::vector<Position> Obstacle_Points_Thick;
    std::vector<Position> Obstacle_Points;
    float rad_ext, rad_int;
    double toDegrees;
};

#endif
