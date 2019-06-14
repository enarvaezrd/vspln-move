#ifndef MAP_GEN
#define MAP_GEN


#include "includes.hpp"


class ObstacleMapGen{
    public:
    ObstacleMapGen(int map_size_, float scale_) :
    MapSize(map_size_+1),
    max_dimm(scale_),
    x_offset(0.18){
        HalfMapSize = (MapSize-1)/2;
        MapResolution = (MapSize-1)/(max_dimm*2.0);
        ObstacleMap.resize(MapSize);

        for (int i=0;i<MapSize;i++) 
                ObstacleMap[i].resize(MapSize);

        ObstacleMapV=ObstacleMap; //resize
        for (int i=0;i<MapSize;i++) 
            for (int j=0;j<MapSize;j++)
                 ObstacleMapV[i][j]=0;
        ObstacleMap=ObstacleMapV;
        sub_Laser    = nh_mp.subscribe("/robot1/front_laser/scan", 1, &ObstacleMapGen::Laser_Handler, this);
    }


    void CreateMap();
    void Laser_Handler(const sensor_msgs::LaserScan& ls);
    int R_to_Cells(float real_point, bool limit );
    std::vector<VectorInt > get_Map(){Map_mtx.lock(); 
                                      std::vector<VectorInt > MapT = ObstacleMap;
                                      Map_mtx.unlock();
                                      return MapT;}

    std::vector<VectorInt > ObstacleMap, ObstacleMapV;
    float max_dimm;
    ros::NodeHandle nh_mp;
    ros::Subscriber sub_Laser ;  //Marker pose
    LaserDataS LaserData;
    int x_offset;
    std::mutex Map_mtx;
    int MapSize;
    int HalfMapSize;
    double MapResolution;
};

#endif
