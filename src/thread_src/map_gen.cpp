#ifndef MAP_GEN_CODE
#define MAP_GEN_CODE

#include "map_gen.hpp"


void ObstacleMapGen::CreateMap() //play sequentially with Laser_Handler
{
    std::vector<VectorInt > ObstacleMapT= ObstacleMapV;
   
    Obstacle_Points.clear();
    std::vector<Position> obs_pos;
    //Print("Laser Data", LaserData.min_angle, LaserData.max_angle, LaserData.range_max, LaserData.range_min, LaserData.state, LaserData.angle_increment,LaserData.size);
    if (LaserData.state){
        for (int i=0;i<LaserData.size;i++)
         {
            float angle = LaserData.min_angle + (i*LaserData.angle_increment);
            if (LaserData.ranges[i]==INFINITY) LaserData.ranges[i] = LaserData.range_max;
            float yr = LaserData.ranges[i]*sin(angle);
            float xr = LaserData.ranges[i]*cos(angle) + x_offset;  //added Lidar offset from manipulator coordinates
            
            int xc = R_to_Cells(xr,false);//no limit, otherwise the limit will appear as obstacle
            int yc = R_to_Cells(yr,false);
            if (xc>=0 && xc<MapSize && yc>=0 && yc<MapSize)
            {
                ObstacleMapT[xc][yc]=1;
            }
        }
    
    ObstacleMapT = Thicken_Map(ObstacleMapT,obs_pos);
    Map_mtx.lock();
    ObstacleMap.assign(ObstacleMapT.begin(),ObstacleMapT.end());
    Map_mtx.unlock();
    
    Pts_mtx.lock();
    Obstacle_Points.assign(obs_pos.begin(),obs_pos.end());
    Pts_mtx.unlock();
    Print("Obstacle Points SIZE",Obstacle_Points.size());
    }
    return;
}

std::vector<VectorInt >  ObstacleMapGen::Thicken_Map(std::vector<VectorInt > Obs_Map, std::vector<Position> &obs_positions)
{
   // obs_positions.clear();

    for (int i=0; i<MapSize; i++){
        for (int j=0; j<MapSize; j++){
            if (Obs_Map[i][j] == 1){
                for (int l=i-k; l<=i+k; l++){
                    int remainingk = k-abs(i-l);
                    for (int m=j-remainingk; m<=j+remainingk; m++){
                        if (l>=0 && m>=0 && l<MapSize && m<MapSize && Obs_Map[l][m]==0){
                            Obs_Map[l][m] = 2;
                        }
                    }
                }
            }
        }
    }
    for (int i=0; i<MapSize; i++){
        for (int j=0; j<MapSize; j++){
            if (Obs_Map[i][j] == 2){
                Obs_Map[i][j] = 1;
                Position pT;
                pT.xval=round(i*map_img_factor);
                pT.yval=round(j*map_img_factor);
                obs_positions.push_back(pT);
            }
        }
    }
    return Obs_Map;
}
    
int ObstacleMapGen::R_to_Cells(float real_point, bool limit=false)  //if limited, the point will be always inside area
{
    if (real_point >= max_dimm && limit) real_point = max_dimm;
    if (real_point <= -max_dimm && limit) real_point = -max_dimm;
    return (int)(HalfMapSize + round(real_point * MapResolution));
}

void ObstacleMapGen::Laser_Handler(const sensor_msgs::LaserScan& laser_msg)
{
      LaserData.size = laser_msg.ranges.size();
      if (LaserData.size>0){
          
          LaserData.state = true;
          VectorDbl rangesT(laser_msg.ranges.begin(),laser_msg.ranges.end());
          VectorDbl intensitiesT(laser_msg.ranges.begin(),laser_msg.ranges.end());
          LaserData.ranges = rangesT;
          LaserData.intensities = intensitiesT;
          LaserData.range_max = laser_msg.range_max;
          LaserData.range_min = laser_msg.range_min;
          
          LaserData.max_angle = laser_msg.angle_max;
          LaserData.min_angle = laser_msg.angle_min;
          LaserData.angle_increment = laser_msg.angle_increment;

        CreateMap();

      }
      else LaserData.state = false;

    return;
}

#endif
