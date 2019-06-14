#ifndef MAP_GEN_CODE
#define MAP_GEN_CODE

#include "map_gen.hpp"


void ObstacleMapGen::CreateMap() //play sequentially with Laser_Handler
{
    std::vector<VectorInt > ObstacleMapT= ObstacleMapV;
    int square_amp=15;
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
                for(int j=-square_amp+1; j<square_amp; j++)
                {
                    for(int k=-square_amp+1; k<square_amp; k++)
                    {
                        if(xc+j>=0 && xc+j<MapSize && yc+k>=0 && yc+k<MapSize)
                        {
                            ObstacleMapT[xc+j][yc+k]=1;
                        }
                    }
                }
            }
        }
    }
    Map_mtx.lock();
    ObstacleMap = ObstacleMapT;
    Map_mtx.unlock();
    return;
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
      }
      else LaserData.state = false;
     // for (int i=0; i<LaserData.size;i++)
     //   Print("LaserData",LaserData.ranges[i]);
    return;
}

#endif
