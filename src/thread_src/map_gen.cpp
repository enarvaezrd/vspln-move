#ifndef MAP_GEN_CODE
#define MAP_GEN_CODE

#include "map_gen.hpp"


void ObstacleMapGen::CreateMap() //play sequentially with Laser_Handler
{
    std::vector<VectorInt > ObstacleMapT= ObstacleMapV;
    
    cv::Mat image_test = cv::Mat::zeros(MapSize,MapSize,CV_8UC1);
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
                image_test.at<int>(xc,yc)=1;
            }
        }
         auto start=std::chrono::high_resolution_clock::now();
       // ObstacleMapT = Thicken_Map_from_Image(image_test,obs_pos);
           ObstacleMapT = Thicken_Map(ObstacleMapT,obs_pos);
auto end=std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    //Print("Dilation time",elapsed.count());
   
   
    
    Map_mtx.lock();
    ObstacleMap.assign(ObstacleMapT.begin(),ObstacleMapT.end());
    Map_mtx.unlock();
    
    Pts_mtx.lock();
    Obstacle_Points.clear();
    Obstacle_Points.assign(obs_pos.begin(),obs_pos.end());
    Pts_mtx.unlock();
    //Print("Obstacle Points SIZE",Obstacle_Points.size());
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

// O(n^2) solution to find the Manhattan distance to "on" pixels in a two dimension array
std::vector<VectorInt >  ObstacleMapGen::Manhattan(std::vector<VectorInt >  Obs_Map){
    // traverse from top left to bottom right
    for (int i=0; i<Obs_Map.size(); i++){
        for (int j=0; j<Obs_Map[i].size(); j++){
            if (Obs_Map[i][j] == 1){
                // first pass and pixel was on, it gets a zero
                Obs_Map[i][j] = 0;
            } else {
                // pixel was off
                // It is at most the sum of the lengths of the array
                // away from a pixel that is on
                Obs_Map[i][j] = Obs_Map.size() + Obs_Map[i].size();
                // or one more than the pixel to the north
                if (i>0) Obs_Map[i][j] = std::min(Obs_Map[i][j], Obs_Map[i-1][j]+1);
                // or one more than the pixel to the west
                if (j>0) Obs_Map[i][j] = std::min(Obs_Map[i][j], Obs_Map[i][j-1]+1);
            }
        }
    }
    // traverse from bottom right to top left
    for (int i=Obs_Map.size()-1; i>=0; i--){
        for (int j=Obs_Map[i].size()-1; j>=0; j--){
            // either what we had on the first pass
            // or one more than the pixel to the south
            if (i+1<Obs_Map.size()) Obs_Map[i][j] = std::min(Obs_Map[i][j], Obs_Map[i+1][j]+1);
            // or one more than the pixel to the east
            if (j+1<Obs_Map[i].size()) Obs_Map[i][j] = std::min(Obs_Map[i][j], Obs_Map[i][j+1]+1);
        }
    }
    return Obs_Map;
}

// n^2 solution with Manhattan oracle
std::vector<VectorInt > ObstacleMapGen::Thicken_Map_Manhattan(std::vector<VectorInt > Obs_Map, std::vector<Position> &obs_positions){
    Obs_Map = Manhattan(Obs_Map);
    for (int i=0; i<Obs_Map.size(); i++){
        for (int j=0; j<Obs_Map[i].size(); j++){
            Obs_Map[i][j] = ((Obs_Map[i][j]<=k)?1:0);
            if(Obs_Map[i][j]==1) {
                Position pT;
                pT.xval=round(i*map_img_factor);
                pT.yval=round(j*map_img_factor);
                obs_positions.push_back(pT);
            }
        }
    }
    return Obs_Map;
}
std::vector<VectorInt > ObstacleMapGen::Thicken_Map_from_Image(cv::Mat Image, std::vector<Position> &obs_positions)
{
    std::vector<VectorInt > ObstacleMap;
     ObstacleMap.resize(Image.rows);
    int morph_size = 3;
    cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 4*morph_size + 1, 2*morph_size+1 ),
                     cv::Point( morph_size, morph_size ) );
    dilate( Image, Image, element );  
    dilate( Image, Image, element );
    dilate( Image, Image, element );

    for (int i=0;i<Image.rows;i++)
    {
        ObstacleMap[i].resize(Image.cols);
        for (int j=0;j<Image.cols;j++)
        {
            ObstacleMap[i][j]=0;
            //Print("img val",Image.at<int>(i,j));
            if(Image.at<int>(i,j)>=1)
            {
               Print("element found",i,j,Image.at<int>(i,j)>=1);
                Position pT;
                pT.xval=round(i*map_img_factor);
                pT.yval=round(j*map_img_factor);
                obs_positions.push_back(pT);
                ObstacleMap[i][j]=1;
            }
        }
    }
    return ObstacleMap;
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
