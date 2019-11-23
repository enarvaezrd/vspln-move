#ifndef MAP_GEN_CODE
#define MAP_GEN_CODE

#include "map_gen.hpp"

void ObstacleMapGen::CreateMap() //play sequentially with Laser_Handler
{
    auto start_t = std::chrono::high_resolution_clock::now();
    std::vector<VectorInt> ObstacleMapT = ObstacleMapV; //void map

    // cv::Mat image_test = cv::Mat::zeros(MapSize, MapSize, CV_8UC1);
    

    if (LaserData.state)
    {
        std::vector<Position> obs_pos_thick, obs_pos;
        bool obstacle_found = false;
        int i = 0;
        for (auto laser_range : LaserData.ranges)
        {
            
            if (laser_range == INFINITY || laser_range > 0.7 || laser_range < 0.005)
            {
                //laser_range = LaserData.range_max;
                continue;
            }
            float angle = LaserData.min_angle + (i * LaserData.angle_increment);
            i++;
            double yr = laser_range * sin(angle);
            double xr = laser_range * cos(angle) + x_offset; //added Lidar offset from manipulator coordinates

            int xc = R_to_Cells(xr, false); //no limit, otherwise the limit will appear as obstacle
            int yc = R_to_Cells(yr, false);
            if (xc >= 0 && xc < MapSize && yc >= 0 && yc < MapSize)
            {
                ObstacleMapT[xc][yc] = 1;
                obstacle_found = true;
                // cout << "value for " << i << ": " << LaserData.ranges[i] << ", angle: " << angle << ", intensities: " << LaserData.intensities[i] << endl;
                //image_test.at<int>(xc, yc) = 1;
            }
        }
        if (obstacle_found)
        {
            AccumulateObstacleMaps(ObstacleMapT);

            // ObstacleMapT = Thicken_Map_from_Image(image_test,obs_pos);
            Get_Obstacle_Points(ObstacleMap_Global, obs_pos);
            ExpandObstacle_Polar(ObstacleMap_Global, obs_pos);
            ObstacleMap_Global = Thicken_Map_Manhattan(ObstacleMap_Global, obs_pos_thick);
            // ObstacleMapT = Thicken_Map(ObstacleMapT, obs_pos_thick);
            //ObstacleMapT = Thicken_Map(ObstacleMapT, obs_pos_thick);
        }
        Map_mtx.lock();
        if (obstacle_found)
        {
            ObstacleMap.assign(ObstacleMap_Global.begin(), ObstacleMap_Global.end());
        }
        else
        {
            ObstacleMap.assign(ObstacleMapV.begin(), ObstacleMapV.end());
        }
        Map_mtx.unlock();

        Pts_mtx.lock();
        Obstacle_Points_Thick.clear();

        Obstacle_Points.clear();
        if (obstacle_found)
        {
            Obstacle_Points_Thick.assign(obs_pos_thick.begin(), obs_pos_thick.end());
            Obstacle_Points.assign(obs_pos.begin(), obs_pos.end());
        }
        Pts_mtx.unlock();
       // auto end_t = std::chrono::high_resolution_clock::now();
    }
    return;
}

void ObstacleMapGen::ExpandObstacle_Polar(std::vector<VectorInt> &ObstacleMapT, std::vector<Position> &obs_pos)
{
    for (int i = 0; i < obs_pos.size(); i++)
    {
        double radius, angle;
        Rect_to_Polar(obs_pos[i].xval / map_img_factor, obs_pos[i].yval / map_img_factor, radius, angle);

        Expand_Obstacle(radius, angle, ObstacleMapT);
    }
    obs_pos.clear();
    Position pT;
    bool is_obstacle;

    for (int i = 15; i < MapSize - 15; i++)
    {
        for (int j = 15; j < MapSize - 15; j++)
        {
            is_obstacle = false;
            if (ObstacleMapT[i][j] == 2 || ObstacleMapT[i][j] == 1)
            {
                is_obstacle = true;
            }
            /* else
            {

                double x_real = Cells_to_Real(i);
                double y_real = Cells_to_Real(j);
                double radius = sqrt((x_real * x_real) + (y_real * y_real));
                if ( radius < rad_int-0.24)
                {
                    is_obstacle = true;
                }
            }*/
            if (is_obstacle)
            {
                pT.xval = (i * map_img_factor);
                pT.yval = (j * map_img_factor);
                obs_pos.push_back(pT);
                ObstacleMapT[i][j] = 1;
            }
        }
    }
    return;
}

void ObstacleMapGen::Rect_to_Polar(int x, int y, double &radius, double &angle)
{
    x -= ceil(MapSize / 2);
    y -= ceil(MapSize / 2);
    double x_d = x, y_d = y;
    radius = sqrt((x * x) + (y * y));
    angle = atan2(y_d, x_d); //* toDegrees;
    return;
}
void ObstacleMapGen::Expand_Obstacle(double radius, double angle, std::vector<VectorInt> &ObstacleMapT)
{
    bool limit_exceeded = false;
    double modified_radius = radius;
    int map_size_half = MapSize / 2;
    double cos_angle = cos(angle);
    double sin_angle = sin(angle);

    while (!limit_exceeded)
    {
        int x = round(modified_radius * cos_angle);
        int y = round(modified_radius * sin_angle);

        if (x >= map_size_half || x <= (-map_size_half) || y >= map_size_half || y <= (-map_size_half))
        {
            limit_exceeded = true;
            break;
        }
        if (x + map_size_half >= MapSize || y + map_size_half >= MapSize || x + map_size_half < 0 || y + map_size_half < 0)
        {
            Print("WARNING MAP EXTENSION", x, y, map_size_half);
            break;
        }
        ObstacleMapT[x + map_size_half][y + map_size_half] = 2;
        modified_radius += 0.1;
    }
    return;
}
void ObstacleMapGen::Get_Obstacle_Points(std::vector<VectorInt> Obs_Map, std::vector<Position> &obs_positions)
{
    Position pT;
    for (int i = 0; i < MapSize; i++)
    {
        for (int j = 0; j < MapSize; j++)
        {
            if (Obs_Map[i][j] == 1.0)
            {
                pT.xval = (i * map_img_factor);
                pT.yval = (j * map_img_factor);
                obs_positions.push_back(pT);
            }
        }
    }

    return;
}

std::vector<VectorInt> ObstacleMapGen::Thicken_Map(std::vector<VectorInt> Obs_Map, std::vector<Position> &obs_positions)
{
    // auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < MapSize; i++)
    {
        for (int j = 0; j < MapSize; j++)
        {
            if (Obs_Map[i][j] == 1)
            {
                for (int l = i - k; l <= i + k; l++)
                {
                    int remainingk = k - abs(i - l);
                    for (int m = j - remainingk; m <= j + remainingk; m++)
                    {
                        if (Obs_Map[l][m] == 0)
                        {
                            if (l >= 0 && m >= 0 && l < MapSize && m < MapSize)
                            {
                                Obs_Map[l][m] = 2;
                            }
                        }
                    }
                }
            }
        }
    }
    // auto end = std::chrono::high_resolution_clock::now();
    //  auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    // start = std::chrono::high_resolution_clock::now();
    Position pT;
    obs_positions.resize(MapSize * MapSize + 10);
    int size_obs_pos = 0;
    int it_i = 0, it_j = 0;
    for (auto i : Obs_Map)
    {
        it_j = 0;
        for (auto j : i)
        {
            if (j == 2)
            {
                j = 1;
                pT.xval = (it_i * map_img_factor);
                pT.yval = (it_j * map_img_factor);
                obs_positions[size_obs_pos] = pT;
                size_obs_pos++;
            }
            it_j++;
        }
        it_i++;
    }
    obs_positions.resize(size_obs_pos);
    // end = std::chrono::high_resolution_clock::now();
    // elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    return Obs_Map;
}

// O(n^2) solution to find the Manhattan distance to "on" pixels in a two dimension array
std::vector<VectorInt> ObstacleMapGen::Manhattan(std::vector<VectorInt> Obs_Map)
{
    // traverse from top left to bottom right
    for (int i = 0; i < MapSize; i++)
    {
        for (int j = 0; j < MapSize; j++)
        {
            if (Obs_Map[i][j] == 1)
            {
                // first pass and pixel was on, it gets a zero
                Obs_Map[i][j] = 0;
            }
            else
            {
                // pixel was off
                // It is at most the sum of the lengths of the array
                // away from a pixel that is on
                Obs_Map[i][j] = MapSize + MapSize;
                // or one more than the pixel to the north
                if (i > 0)
                    Obs_Map[i][j] = std::min(Obs_Map[i][j], Obs_Map[i - 1][j] + 1);
                // or one more than the pixel to the west
                if (j > 0)
                    Obs_Map[i][j] = std::min(Obs_Map[i][j], Obs_Map[i][j - 1] + 1);
            }
        }
    }
    // traverse from bottom right to top left
    for (int i = MapSize - 1; i >= 0; i--)
    {
        for (int j = MapSize - 1; j >= 0; j--)
        {
            // either what we had on the first pass
            // or one more than the pixel to the south
            if (i + 1 < MapSize)
                Obs_Map[i][j] = std::min(Obs_Map[i][j], Obs_Map[i + 1][j] + 1);
            // or one more than the pixel to the east
            if (j + 1 < MapSize)
                Obs_Map[i][j] = std::min(Obs_Map[i][j], Obs_Map[i][j + 1] + 1);
        }
    }
    return Obs_Map;
}

// n^2 solution with Manhattan oracle
std::vector<VectorInt> ObstacleMapGen::Thicken_Map_Manhattan(std::vector<VectorInt> Obs_Map, std::vector<Position> &obs_positions)
{
    Obs_Map = Manhattan(Obs_Map);
    for (int i = 0; i < MapSize; i++)
    {
        for (int j = 0; j < MapSize; j++)
        {
            Obs_Map[i][j] = ((Obs_Map[i][j] <= k) ? 1 : 0);

            if (Obs_Map[i][j] == 1)
            {
                Position pT;
                pT.xval = round(i * map_img_factor);
                pT.yval = round(j * map_img_factor);
                obs_positions.push_back(pT);
            }
        }
    }
    return Obs_Map;
}
std::vector<VectorInt> ObstacleMapGen::Thicken_Map_from_Image(cv::Mat Image, std::vector<Position> &obs_positions)
{
    std::vector<VectorInt> ObstacleMap;
    ObstacleMap.resize(Image.rows);
    int morph_size = 3;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(4 * morph_size + 1, 2 * morph_size + 1),
                                                cv::Point(morph_size, morph_size));
    dilate(Image, Image, element);
    dilate(Image, Image, element);
    dilate(Image, Image, element);
    for (int i = 0; i < Image.rows; i++)
    {
        ObstacleMap[i].resize(Image.cols);
        for (int j = 0; j < Image.cols; j++)
        {
            ObstacleMap[i][j] = 0;
            if (Image.at<int>(i, j) >= 1)
            {
                Position pT;
                pT.xval = round(i * map_img_factor);
                pT.yval = round(j * map_img_factor);
                obs_positions.push_back(pT);
                ObstacleMap[i][j] = 1;
            }
        }
    }
    return ObstacleMap;
}

void ObstacleMapGen::AccumulateObstacleMaps(std::vector<VectorInt> Obs_map)
{
    ugv_state_mtx.lock();
    RobotState_ ugv_State = UGV_state;
    double delta_time = map_build_milliseconds / 1000;
    ugv_state_mtx.unlock();

    double movement_x_real = (ugv_State.velocity_linear.dx * delta_time);
    double movement_y_real = (ugv_State.velocity_linear.dy * delta_time);

    double px_movement_x = round(movement_x_real * MapSize / max_dimm);
    double px_movement_y = round(movement_y_real * MapSize / max_dimm);

    num.MinMax_Correction(px_movement_x, 4.0);
    num.MinMax_Correction(px_movement_y, 4.0);
    // cout << "~~~Movements: x: " << px_movement_x << ", y: " << px_movement_y << "; time: " << delta_time << ", velocities: " << ugv_State.velocity_linear.dx << ", " << ugv_State.velocity_linear.dy << "\n";
    map_img_factor; //real size in m of each pixel

    ObstacleOldMaps[0] = Obs_map;

    int px_x = 0;
    int px_y = 0;
    int oldMapsSize = ObstacleOldMaps.size();
    for (int i = 0; i < MapSize; i++)
    {
        for (int j = 0; j < MapSize; j++)
        {
            if (ObstacleOldMaps[0][i][j] == 1)
            {
                ObstacleOldMaps[1][i][j] = 2;
                ObstacleOldMaps[0][i][j] = 2;
            }
        }
    }

    int global_value = 0;
    ObstacleMap_Global = ObstacleMapV;
    if (px_movement_x != 0.0 || px_movement_y != 0.0)
    {
        for (int i = k; i < MapSize - k; i++)
        {
            for (int j = k; j < MapSize - k; j++)
            {
                px_x = i - (int)(px_movement_x);
                px_y = j - (int)(px_movement_y);
                if (px_x >= 0 && px_x < MapSize && px_y >= 0 && px_y < MapSize)
                {
                    if ((ObstacleOldMaps[1][i][j] == 1 || ObstacleOldMaps[1][i][j] == 2) && ObstacleOldMaps[1][px_x][px_y] == 0)
                    {
                        ObstacleOldMaps[1][px_x][px_y] = 2;
                    }
                }
            }
        }
    }
    for (int i = 0; i < MapSize; i++)
    {
        for (int j = 0; j < MapSize; j++)
        {
            global_value = 0;
            std::for_each(ObstacleOldMaps.begin(),
                          ObstacleOldMaps.end(),
                          [&](std::vector<VectorInt> &Obs_m) {
                              if (Obs_m[i][j] == 2)
                              {
                                  Obs_m[i][j] = 1;
                                  global_value = 1;
                              }
                              else
                              {
                                  Obs_m[i][j] = 0;
                              }
                          });
            ObstacleMap_Global[i][j] = global_value;
        }
    }
    // cout << "map generated step1 " << ObstacleOldMaps.size() << "\n";

    return;
}
int ObstacleMapGen::R_to_Cells(double real_point, bool limit = false) //if limited, the point will be always inside area
{
    if (real_point >= max_dimm && limit)
        real_point = max_dimm;
    if (real_point <= -max_dimm && limit)
        real_point = -max_dimm;
    return (int)(HalfMapSize + round(real_point * MapResolution));
}
double ObstacleMapGen::Cells_to_Real(float cells_point) //if limited, the point will be always inside area
{

    return ((double)(cells_point) - (double)(HalfMapSize)) / (double)(MapResolution);
}

void ObstacleMapGen::Laser_Handler(const sensor_msgs::LaserScan &laser_msg)
{
    LaserData.size = laser_msg.ranges.size();
    if (LaserData.size > 0)
    {
        LaserData.state = true;
        VectorDbl rangesT(laser_msg.ranges.begin(), laser_msg.ranges.end());
        VectorDbl intensitiesT(laser_msg.ranges.begin(), laser_msg.ranges.end());
        LaserData.ranges = rangesT;
        LaserData.intensities = intensitiesT;
        LaserData.range_max = laser_msg.range_max;
        LaserData.range_min = laser_msg.range_min;

        LaserData.max_angle = laser_msg.angle_max;
        LaserData.min_angle = laser_msg.angle_min;
        LaserData.angle_increment = laser_msg.angle_increment;

        CreateMap();
    }
    else
        LaserData.state = false;

    return;
}

#endif
