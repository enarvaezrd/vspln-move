#ifndef MAP_GEN_CODE
#define MAP_GEN_CODE

#include "map_gen.hpp"

void ObstacleMapGen::CreateMap() //play sequentially with Laser_Handler
{
    std::vector<VectorInt> ObstacleMapT = ObstacleMapV;

    // cv::Mat image_test = cv::Mat::zeros(MapSize, MapSize, CV_8UC1);
    std::vector<Position> obs_pos_thick, obs_pos;
    //Print("Laser Data", LaserData.min_angle, LaserData.max_angle, LaserData.range_max, LaserData.range_min, LaserData.state, LaserData.angle_increment,LaserData.size);

    if (LaserData.state)
    {
        int i = 0;
        for (auto laser_range : LaserData.ranges)
        {
            float angle = LaserData.min_angle + (i * LaserData.angle_increment);
            i++;
            if (laser_range == INFINITY)
                laser_range = LaserData.range_max;
            double yr = laser_range * sin(angle);
            double xr = laser_range * cos(angle) + x_offset; //added Lidar offset from manipulator coordinates

            int xc = R_to_Cells(xr, false); //no limit, otherwise the limit will appear as obstacle
            int yc = R_to_Cells(yr, false);
            if (xc >= 0 && xc < MapSize && yc >= 0 && yc < MapSize && laser_range > 0.008)
            {
                ObstacleMapT[xc][yc] = 1;
                //  cout << "value for " << i << ": " << LaserData.ranges[i] << ", angle: " << angle <<", intensities: "<<LaserData.intensities[i]<< endl;
                //image_test.at<int>(xc, yc) = 1;
            }
        }
        // ObstacleMapT = Thicken_Map_from_Image(image_test,obs_pos);
        Get_Obstacle_Points(ObstacleMapT, obs_pos);
        ExpandObstacle_Polar(ObstacleMapT, obs_pos);
       // auto start = std::chrono::high_resolution_clock::now();
        ObstacleMapT = Thicken_Map_Manhattan(ObstacleMapT, obs_pos_thick);
        // ObstacleMapT = Thicken_Map(ObstacleMapT, obs_pos_thick);
        //ObstacleMapT = Thicken_Map(ObstacleMapT, obs_pos_thick);
       // auto end = std::chrono::high_resolution_clock::now();
       // auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
       // Print("=== OOOOO == Dilation time", elapsed.count());

        Map_mtx.lock();
        ObstacleMap.assign(ObstacleMapT.begin(), ObstacleMapT.end());
        Map_mtx.unlock();

        Pts_mtx.lock();
        Obstacle_Points_Thick.clear();
        Obstacle_Points_Thick.assign(obs_pos_thick.begin(), obs_pos_thick.end());
        Obstacle_Points.clear();
        Obstacle_Points.assign(obs_pos.begin(), obs_pos.end());
        Pts_mtx.unlock();
        //Print("Obstacle Points SIZE",Obstacle_Points.size());
    }
    return;
}

void ObstacleMapGen::ExpandObstacle_Polar(std::vector<VectorInt> &ObstacleMapT, std::vector<Position> &obs_pos)
{
    for (int i = 0; i < obs_pos.size(); i++)
    {
        double radius, angle;
        Rect_to_Polar(obs_pos[i].xval / map_img_factor, obs_pos[i].yval / map_img_factor, radius, angle);
        // Print("angle",i,j,angle);
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
                    //Print("radius", radius,x_real,y_real,i,j);
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
    //  Print("modified radius",modified_radius,angle);
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
    auto start = std::chrono::high_resolution_clock::now();

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
    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    //Print("=== OOOOO == thick time", elapsed.count());
    start = std::chrono::high_resolution_clock::now();
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
    end = std::chrono::high_resolution_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    Print("=== OOOOO == assign time", elapsed.count());
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
            //Print("img val",Image.at<int>(i,j));
            if (Image.at<int>(i, j) >= 1)
            {
                Print("element found", i, j, Image.at<int>(i, j) >= 1);
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
