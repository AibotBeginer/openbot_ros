/*
 * Copyright 2024 The OpenRobotic Beginner Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "openbot_ros/map_generator.hpp"


namespace openbot_ros {

RandomMapGenerator::RandomMapGenerator()
{
    InitDefaultConfig();
}

RandomMapGenerator::RandomMapGenerator(const MapOption& config)
{
    *default_config_ = config;
}

sensor_msgs::msg::PointCloud2& RandomMapGenerator::Generate()
{
    std::random_device rd;
    std::default_random_engine eng(rd());

    std::uniform_real_distribution<double> rand_x = std::uniform_real_distribution<double>(default_config_->x_l, default_config_->x_h);
    std::uniform_real_distribution<double> rand_y = std::uniform_real_distribution<double>(default_config_->y_l, default_config_->y_h );
    std::uniform_real_distribution<double> rand_w = std::uniform_real_distribution<double>(default_config_->w_l, default_config_->w_h);
    std::uniform_real_distribution<double> rand_h = std::uniform_real_distribution<double>(default_config_->h_l, default_config_->h_h);

    std::uniform_real_distribution<double> rand_x_circle = std::uniform_real_distribution<double>(default_config_->x_l + 1.0, default_config_->x_h - 1.0);
    std::uniform_real_distribution<double> rand_y_circle = std::uniform_real_distribution<double>(default_config_->y_l + 1.0, default_config_->y_h - 1.0);
    std::uniform_real_distribution<double> rand_r_circle = std::uniform_real_distribution<double>(default_config_->w_c_l    , default_config_->w_c_h    );

    std::uniform_real_distribution<double> rand_roll      = std::uniform_real_distribution<double>(- M_PI,     + M_PI);
    std::uniform_real_distribution<double> rand_pitch     = std::uniform_real_distribution<double>(+ M_PI/4.0, + M_PI/2.0);
    std::uniform_real_distribution<double> rand_yaw       = std::uniform_real_distribution<double>(+ M_PI/4.0, + M_PI/2.0);
    std::uniform_real_distribution<double> rand_ellipse_c = std::uniform_real_distribution<double>(0.5, 2.0);
    std::uniform_real_distribution<double> rand_num       = std::uniform_real_distribution<double>(0.0, 1.0);

    pcl::PointXYZ pt_random;

    // firstly, we put some circles
    for(int i = 0; i < default_config_->cir_num; i ++)
    {
        double x0, y0, z0, R;
        std::vector<Eigen::Vector3d> circle_set;

        x0   = rand_x_circle(eng);
        y0   = rand_y_circle(eng);
        z0   = rand_h(eng) / 2.0;  
        R    = rand_r_circle(eng);

        if(std::sqrt(std::pow(x0 - default_config_->init_x, 2) + std::pow(y0 -default_config_->init_y, 2) ) < 2.0 ) 
            continue;

        double a, b;
        a = rand_ellipse_c(eng);
        b = rand_ellipse_c(eng);

        double x, y, z;
        Eigen::Vector3d pt3, pt3_rot;
        for(double theta = -M_PI; theta < M_PI; theta += 0.025)
        {  
            x = a * cos(theta) * R;
            y = b * sin(theta) * R;
            z = 0;
            pt3 << x, y, z;
            circle_set.push_back(pt3);
        }
        // Define a random 3d rotation matrix
        Eigen::Matrix3d Rot;
        double roll,  pitch, yaw;
        double alpha, beta,  gama;
        roll  = rand_roll(eng); // alpha
        pitch = rand_pitch(eng); // beta
        yaw   = rand_yaw(eng); // gama

        alpha = roll;
        beta  = pitch;
        gama  = yaw;

        double p = rand_num(eng);
        if(p < 0.5)
        {
            beta = M_PI / 2.0;
            gama = M_PI / 2.0;
        }

        Rot << cos(alpha) * cos(gama)  - cos(beta) * sin(alpha) * sin(gama), - cos(beta) * cos(gama) * sin(alpha) - cos(alpha) * sin(gama),   sin(alpha) * sin(beta),
                cos(gama)  * sin(alpha) + cos(alpha) * cos(beta) * sin(gama),   cos(alpha) * cos(beta) * cos(gama) - sin(alpha) * sin(gama), - cos(alpha) * sin(beta),        
                sin(beta)  * sin(gama),                                         cos(gama) * sin(beta),                                         cos(beta);

        for(auto pt : circle_set)
        {
            pt3_rot = Rot * pt;
            pt_random.x = pt3_rot(0) + x0 + 0.001;
            pt_random.y = pt3_rot(1) + y0 + 0.001;
            pt_random.z = pt3_rot(2) + z0 + 0.001;

            if(pt_random.z >= 0.0)
            cloudMap.points.push_back( pt_random );
        }
    }

    bool is_kdtree_empty = false;
    if(cloudMap.points.size() > 0)
        kdtreeMap.setInputCloud(cloudMap.makeShared()); 
    else
        is_kdtree_empty = true;

    // then, we put some pilar
    for(int i = 0; i < default_config_->obs_num; i ++)
    {
        double x, y, w, h; 
        x    = rand_x(eng);
        y    = rand_y(eng);
        w    = rand_w(eng);

        //if(sqrt( pow(x - _init_x, 2) + pow(y - _init_y, 2) ) < 2.0 ) 
        if(sqrt( pow(x - default_config_->init_x, 2) + pow(y - default_config_->init_y, 2) ) < 0.8 ) 
            continue;
        
        pcl::PointXYZ searchPoint(x, y, (default_config_->h_l + default_config_->h_h)/2.0);
        pointIdxSearch.clear();
        pointSquaredDistance.clear();
        
        if(is_kdtree_empty == false)
        {
            if ( kdtreeMap.nearestKSearch (searchPoint, 1, pointIdxSearch, pointSquaredDistance) > 0 )
            {
            if(std::sqrt(pointSquaredDistance[0]) < 1.0 )
                continue;
            }
        }

        x = std::floor(x/default_config_->resolution) * default_config_->resolution + default_config_->resolution / 2.0;
        y = std::floor(y/default_config_->resolution) * default_config_->resolution + default_config_->resolution / 2.0;

        int widNum = std::ceil(w / default_config_->resolution);
        for(int r = -widNum/2.0; r < widNum / 2.0; r++ )
        {
            for(int s = -widNum/2.0; s < widNum/2.0; s ++ )
            {
            h    = rand_h(eng);  
            int heiNum = 2.0 * std::ceil(h/default_config_->resolution);
            for(int t = 0; t < heiNum; t ++ ){
                pt_random.x = x + (r+0.0) * default_config_->resolution + 0.001;
                pt_random.y = y + (s+0.0) * default_config_->resolution + 0.001;
                pt_random.z =     (t+0.0) * default_config_->resolution * 0.5 + 0.001;
                cloudMap.points.push_back( pt_random );
            }
            }
        }
    }

    cloudMap.width = cloudMap.points.size();
    cloudMap.height = 1;
    cloudMap.is_dense = true;

    _has_map = true;

    pcl::toROSMsg(cloudMap, globalMap_pcd);
    globalMap_pcd.header.frame_id = "map";
    return globalMap_pcd;
}

bool RandomMapGenerator::GetPointCloud2Data(sensor_msgs::msg::PointCloud2& data)
{
    if ( !_has_map ) {
        return false;
    } 
    data = globalMap_pcd;
    return true;
}

void RandomMapGenerator::InitDefaultConfig()
{
    if (default_config_ == nullptr) {
        default_config_ = new MapOption();
    }

    default_config_->init_x = 0.0;
    default_config_->init_y = 0.0;

    default_config_->x_size = 15.0;
    default_config_->y_size = 15.0;
    default_config_->z_size = 2.0;

    default_config_->obs_num = 300;
    default_config_->cir_num = 40;
    default_config_->resolution = 0.1;

    // ObstacleShape
    default_config_->w_l = 0.1;  // lower_rad
    default_config_->w_h = 0.7;  // upper_rad
    default_config_->h_l = 0.1;  // lower_hei
    default_config_->h_h = 3.0;  // upper_hei

    // CircleShape
    default_config_->w_c_l = 0.2;  // lower_circle_rad
    default_config_->w_c_h = 2.0;  // upper_circle_rad

    default_config_->x_l = - default_config_->x_size / 2.0;
    default_config_->x_h = + default_config_->x_size / 2.0;

    default_config_->y_l = - default_config_->y_size / 2.0;
    default_config_->y_h = + default_config_->y_size / 2.0;
}

}  // namespace openbot_ros