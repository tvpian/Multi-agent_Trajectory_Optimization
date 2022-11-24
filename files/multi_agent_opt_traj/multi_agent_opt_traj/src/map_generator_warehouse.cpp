#include <iostream>
#include <map_generator.h>

class WareHouseMapGenerator : public MapGenerator
{
public:
    void RandomMapGenerate(const TrajPlanning::Mission &mission)
    {
        random_device rd;
        default_random_engine eng(rd());
        double numel_e = 0.00001;
        pcl::PointXYZ pt_random;

        rand_x = uniform_real_distribution<double>(x_min, x_max);
        rand_y = uniform_real_distribution<double>(y_min, y_max);
        rand_w = uniform_real_distribution<double>(r_min, r_max);
        rand_h = uniform_real_distribution<double>(h_min, h_max);

        int obs_iter = 0;
        while (obs_iter < 12)
        {
            double x, y, w, h;
            if (obs_iter % 2 == 0)
                x = -3;
            else
                x = 3;
            if (obs_iter % 3 == 0)
                y = 0;
            else if (obs_iter % 3 == 1)
                y = 2;
            else
                y = -2;

            x = floor(x / resolution) * resolution + resolution / 2.0;
            y = floor(y / resolution) * resolution + resolution / 2.0;

            int widNum = ceil(0.6 / resolution);
            int longNum = ceil(3.0 / resolution);

            for (int r = -longNum / 2.0; r < longNum / 2.0; r++)
            {
                for (int s = -widNum / 2.0; s < widNum / 2.0; s++)
                {
                    h = rand_h(eng);
                    int heiNum = ceil(h / resolution);
                    for (int t = 0; t < heiNum; t++)
                    {
                        pt_random.x = x + (r + 0.5) * resolution + numel_e;
                        pt_random.y = y + (s + 0.5) * resolution + numel_e;
                        pt_random.z = (t + 0.5) * resolution + numel_e;
                        cloudMap.points.push_back(pt_random);
                    }
                }
            }

            obs_iter++;
        }

        obs_iter = 0;
        while (obs_iter < 12)
        {
            double x, y, w, hh, h;
            if (obs_iter == 0)
            {
                x = -6;
                y = 0;
                w = 10;
                hh = 0.05;
            }
            else if (obs_iter == 1)
            {
                x = 6;
                y = 0;
                w = 10;
                hh = 0.05;
            }
            else if (obs_iter == 2)
            {
                x = -3.5;
                y = 5;
                w = 0.05;
                hh = 5;
            }
            else if (obs_iter == 3)
            {
                x = 3.5;
                y = 5;
                w = 0.05;
                hh = 5;
            }
            else if (obs_iter == 4)
            {
                x = -3.5;
                y = -5;
                w = 0.05;
                hh = 5;
            }
            else if (obs_iter == 5)
            {
                x = 3.5;
                y = -5;
                w = 0.05;
                hh = 5;
            }

            x = floor(x / resolution) * resolution + resolution / 2.0;
            y = floor(y / resolution) * resolution + resolution / 2.0;

            int widNum = ceil(w / resolution);
            int longNum = ceil(hh / resolution);

            for (int r = -longNum / 2.0; r < longNum / 2.0; r++)
            {
                for (int s = -widNum / 2.0; s < widNum / 2.0; s++)
                {
                    h = rand_h(eng);
                    int heiNum = ceil(0.2 / resolution);
                    for (int t = 0; t < heiNum; t++)
                    {
                        pt_random.x = x + (r + 0.5) * resolution + numel_e;
                        pt_random.y = y + (s + 0.5) * resolution + numel_e;
                        pt_random.z = (t + 0.5) * resolution + numel_e;
                        cloudMap.points.push_back(pt_random);
                    }
                }
            }

            obs_iter++;
        }

        cloudMap.width = cloudMap.points.size();
        cloudMap.height = 1;
        cloudMap.is_dense = true;

        ROS_WARN("Finished generate random map ");

        kdtreeLocalMap.setInputCloud(cloudMap.makeShared());
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "warehouse_map_generator");
    ros::NodeHandle n("~");
    ros::V_string args;
    ros::removeROSArgs(argc, argv, args);

    WareHouseMapGenerator map_generator;
    map_generator.all_map_pub = n.advertise<sensor_msgs::PointCloud2>("/warehouse_map_generator/all_map", 1);

    n.param<double>("world/x_min", map_generator.x_min, -5);
    n.param<double>("world/y_min", map_generator.y_min, -5);
    n.param<double>("world/z_min", map_generator.z_min, 0);
    n.param<double>("world/x_max", map_generator.x_max, 5);
    n.param<double>("world/y_max", map_generator.y_max, 5);
    n.param<double>("world/z_max", map_generator.z_max, 2.5);
    n.param<double>("world/margin", map_generator.margin, 1.5);

    n.param<int>("world/obs_num", map_generator.obs_num, 6);
    n.param<double>("world/resolution", map_generator.resolution, 0.1);
    n.param<double>("world/r_min", map_generator.r_min, 0.3);
    n.param<double>("world/r_max", map_generator.r_max, 0.8);
    n.param<double>("world/h_min", map_generator.h_min, 1.0);
    n.param<double>("world/h_max", map_generator.h_max, 2.5);

    TrajPlanning::Mission mission;
    if (!mission.setMission(n))
    {
        return -1;
    }

    // generate map msg
    map_generator.RandomMapGenerate(mission);

    ros::Rate rate(10);
    int count = 0;
    while (ros::ok())
    {
        if (count < 100)
        {
            map_generator.pubSensedPoints();
            count++;
        }
        ros::spinOnce();
        rate.sleep();
    }
}
