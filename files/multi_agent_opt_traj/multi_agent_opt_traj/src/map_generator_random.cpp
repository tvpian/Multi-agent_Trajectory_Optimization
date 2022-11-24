#include <iostream>
#include <map_generator.h>
#include <mission.hpp>

using namespace std;

class RandomMapGenerator : public MapGenerator
{
public:
    void RandomMapGenerate(const TrajPlanning::Mission &mission)
    {
        double numel_e = 0.00001;
        pcl::PointXYZ pt_random;

        random_device rd;
        default_random_engine eng(rd());
        
        rand_x = uniform_real_distribution<double>(x_min, x_max);
        rand_y = uniform_real_distribution<double>(y_min, y_max);
        rand_w = uniform_real_distribution<double>(r_min, r_max);
        rand_h = uniform_real_distribution<double>(h_min, h_max);

        int obs_iter = 0;
        while (obs_iter < obs_num)
        {
            double x, y, w, h;
            x = rand_x(eng);
            y = rand_y(eng);
            w = rand_w(eng);

            bool quadInObs = false;
            for (int qi = 0; qi < mission.qn; qi++)
            {
                if (sqrt(pow(x - mission.startState[qi][0], 2) + pow(y - mission.startState[qi][1], 2)) < mission.quad_size[qi] + w + margin ||
                    sqrt(pow(x - mission.goalState[qi][0], 2) + pow(y - mission.goalState[qi][1], 2)) < mission.quad_size[qi] + w + margin)
                {
                    quadInObs = true;
                    break;
                }
            }
            if (quadInObs)
            {
                continue;
            }
            x = floor(x / resolution) * resolution + resolution / 2.0;
            y = floor(y / resolution) * resolution + resolution / 2.0;

            int widNum = ceil(w / resolution);

            for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
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

        cloudMap.width = cloudMap.points.size();
        cloudMap.height = 1;
        cloudMap.is_dense = true;

        ROS_WARN("Finished generate random map ");

        kdtreeLocalMap.setInputCloud(cloudMap.makeShared());
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "random_map_generator");
    ros::NodeHandle n("~");
    ros::V_string args;
    ros::removeROSArgs(argc, argv, args);

    RandomMapGenerator random_map_generator;
    random_map_generator.all_map_pub = n.advertise<sensor_msgs::PointCloud2>("all_map", 1);

    n.param<double>("world/x_min", random_map_generator.x_min, -5);
    n.param<double>("world/y_min", random_map_generator.y_min, -5);
    n.param<double>("world/z_min", random_map_generator.z_min, 0);
    n.param<double>("world/x_max", random_map_generator.x_max, 5);
    n.param<double>("world/y_max", random_map_generator.y_max, 5);
    n.param<double>("world/z_max", random_map_generator.z_max, 2.5);
    n.param<double>("world/margin", random_map_generator.margin, 1.5);

    n.param<int>("world/obs_num", random_map_generator.obs_num, 6);
    n.param<double>("world/resolution", random_map_generator.resolution, 0.1);
    n.param<double>("world/r_min", random_map_generator.r_min, 0.3);
    n.param<double>("world/r_max", random_map_generator.r_max, 0.8);
    n.param<double>("world/h_min", random_map_generator.h_min, 1.0);
    n.param<double>("world/h_max", random_map_generator.h_max, 2.5);

    TrajPlanning::Mission mission;
    if (!mission.setMission(n))
    {
        return -1;
    }

    // generate map msg
    random_map_generator.RandomMapGenerate(mission);

    ros::Rate rate(10);
    int count = 0;
    while (ros::ok())
    {
        if (count < 100)
        {
            random_map_generator.pubSensedPoints();
            count++;
        }
        ros::spinOnce();
        rate.sleep();
    }
}
