#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/NavSatFix.h>
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>
#include "rosbag_to_2000/wgs84_do_puwg92.h"

int main()
{
    const std::string fn {"/media/michal/ext/orto_photo_skierniewice/hd_skierniewice"};

    using namespace boost::filesystem;
    recursive_directory_iterator dir_it ( fn );

    std::vector<std::string> patches;
    while (dir_it!=recursive_directory_iterator{})
    {
        if (dir_it->path().extension() == ".bag"){
            patches.push_back(dir_it->path().string());
        }
        dir_it++;
    }

    std::ofstream fst("/tmp/pc.txt");
    for (const auto &p : patches){
        std::cout << "processing " << p << std::endl;
        rosbag::Bag bag;
        bag.open(p);  // BagMode is Read by default
        for(rosbag::MessageInstance const m: rosbag::View(bag)) {
            sensor_msgs::NavSatFix::ConstPtr i = m.instantiate<sensor_msgs::NavSatFix>();
            if (i != nullptr) {
//                std::cout << std::fixed << i->latitude << "," << i->longitude << std::endl;

                double L = i->longitude;
                double B = i->latitude;

                std::array<double, 2> xy;
                wgs84_do_puwg92(B, L, xy.data(), xy.data() + 1);
                fst <<std::fixed<< xy[1] << "\t" << xy[0] << "\t" << i->altitude << std::endl;
            }
        }

        bag.close();
    }
    




}