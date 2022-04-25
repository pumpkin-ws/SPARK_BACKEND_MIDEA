// pcl 
#include "io/plc/include/omron_plc.hpp"

// ros related
#include "ros/ros.h"
#include "spark_backend/OmronPLCInfo.h"


const unsigned PLC_PORT{9600};
const std::string PLC_IP = "192.168.250.1";

int main(int argc, char** argv) {
    // initialize connection to omron plc
    // TODO: Assumed connection is successful, what to perform if failed?
    OmronPLC plc(PLC_PORT, PLC_IP);

    // initialize node and the publisher
    ros::init(argc, argv, "omron");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<spark_backend::OmronPLCInfo>("omron_io", 1, false);// latch=false indicates that the last message in queue will not be saved
    spark_backend::OmronPLCInfo omron_info;
    ros::Rate loop_rate(200);

    std::vector<bool> IO_vals;

    auto assignOmronDIInfo = [](boost::array<uint8_t, 8UL>& input, const std::vector<bool>& IO_vals) mutable {
        for (int i = 0; i < input.size(); i++) {
            if (IO_vals[i] == true) {
                input[i] = 1;
            } else {
                input[i] = 0;
            }
        }
    };    
    auto assignOmronDOInfo = [](boost::array<uint8_t, 12UL>& input, const std::vector<bool>& IO_vals) mutable {
        for (int i = 0; i < input.size(); i++) {
            if (IO_vals[i] == true) {
                input[i] = 1;
            } else {
                input[i] = 0;
            }
        }
    };
    while(ros::ok()) { // exit if rosmaster is shut
        // acquire and publish message

        // DI ch0
        IO_vals.clear();
        plc.getIOArray(0, IO_vals);
        assignOmronDIInfo(omron_info.DI_ch0, IO_vals);
        // DI ch1
        IO_vals.clear();
        plc.getIOArray(1, IO_vals);
        assignOmronDIInfo(omron_info.DI_ch1, IO_vals);
        // DI ch2
        IO_vals.clear();
        plc.getIOArray(2, IO_vals);
        assignOmronDIInfo(omron_info.DI_ch2, IO_vals);
        // DO ch1
        IO_vals.clear();
        plc.getIOArray(100, IO_vals);
        assignOmronDOInfo(omron_info.DO_ch0, IO_vals);
        // DO ch2
        IO_vals.clear();
        plc.getIOArray(101, IO_vals);
        assignOmronDOInfo(omron_info.DO_ch1, IO_vals);
        // DO ch3
        IO_vals.clear();
        plc.getIOArray(102, IO_vals);
        assignOmronDOInfo(omron_info.DO_ch2, IO_vals);

        pub.publish(omron_info);

        loop_rate.sleep();
        ros::spinOnce();
    }

    ros::shutdown();
}