#include "mrl_ipcamera/mrl_ipcamera.hpp"

int main(int argc, char** argv)
{
    ros::init(argc,argv,"mrl_ipcamera");
    ros::NodeHandle nh("~");
    MrlIpCamera ipCamera(&nh);
    ipCamera.publish();
    return 0;
}
