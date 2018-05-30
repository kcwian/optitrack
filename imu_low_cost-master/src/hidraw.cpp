/* ROS */

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

/* Linux */
#include <linux/types.h>
#include <linux/input.h>
#include <linux/hidraw.h>

/*
* Ugly hack to work around failing compilation on systems that don't
* yet populate new version of hidraw.h to userspace.
*/
#ifndef HIDIOCSFEATURE
#warning Please have your distro update the userspace kernel headers
#define HIDIOCSFEATURE(len)    _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x06, len)
#define HIDIOCGFEATURE(len)    _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x07, len)
#endif

/* Unix */
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

/* C */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>

#include <vector>

const char *bus_str(int bus);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lowCostImu");
    ros::NodeHandle imu;
    
    ros::Publisher imuPub = imu.advertise<sensor_msgs::Imu>("/low_cost/imu", 1000);
    ros::Rate loop_rate(100);
    
    int fd;
    int i, res, desc_size = 0;
    char buf[256];
    struct hidraw_report_descriptor rpt_desc;
    struct hidraw_devinfo info;
    char *device = "/dev/hidraw1";
    
    std::vector<float> data; 

        if (argc > 1)
                device = argv[1];

        fd = open(device, O_RDWR|O_NONBLOCK);

        if (fd < 0) {
                perror("Unable to open device");
                return 1;
        }
        
        while(ros::ok()) {
            
        memset(&rpt_desc, 0x0, sizeof(rpt_desc));
        memset(&info, 0x0, sizeof(info));
        memset(buf, 0x0, sizeof(buf));

        /* Set Feature */
        buf[0] = 0x9; /* Report Number */
        buf[1] = 0xff;
        buf[2] = 0xff;
        buf[3] = 0xff;
        res = ioctl(fd, HIDIOCSFEATURE(4), buf);

        /* Send a Report to the Device */
        buf[0] = 0x00; /* Report Number */
        for(i=1; i<29; i++){
                if(i<9)
                        buf[i] = 0xCA;
                else if(i>=9 && i<13)
                        buf[i] = 0x00;
                else if(i>=13 && i<16)
                        buf[i] = 0;
                else if(i==16)
                        buf[i]=0x7F;
                else
                        buf[i]=0xFF;
        }
        res = write(fd, buf, 29);
        if (res < 0) {
                printf("Error: %d\n", errno);
                perror("write");
        } else {
                printf("write() wrote %d bytes\n", res);
        }

        /* Get a report from the device */
        res = read(fd, buf, 256);
        if (res < 0) {
                perror("read");
        } else {

                for (i = 0; i < res; i+=4){
                        data.push_back((*(float*)&buf[i]));

                }
                
                sensor_msgs::Imu msg;
                
                msg.header.frame_id = "imu_low_cost";
                msg.header.stamp = ros::Time::now();
                
                msg.orientation.w = data[2];
                msg.orientation.x = data[3];
                msg.orientation.y = data[4];
                msg.orientation.z = data[5];
 
                msg.angular_velocity.x = data[6];
                msg.angular_velocity.y = data[7];
                msg.angular_velocity.z = data[8];
                
                msg.linear_acceleration.x = data[9]*9.80665;
                msg.linear_acceleration.y = data[10]*9.80665;
                msg.linear_acceleration.z = data[11]*9.80665;
                
                data.clear();
                //puts("\n");
                
                ROS_INFO("LECIMY");
                imuPub.publish(msg);
                
                ros::spinOnce();
                
                loop_rate.sleep();
        }
        
        
    }
    close(fd);      
    return 0;
}
