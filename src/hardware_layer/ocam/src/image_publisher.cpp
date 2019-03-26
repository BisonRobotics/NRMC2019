#include <stdio.h>
#include <errno.h>

#include <ocam/camera.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>


int main (int argc, char* argv[])
{
    // Initialize camera
    const char* devPath = "/dev/video0";
    unsigned int  width = 1280, height = 720;
    ocam::Camera camera(devPath);
    camera.set_format(width, height, ocam::fourcc_to_pixformat('G','R','E','Y'), 1, 50);
    ocam::camera_format camFormat;
    camera.get_current_format(camFormat);

    // Print info
    std::string camName = camera.get_dev_name();
    std::string camSerialNumber = camera.get_serial_number();
    ROS_INFO("dev: %s, serial number: %s", camName.c_str(), camSerialNumber.c_str());
    printf("----------------- Current format information -----------------\n");
    camFormat.print();
    printf("---------------------------------------------------------------\n");

    // Additional controls
    int brightness = camera.get_control("Brightness");
    int exposure = camera.get_control("Exposure (Absolute)");
    camera.set_control("Brightness", brightness);
    camera.set_control("Exposure (Absolute)", exposure);

    // Start streaming
    if (!camera.start()) {
        ROS_ERROR("Failed to start.");
        exit(0);
    }

    // Initialize ROS
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);
    sensor_msgs::Image image;
    image.header.frame_id = "camera";
    image.header.seq = 0;
    image.header.stamp = ros::Time::now();
    image.height = height;
    image.width = width;
    image.encoding = "mono8";
    image.is_bigendian = 0;
    image.step = width;
    image.data.resize(width * height);

    // Main loop
    while (ros::ok())
    {
        // Copy a single frame(image) from camera(oCam-1MGN). This is a blocking function.
        int size = camera.get_frame(image.data.data(), camFormat.image_size, 1);

        // If the error occured, restart the camera.
        if (size == -1)
        {
            ROS_WARN("Error number: %d", errno);
            ROS_WARN("Cannot get image from camera");
            camera.stop();
            camera.start();
        }
        else
        {
            pub.publish(image);
        }

    }

    // Exit
    camera.stop();
    return 0;
}
