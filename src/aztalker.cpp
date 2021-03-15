#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <assert.h> 
#include <k4a/k4a.hpp>
#include <k4abt.hpp>  


int main(int argc, char** argv)
{
  try
    {
        k4a_device_configuration_t device_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
        device_config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;

        k4a::device device = k4a::device::open(0);
        device.start_cameras(&device_config);

        k4a::calibration sensor_calibration = device.get_calibration(device_config.depth_mode, device_config.color_resolution);

        k4abt::tracker tracker = k4abt::tracker::create(sensor_calibration);

  ros::init(argc, argv, "aztalker");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray>("array", 100);
  ros::Rate loop_rate(20);

  while (ros::ok())
  {


    k4a::capture sensor_capture;
            if (device.get_capture(&sensor_capture, std::chrono::milliseconds(K4A_WAIT_INFINITE)))
            {


                if (!tracker.enqueue_capture(sensor_capture))
                {
                    // It should never hit timeout when K4A_WAIT_INFINITE is set.
                    std::cout << "Error! Add capture to tracker process queue timeout!" << std::endl;
                    break;
                }

                k4abt::frame body_frame = tracker.pop_result();
                if (body_frame != nullptr)
                {
                    uint32_t num_bodies = body_frame.get_num_bodies();

                    for (uint32_t i = 0; i < num_bodies; i++)
                    {
                        k4abt_body_t body = body_frame.get_body(i);
                        if(body.id==1){
                                  std_msgs::Float32MultiArray array;
                        for (int i = 0; i < (int)K4ABT_JOINT_COUNT; i++)
                           {
                              k4a_float3_t position = body.skeleton.joints[i].position;
                              k4a_quaternion_t orientation = body.skeleton.joints[i].orientation;
                              std::cout << "position(" << position.v[0] << "," << position.v[1]<<"," << position.v[2] << ")" << std::endl;


                                  array.data.resize((int)K4ABT_JOINT_COUNT*3);
                                  array.data[i*3]=position.v[0];
                                  array.data[i*3+1]=position.v[1];
                                  array.data[i*3+2]=position.v[2];
                           }
                                  ROS_INFO("publish now");
                                  pub.publish(array);
                                  ros::spinOnce();
                                  loop_rate.sleep();
                        }

                }
                }
                else
                {
                    //  It should never hit timeout when K4A_WAIT_INFINITE is set.
                    std::cout << "Error! Pop body frame result time out!" << std::endl;
                    break;
                }
            }
            else
            {
                // It should never hit time out when K4A_WAIT_INFINITE is set.
                std::cout << "Error! Get depth frame time out!" << std::endl;
                break;
            }
  }
  }
  catch (const std::exception& e)
    {
        std::cerr << "Failed with exception:" << std::endl
            << "    " << e.what() << std::endl;
        return 1;
    }

  return 0;
}
