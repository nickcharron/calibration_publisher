#include <filesystem>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <beam_calibration/TfTree.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "calibration_publisher");

  ros::NodeHandle node_handle("~");
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;

  std::string extrinsics_file_path;
  node_handle.getParam("extrinsics_file_path", extrinsics_file_path);

  if (extrinsics_file_path.empty())
  {
    BEAM_ERROR("Invalid parameters, extrinsics_file_path cannot be empty");
    throw std::invalid_argument{" extrinsics_file_path cannot be empty"};
  }

  // check file exists
  if (!std::filesystem::exists(extrinsics_file_path))
  {
    ROS_ERROR("Extrinsics file path does not exist: %s",
              extrinsics_file_path.c_str());
    throw std::invalid_argument{"Extrinsics file path does not exist."};
  }

  beam_calibration::TfTree tf_tree;
  tf_tree.LoadJSON(extrinsics_file_path);

  auto frames = tf_tree.GetAllFrames();
  std::string child_frame;
  for (auto frame : frames)
  {
    child_frame = frame.first;
    for (auto parent_frame : frame.second)
    {
      ros::Time time_now = ros::Time::now();
      auto tf_msg =
          tf_tree.GetTransformROS(parent_frame, child_frame, time_now);
      static_broadcaster.sendTransform(tf_msg);
    }
  }

  ROS_INFO("Successfully calibration publisher launched node.");
  ros::spin();

  return 0;
}
