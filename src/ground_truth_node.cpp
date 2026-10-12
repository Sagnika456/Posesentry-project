#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <cmath>

using std::placeholders::_1;

class GroundTruthNode : public rclcpp::Node
{
public:
  GroundTruthNode() : Node("ground_truth_node")
  {
    gt_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/ground_truth/actor_position", 10);
    error_pub_ = this->create_publisher<std_msgs::msg::Float64>("/pose_estimation/accuracy_error", 10);

    keypoints_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/pose_estimation/keypoints", 10,
      std::bind(&GroundTruthNode::keypointsCallback, this, _1));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&GroundTruthNode::publishGroundTruth, this));

    start_time_ = this->now();

    // Camera parameters (from world file)
    cam_x_ = 0.0; cam_y_ = -5.0; cam_z_ = 1.5;
    cam_yaw_ = 1.5708;
    img_width_ = 640; img_height_ = 480;
    h_fov_ = 1.047;

    RCLCPP_INFO(this->get_logger(), "Ground truth node started (with accuracy comparison).");
  }

private:
  double actor_x_, actor_y_, actor_z_ = 1.0;
  double cam_x_, cam_y_, cam_z_, cam_yaw_;
  double total_error_ = 0.0;
  int error_count_ = 0;
  int img_width_, img_height_;
  double h_fov_;

  void publishGroundTruth()
  {
    double t = (this->now() - start_time_).seconds();
    double cycle_t = std::fmod(t, 10.0);

    if (cycle_t <= 5.0) {
      double ratio = cycle_t / 5.0;
      actor_x_ = 2.0 + ratio * (-2.0 - 2.0);
      actor_y_ = 0.0;
    } else {
      double ratio = (cycle_t - 5.0) / 5.0;
      actor_x_ = -2.0 + ratio * (2.0 - (-2.0));
      actor_y_ = 0.0;
    }

    geometry_msgs::msg::Point pt;
    pt.x = actor_x_;
    pt.y = actor_y_;
    pt.z = actor_z_;
    gt_pub_->publish(pt);
  }

  // Project a 3D world point into 2D pixel coords using simple pinhole camera model
  bool projectToImage(double wx, double wy, double wz, double & px, double & py)
  {
    // Translate point relative to camera
    double dx = wx - cam_x_;
    double dy = wy - cam_y_;
    double dz = wz - cam_z_;

    // Rotate into camera frame (camera yaw = 1.5708 means camera looks along +Y)
    double cos_yaw = std::cos(-cam_yaw_);
    double sin_yaw = std::sin(-cam_yaw_);
    double cam_forward = dx * sin_yaw + dy * cos_yaw;
    double cam_right   = dx * cos_yaw - dy * sin_yaw;
    double cam_up = dz;

    if (cam_forward <= 0.01) return false;

    double focal_px = (img_width_ / 2.0) / std::tan(h_fov_ / 2.0);

    px = (img_width_ / 2.0) + (cam_right / cam_forward) * focal_px;
    py = (img_height_ / 2.0) - (cam_up / cam_forward) * focal_px;

    return true;
  }

  void keypointsCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    if (msg->poses.empty()) return;

    // Use the average of all detected keypoints as the "detected body center"
    double sum_x = 0, sum_y = 0;
    for (auto & p : msg->poses) {
      sum_x += p.position.x;
      sum_y += p.position.y;
    }
    double detected_x = sum_x / msg->poses.size();
    double detected_y = sum_y / msg->poses.size();

    // Project ground-truth actor position (use mid-body height ~1.0m)
    double gt_px, gt_py;
    if (!projectToImage(actor_x_, actor_y_, 1.0, gt_px, gt_py)) {
      return;
    }

    double error = std::sqrt(
      std::pow(detected_x - gt_px, 2) + std::pow(detected_y - gt_py, 2));

    std_msgs::msg::Float64 error_msg;
    error_msg.data = error;
    error_pub_->publish(error_msg);

    total_error_ += error;
    error_count_++;
    double avg_error = total_error_ / error_count_;

    RCLCPP_INFO(this->get_logger(),
      "Error: %.2f px | Running Average (%d frames): %.2f px",
      error, error_count_, avg_error);
  }

  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr gt_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr error_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr keypoints_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundTruthNode>());
  rclcpp::shutdown();
  return 0;
}
