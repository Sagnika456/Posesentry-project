#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cmath>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <vector>
#include <string>

using std::placeholders::_1;

class PoseEstimationNode : public rclcpp::Node
{
public:
  PoseEstimationNode() : Node("pose_estimation_node")
  {
    // ---- Load the pretrained MPI model ----
    std::string package_share = ament_index_cpp::get_package_share_directory("human_pose_estimation");
    std::string proto_path = package_share + "/models/pose_deploy.prototxt";
    std::string weights_path = package_share + "/models/pose_weights.caffemodel";

    RCLCPP_INFO(this->get_logger(), "Loading pose model from: %s", weights_path.c_str());

    net_ = cv::dnn::readNetFromCaffe(proto_path, weights_path);
    if (net_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load pose model!");
    } else {
      RCLCPP_INFO(this->get_logger(), "Pose model loaded successfully.");
    }

    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

    // ---- Subscriber: raw camera image ----
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera", 10,
      std::bind(&PoseEstimationNode::imageCallback, this, _1));

    // ---- Publisher: annotated image with skeleton drawn ----
    annotated_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/pose_estimation/annotated_image", 10);

    // ---- Publisher: keypoints as PoseArray ----
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
      "/pose_estimation/keypoints", 10);

    posture_pub_ = this->create_publisher<std_msgs::msg::String>("/human/posture_status", 10);
    fall_pub_ = this->create_publisher<std_msgs::msg::Bool>("/human/fall_detected", 10);

    RCLCPP_INFO(this->get_logger(), "Human Pose Estimation Node started. Waiting for images on /camera...");
  }

private:
  // MPI model: 15 keypoints, 14 limb pairs
  const int n_points_ = 15;
  const std::vector<std::pair<int,int>> pose_pairs_ = {
    {0,1}, {1,2}, {2,3}, {3,4}, {1,5}, {5,6}, {6,7},
    {1,14}, {14,8}, {8,9}, {9,10}, {14,11}, {11,12}, {12,13}
  };

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat frame = cv_ptr->image;
    if (frame.empty()) {
      return;
    }

    int frame_width = frame.cols;
    int frame_height = frame.rows;

    // ---- Prepare input blob for the network ----
    int in_width = 368;
    int in_height = 368;
    cv::Mat input_blob = cv::dnn::blobFromImage(
      frame, 1.0 / 255.0, cv::Size(in_width, in_height),
      cv::Scalar(0, 0, 0), false, false);

    net_.setInput(input_blob);
    cv::Mat output = net_.forward();

    int H = output.size[2];
    int W = output.size[3];

    std::vector<cv::Point> points(n_points_);
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header = msg->header;

    for (int n = 0; n < n_points_; n++) {
      cv::Mat prob_map(H, W, CV_32F, output.ptr(0, n));

      cv::Point max_loc;
      double prob;
      cv::minMaxLoc(prob_map, 0, &prob, 0, &max_loc);

      int x = static_cast<int>((frame_width * max_loc.x) / static_cast<double>(W));
      int y = static_cast<int>((frame_height * max_loc.y) / static_cast<double>(H));

      if (prob > 0.1) {
        points[n] = cv::Point(x, y);
        cv::circle(frame, points[n], 6, cv::Scalar(0, 255, 255), -1);

        geometry_msgs::msg::Pose p;
        p.position.x = x;
        p.position.y = y;
        p.position.z = 0.0;
        pose_array.poses.push_back(p);
      } else {
        points[n] = cv::Point(-1, -1);
      }
    }

    // ---- Draw skeleton lines connecting valid joint pairs ----
    for (auto & pair : pose_pairs_) {
      cv::Point p1 = points[pair.first];
      cv::Point p2 = points[pair.second];
      if (p1.x <= 0 || p1.y <= 0 || p2.x <= 0 || p2.y <= 0) {
        continue;
      }
      cv::line(frame, p1, p2, cv::Scalar(0, 255, 0), 2);
    }

    // ---- Posture classification & fall detection ----
    {
      cv::Point neck = points[1];
      cv::Point rhip = points[8];
      cv::Point lhip = points[11];

      std::string posture = "UNKNOWN";
      bool fall_detected = false;

      if (neck.x > 0 && neck.y > 0 && (rhip.x > 0 || lhip.x > 0)) {
        cv::Point hip;
        if (rhip.x > 0 && lhip.x > 0) {
          hip = cv::Point((rhip.x + lhip.x) / 2, (rhip.y + lhip.y) / 2);
        } else if (rhip.x > 0) {
          hip = rhip;
        } else {
          hip = lhip;
        }

        double dx = hip.x - neck.x;
        double dy = hip.y - neck.y;
        double angle_from_vertical = std::atan2(std::abs(dx), std::abs(dy)) * 180.0 / CV_PI;

        if (angle_from_vertical < 30.0) {
          posture = "STANDING";
        } else if (angle_from_vertical < 60.0) {
          posture = "BENDING";
        } else {
          posture = "FALLEN";
          fall_detected = true;
        }
      }

      std_msgs::msg::String posture_msg;
      posture_msg.data = posture;
      posture_pub_->publish(posture_msg);

      std_msgs::msg::Bool fall_msg;
      fall_msg.data = fall_detected;
      fall_pub_->publish(fall_msg);

      if (fall_detected) {
        RCLCPP_WARN(this->get_logger(), "FALL DETECTED!");
      }
    }

    // ---- Publish annotated image ----
    sensor_msgs::msg::Image::SharedPtr annotated_msg =
      cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
    annotated_pub_->publish(*annotated_msg);

    // ---- Publish keypoints ----
    pose_pub_->publish(pose_array);
  }

  cv::dnn::Net net_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr posture_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fall_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseEstimationNode>());
  rclcpp::shutdown();
  return 0;
}

