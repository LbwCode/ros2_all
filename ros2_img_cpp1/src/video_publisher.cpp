#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImagePublisher : public rclcpp::Node
{
public:
    int num = 0;

    ImagePublisher(std::string name) : Node(name)
    {
        // 创建图像发布者，话题名为"image_raw"，队列长度10
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
        
        // 创建定时器，每40ms(25FPS)触发一次回调函数
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&ImagePublisher::timerCallback, this)
        );
        
        // 视频文件路径
        video_path_ = "/home/lbw/ros2_all/ros2_img_cpp1/src/4.mp4";
        
        // 打开视频文件
        cap_ = cv::VideoCapture(video_path_);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "无法打开视频文件: %s", video_path_.c_str());
        }
        
        RCLCPP_INFO(this->get_logger(), "视频发布节点已启动");
    }

private:
    
    void timerCallback()
    {
        cv::Mat frame;
        bool ret = cap_.read(frame);
        
        if (ret) {
            num += 1;
            if (num >= 5000){
                rclcpp::shutdown();
            }
            cv::resize(frame, frame, cv::Size(640, 640));

            // 将OpenCV图像转换为ROS消息
            sensor_msgs::msg::Image::SharedPtr msg = 
                cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            
            // 添加时间戳
            msg->header.stamp = this->get_clock()->now();
            
            // 发布图像消息
            publisher_->publish(*msg);
            
            RCLCPP_INFO(this->get_logger(), "发布视频帧 %d", num);
        } else {
            // 视频播放完毕，重新打开文件
            RCLCPP_WARN(this->get_logger(), "视频播放完毕，重新开始");
            cap_.release();
            cap_ = cv::VideoCapture(video_path_);
        }
        
    }
    
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
    std::string video_path_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImagePublisher>("topic_webcam_pub");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}