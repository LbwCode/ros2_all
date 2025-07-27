#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImageSubscriber : public rclcpp::Node
{
public:
    int num = 0;

    ImageSubscriber(std::string name) : Node(name)
    {
        // 创建图像订阅者，话题名为"image_raw"，队列长度10
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_raw", 10, std::bind(&ImageSubscriber::listenerCallback, this, std::placeholders::_1));
        
        // 创建OpenCV窗口
        cv::namedWindow("object", cv::WINDOW_NORMAL);
        cv::resizeWindow("object", 640, 640);
        
        RCLCPP_INFO(this->get_logger(), "图像订阅节点已启动");
    }
    
    ~ImageSubscriber()
    {
        // 关闭OpenCV窗口
        cv::destroyWindow("object");
    }

private:
    void objectDetect(cv::Mat &image)
    {

        // 获取图像尺寸
        int height = image.rows;
        int width = image.cols;
        
        // 计算中心点坐标
        int center_x = width / 2;
        int center_y = height / 2;
        

        // 绘制中心点（红色实心圆，半径30px，线宽2px）
        cv::circle(image, cv::Point(center_x, center_y), 30, cv::Scalar(0, 0, 255), 2);
        
        // 显示图像
        cv::imshow("object", image);
        cv::waitKey(1);  // 必须调用，否则窗口不会刷新
    }
    
    void listenerCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "收到视频帧");
        
        try {
            num += 1;
            RCLCPP_INFO(this->get_logger(), "收到视频帧 %d", num);
            // 将ROS图像消息转换为OpenCV图像（BGR8格式）
            cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
            
            // 目标检测处理
            objectDetect(image);
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "图像转换失败: %s", e.what());
        }
    }
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageSubscriber>("topic_webcam_sub");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}