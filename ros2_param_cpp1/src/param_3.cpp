#include <rclcpp/rclcpp.hpp>
/*
客户端
*/
using namespace std::chrono_literals;

class paramClient : public rclcpp::Node {
public:
    paramClient() : Node("my_param_client_cpp") {  // 节点名无空格
        RCLCPP_INFO(get_logger(), "参数客户端: 开始");

        // 创建客户端
        param_client_ = std::make_shared<rclcpp::SyncParametersClient>(this, "my_param_Server_cpp");
        }

        //3-2. 连接服务端；
        bool connect_server(){
            std::cout << "在连接" << std::endl;
            // 连接服务器 超时时间设置1秒
            while (!param_client_->wait_for_service(1s))
            {   
                std::cout << "已连接" << std::endl;
                // 结束
                if (!rclcpp::ok())
                {   
                    RCLCPP_INFO(this->get_logger(),"连接结束");
                    return false;
                }
                RCLCPP_INFO(this->get_logger(),"服务连接中....");
            }
            std::cout << "连接循环结束" << std::endl;
            return true;
        }
        // 查
        void get_param(){
            RCLCPP_INFO(this->get_logger(), "---------------------查---------------------");
            // 获取某个参数
            std::string car_name = param_client_->get_parameter<std::string>("car_name");
            double width = param_client_->get_parameter<double>("width");
            RCLCPP_INFO(this->get_logger(), "car_name = %s", car_name.c_str());
            RCLCPP_INFO(this->get_logger(), "width = %.2f", width);

            // 查询多个参数
            auto params = param_client_->get_parameters({"car_name", "width", "wheels"});
            for(auto &&param : params){
                RCLCPP_INFO(this->get_logger(), "%s = %s", param.get_name().c_str(),param.value_to_string().c_str());
            }
            // 查询是否包含某个参数
            auto car_ = param_client_->has_parameter("car_name");
            std::cout << "car_name 的查询结果:" << car_ << std::endl;
        }
        // 改 
        void update_param(){
            RCLCPP_INFO(this->get_logger(), "---------------------改---------------------");
            param_client_->set_parameters({
                rclcpp::Parameter("car_name", "pig"),
                rclcpp::Parameter("width", 3.0),
                rclcpp::Parameter("length", 5.0)
            });
            
        }

private:
    rclcpp::SyncParametersClient::SharedPtr param_client_;
        
};


int main(int argc, char *argv[]) {



    //2. 初始化 ROS2 客户端；
    rclcpp::init (argc,argv);
    auto client = std::make_shared<paramClient>();
    bool flag = client->connect_server();

    std::cout << "连接?" << std::endl;
    std::cout << flag << std::endl;
    
    if(!flag){
        std::cout << "未能连接" << std::endl;
        return 0;
    }
    client->get_param();
    client->update_param();
    client->get_param();
    

    rclcpp::shutdown();
    return 0;
}