#include <rclcpp/rclcpp.hpp>
/*
服务端
*/

class paramServer : public rclcpp::Node {
public:
    // rclcpp::NodeOptions().allow_undeclared_parameters(true) 让删除操作允许
    paramServer() : Node("my_param_Server_cpp", rclcpp::NodeOptions().allow_undeclared_parameters(true)) {  // 节点名无空格
        RCLCPP_INFO(get_logger(), "参数服务端: 开始");
        // 静态类型参数 不可删除
        // this->declare_parameter("height","qwer_df");
        // 动态类型参数 可删除
        this->set_parameter(rclcpp::Parameter("height", "qwer_df"));
    }

    // 增
    void declare_param(){
        RCLCPP_INFO(this->get_logger(), "---------------------增---------------------");
        this->declare_parameter("car_name","tiger");
        this->declare_parameter("width",1.55);
        this->declare_parameter("wheels",5);
        
        // 这个需要 "让删除操作允许" rclcpp::NodeOptions().allow_undeclared_parameters(true) 才能用这个
        this->set_parameter(rclcpp::Parameter("h1", 2.00));

    };
    // 查
    void get_param(){
        RCLCPP_INFO(this->get_logger(), "---------------------查---------------------");
        // 获取指定参数
        auto car = this->get_parameter("car_name");
        RCLCPP_INFO(this->get_logger(),"key = %s, value = %s", car.get_name().c_str(),car.as_string().c_str());
        // 获取一些参数
        auto params = this->get_parameters ({"car_name","width","height"});
        for (auto &&param : params){
            RCLCPP_INFO(this->get_logger(), "(%s = %s)", param.get_name().c_str(), param.value_to_string().c_str());
        }
        
        // 判断是否包含
        RCLCPP_INFO(this->get_logger(), "是否包含 car_namer? %d", this->has_parameter("car_name"));
        RCLCPP_INFO(this->get_logger(), "是否包含 ce1 ? %d", this->has_parameter("ce1"));
        RCLCPP_INFO(this->get_logger(), "是否包含 h1 ? %d", this->has_parameter("h1"));
    };
    // 改
    void update_param(){
        RCLCPP_INFO(this->get_logger(), "---------------------改---------------------");
        this->set_parameter(rclcpp::Parameter("width", 1.15));
        RCLCPP_INFO(this->get_logger(), "width = %.2f", this->get_parameter("width").as_double());


    };
    // 删
    void del_param(){
        RCLCPP_INFO(this->get_logger(), "---------------------删---------------------");
        //this->undeclare_parameter("car_name");// 不能删除声明的参数
        RCLCPP_INFO(this->get_logger()," 删除前还包含 height 吗 %d",this->has_parameter("height"));
        // 删除操作需要判断一下，若不存在，还删除，会报错
        if (this->has_parameter("height")){
            this->undeclare_parameter("height");
        }
        RCLCPP_INFO(this->get_logger()," 删除后还包含 height 吗 %d",this->has_parameter("height"));
    };
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    // 调用spain函数， 传入对象指针
    auto node = std::make_shared<paramServer>();
    // 增
    node->declare_param();
    // 查
    node->get_param();
    // 改
    node->update_param();
    // 删
    node->del_param();
    rclcpp::spin(node);
    // rclcpp::spin(std::make_shared<MyParam>());

    rclcpp::shutdown();
    return 0;
}