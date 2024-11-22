#include <rclcpp/rclcpp.hpp>

class CppNode : public rclcpp::Node {
public:
    CppNode() : Node("cpp_node") {
        RCLCPP_INFO(this->get_logger(), "C++ Node is running!");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CppNode>());
    rclcpp::shutdown();
    return 0;
}
