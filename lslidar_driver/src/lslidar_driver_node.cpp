#include "lslidar_driver/lslidar_driver.h"
#include "std_msgs/msg/string.h"
#include <tinyxml2.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace tinyxml2;
using namespace lslidar_ch_driver;
volatile sig_atomic_t flag = 1;

static void my_handler(int sig) {
    flag = 0;
    rclcpp::shutdown();
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    signal(SIGINT, my_handler);
    std::string package_name = "lslidar_driver";
    std::string package_path = ament_index_cpp::get_package_share_directory(package_name);
    std::string package_xml_path = package_path + "/package.xml";
    //std::cout << "Package path: " << package_path << std::endl;

    XMLDocument doc;
    doc.LoadFile(package_xml_path.c_str());

    tinyxml2::XMLElement* version_element = doc.FirstChildElement("package")->FirstChildElement("version");
    const char* version_text = version_element->GetText();


    //std::cout << "Package version: " << version_text << std::endl;

    auto node = std::make_shared<lslidar_ch_driver::LslidarChDriver>();
    if(version_text){
        RCLCPP_INFO(node->get_logger(),"CHX1 ROS2 VERSION: %s",version_text);
    }else{
        RCLCPP_WARN(node->get_logger(),"Error,Failed to get CHX1 ROS2 VERSION");
    }

    if (!node->initialize()) {
        RCLCPP_ERROR(node->get_logger(), "cannot initialize lslidar driver.");
        return 0;
    }
    while (rclcpp::ok()) {
        node->polling();

    }
    rclcpp::shutdown();
    return 0;
}
