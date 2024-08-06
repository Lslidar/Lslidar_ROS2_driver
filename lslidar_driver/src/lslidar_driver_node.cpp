#include "lslidar_driver/lslidar_driver.h"
#include "std_msgs/msg/string.h"

using namespace lslidar_driver;
volatile sig_atomic_t flag = 1;

static void my_handler([[maybe_unused]] int sig) {
    LS_ERROR << "Signal received, ending process." << LS_END;
    flag = 0;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    signal(SIGINT, my_handler);
    auto node = std::make_shared<lslidar_driver::LslidarDriver>();
    if (!node->initialize()) {
        LS_ERROR << "cannot initialize lslidar driver." << LS_END;
        return 0;
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
