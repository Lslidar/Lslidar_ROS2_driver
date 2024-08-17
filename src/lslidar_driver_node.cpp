#include "lslidar_ls_driver/lslidar_driver.h"

using namespace lslidar_driver;
volatile sig_atomic_t flag = 1;

static void my_handler([[maybe_unused]] int sig) {
    flag = 0;
    exit(0);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    signal(SIGINT, my_handler);
    auto node = std::make_shared<lslidar_driver::lslidarDriver>();
    if (!node->initialize()) {
        RCLCPP_ERROR(node->get_logger(), "cannot initialize lslidar driver.");
        return 0;
    }

    rclcpp::spin(node);
    // rclcpp::spin_some(node);
    rclcpp::shutdown();
    return 0;
}
