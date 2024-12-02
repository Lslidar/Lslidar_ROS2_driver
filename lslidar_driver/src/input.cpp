#include "lslidar_driver/input.h"
#include <cmath>

extern volatile sig_atomic_t flag;
namespace lslidar_driver {
    static const size_t packet_size = sizeof(lslidar_msgs::msg::LslidarPacket().data);
////////////////////////////////////////////////////////////////////////
// Input base class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
 *
 *  @param private_nh ROS private handle for calling node.
 *  @param port UDP port number.
 */
    Input::Input(rclcpp::Node *private_nh, uint16_t port) : private_nh_(private_nh), port_(port) {
        npkt_update_flag_ = false;
        cur_rpm_ = 0;
        return_mode_ = 1;
//        private_nh->declare_parameter<std::string>("device_ip", "");
//        private_nh->declare_parameter<bool>("add_multicast", false);
//        private_nh->declare_parameter<std::string>("group_ip", "224.1.1.2");

        private_nh->get_parameter("device_ip", devip_str_);
        private_nh->get_parameter("add_multicast", add_multicast);
        private_nh->get_parameter("group_ip", group_ip);

        if (!devip_str_.empty())
            RCLCPP_INFO(private_nh->get_logger(), "[driver][input] accepting packets from IP address: %s  port: %d",
                        devip_str_.c_str(),port);
    }


////////////////////////////////////////////////////////////////////////
// InputSocket class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
   *
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number
*/
    InputSocket::InputSocket(rclcpp::Node *private_nh, uint16_t port) : Input(private_nh, port) {
        sockfd_ = -1;

        if (!devip_str_.empty()) {
            inet_aton(devip_str_.c_str(), &devip_);
        }

        RCLCPP_INFO(private_nh_->get_logger(), "[driver][socket] Opening UDP socket: port %d", port);
        sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
        if (sockfd_ == -1) {
            perror("socket");  // TODO: ROS_ERROR errno
            return;
        }

        int opt = 1;
        if (setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, (const void *) &opt, sizeof(opt))) {
            perror("setsockopt error!\n");
            return;
        }

        sockaddr_in my_addr{};                   // my address information
        memset(&my_addr, 0, sizeof(my_addr));  // initialize to zeros
        my_addr.sin_family = AF_INET;          // host byte order
        my_addr.sin_port = htons(port);        // port in network byte order
        my_addr.sin_addr.s_addr = INADDR_ANY;  // automatically fill in my IP

        if (bind(sockfd_, (sockaddr * ) & my_addr, sizeof(sockaddr)) == -1) {
            perror("bind");  // TODO: ROS_ERROR errno
            return;
        }

        if (add_multicast) {
            struct ip_mreq group{};
            group.imr_multiaddr.s_addr = inet_addr(group_ip.c_str());
            group.imr_interface.s_addr = htonl(INADDR_ANY);
            //group.imr_interface.s_addr = inet_addr("192.168.1.102");

            if (setsockopt(sockfd_, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *) &group, sizeof(group)) < 0) {
                perror("Adding multicast group error ");
                close(sockfd_);
                exit(1);
            } else
                printf("Adding multicast group...OK.\n");
        }
        if (fcntl(sockfd_, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
            perror("non-block");
            return;
        }
    }

/** @brief destructor */
    InputSocket::~InputSocket(void) {
        (void) close(sockfd_);
    }

/** @brief Get one lslidar packet. */
    int InputSocket::getPacket(lslidar_msgs::msg::LslidarPacket::UniquePtr &pkt) {
        struct pollfd fds[1];
        fds[0].fd = sockfd_;
        fds[0].events = POLLIN;
        static const int POLL_TIMEOUT = 3000;  // one second (in msec)

        sockaddr_in sender_address{};
        socklen_t sender_address_len = sizeof(sender_address);
        while (flag == 1)
            // while (true)
        {
            // Receive packets that should now be available from the
            // socket using a blocking read.
            // poll() until input available
            do {
                int retval = poll(fds, 1, POLL_TIMEOUT);
                if (retval < 0)  // poll() error?
                {
                    if (errno != EINTR)
                        RCLCPP_ERROR(private_nh_->get_logger(), "[driver][socket] poll() error: %s", strerror(errno));
                    return 1;
                }
                if (retval == 0)  // poll() timeout?
                {
                    time_t curTime = time(NULL);
                    struct tm *curTm = localtime(&curTime);
                    char bufTime[40] = {0};
                    sprintf(bufTime, "%d-%d-%d %d:%d:%d", curTm->tm_year + 1900, curTm->tm_mon + 1,
                            curTm->tm_mday, curTm->tm_hour, curTm->tm_min, curTm->tm_sec);

                    RCLCPP_WARN(private_nh_->get_logger(), "%s  lslidar poll() timeout, port: %d", bufTime,port_);

                    return 1;
                }
                if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) ||
                    (fds[0].revents & POLLNVAL))  // device error?
                {
                    RCLCPP_ERROR(private_nh_->get_logger(),"poll() reports lslidar error");
                    return 1;
                }
            } while ((fds[0].revents & POLLIN) == 0);
            ssize_t nbytes = recvfrom(sockfd_, &pkt->data[0], packet_size, 0, (sockaddr * ) & sender_address,
                                      &sender_address_len);

            if ((size_t) nbytes == (size_t) packet_size) {
                if (devip_str_ != "" && sender_address.sin_addr.s_addr != devip_.s_addr) {
                    RCLCPP_ERROR(private_nh_->get_logger(), "lidar IP parameter set error,please reset in launch file");
                    continue;
                } else
                    break;  // done
            }
            RCLCPP_WARN(private_nh_->get_logger(), "[driver][socket] incomplete lslidar packet read: %d bytes", nbytes);
        }
        if (flag == 0) {
            abort();
        }

        return 0;
    }

////////////////////////////////////////////////////////////////////////
// InputPCAP class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
   *
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number
   *  @param packet_rate expected device packet frequency (Hz)
   *  @param filename PCAP dump file name
   */
    InputPCAP::InputPCAP(rclcpp::Node *private_nh, uint16_t port, double packet_rate, std::string filename)
            : Input(private_nh, port), packet_rate_(packet_rate), filename_(filename) {
        pcap_ = NULL;
        empty_ = true;
        read_once_ = false;
        read_fast_ = false;
        repeat_delay_ = 0.0;
//        private_nh->declare_parameter("read_once", false);
//        private_nh->declare_parameter("read_fast", false);
//        private_nh->declare_parameter("repeat_delay", 0.0);

        private_nh->get_parameter("read_once", read_once_);
        private_nh->get_parameter("read_fast", read_fast_);
        private_nh->get_parameter("repeat_delay", repeat_delay_);

        if (read_once_)
            RCLCPP_WARN(private_nh_->get_logger(),"Read input file only once.");
        if (read_fast_)
            RCLCPP_WARN(private_nh_->get_logger(),"Read input file as quickly as possible.");
        if (repeat_delay_ > 0.0)
            RCLCPP_WARN(private_nh_->get_logger(),"Delay %.3f seconds before repeating input file.", repeat_delay_);

        // Open the PCAP dump file
        // ROS_INFO("Opening PCAP file \"%s\"", filename_.c_str());
        RCLCPP_WARN(private_nh_->get_logger(),"Opening PCAP file %s",filename_.c_str());
        if ((pcap_ = pcap_open_offline(filename_.c_str(), errbuf_)) == NULL) {
            RCLCPP_WARN(private_nh_->get_logger(),"Error opening lslidar socket dump file.");
            return;
        }

        std::stringstream filter;
        if (devip_str_ != "")  // using specific IP?
        {
            filter << "src host " << devip_str_ << " && ";
        }
        filter << "udp dst port " << port;
        pcap_compile(pcap_, &pcap_packet_filter_, filter.str().c_str(), 1, PCAP_NETMASK_UNKNOWN);
    }

/** destructor */
    InputPCAP::~InputPCAP(void) {
        pcap_close(pcap_);
    }

/** @brief Get one lslidar packet. */
    int InputPCAP::getPacket(lslidar_msgs::msg::LslidarPacket::UniquePtr &pkt) {
        struct pcap_pkthdr *header;
        const u_char *pkt_data;

        // while (flag == 1)
        while (flag == 1) {
            int res;
            if ((res = pcap_next_ex(pcap_, &header, &pkt_data)) >= 0) {
                // Skip packets not for the correct port and from the
                // selected IP address.
                if (!devip_str_.empty() && (0 == pcap_offline_filter(&pcap_packet_filter_, header, pkt_data)))
                    continue;

                // Keep the reader from blowing through the file.
                if (read_fast_ == false)
                    packet_rate_.sleep();


                memcpy(&pkt->data[0], pkt_data + 42, packet_size);

                // if (pkt->data[0] == 0xA5 && pkt->data[1] == 0xFF && pkt->data[2] == 0x00 &&
                //     pkt->data[3] == 0x5A) {//difop
                //     int rpm = (pkt->data[8] << 8) | pkt->data[9];
                //     RCLCPP_WARN_ONCE(private_nh_->get_logger(),"lidar rpm: %d", rpm);
                // }
                empty_ = false;
                return 0;  // success
            }

            if (empty_)  // no data in file?
            {
                RCLCPP_WARN(private_nh_->get_logger(),"Error %d reading lslidar packet: %s", res, pcap_geterr(pcap_));
                return -1;
            }

            if (read_once_) {
                RCLCPP_WARN(private_nh_->get_logger(),"end of file reached -- done reading.");
                return -1;
            }

            if (repeat_delay_ > 0.0) {
                RCLCPP_WARN(private_nh_->get_logger(),"end of file reached -- delaying %.3f seconds.", repeat_delay_);
                usleep(rint(repeat_delay_ * 1000000.0));
            }

            RCLCPP_WARN(private_nh_->get_logger(),"replaying lslidar dump file");

            // I can't figure out how to rewind the file, because it
            // starts with some kind of header.  So, close the file
            // and reopen it with pcap.
            pcap_close(pcap_);
            pcap_ = pcap_open_offline(filename_.c_str(), errbuf_);
            empty_ = true;  // maybe the file disappeared?
        }                 // loop back and try again

        if (flag == 0) {
            abort();
        }
        return 0;
    }
}
