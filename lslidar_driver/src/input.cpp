/******************************************************************************
 * This file is part of lslidar_driver.
 *
 * Copyright 2022 LeiShen Intelligent Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "lslidar_driver/input.hpp"

extern volatile sig_atomic_t flag;
namespace lslidar_driver {

////////////////////////////////////////////////////////////////////////
// Input base class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
 *
 *  @param private_nh ROS private handle for calling node.
 *  @param port UDP port number.
 */
    Input::Input(rclcpp::Node::SharedPtr private_nh, uint16_t port, int packet_size) 
        : private_nh_(private_nh), port_(port), packet_size_(packet_size) {
        npkt_update_flag_ = false;
        cur_rpm_ = 0;
        return_mode_ = 1;

        private_nh_->get_parameter("device_ip", devip_str_);
        private_nh_->get_parameter("add_multicast", add_multicast);
        private_nh_->get_parameter("group_ip", group_ip);
        private_nh_->get_parameter("difop_port", difop_port_);

        // if (!devip_str_.empty())
        //     LS_INFO << "Only accepting packets from IP address: " << devip_str_ << LS_END;
    }

////////////////////////////////////////////////////////////////////////
// InputSocket class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
   *
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number
*/
    InputSocket::InputSocket(rclcpp::Node::SharedPtr private_nh, uint16_t port, int packet_size) 
        : Input(private_nh, port, packet_size) {
        sockfd_ = -1;

        if (!devip_str_.empty()) {
            inet_aton(devip_str_.c_str(), &devip_);
        }
        
        // LS_INFO << "Opening UDP socket port: " << port << LS_END;
        sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
        if (sockfd_ == -1) {
            perror("socket");
            return;
        }

        int opt = 1;
        if (setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, (const void *)&opt, sizeof(opt))) {
            perror("setsockopt error!\n");
            return;
        }

        sockaddr_in my_addr;                   
        memset(&my_addr, 0, sizeof(my_addr));  
        my_addr.sin_family = AF_INET;          
        my_addr.sin_port = htons(port);        
        my_addr.sin_addr.s_addr = INADDR_ANY;  

        if (bind(sockfd_, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1) {
            perror("bind");
            return;
        }

        if (add_multicast) {
            struct ip_mreq group;
            group.imr_multiaddr.s_addr = inet_addr(group_ip.c_str());
            group.imr_interface.s_addr = htonl(INADDR_ANY);

            if (setsockopt(sockfd_, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&group, sizeof(group)) < 0) {
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
        static const int POLL_TIMEOUT = 2000;  

        sockaddr_in sender_address{};
        socklen_t sender_address_len = sizeof(sender_address);

        int retval = poll(fds, 1, POLL_TIMEOUT);

        if (retval > 0 && (fds[0].revents & POLLIN)) {
            ssize_t nbytes = recvfrom(sockfd_, &pkt->data[0], packet_size_, 0, (sockaddr *)&sender_address, &sender_address_len);

            if (sender_address.sin_addr.s_addr == devip_.s_addr) {
                return nbytes; 
            } else {
                LS_WARN << "Lidar IP parameter mismatch. Received IP: " << inet_ntoa(sender_address.sin_addr) 
                        << ". Please reset lidar IP in the launch file." << LS_END;
                return 0;
            }
        } else {
            if (retval == 0) {  
                time_t curTime = time(NULL);
                struct tm *curTm = localtime(&curTime);
                char bufTime[72] = {0};
                sprintf(bufTime, "%d-%d-%d %d:%d:%d", curTm->tm_year + 1900, curTm->tm_mon + 1,
                        curTm->tm_mday, curTm->tm_hour, curTm->tm_min, curTm->tm_sec);
                LS_WARN << bufTime << "  Lidar poll() timeout, port:" << port_ << LS_END;
                return 0; 
            }

            if (retval < 0) { 
                if (errno != EINTR) {
                    LS_ERROR << "poll() error: " << strerror(errno) << LS_END;
                }
                return -1; 
            }

            if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) || (fds[0].revents & POLLNVAL)) {
                LS_ERROR << "poll() reports lidar error" << LS_END;
                return -1;
            }
        }

        return 0;
    }

    ssize_t InputSocket::sendPacket(const unsigned char *data, size_t length) {
        if (data == nullptr || length <= 0) {
            LS_ERROR << "Invalid input data or length." << LS_END;
            return -1;
        }

        sockaddr_in server_sai;
        server_sai.sin_family = AF_INET;
        server_sai.sin_port = htons(difop_port_);
        server_sai.sin_addr.s_addr = inet_addr(devip_str_.c_str());

        ssize_t nbytes = sendto(sockfd_, data, length, 0, (struct sockaddr *)&server_sai, sizeof(server_sai));

        if (nbytes < 0) {
            LS_ERROR << "Data packet sending failed: " << strerror(errno) << LS_END;
        } else if (nbytes != length) {
            LS_WARN << "Partial data sent:" << nbytes << "/" << length << " bytes." << LS_END;
        } else {
            LS_INFO << "Successfully sent " <<  nbytes << " bytes!" << LS_END;
        }

        return nbytes;
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
    InputPCAP::InputPCAP(rclcpp::Node::SharedPtr private_nh, uint16_t port, int packet_size, double packet_rate, std::string filename)
             : Input(private_nh, port, packet_size), packet_rate_(packet_rate), filename_(filename) {
        pcap_ = NULL;
        empty_ = true;
        
        private_nh_->get_parameter("read_once", read_once_);
        private_nh_->get_parameter("read_fast", read_fast_);
        private_nh_->get_parameter("repeat_delay", repeat_delay_);

        if (read_once_)
            LS_INFO << "Read input file only once." << LS_END;
        if (read_fast_)
            LS_INFO << "Read input file as quickly as possible." << LS_END;
        if (repeat_delay_ > 0.0)
            LS_INFO << "Delay " << repeat_delay_ << " seconds before repeating input file." << LS_END;

        LS_INFO << "Opening PCAP file " << filename_ << LS_END;
        if ((pcap_ = pcap_open_offline(filename_.c_str(), errbuf_)) == NULL) {
            LS_ERROR << "Error opening lidar socket dump file." << LS_END;
            return;
        }

        std::stringstream filter;
        if (devip_str_ != "")  {
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
        
        while (flag == 1) {
            int res;
            if ((res = pcap_next_ex(pcap_, &header, &pkt_data)) >= 0) {
                if (!devip_str_.empty() && (0 == pcap_offline_filter(&pcap_packet_filter_, header, pkt_data)))
                    continue;

                if (read_fast_ == false)
                    packet_rate_.sleep();

                memcpy(&pkt->data[0], pkt_data + 42, packet_size_);

                // if (pkt->data[0] == 0xA5 && pkt->data[1] == 0xFF && pkt->data[2] == 0x00 &&
                //     pkt->data[3] == 0x5A) {
                //     int rpm = (pkt->data[8] << 8) | pkt->data[9];
                //     LS_PCAP << "Lidar RPM: " << rpm << LS_END;
                // }

                pkt->stamp = rclcpp::Clock().now();  
                empty_ = false;
                return packet_size_;  
            }

            if (empty_) {
                LS_WARN << "Error " << res << " reading lidar packet: " << pcap_geterr(pcap_) << LS_END;
                return -1;
            }

            if (read_once_) {
                LS_INFO << "End of file reached -- done reading." << LS_END;
                return -1;
            }

            if (repeat_delay_ > 0.0) {
                LS_INFO << "End of file reached -- delaying " << repeat_delay_ << " seconds." << LS_END;
                usleep(rint(repeat_delay_ * 1000000.0));
            }

            LS_PCAP << "Replaying lidar dump file" << LS_END;

            pcap_close(pcap_);
            pcap_ = pcap_open_offline(filename_.c_str(), errbuf_);
            empty_ = true;
        }

        if (flag == 0) {
            abort();
        }

        return 0;
    }

    ssize_t InputPCAP::sendPacket(const unsigned char *data, size_t length) {
        LS_WARN << "Offline settings are not currently supported." << LS_END;
        return -1;
    }

} //namespace
