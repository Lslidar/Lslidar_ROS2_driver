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

#include "lslidar_driver/lsiosr.hpp"

namespace lslidar_driver {

    LSIOSR::LSIOSR(const std::string& port, BaudRate baud_rate,
                DataBits data_bits, Parity parity, StopBits stop_bits)
        : port_(port), baud_rate_(baud_rate), data_bits_(data_bits),
        parity_(parity), stop_bits_(stop_bits), fd_(-1) {
        // fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY);
        fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        
        if (fd_ < 0) {
            throw std::system_error(errno, std::system_category(),
                                    "Unable to open serial port: " + port);
        }

        flushIO();

        setOpt(data_bits_, parity_, stop_bits_);

        LS_INFO << "Successfully opened serial port" << port_.c_str() << " at " << static_cast<int>(baud_rate_) << " baud." << LS_END;
    }

    LSIOSR::~LSIOSR() {
        if (fd_ >= 0) {
            ::close(fd_);
        }
    }

    void LSIOSR::flushIO() {
        if (fd_ >= 0) {
            tcflush(fd_, TCIOFLUSH);
        }
    }

    void LSIOSR::setOpt(DataBits nBits, Parity nEvent, StopBits nStop) {
        struct termios newtio;
        if (tcgetattr(fd_, &newtio) != 0) {
            throw std::system_error(errno, std::system_category(),
                                    "Failed to retrieve serial port properties");
        }

        // 清空配置并设置基本参数
        // bzero(&newtio, sizeof(newtio));
        cfmakeraw(&newtio);
        newtio.c_cflag |= CLOCAL | CREAD;

        // 设置数据位
        switch (nBits) {
            case DataBits::SEVEN: 
                newtio.c_cflag |= CS7; 
                break;
            case DataBits::EIGHT: 
                newtio.c_cflag |= CS8; 
                break;
            default:
                throw std::invalid_argument("Invalid DataBits value");
        }

        // 设置奇偶校验
        switch (nEvent) {
            case Parity::NONE:
                newtio.c_cflag &= ~PARENB;
                break;
            case Parity::ODD:
                newtio.c_cflag |= (PARENB | PARODD);
                newtio.c_iflag |= (INPCK | ISTRIP);
                break;
            case Parity::EVEN:
                newtio.c_cflag |= PARENB;
                newtio.c_cflag &= ~PARODD;
                newtio.c_iflag |= (INPCK | ISTRIP);
                break;
            default:
                throw std::invalid_argument("Invalid Baud rate value");
        }

        // 设置波特率
        speed_t baud;
        switch (baud_rate_) {
            case BaudRate::BAUD_230400:
                baud = B230400;
                break;
            case BaudRate::BAUD_460800:
                baud = B460800;
                break;
            case BaudRate::BAUD_500000:
                baud = B500000;
                break;
            case BaudRate::BAUD_921600:
                baud = B921600;
                break;
            default:
                throw std::invalid_argument("Unsupported baud rate");
        }

        cfsetispeed(&newtio, baud);
        cfsetospeed(&newtio, baud);

        // 设置停止位
        switch (nStop) {
            case StopBits::ONE:
                newtio.c_cflag &= ~CSTOPB;
                break;
            case StopBits::TWO:
                newtio.c_cflag |= CSTOPB;
                break;
            default:
                throw std::invalid_argument("Invalid StopBits value");
        }


        // 设置超时和最小字符
        newtio.c_cc[VTIME] = 0;
        newtio.c_cc[VMIN] = 0;

        // 应用配置
        flushIO();
        if (tcsetattr(fd_, TCSANOW, &newtio) != 0) {
            throw std::system_error(errno, std::system_category(),
                                    "Failed to set serial port!");
        }
    }

    bool LSIOSR::waitEvent(short events, int timeout_ms) {
        if (fd_ < 0) return false;

        struct pollfd fds[1];
        fds[0].fd = fd_;
        fds[0].events = events;

        int retval = poll(fds, 1, timeout_ms);

        if (retval > 0) {
            return (fds[0].revents & events) != 0;
        } else if (retval == 0) {
            time_t curTime = time(NULL);
            struct tm *curTm = localtime(&curTime);
            char bufTime[72] = {0};
            sprintf(bufTime, "%d-%d-%d %d:%d:%d", 
                    curTm->tm_year + 1900, curTm->tm_mon + 1,
                    curTm->tm_mday, curTm->tm_hour, 
                    curTm->tm_min, curTm->tm_sec);
            LS_WARN << bufTime << "  Lidar poll() timeout, serial port:" << port_.c_str() << LS_END;
        }
        else {
            if (errno != EINTR) {
                LS_ERROR << "poll() error: " << strerror(errno) << LS_END;
            }
        }

        return false;
    }

    ssize_t LSIOSR::read(unsigned char* buffer, size_t length) {
        if (fd_ < 0) {
            throw std::runtime_error("Serial port not open");
        }

        ssize_t totalBytesRead = 0;
        if (!waitEvent(POLLIN, 2000)) {
            return 0;
        }

        while (length > 0) {
            ssize_t rc = ::read(fd_, buffer, length);
            if (rc > 0) {
                length -= rc;
                buffer += rc;
                totalBytesRead += rc;
            } else if (rc < 0) {
                if (errno != EINTR && errno != EAGAIN) {
                    return -1;
                }
            } else {
                break;
            }
        }
        return totalBytesRead;
    }
    
    //  获取单线雷达串口数据
    int LSIOSR::getSerialData(lslidar_msgs::msg::LslidarPacket::UniquePtr &packet, int packet_length, std::string lidar_model) 
	{
		int count = 0;
		int count_2 = 0;

		while (count < 1) {
			count = read(&packet->data[count], 1);
		}
		if (packet->data[0] != 0xA5) return 0;

		while (count_2 < 1) {
			count_2 = read(&packet->data[count], 1);
			if (count_2 > 0) count += count_2;
		}
		if (packet->data[1] != 0x5A) return 0;

		count_2 = 0;
		if (lidar_model == "M10P") {
			while (count_2 < 2) {
				count_2 = read(&packet->data[count], 2 - count_2);
				if (count_2 > 0) {
					count += count_2;
				}
			}

			packet_length = (packet->data[2] << 8) + packet->data[3];
			if (packet_length > 180) return 0;
		}

		count_2 = 0;
		while (count < packet_length) {
			count_2 = read(&packet->data[count], packet_length - count);
			if (count_2 < 0) {
				return -1;
			}
			count += count_2;
		}

		return count;
	}

    ssize_t LSIOSR::send(const char* buffer, size_t length) {
        if (fd_ < 0) {
            throw std::runtime_error("Serial port not open.");
        }

        ssize_t totalBytesWrite = 0;
        if (!waitEvent(POLLOUT, 2000)) {
            return -1;
        }

        char* pb = (char*)buffer;

        while (length > 0) {
            ssize_t rc = ::write(fd_, pb, length);
            if (rc > 0) {
                length -= rc;
                pb += rc;
                totalBytesWrite += rc;
            } else if (rc < 0) {
                if (errno != EINTR && errno != EAGAIN) {
                    throw std::system_error(errno, std::system_category(),
                                            "Writing to serial port failed!");
                }
            } else {
                break;
            }
        }

        return totalBytesWrite;
    }

} // namespace lslidar_driver
