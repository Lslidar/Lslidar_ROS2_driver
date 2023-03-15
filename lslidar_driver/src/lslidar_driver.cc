/*
 * This file is part of lslidar driver.
 *
 * The driver is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the driver.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <string>
#include <cmath>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>
#include <stdio.h>
#include <stdlib.h>
 
#include "rclcpp/rclcpp.hpp"
#include "lslidar_driver/lslidar_driver.h"
#include <functional>


namespace lslidar_driver {

static void my_hander(int sig)
{
    printf("sig: %d",sig);
	abort();
}
LslidarDriver::LslidarDriver() : LslidarDriver(rclcpp::NodeOptions()) {}
LslidarDriver::LslidarDriver(const rclcpp::NodeOptions& options) : 
    Node("lslidar_driver_node", options), diagnostics(this)
{
	signal(SIGINT, my_hander);
	
	if (!this->initialize()) 
		RCLCPP_ERROR(this->get_logger(), "Could not initialize the driver...");
	else
		RCLCPP_INFO(this->get_logger(), "Successfully initialize driver...");
}

LslidarDriver::~LslidarDriver() {
    return;
}

bool LslidarDriver::loadParameters() {
    pubscan_thread_ = new boost::thread(boost::bind(&LslidarDriver::pubScanThread, this));
	frame_id = std::string("laser_link");
    scan_topic = std::string("/scan");
    pointcloud_topic = std::string("/lslidar_point_cloud");
	is_start = true;
    min_range = 0.3;
    max_range = 100.0;
	use_gps_ts = true;
	pubScan = true;
    pubPointCloud2 = true;
    angle_disable_min = 0.0;
    angle_disable_max = 0.0;
    agreement_type = 1.6;

	this->declare_parameter<std::string>("frame_id","laser_link");
	this->declare_parameter<std::string>("scan_topic","/scan");
    this->declare_parameter<std::string>("pointcloud_topic","/lslidar_point_cloud");

	this->declare_parameter<bool>("pubScan",false);
	this->declare_parameter<bool>("use_gps_ts",false);
    this->declare_parameter<bool>("pubPointCloud2",false);
	this->declare_parameter<double>("angle_disable_min",0.0);
	this->declare_parameter<double>("angle_disable_max",0.0);
    this->declare_parameter<double>("agreement_type",1.6);
	this->declare_parameter<double>("min_range",0.3);
	this->declare_parameter<double>("max_range",100.0);

	this->get_parameter("frame_id", frame_id);
	this->get_parameter("scan_topic", scan_topic);
	this->get_parameter("min_range", min_range);
	this->get_parameter("max_range", max_range);
	this->get_parameter("use_gps_ts", use_gps_ts);
    this->get_parameter("pointcloud_topic", pointcloud_topic);
    this->get_parameter("pubScan", pubScan);
    this->get_parameter("pubPointCloud2", pubPointCloud2);
	this->get_parameter("angle_disable_min", angle_disable_min);
	this->get_parameter("angle_disable_max", angle_disable_max);
    this->get_parameter("agreement_type", agreement_type);
    while(angle_disable_min<0)	angle_disable_min+=360;
    while(angle_disable_max<0)	angle_disable_max+=360;
    while(angle_disable_min>360)	angle_disable_min-=360;
    while(angle_disable_max>360)	angle_disable_max-=360;
    if(angle_disable_max == angle_disable_min ){
        angle_able_min = 0;
        angle_able_max = 360;
    }
    else{
        if(angle_disable_min<angle_disable_max && angle_disable_min !=0.0){
            angle_able_min = angle_disable_max;
            angle_able_max = angle_disable_min+360;
        }
        if (angle_disable_min<angle_disable_max && angle_disable_min == 0.0){
            angle_able_min = angle_disable_max;
            angle_able_max = 360;
        }
        if (angle_disable_min>angle_disable_max ){
            angle_able_min = angle_disable_max;
            angle_able_max = angle_disable_min;
        }
    }
    count_num = 0;
    scan_points_.resize(4000);
        
    if (agreement_type > 1.65){
        block_point_num = 15;
        DISTANCE_RESOLUTION = 0.004;
        printf("Lidar is 1.7 \n");
    }
    else{
        block_point_num = 1;
        DISTANCE_RESOLUTION = 0.002;
        printf("Lidar is 1.6 \n");
    }
    if(pubScan)         scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic, 10);	
    if(pubPointCloud2)  point_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic, 10);	
	return true;
}

bool LslidarDriver::createRosIO() {
    UDP_PORT_NUMBER = 2368;
    this->declare_parameter<int>("msop_port",2368);
    this->get_parameter("msop_port", UDP_PORT_NUMBER);
    RCLCPP_INFO_STREAM(this->get_logger(), "Opening UDP socket: port " << UDP_PORT_NUMBER);
    dump_file = std::string("");
    this->declare_parameter<std::string>("pcap","");
    this->get_parameter("pcap", dump_file);
    // ROS diagnostics
    diagnostics.setHardwareID("Lslidar_N301");
	int hz = 1;
	if(agreement_type > 1.65) hz = 15;	
    const double diag_freq = 20000.0 / (24*hz);
    diag_max_freq = diag_freq;
    diag_min_freq = diag_freq;
    RCLCPP_INFO(this->get_logger(), "expected frequency: %.3f (Hz)", diag_freq);

    using namespace diagnostic_updater;
        diag_topic.reset(new TopicDiagnostic(
                        "lslidar_packets", diagnostics,
                        FrequencyStatusParam(&diag_min_freq, &diag_max_freq, 0.1, 10),
                        TimeStampStatusParam()));

	double packet_rate = 20000/(24*hz);
    if(dump_file !="")
    {
        msop_input_.reset(new lslidar_driver::InputPCAP(this,UDP_PORT_NUMBER,packet_rate,dump_file));
    }else{
        msop_input_.reset(new lslidar_driver::InputSocket(this,UDP_PORT_NUMBER));

    }
	
    // Output
	return true;
}

int LslidarDriver::getScan(std::vector<ScanPoint> &points, rclcpp::Time &scan_time, float &scan_duration)
{
  boost::unique_lock<boost::mutex> lock(mutex_);
  points.assign(scan_points_bak_.begin(), scan_points_bak_.end());
  scan_time = pre_time_;
  scan_duration = time_.seconds()-pre_time_.seconds();
  return 1;
}

uint64_t LslidarDriver::get_gps_stamp(struct tm t){

   uint64_t ptime =static_cast<uint64_t>(timegm(&t));
   return ptime;
}

bool LslidarDriver::initialize() {
    if (!loadParameters()) {
        RCLCPP_ERROR(this->get_logger(), "Cannot load all required ROS parameters...");
        return false;
    }

    if (!createRosIO()) {
        RCLCPP_ERROR(this->get_logger(), "Cannot create all ROS IO...");
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Initialised lslidar without error");
    return true;
}

void LslidarDriver::data_processing(unsigned char *packet_bytes)                                 //处理每一包的数据
{
    for(int i = 0; i < 12; i++) {
        int end_degree_id = 0;
        if(packet_bytes[i*100] != 0xff || packet_bytes[i*100+1] != 0xee)  continue;
        int s = packet_bytes[i * 100 + 2];
        int z = packet_bytes[i * 100 + 3];
        double degree = (double)((z * 256 + s) / 100.f);
        
        if(i<11)                end_degree_id = i+1;
        else                    end_degree_id = i-1;
        s = packet_bytes[end_degree_id * 100 + 2];
        z = packet_bytes[end_degree_id * 100 + 3];
        double end_degree = (double)((z * 256 + s) / 100.f);

        if(end_degree > 360)    end_degree-=360;
        if(degree > 360)        degree-=360;

        double degree_diff = fabs(degree - end_degree);
        if(degree_diff>180)   degree_diff = fabs(360-degree_diff);
        degree_diff = degree_diff / block_point_num/2.0;

        for(int j = 0; j < 2; j++){
            for(int k = 0; k < block_point_num; k++){
                double point_degree = (degree+(j*block_point_num+k)*degree_diff);
                if(point_degree >= 360)   point_degree-=360;
                s = packet_bytes[i * 100 + j * 48 + k * 3 + 4];
                z = packet_bytes[i * 100 + j * 48 + k * 3 + 5];
                double point_distance = (double)((z * 256 + s)*DISTANCE_RESOLUTION);
                int point_Intensity = packet_bytes[i * 100 + j * 48 + k * 3 + 6];
                scan_points_[idx].degree    = point_degree;
                scan_points_[idx].range     = point_distance;
                scan_points_[idx].intensity = point_Intensity;
                
                if ((scan_points_[idx].degree < last_degree && scan_points_[idx].degree < 5 && last_degree > 355)) 	
                {
                    last_degree = scan_points_[idx].degree;
                    count_num = idx;
                    idx = 0;
                    for(long unsigned int k=0;k<scan_points_.size();k++)
                    {	
                        if(scan_points_[k].range < min_range || scan_points_[k].range > max_range)
                            scan_points_[k].range = 0;
                    }
                    boost::unique_lock<boost::mutex> lock(mutex_);
                    scan_points_bak_.resize(scan_points_.size());
                    scan_points_bak_.assign(scan_points_.begin(), scan_points_.end());
                    for(long unsigned int k=0; k<scan_points_.size(); k++)
                    {
                        scan_points_[k].range = 0;
                        scan_points_[k].degree = 0;
                        scan_points_[k].intensity = 0;
                    }
                    pre_time_ = time_;
                    lock.unlock();
                    pubscan_cond_.notify_one();
                    time_ = get_clock()->now();
                }
                else
                {
                    last_degree = scan_points_[idx].degree;
                    idx++;
                }
            }
        }
		if(use_gps_ts)
		{
			if (agreement_type > 1.65){
			pTime.tm_year = packet_bytes[i * 100 + 49]+2000-1900;
			pTime.tm_mon  = packet_bytes[i * 100 + 50]-1;
			pTime.tm_mday = packet_bytes[i * 100 + 51];
			pTime.tm_hour = packet_bytes[i * 100 + 97];
			pTime.tm_min  = packet_bytes[i * 100 + 98];
			pTime.tm_sec  = packet_bytes[i * 100 + 99];
			}
			else{
			pTime.tm_year = packet_bytes[i * 100 + 94]+2000-1900;
			pTime.tm_mon  = packet_bytes[i * 100 + 95]-1;
			pTime.tm_mday = packet_bytes[i * 100 + 96];
			pTime.tm_hour = packet_bytes[i * 100 + 97];
			pTime.tm_min  = packet_bytes[i * 100 + 98];
			pTime.tm_sec  = packet_bytes[i * 100 + 99];
			}
			sub_second	  =(packet_bytes[1200] +
							packet_bytes[1201] * pow(2, 8) +
							packet_bytes[1202] * pow(2, 16) +
							packet_bytes[1203] * pow(2, 24)) * 1e3;
			sweep_end_time_gps = get_gps_stamp(pTime);
			sweep_end_time_hardware = sub_second%1000000000;
		}
    }
    packet_bytes = {0x00};
    if (packet_bytes)
    {
        packet_bytes = NULL;
        delete packet_bytes;
    }
}

void LslidarDriver::pubScanThread()
{
  bool wait_for_wake = true;
  boost::unique_lock<boost::mutex> lock(pubscan_mutex_);

  while (rclcpp::ok())
  {

    while (wait_for_wake)
    {
      pubscan_cond_.wait(lock);
      wait_for_wake = false;
    }
	if(count_num <= 10 )		
        continue;
    if(pubScan){
        auto scan = sensor_msgs::msg::LaserScan::UniquePtr(new sensor_msgs::msg::LaserScan());
        int scan_num = ceil((angle_able_max-angle_able_min)/360*count_num)+1;
        std::vector<ScanPoint> points;
        rclcpp::Time start_time;
        float scan_time;
        this->getScan(points, start_time, scan_time);
        scan->header.frame_id = frame_id;
        if (use_gps_ts){
            scan->header.stamp = rclcpp::Time(sweep_end_time_gps, sweep_end_time_hardware);
        }
        else{
            scan->header.stamp = this->now();  // timestamp will obtained from sweep data stamp
        }

        if(angle_able_max >360){
        scan->angle_min = 2 * M_PI * (angle_able_min-360) / 360;
        scan->angle_max = 2 * M_PI * (angle_able_max-360) / 360;
        }
        else{
        scan->angle_min = 2 * M_PI * angle_able_min / 360;
        scan->angle_max = 2 * M_PI * angle_able_max / 360;
        }
        scan->angle_increment = 2 * M_PI / (double)(count_num - 1);

        scan->range_min = min_range;
        scan->range_max = max_range;
        scan->ranges.reserve(scan_num);
        scan->ranges.assign(scan_num, std::numeric_limits<float>::infinity());
        scan->intensities.reserve(scan_num);
        scan->intensities.assign(scan_num, std::numeric_limits<float>::infinity());
        scan->scan_time = scan_time;
        scan->time_increment = scan_time / (double)(count_num - 1);
        
        int start_num = floor(angle_able_min * count_num / 360);
        int end_num = floor(angle_able_max * count_num / 360);

        for (int i = 0; i < count_num; i++) {
            int point_idx = round((360 - points[i].degree) * count_num / 360);
            if(point_idx<(end_num-count_num))
                point_idx += count_num;
            point_idx =  point_idx - start_num;
            if(point_idx < 0 || point_idx >= scan_num) 
                continue;
            if (points[i].range == 0.0) {
                scan->ranges[point_idx] = std::numeric_limits<float>::infinity();
            }
            else {
                double dist = points[i].range;
                scan->ranges[point_idx] = (float) dist;
            }
            scan->intensities[point_idx] = points[i].intensity;
        }
        scan_pub->publish(std::move(scan));
    }
    if(pubPointCloud2){
        std::vector<ScanPoint> points;
        rclcpp::Time start_time;
        float scan_time;
        this->getScan(points, start_time, scan_time);
        VPointCloud::Ptr point_cloud(new VPointCloud());
        if (use_gps_ts){
            start_time = rclcpp::Time(sweep_end_time_gps, sweep_end_time_hardware);
        }
        double timestamp = start_time.seconds();
        point_cloud->header.stamp = static_cast<uint64_t>(timestamp * 1e6);
        point_cloud->header.frame_id = frame_id;
        point_cloud->height = 1;
        for(uint16_t i = 0; i < count_num; i++)
        {
            double degree = 360.0 - points[i].degree;
            bool pass_point = false;
            if(angle_able_max < 360){
                if(degree < angle_able_min || degree > angle_able_max)              pass_point = true;
            }
            else{
                if(degree < angle_able_min && degree > (angle_able_max-360))        pass_point = true;
            }
            if(points[i].range < 0.001)                                             pass_point = true;
            if(!pass_point)
            {
                VPoint point;
                int point_idx = round(degree * count_num / 360);
                point.timestamp = timestamp - point_idx*(scan_time/count_num);
                point.x = points[i].range*cos(M_PI/180*points[i].degree);
                point.y = -points[i].range*sin(M_PI/180*points[i].degree);
                point.z = 0;
                point.intensity = points[i].intensity;
                point_cloud->points.push_back(point);
                ++point_cloud->width;
            }
        }
        sensor_msgs::msg::PointCloud2 pc_msg;
        pcl::toROSMsg(*point_cloud, pc_msg);
        point_cloud_pub->publish(pc_msg);
    }
	count_num = 0;    
    wait_for_wake = true;
  }
}

bool LslidarDriver::polling()
{
    if(!is_start) return true;
    // Allocate a new shared pointer for zero-copy sharing with other nodelets.
    unsigned char * packet_bytes = new unsigned char[1206];
    int len = 0;
    auto packet =  lslidar_msgs::msg::LslidarPacket::UniquePtr(
                new lslidar_msgs::msg::LslidarPacket());
		
    std_msgs::msg::Byte msg;
    while (true)
    {
        len = 0;
        // keep reading until full packet received
        len = msop_input_->getPacket(packet);
        if(len > 1)
        break;
    } 
    for (int i = 0; i < 1206; i++)
    {
        packet_bytes[i] = packet->data[i];
    }
    LslidarDriver::data_processing(packet_bytes);    
    delete packet_bytes;
    return true;
}

} // namespace lslidar_driver
