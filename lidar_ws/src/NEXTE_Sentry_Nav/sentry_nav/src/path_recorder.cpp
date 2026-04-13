#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <fstream>
#include <string>
#include <vector>
#include <sys/stat.h> // for mkdir
#include <std_srvs/Empty.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <signal.h>
#include <csignal>

class EnhancedPathRecorder
{
public:
    EnhancedPathRecorder() : nh_("~"), should_save_on_exit_(true), already_saved_(false), target_frame_("")
    {
        // 设置静态实例指针
        instance_ = this;
        // 获取参数
        nh_.param<std::string>("map_frame", map_frame_, "map");  // 默认改为map，更常见
        nh_.param<std::string>("body_frame", body_frame_, "body");
        nh_.param<std::string>("output_dir", output_dir_, ".");
        nh_.param<double>("record_interval", record_interval_, 0.1);
        nh_.param<bool>("save_planned_path", save_planned_path_, true);
        nh_.param<bool>("save_actual_path", save_actual_path_, true);
        nh_.param<bool>("save_local_plan", save_local_plan_, false);
        nh_.param<std::string>("global_plan_topic", global_plan_topic_, "/move_base/GlobalPlanner/plan");
        nh_.param<std::string>("local_plan_topic", local_plan_topic_, "/move_base/DWAPlannerROS/local_plan");
        nh_.param<std::string>("nav_status_topic", nav_status_topic_, "/move_base/status");
        
        // 如果输出目录为"."，则使用默认目录
        if (output_dir_ == ".")
        {
            output_dir_ = std::string(getenv("HOME")) + "/lidar_ws/navigation_logs";
            ROS_INFO("Using default output directory: %s", output_dir_.c_str());
        }
        
        // 创建输出目录
        createDirectory(output_dir_);
        
        // 初始化可能的地图坐标系列表（按优先级排序）
        possible_map_frames_.push_back(map_frame_);  // 用户指定的坐标系
        possible_map_frames_.push_back("map");
        possible_map_frames_.push_back("odom");
        possible_map_frames_.push_back("camera_init");
        // 移除"world"，因为通常不存在
        
        // 订阅话题
        if (save_planned_path_)
        {
            global_plan_sub_ = nh_.subscribe(global_plan_topic_, 1, 
                                             &EnhancedPathRecorder::globalPlanCallback, this);
            ROS_INFO("Subscribed to global plan topic: %s", global_plan_topic_.c_str());
        }
        
        if (save_local_plan_)
        {
            local_plan_sub_ = nh_.subscribe(local_plan_topic_, 1,
                                            &EnhancedPathRecorder::localPlanCallback, this);
            ROS_INFO("Subscribed to local plan topic: %s", local_plan_topic_.c_str());
        }
        
        // 订阅导航状态，用于检测导航任务开始/结束
        nav_status_sub_ = nh_.subscribe(nav_status_topic_, 1,
                                        &EnhancedPathRecorder::navStatusCallback, this);
        ROS_INFO("Subscribed to navigation status topic: %s", nav_status_topic_.c_str());
        
        // 发布可视化标记
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
        
        // 初始化TF监听器
        tf_listener_ = new tf::TransformListener();
        
        // 设置定时器以记录实际路径
        timer_ = nh_.createTimer(ros::Duration(record_interval_), 
                                &EnhancedPathRecorder::timerCallback, this);
        
        // 初始化服务
        save_service_ = nh_.advertiseService("save_paths", 
                                            &EnhancedPathRecorder::savePathsService, this);
        
        // 初始化标记
        initMarkers();
        
        ROS_INFO("Enhanced Path Recorder initialized.");
        ROS_INFO("Map frame: %s, Body frame: %s", map_frame_.c_str(), body_frame_.c_str());
        ROS_INFO("Possible map frames to try: ");
        for (size_t i = 0; i < possible_map_frames_.size(); ++i)
        {
            ROS_INFO("  %zu. %s", i+1, possible_map_frames_[i].c_str());
        }
        ROS_INFO("Output directory: %s", output_dir_.c_str());
    }
    
    ~EnhancedPathRecorder()
    {
        // 节点关闭时自动保存
        if (should_save_on_exit_)
        {
            ROS_INFO("Saving paths before shutdown...");
            saveAllPaths();
        }
        else
        {
            ROS_INFO("Skipping save on exit (already saved).");
        }
        delete tf_listener_;
        
        // 清除静态实例指针
        if (instance_ == this)
        {
            instance_ = nullptr;
        }
    }
    
    // 修改信号处理函数，强制保存
    static void signalHandler(int signal)
    {
        if (signal == SIGINT || signal == SIGTERM)
        {
            ROS_WARN("Received signal %d, shutting down gracefully...", signal);
            // 如果实例存在，立即强制保存
            if (instance_ != nullptr)
            {
                instance_->saveAllPaths(true);  // 强制保存
            }
            // 短暂延迟以确保保存完成
            ros::Duration(0.5).sleep();
            ros::shutdown();
        }
    }
    
    // 静态实例指针
    static EnhancedPathRecorder* instance_;
    
    void setSaveOnExit(bool save) 
    { 
        should_save_on_exit_ = save; 
    }
    
    void globalPlanCallback(const nav_msgs::Path::ConstPtr& msg)
    {
        boost::mutex::scoped_lock lock(plan_mutex_);
        global_plan_ = *msg;
        global_plan_received_ = true;
        
        // 设置目标坐标系为规划路径的坐标系（首次接收到时）
        if (target_frame_.empty()) {
            target_frame_ = msg->header.frame_id;
            ROS_INFO("Set target frame to: %s", target_frame_.c_str());
            // 一旦设置了目标坐标系，就只使用这个坐标系
            use_target_frame_only_ = true;
            // 清空其他坐标系列表，只保留目标坐标系
            possible_map_frames_.clear();
            possible_map_frames_.push_back(target_frame_);
            ROS_INFO("Now using target frame only for TF lookup.");
        }
        
        // 更新标记
        updatePlannedPathMarker();
    }
    
    void localPlanCallback(const nav_msgs::Path::ConstPtr& msg)
    {
        boost::mutex::scoped_lock lock(plan_mutex_);
        local_plan_ = *msg;
    }
    
    void navStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
    {
        // 检测导航状态变化
        if (!msg->status_list.empty())
        {
            int current_status = msg->status_list[0].status;
            ROS_DEBUG("Received navigation status: %d, navigation_active_: %d", current_status, navigation_active_);
            
            // 状态定义: 1=PENDING, 2=ACTIVE, 3=PREEMPTED, 4=SUCCEEDED, 5=ABORTED
            if (current_status == actionlib_msgs::GoalStatus::ACTIVE && !navigation_active_)
            {
                ROS_INFO("Navigation started. Beginning path recording.");
                navigation_active_ = true;
                // 清空之前记录的实际路径，开始新的记录
                actual_path_.poses.clear();
                // 重置保存标志，允许新的保存
                already_saved_ = false;
                should_save_on_exit_ = true;
            }
            else if ((current_status == actionlib_msgs::GoalStatus::SUCCEEDED || 
                     current_status == actionlib_msgs::GoalStatus::ABORTED ||
                     current_status == actionlib_msgs::GoalStatus::PREEMPTED) && 
                     navigation_active_)
            {
                ROS_INFO("Navigation ended with status %d. Stopping path recording.", current_status);
                navigation_active_ = false;
                // 保存本次导航的数据
                saveAllPaths();
            }
        }
        else
        {
            ROS_DEBUG_THROTTLE(5.0, "Received empty navigation status list");
        }
    }
    
    void timerCallback(const ros::TimerEvent& e)
    {
        // 如果未启用实际路径保存，直接返回
        if (!save_actual_path_)
        {
            ROS_DEBUG_THROTTLE(5.0, "Actual path recording is disabled. save_actual_path_=%d", save_actual_path_);
            return;
        }
        
        // 记录实际路径不再依赖navigation_active_标志
        // 这样可以确保无论导航状态如何，只要节点运行就记录实际路径
        // 导航状态仍然用于触发保存和清空路径
        if (!navigation_active_)
        {
            ROS_DEBUG_THROTTLE(5.0, "Navigation not active, but still recording actual path for debugging. navigation_active_=%d", 
                              navigation_active_);
        }
            
        // 获取机器人当前位置
        tf::StampedTransform transform;
        bool tf_success = false;
        std::string successful_frame = "";
        
        // 尝试所有可能的地图坐标系
        for (size_t i = 0; i < possible_map_frames_.size(); ++i)
        {
            try
            {
                // 增加超时时间，避免坐标系尚未发布
                tf_listener_->waitForTransform(possible_map_frames_[i], body_frame_, ros::Time(0), ros::Duration(0.1));
                tf_listener_->lookupTransform(possible_map_frames_[i], body_frame_, ros::Time(0), transform);
                tf_success = true;
                successful_frame = possible_map_frames_[i];
                
                // 更新当前使用的地图坐标系
                {
                    boost::mutex::scoped_lock lock(frame_mutex_);
                    if (current_map_frame_ != successful_frame)
                    {
                        current_map_frame_ = successful_frame;
                        ROS_INFO("Switched to map frame: %s", current_map_frame_.c_str());
                        
                        // 更新标记的坐标系
                        planned_path_marker_.header.frame_id = current_map_frame_;
                        actual_path_marker_.header.frame_id = current_map_frame_;
                    }
                }
                
                // 调试日志：记录转换结果
                ROS_DEBUG_THROTTLE(5.0, "TF lookup successful: %s -> %s, transform: (%.3f, %.3f, %.3f)", 
                                 successful_frame.c_str(), body_frame_.c_str(),
                                 transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
                break; // 成功找到，跳出循环
            }
            catch (tf::TransformException& ex)
            {
                // 继续尝试下一个坐标系
                if (i == possible_map_frames_.size() - 1)
                {
                    ROS_WARN_THROTTLE(1.0, "TF lookup failed for all frames. Last attempt: %s -> %s: %s", 
                                     possible_map_frames_[i].c_str(), body_frame_.c_str(), ex.what());
                }
                else
                {
                    ROS_DEBUG_THROTTLE(1.0, "TF lookup failed for %s -> %s: %s", 
                                      possible_map_frames_[i].c_str(), body_frame_.c_str(), ex.what());
                }
            }
        }
        
        if (!tf_success)
        {
            ROS_WARN_THROTTLE(2.0, "All TF lookups failed. Cannot record actual path.");
            return; // 所有坐标系都尝试失败，直接返回
        }
        
        // 使用成功找到的坐标系
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = successful_frame;
        pose.pose.position.x = transform.getOrigin().x();
        pose.pose.position.y = transform.getOrigin().y();
        pose.pose.position.z = transform.getOrigin().z();
        pose.pose.orientation.x = transform.getRotation().x();
        pose.pose.orientation.y = transform.getRotation().y();
        pose.pose.orientation.z = transform.getRotation().z();
        pose.pose.orientation.w = transform.getRotation().w();
        
        // 调试日志：如果坐标值异常小，发出警告
        if (fabs(transform.getOrigin().x()) < 0.1 && fabs(transform.getOrigin().y()) < 0.1)
        {
            ROS_WARN_THROTTLE(2.0, "Actual path coordinates are near zero (%.3f, %.3f). Possible TF issue with frame %s -> %s",
                            transform.getOrigin().x(), transform.getOrigin().y(),
                            successful_frame.c_str(), body_frame_.c_str());
        }
        
        // 添加到实际路径
        {
            boost::mutex::scoped_lock lock(actual_path_mutex_);
            actual_path_.header = pose.header;
            actual_path_.poses.push_back(pose);
            ROS_DEBUG("Recorded actual path point #%zu at (%.3f, %.3f, %.3f)", 
                     actual_path_.poses.size(), pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
        }
        
        // 更新实际路径标记
        updateActualPathMarker();
        
        // 发布标记
        if (global_plan_received_)
        {
            marker_pub_.publish(planned_path_marker_);
        }
        marker_pub_.publish(actual_path_marker_);
    }
    
    bool savePathsService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
    {
        ROS_INFO("Manual save request received.");
        // 手动保存时强制保存，并重置标志允许后续自动保存
        saveAllPaths(true);  // 强制保存
        already_saved_ = false;  // 重置标志，允许再次自动保存
        should_save_on_exit_ = true;  // 确保退出时仍然保存
        return true;
    }
    
    void saveAllPaths(bool force_save = false)
    {
        // 防止重复保存
        if (already_saved_ && !force_save)
        {
            ROS_WARN("Paths already saved, skipping duplicate save.");
            return;
        }
        
        already_saved_ = true;
        should_save_on_exit_ = false;  // 析构函数中不再保存
        
        // 生成时间戳
        ros::Time now = ros::Time::now();
        char time_buf[100];
        time_t raw_time = static_cast<time_t>(now.toSec());
        std::strftime(time_buf, sizeof(time_buf), "%Y%m%d_%H%M%S", 
                     std::localtime(&raw_time));
        
        std::string timestamp = std::string(time_buf);
        
        bool any_data_saved = false;
        
        // 保存全局规划路径
        if (save_planned_path_ && global_plan_received_)
        {
            savePathToCSV(global_plan_, output_dir_ + "/global_plan_" + timestamp + ".csv", 
                         "timestamp,x,y,z,qx,qy,qz,qw");
            any_data_saved = true;
        }
        
        // 保存局部规划路径
        if (save_local_plan_)
        {
            boost::mutex::scoped_lock lock(plan_mutex_);
            if (!local_plan_.poses.empty())
            {
                savePathToCSV(local_plan_, output_dir_ + "/local_plan_" + timestamp + ".csv",
                             "timestamp,x,y,z,qx,qy,qz,qw");
                any_data_saved = true;
            }
        }
        
        // 保存实际路径
        if (save_actual_path_)
        {
            boost::mutex::scoped_lock lock(actual_path_mutex_);
            if (!actual_path_.poses.empty())
            {
                savePathToCSV(actual_path_, output_dir_ + "/actual_path_" + timestamp + ".csv",
                             "timestamp,x,y,z,qx,qy,qz,qw");
                any_data_saved = true;
            }
        }
        
        // 如果没有数据被保存，创建一个标记文件表示保存被触发但没有数据
        if (!any_data_saved)
        {
            std::string marker_file = output_dir_ + "/no_path_data_" + timestamp + ".txt";
            std::ofstream file(marker_file.c_str());
            if (file.is_open())
            {
                file << "Path recorder saved at " << timestamp << " but no data was available.\n";
                file << "Conditions:\n";
                file << "  global_plan_received_: " << (global_plan_received_ ? "true" : "false") << "\n";
                file << "  save_planned_path_: " << (save_planned_path_ ? "true" : "false") << "\n";
                file << "  save_actual_path_: " << (save_actual_path_ ? "true" : "false") << "\n";
                file << "  save_local_plan_: " << (save_local_plan_ ? "true" : "false") << "\n";
                file << "  navigation_active_: " << (navigation_active_ ? "true" : "false") << "\n";
                file.close();
                ROS_INFO("No path data available. Marker file created: %s", marker_file.c_str());
            }
        }
        
        ROS_INFO("All paths saved with timestamp: %s", timestamp.c_str());
    }
    
private:
    void createDirectory(const std::string& path)
    {
        if (path.empty() || path == ".")
            return;
            
        // 检查目录是否存在
        struct stat st;
        if (stat(path.c_str(), &st) != 0)
        {
            // 目录不存在，创建它
            if (mkdir(path.c_str(), 0755) != 0)
            {
                ROS_WARN("Failed to create directory: %s. Using current directory.", path.c_str());
                output_dir_ = ".";
            }
            else
            {
                ROS_INFO("Created output directory: %s", path.c_str());
            }
        }
        else if (!S_ISDIR(st.st_mode))
        {
            ROS_WARN("Path exists but is not a directory: %s. Using current directory.", path.c_str());
            output_dir_ = ".";
        }
    }
    
    void savePathToCSV(const nav_msgs::Path& path, const std::string& filename, 
                      const std::string& header)
    {
        std::ofstream file(filename.c_str());
        if (!file.is_open())
        {
            ROS_ERROR("Failed to open file for writing: %s", filename.c_str());
            return;
        }
        
        file << header << "\n";
        for (const auto& pose : path.poses)
        {
            file << pose.header.stamp.toSec() << ","
                 << pose.pose.position.x << ","
                 << pose.pose.position.y << ","
                 << pose.pose.position.z << ","
                 << pose.pose.orientation.x << ","
                 << pose.pose.orientation.y << ","
                 << pose.pose.orientation.z << ","
                 << pose.pose.orientation.w << "\n";
        }
        
        // 确保数据写入磁盘
        file.flush();
        file.close();
        
        ROS_INFO("Saved path to: %s (%zu points)", filename.c_str(), path.poses.size());
    }
    
    void initMarkers()
    {
        // 初始化规划路径标记（红色）
        planned_path_marker_.header.frame_id = map_frame_;
        planned_path_marker_.ns = "paths";
        planned_path_marker_.id = 0;
        planned_path_marker_.type = visualization_msgs::Marker::LINE_STRIP;
        planned_path_marker_.action = visualization_msgs::Marker::ADD;
        planned_path_marker_.pose.orientation.w = 1.0;
        planned_path_marker_.scale.x = 0.05; // 线宽
        planned_path_marker_.color.r = 1.0;
        planned_path_marker_.color.g = 0.0;
        planned_path_marker_.color.b = 0.0;
        planned_path_marker_.color.a = 0.8;
        planned_path_marker_.lifetime = ros::Duration();
        
        // 初始化实际路径标记（绿色）
        actual_path_marker_.header.frame_id = map_frame_;
        actual_path_marker_.ns = "paths";
        actual_path_marker_.id = 1;
        actual_path_marker_.type = visualization_msgs::Marker::LINE_STRIP;
        actual_path_marker_.action = visualization_msgs::Marker::ADD;
        actual_path_marker_.pose.orientation.w = 1.0;
        actual_path_marker_.scale.x = 0.05; // 线宽
        actual_path_marker_.color.r = 0.0;
        actual_path_marker_.color.g = 1.0;
        actual_path_marker_.color.b = 0.0;
        actual_path_marker_.color.a = 0.8;
        actual_path_marker_.lifetime = ros::Duration();
    }
    
    void updatePlannedPathMarker()
    {
        boost::mutex::scoped_lock lock(plan_mutex_);
        planned_path_marker_.points.clear();
        for (const auto& pose : global_plan_.poses)
        {
            geometry_msgs::Point point;
            point.x = pose.pose.position.x;
            point.y = pose.pose.position.y;
            point.z = pose.pose.position.z;
            planned_path_marker_.points.push_back(point);
        }
        planned_path_marker_.header.stamp = ros::Time::now();
    }
    
    void updateActualPathMarker()
    {
        boost::mutex::scoped_lock lock(actual_path_mutex_);
        actual_path_marker_.points.clear();
        for (const auto& pose : actual_path_.poses)
        {
            geometry_msgs::Point point;
            point.x = pose.pose.position.x;
            point.y = pose.pose.position.y;
            point.z = pose.pose.position.z;
            actual_path_marker_.points.push_back(point);
        }
        actual_path_marker_.header.stamp = ros::Time::now();
    }
    
private:
    ros::NodeHandle nh_;
    
    // 静态实例指针定义（必须在类外定义）
    
    // 订阅者和发布者
    ros::Subscriber global_plan_sub_;
    ros::Subscriber local_plan_sub_;
    ros::Subscriber nav_status_sub_;
    ros::Publisher marker_pub_;
    ros::Timer timer_;
    ros::ServiceServer save_service_;
    
    // TF监听器
    tf::TransformListener* tf_listener_;
    
    // 路径数据
    nav_msgs::Path global_plan_;
    nav_msgs::Path local_plan_;
    nav_msgs::Path actual_path_;
    
    // 标记
    visualization_msgs::Marker planned_path_marker_;
    visualization_msgs::Marker actual_path_marker_;
    
    // 参数
    std::string map_frame_;
    std::string body_frame_;
    std::string output_dir_;
    double record_interval_;
    bool save_planned_path_;
    bool save_actual_path_;
    bool save_local_plan_;
    std::string global_plan_topic_;
    std::string local_plan_topic_;
    std::string nav_status_topic_;
    
    // 状态标志
    bool global_plan_received_ = false;
    bool navigation_active_ = false;
    bool should_save_on_exit_;  // 控制退出时是否保存
    bool already_saved_;        // 防止重复保存
    std::string target_frame_;  // 目标坐标系，优先使用规划路径坐标系
    bool use_target_frame_only_ = false;  // 是否只使用目标坐标系
    
    // 互斥锁
    boost::mutex plan_mutex_;
    boost::mutex actual_path_mutex_;
    boost::mutex frames_mutex_;  // 保护坐标系列表
    
    // 可能的地图坐标系列表
    std::vector<std::string> possible_map_frames_;
    // 当前使用的地图坐标系
    std::string current_map_frame_;
    // 用于保护current_map_frame_的互斥锁
    boost::mutex frame_mutex_;
};

// 静态成员变量定义
EnhancedPathRecorder* EnhancedPathRecorder::instance_ = nullptr;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "enhanced_path_recorder");
    
    // 注册信号处理器
    signal(SIGINT, EnhancedPathRecorder::signalHandler);
    signal(SIGTERM, EnhancedPathRecorder::signalHandler);
    
    EnhancedPathRecorder recorder;
    ros::spin();
    return 0;
}
