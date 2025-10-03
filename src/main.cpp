#include <rclcpp/rclcpp.hpp>
#include <motoros2_interfaces/srv/start_rt_mode.hpp>
#include <motoros2_interfaces/srv/reset_error.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <array>
#include <thread>
#include <atomic>
#include <cmath> // For M_PI

// Linux/Network specific headers
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <termios.h>
#include <stdio.h>
#include <sys/select.h>


#pragma pack(push, 1)
typedef enum
{
    Group_1 = 0,
    Group_2,
    Group_3,
    Group_4,
    Group_5,
    Group_6,
    Group_7,
    Group_8,

    MAX_GROUPS
} GroupIndeces;

typedef enum
{
    Joint_S = 0,    //radians
    Joint_L,
    Joint_U,
    Joint_R,
    Joint_B,
    Joint_T,
    Joint_E,
    Joint_8,

    MAX_JOINTS
} JointIndeces;

typedef enum
{
    TCP_X = 0,      //meters
    TCP_Y,
    TCP_Z,

    TCP_Rx,         //radians
    TCP_Ry,
    TCP_Rz,
    TCP_Re,

    TCP_8,          //pulse

    MAX_AXES
} CartesianIndeces;

#define MP_GRP_AXES_NUM 8

struct RtPacket
{
    unsigned int sequenceId;
    double delta[MAX_GROUPS][MP_GRP_AXES_NUM];
    int toolIndex[MAX_GROUPS];
} ;

struct RtReply
{
    unsigned int sequenceEcho;
    double feedbackPositionJoints[MAX_GROUPS][MP_GRP_AXES_NUM];
    double feedbackPositionCartesian[MAX_GROUPS][MP_GRP_AXES_NUM];
    double previousCommandPositionJoints[MAX_GROUPS][MP_GRP_AXES_NUM];
    double previousCommandPositionCartesian[MAX_GROUPS][MP_GRP_AXES_NUM];
    bool fsuInterferenceDetected;
} ;
#pragma pack(pop)

// Constants
constexpr int CONTROL_INTERVAL_MS = 8;
constexpr double CONTROL_INTERVAL_S = CONTROL_INTERVAL_MS / 1000.0;
const int ROBOT_UDP_PORT = 8889;
constexpr int MIN_ACTIVE_DURATION_MS = 500;

// Struct to hold the timestamp of the last press for each key
struct KeyPressTimestamps
{
    using TimePoint = std::chrono::steady_clock::time_point;
    std::atomic<TimePoint> w, s, a, d, r, f; // Translation
    std::atomic<TimePoint> i, k, j, l, u, o; // Rotation
};


class KeyboardRtController : public rclcpp::Node 
{
public:
    KeyboardRtController() : Node("rt_keyboard_controller_node"), sequence_id_(0), udp_socket_fd_(-1), running_(true)
    {
        this->declare_parameter<std::string>("robot_ip", "192.168.1.31");
        this->declare_parameter<double>("speed_limit_mps", 0.3);
        this->declare_parameter<double>("rot_speed_limit_dps", 30.0); // 30 deg/sec
        
        robot_ip_ = this->get_parameter("robot_ip").as_string();
        speed_limit_mps_ = this->get_parameter("speed_limit_mps").as_double();
        double rot_speed_dps = this->get_parameter("rot_speed_limit_dps").as_double();
        rot_speed_rps_ = rot_speed_dps * (M_PI / 180.0);

        // Initialize all timestamps to a long time ago
        auto distant_past = std::chrono::steady_clock::now() - std::chrono::hours(1);
        key_timestamps_.w.store(distant_past); key_timestamps_.s.store(distant_past);
        key_timestamps_.a.store(distant_past); key_timestamps_.d.store(distant_past);
        key_timestamps_.r.store(distant_past); key_timestamps_.f.store(distant_past);
        key_timestamps_.i.store(distant_past); key_timestamps_.k.store(distant_past);
        key_timestamps_.j.store(distant_past); key_timestamps_.l.store(distant_past);
        key_timestamps_.u.store(distant_past); key_timestamps_.o.store(distant_past);
    }

    ~KeyboardRtController() 
    {
        running_ = false;
        // Make one last write to stdin to unblock the read() call
        write(STDIN_FILENO, "q", 1);
        if (keyboard_thread_.joinable()) {
            keyboard_thread_.join();
        }
        if (control_thread_.joinable()) {
            control_thread_.join();
        }

        if (udp_socket_fd_ != -1) { close(udp_socket_fd_); }
        restore_keyboard_input();
        RCLCPP_INFO(this->get_logger(), "Resources cleaned up. Shutting down.");
    }

    bool initialize() 
    {
        // ROS Service calls (unchanged)
        auto reset_client = this->create_client<motoros2_interfaces::srv::ResetError>("reset_error");
        if (!reset_client->wait_for_service(std::chrono::seconds(5))) {
             RCLCPP_ERROR(this->get_logger(), "Service 'reset_error' not available."); return false;
        }
        reset_client->async_send_request(std::make_shared<motoros2_interfaces::srv::ResetError::Request>());
        RCLCPP_INFO(this->get_logger(), "Resetting errors...");
        std::this_thread::sleep_for(std::chrono::milliseconds(500)); 

        auto stop_traj_client = this->create_client<std_srvs::srv::Trigger>("stop_traj_mode");
        if (!stop_traj_client->wait_for_service(std::chrono::seconds(5))) {
             RCLCPP_ERROR(this->get_logger(), "Service 'stop_traj_mode' not available."); return false;
        }
        stop_traj_client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
        RCLCPP_INFO(this->get_logger(), "Stopping trajectory mode...");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        client_ = this->create_client<motoros2_interfaces::srv::StartRtMode>("start_rt_mode");
        if (!client_->wait_for_service(std::chrono::seconds(5))) {
             RCLCPP_ERROR(this->get_logger(), "Service 'start_rt_mode' not available."); return false;
        }

        auto request = std::make_shared<motoros2_interfaces::srv::StartRtMode::Request>();
        request->control_mode.value = motoros2_interfaces::msg::ControlModeEnum::CARTESIAN;
        auto result_future = client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service start_rt_mode"); return false;
        }
        auto start_rt_mode_result = result_future.get();
        if (start_rt_mode_result->result_code.value != motoros2_interfaces::msg::MotionReadyEnum::READY) {
            RCLCPP_ERROR(this->get_logger(), "start_rt_mode failed: %s", start_rt_mode_result->message.c_str());
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Successfully started real-time mode.");
        
        if (!setup_keyboard_input() || !setup_udp_socket()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to setup hardware.");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "--- Robot Control Keys ---");
        RCLCPP_INFO(this->get_logger(), "Translation: [w/s] fwd/back, [a/d] left/right, [r/f] up/down");
        RCLCPP_INFO(this->get_logger(), "Rotation:    [i/k] pitch, [j/l] roll, [u/o] yaw");
        RCLCPP_INFO(this->get_logger(), "Exit:        [q]");

        keyboard_thread_ = std::thread(&KeyboardRtController::keyboard_poll_thread, this);
        control_thread_ = std::thread(&KeyboardRtController::control_loop_thread, this);

        return true;
    }

private:
    // This thread now only updates the timestamp of the last key press.
    void keyboard_poll_thread()
    {
        RCLCPP_INFO(this->get_logger(), "Keyboard polling thread started.");
        
        while(running_)
        {
            char c = 0;
            // This is a blocking call; it will wait until a key is pressed.
            if (read(STDIN_FILENO, &c, 1) > 0)
            {
                auto now = std::chrono::steady_clock::now();
                switch(c) {
                    case 'w': key_timestamps_.w.store(now); break;
                    case 's': key_timestamps_.s.store(now); break;
                    case 'a': key_timestamps_.a.store(now); break;
                    case 'd': key_timestamps_.d.store(now); break;
                    case 'r': key_timestamps_.r.store(now); break;
                    case 'f': key_timestamps_.f.store(now); break;
                    case 'j': key_timestamps_.j.store(now); break;
                    case 'l': key_timestamps_.l.store(now); break;
                    case 'i': key_timestamps_.i.store(now); break;
                    case 'k': key_timestamps_.k.store(now); break;
                    case 'u': key_timestamps_.u.store(now); break;
                    case 'o': key_timestamps_.o.store(now); break;
                    case 'q': running_ = false; rclcpp::shutdown(); break;
                }
            }
        }
        RCLCPP_INFO(this->get_logger(), "Keyboard polling thread stopped.");
    }
    
    // This thread now checks timestamps to determine if a key is "active".
    void control_loop_thread() 
    {
        RCLCPP_INFO(this->get_logger(), "Control loop (UDP) thread started.");

        double trans_increment = speed_limit_mps_ * CONTROL_INTERVAL_S;
        double rot_increment = rot_speed_rps_ * CONTROL_INTERVAL_S;
        
        // Local state for this thread to track the last-seen timestamps
        KeyPressTimestamps::TimePoint local_last_seen_w, local_last_seen_s, local_last_seen_a, local_last_seen_d, local_last_seen_r, local_last_seen_f;
        KeyPressTimestamps::TimePoint local_last_seen_i, local_last_seen_k, local_last_seen_j, local_last_seen_l, local_last_seen_u, local_last_seen_o;

        while(running_)
        {
            RtPacket packet{};
            packet.sequenceId = sequence_id_++;
            memset(packet.delta, 0, sizeof(packet.delta));

            auto now = std::chrono::steady_clock::now();
            
            // Lambda to check and update the state for a single key
            auto is_active = [&](const std::atomic<KeyPressTimestamps::TimePoint>& shared_ts, KeyPressTimestamps::TimePoint& local_ts) {
                auto shared_val = shared_ts.load();
                // Check for a new press (rising edge)
                if (shared_val > local_ts) {
                    local_ts = shared_val;
                }
                // Key is active if the last known press was within the time window
                return std::chrono::duration_cast<std::chrono::milliseconds>(now - local_ts).count() < MIN_ACTIVE_DURATION_MS;
            };

            // Build deltas based on which keys are active
            if (is_active(key_timestamps_.w, local_last_seen_w)) { packet.delta[0][TCP_X] += trans_increment; }
            if (is_active(key_timestamps_.s, local_last_seen_s)) { packet.delta[0][TCP_X] -= trans_increment; }
            if (is_active(key_timestamps_.a, local_last_seen_a)) { packet.delta[0][TCP_Y] += trans_increment; }
            if (is_active(key_timestamps_.d, local_last_seen_d)) { packet.delta[0][TCP_Y] -= trans_increment; }
            if (is_active(key_timestamps_.r, local_last_seen_r)) { packet.delta[0][TCP_Z] += trans_increment; }
            if (is_active(key_timestamps_.f, local_last_seen_f)) { packet.delta[0][TCP_Z] -= trans_increment; }
            
            if (is_active(key_timestamps_.j, local_last_seen_j)) { packet.delta[0][TCP_Rx] += rot_increment; }
            if (is_active(key_timestamps_.l, local_last_seen_l)) { packet.delta[0][TCP_Rx] -= rot_increment; }
            if (is_active(key_timestamps_.i, local_last_seen_i)) { packet.delta[0][TCP_Ry] += rot_increment; }
            if (is_active(key_timestamps_.k, local_last_seen_k)) { packet.delta[0][TCP_Ry] -= rot_increment; }
            if (is_active(key_timestamps_.u, local_last_seen_u)) { packet.delta[0][TCP_Rz] += rot_increment; }
            if (is_active(key_timestamps_.o, local_last_seen_o)) { packet.delta[0][TCP_Rz] -= rot_increment; }

            // Duplicate motion commands to Group 2, as in your uploaded file
            packet.delta[1][TCP_X] = packet.delta[0][TCP_X];
            packet.delta[1][TCP_Y] = packet.delta[0][TCP_Y];
            packet.delta[1][TCP_Z] = packet.delta[0][TCP_Z];
            packet.delta[1][TCP_Rx] = packet.delta[0][TCP_Rx];
            packet.delta[1][TCP_Ry] = packet.delta[0][TCP_Ry];
            packet.delta[1][TCP_Rz] = packet.delta[0][TCP_Rz];
            
            sendto(udp_socket_fd_, &packet, sizeof(packet), 0, (struct sockaddr*)&robot_addr_, sizeof(robot_addr_));

            RtReply reply{};
            socklen_t addr_len = sizeof(robot_addr_);
            recvfrom(udp_socket_fd_, &reply, sizeof(reply), 0, (struct sockaddr*)&robot_addr_, &addr_len);

            if (reply.fsuInterferenceDetected) 
            {
                RCLCPP_ERROR(this->get_logger(), "FSU interference detected.");
            }
        }
        RCLCPP_INFO(this->get_logger(), "Control loop (UDP) thread stopped.");
    }

    bool setup_keyboard_input() {
        if (tcgetattr(STDIN_FILENO, &oldt_) == -1) return false;
        newt_ = oldt_;
        newt_.c_lflag &= ~(ICANON | ECHO);
        if (tcsetattr(STDIN_FILENO, TCSANOW, &newt_) == -1) return false;
        return true;
    }
    
    void restore_keyboard_input() {
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt_);
    }

    bool setup_udp_socket() {
        udp_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_socket_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create UDP socket."); return false;
        }

        memset(&robot_addr_, 0, sizeof(robot_addr_));
        robot_addr_.sin_family = AF_INET;
        robot_addr_.sin_port = htons(ROBOT_UDP_PORT);
        if (inet_pton(AF_INET, robot_ip_.c_str(), &robot_addr_.sin_addr) <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Invalid robot IP address: %s", robot_ip_.c_str());
            close(udp_socket_fd_); return false;
        }
        RCLCPP_INFO(this->get_logger(), "UDP socket created for robot at %s:%d", robot_ip_.c_str(), ROBOT_UDP_PORT);
        return true;
    }

    rclcpp::Client<motoros2_interfaces::srv::StartRtMode>::SharedPtr client_;
    std::string robot_ip_;
    double speed_limit_mps_;
    double rot_speed_rps_;
    
    uint32_t sequence_id_;
    int udp_socket_fd_;
    struct sockaddr_in robot_addr_;
    
    std::thread keyboard_thread_;
    std::thread control_thread_;
    std::atomic<bool> running_;

    KeyPressTimestamps key_timestamps_;

    struct termios oldt_, newt_;
};

int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardRtController>();
    if (node->initialize()) {
        rclcpp::spin(node);
    }
    return 0;
}