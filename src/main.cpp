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
#include <linux/joystick.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>

// CHANGED: The packet structure now sends 6 cartesian increments.
// X, Y, Z in meters; Rx, Ry, Rz in radians.
#pragma pack(push, 1)
struct RtPacket 
{
    uint32_t sequenceId;
    double delta[8][8];
};

struct RtReply 
{
    uint32_t sequenceId;
    double feedbackPosition[8][8];
};
#pragma pack(pop)

// Constants
constexpr int CONTROL_INTERVAL_MS = 4;
constexpr double CONTROL_INTERVAL_S = CONTROL_INTERVAL_MS / 1000.0;
constexpr double MAX_JOYSTICK_AXIS_VALUE = 32767.0;
const int MAX_JOYSTICK_AXES = 8;
const int MAX_JOYSTICK_BUTTONS = 12; // Max buttons to consider
const int ROBOT_UDP_PORT = 8889; // Standard RT motion port

class JoystickRtController : public rclcpp::Node 
{
public:
    JoystickRtController() : Node("rt_joystick_controller_node"), sequence_id_(0), joystick_fd_(-1), udp_socket_fd_(-1), running_(true), trigger_pressed_(false)
    {
        this->declare_parameter<std::string>("robot_ip", "192.168.1.31");
        
        // NEW: Parameters for Cartesian control
        this->declare_parameter<std::string>("joystick_device", "/dev/input/js0");
        this->declare_parameter<double>("speed_limit_mps", 1000); // 200 mm/sec
        this->declare_parameter<double>("rot_speed_limit_dps", 60.0); // 60 deg/sec
        
        // NEW: Joystick axis and button mapping parameters
        this->declare_parameter<int>("axis_x", 1); // Fwd/Back on left stick
        this->declare_parameter<int>("axis_y", 0); // Left/Right on left stick
        this->declare_parameter<int>("axis_z_dpad", 7); // Up/Down on D-pad/hat
        this->declare_parameter<int>("button_trigger", 0); // Main trigger button

        // Read parameters
        robot_ip_ = this->get_parameter("robot_ip").as_string();
        joystick_device_ = this->get_parameter("joystick_device").as_string();
        speed_limit_mps_ = this->get_parameter("speed_limit_mps").as_double();
        axis_x_ = this->get_parameter("axis_x").as_int();
        axis_y_ = this->get_parameter("axis_y").as_int();
        axis_z_dpad_ = this->get_parameter("axis_z_dpad").as_int();
        button_trigger_ = this->get_parameter("button_trigger").as_int();

        double rot_speed_dps = this->get_parameter("rot_speed_limit_dps").as_double();
        rot_speed_rps_ = rot_speed_dps * (M_PI / 180.0);
        
        // Initialize atomic state arrays
        for(auto& state : axis_states_) { state.store(0); }
    }

    ~JoystickRtController() 
    {
        running_ = false;
        if (joystick_thread_.joinable()) {
            joystick_thread_.join();
        }

        if (joystick_fd_ != -1) { close(joystick_fd_); }
        if (udp_socket_fd_ != -1) { close(udp_socket_fd_); }
        RCLCPP_INFO(this->get_logger(), "Resources cleaned up. Shutting down.");
    }

    void initialize() 
    {
        auto reset_client = this->create_client<motoros2_interfaces::srv::ResetError>("reset_error");
        RCLCPP_INFO(this->get_logger(), "Waiting for 'reset_error' service...");
        if (!reset_client->wait_for_service(std::chrono::seconds(5))) {
             RCLCPP_ERROR(this->get_logger(), "Service 'reset_error' not available. Exiting.");
             rclcpp::shutdown();
             return;
        }
        auto reset_request = std::make_shared<motoros2_interfaces::srv::ResetError::Request>();
        auto reset_result_future = reset_client->async_send_request(reset_request);
        RCLCPP_INFO(this->get_logger(), "Calling ResetError service...");
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), reset_result_future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service reset_error");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Successfully reset errors.");

        auto stop_traj_client = this->create_client<std_srvs::srv::Trigger>("stop_traj_mode");
        RCLCPP_INFO(this->get_logger(), "Waiting for 'stop_traj_mode' service...");
        if (!stop_traj_client->wait_for_service(std::chrono::seconds(5))) {
             RCLCPP_ERROR(this->get_logger(), "Service 'stop_traj_mode' not available. Exiting.");
             rclcpp::shutdown();
             return;
        }
        auto stop_traj_request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto stop_traj_future = stop_traj_client->async_send_request(stop_traj_request);
        RCLCPP_INFO(this->get_logger(), "Calling StopTrajMode service...");
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), stop_traj_future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service stop_traj_mode");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Successfully stopped trajectory mode.");

        client_ = this->create_client<motoros2_interfaces::srv::StartRtMode>("start_rt_mode");
        RCLCPP_INFO(this->get_logger(), "Waiting for 'start_rt_mode' service...");
        if (!client_->wait_for_service(std::chrono::seconds(5))) {
             RCLCPP_ERROR(this->get_logger(), "Service 'start_rt_mode' not available. Exiting.");
             rclcpp::shutdown();
             return;
        }

        auto request = std::make_shared<motoros2_interfaces::srv::StartRtMode::Request>();
        request->control_mode.value = motoros2_interfaces::msg::ControlModeEnum::CARTESIAN;
        auto result_future = client_->async_send_request(request);

        RCLCPP_INFO(this->get_logger(), "Calling StartRtMode service...");
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service start_rt_mode");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Successfully started real-time mode.");
        if (!setup_joystick() || !setup_udp_socket()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to setup hardware. Shutting down.");
            rclcpp::shutdown();
            return;
        }

        joystick_thread_ = std::thread(&JoystickRtController::joystick_poll_thread, this);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(CONTROL_INTERVAL_MS),
            std::bind(&JoystickRtController::control_loop, this));
    }

private:
    // UPDATED: This thread now also reads button events.
    void joystick_poll_thread()
    {
        RCLCPP_INFO(this->get_logger(), "Joystick polling thread started.");
        js_event event;
        while(running_)
        {
            if (read(joystick_fd_, &event, sizeof(event)) > 0)
            {
                if (event.type & JS_EVENT_AXIS)
                {
                    if (event.number < MAX_JOYSTICK_AXES)
                    {
                        axis_states_[event.number].store(event.value);
                    }
                }
                else if (event.type & JS_EVENT_BUTTON)
                {
                    if (event.number == button_trigger_)
                    {
                        trigger_pressed_.store(event.value != 0);
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        RCLCPP_INFO(this->get_logger(), "Joystick polling thread stopped.");
    }

    // UPDATED: Control loop logic for Cartesian translation and rotation.
    void control_loop() 
    {
        RtPacket packet{};
        packet.sequenceId = sequence_id_++;
        
        memset(packet.delta, 0x00, sizeof(packet.delta));

        // Get current joystick state
        double x_axis_val = static_cast<double>(axis_states_[axis_x_].load());
        double y_axis_val = static_cast<double>(axis_states_[axis_y_].load());
        double z_dpad_val = static_cast<double>(axis_states_[axis_z_dpad_].load());
        bool is_rotating = trigger_pressed_.load();

        if (is_rotating)
        {
            // --- ROTATION MODE ---
            // Fwd/Back stick -> Pitch (rotation around Y)
            packet.delta[0][4] = rot_speed_rps_ * (x_axis_val / MAX_JOYSTICK_AXIS_VALUE) * CONTROL_INTERVAL_S;
            // Left/Right stick -> Roll (rotation around X)
            packet.delta[0][3] = rot_speed_rps_ * (y_axis_val / MAX_JOYSTICK_AXIS_VALUE) * CONTROL_INTERVAL_S;
            // D-Pad Up/Down -> Yaw (rotation around Z)
            packet.delta[0][5] = rot_speed_rps_ * (z_dpad_val / MAX_JOYSTICK_AXIS_VALUE) * CONTROL_INTERVAL_S;
        }
        else
        {
            // --- TRANSLATION MODE ---
            // Fwd/Back stick -> +/- X
            packet.delta[0][0] = speed_limit_mps_ * (x_axis_val / MAX_JOYSTICK_AXIS_VALUE) * CONTROL_INTERVAL_S;
            // Left/Right stick -> +/- Y
            packet.delta[0][1] = speed_limit_mps_ * (y_axis_val / MAX_JOYSTICK_AXIS_VALUE) * CONTROL_INTERVAL_S;
            // D-Pad Up/Down -> +/- Z
            packet.delta[0][2] = speed_limit_mps_ * (z_dpad_val / MAX_JOYSTICK_AXIS_VALUE) * CONTROL_INTERVAL_S;

            //RCLCPP_INFO(this->get_logger(), "packet.delta[0][0] = %.5f", packet.delta[0][0]);
        }
        
        // Send UDP Packet
        sendto(udp_socket_fd_, &packet, sizeof(packet), 0, (struct sockaddr*)&robot_addr_, sizeof(robot_addr_));

        // Listen for Reply
        RtReply reply{};
        socklen_t addr_len = sizeof(robot_addr_);
        recvfrom(udp_socket_fd_, &reply, sizeof(reply), 0, (struct sockaddr*)&robot_addr_, &addr_len);
        // Error/mismatch check would go here
    }

    bool setup_joystick()
    {
        joystick_fd_ = open(joystick_device_.c_str(), O_RDONLY | O_NONBLOCK);
        if (joystick_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open joystick at %s", joystick_device_.c_str());
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Joystick '%s' opened successfully.", joystick_device_.c_str());
        return true;
    }

    bool setup_udp_socket() 
    {
        udp_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_socket_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create UDP socket.");
            return false;
        }

        memset(&robot_addr_, 0, sizeof(robot_addr_));
        robot_addr_.sin_family = AF_INET;
        robot_addr_.sin_port = htons(ROBOT_UDP_PORT);
        if (inet_pton(AF_INET, robot_ip_.c_str(), &robot_addr_.sin_addr) <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Invalid robot IP address: %s", robot_ip_.c_str());
            close(udp_socket_fd_);
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "UDP socket created for robot at %s:%d", robot_ip_.c_str(), ROBOT_UDP_PORT);
        return true;
    }

    // ROS2 Members
    rclcpp::Client<motoros2_interfaces::srv::StartRtMode>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Configuration Members
    std::string robot_ip_;
    std::string joystick_device_;
    double speed_limit_mps_;
    double rot_speed_rps_;
    int axis_x_, axis_y_, axis_z_dpad_, button_trigger_;
    
    // State & Networking Members
    uint32_t sequence_id_;
    int joystick_fd_;
    int udp_socket_fd_;
    struct sockaddr_in robot_addr_;
    
    // Threading Members
    std::thread joystick_thread_;
    std::atomic<bool> running_;
    std::atomic<bool> trigger_pressed_;
    std::array<std::atomic<int16_t>, MAX_JOYSTICK_AXES> axis_states_;
};

int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoystickRtController>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}