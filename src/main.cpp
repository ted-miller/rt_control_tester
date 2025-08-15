#include <rclcpp/rclcpp.hpp>
#include <motoros2_interfaces/srv/start_rt_mode.hpp>

#include <chrono>
#include <cstdint>
#include <fcntl.h>
#include <linux/joystick.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <memory>
#include <array>
#include <thread>   // Added for std::thread
#include <atomic>   // Added for std::atomic

// Define the UDP packet structures based on RealTimeMotionControl.h
#pragma pack(push, 1)
struct RtPacket 
{
    uint32_t sequenceId;
    double deltaRad[8][8];
};

struct RtReply 
{
    uint32_t sequenceId;
    double feedbackPosition[8][8];
};
#pragma pack(pop)

// Constants
constexpr double DEGREES_PER_SECOND = 60.0;
constexpr double RADIANS_PER_SECOND = DEGREES_PER_SECOND * (M_PI / 180.0);
constexpr int CONTROL_INTERVAL_MS = 4;
constexpr double CONTROL_INTERVAL_S = CONTROL_INTERVAL_MS / 1000.0;
constexpr double MAX_JOYSTICK_AXIS_VALUE = 32767.0;
const int JOYSTICK_AXIS_ID_S = 0; // Left/Right on most gamepads
const int JOYSTICK_AXIS_ID_L = 1; // Up/Down on most gamepads
const int MAX_JOYSTICK_AXES = 8;
const char* JOYSTICK_DEVICE_PATH = "/dev/input/js0";
const int ROBOT_UDP_PORT = 8889;

class JoystickRtController : public rclcpp::Node 
{
public:
    JoystickRtController() : Node("rt_joystick_controller_node"), sequence_id_(0), joystick_fd_(-1), udp_socket_fd_(-1), running_(true) 
    {
        this->declare_parameter<std::string>("robot_ip", "192.168.1.31");
        robot_ip_ = this->get_parameter("robot_ip").as_string();
        client_ = this->create_client<motoros2_interfaces::srv::StartRtMode>("start_rt_mode");
        
        // Initialize the atomic axis states array
        for(auto& state : axis_states_) {
            state.store(0);
        }
    }

    ~JoystickRtController() 
    {
        // *** UPDATED: Cleanly shut down the joystick thread. ***
        running_ = false;
        if (joystick_thread_.joinable()) {
            joystick_thread_.join();
        }

        if (joystick_fd_ != -1) {
            close(joystick_fd_);
        }
        if (udp_socket_fd_ != -1) {
            close(udp_socket_fd_);
        }
        RCLCPP_INFO(this->get_logger(), "Resources cleaned up. Shutting down.");
    }

    void initialize() 
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for 'start_rt_mode' service...");
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Client interrupted while waiting for service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }

        auto request = std::make_shared<motoros2_interfaces::srv::StartRtMode::Request>();
        auto result_future = client_->async_send_request(request);

        RCLCPP_INFO(this->get_logger(), "Calling StartRtMode service...");
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service start_rt_mode");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Successfully started real-time mode.");
        if (!setup_joystick() || !setup_udp_socket()) {
            return;
        }

        // *** NEW: Start the joystick polling thread. ***
        joystick_thread_ = std::thread(&JoystickRtController::joystick_poll_thread, this);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(CONTROL_INTERVAL_MS),
            std::bind(&JoystickRtController::control_loop, this));
    }

private:
    // *** NEW METHOD: This function runs in its own thread. ***
    void joystick_poll_thread()
    {
        RCLCPP_INFO(this->get_logger(), "Joystick polling thread started.");
        js_event event;
        while(running_)
        {
            // Read all available events from the queue
            while (read(joystick_fd_, &event, sizeof(event)) > 0)
            {
                if (event.type & JS_EVENT_AXIS)
                {
                    if (event.number < MAX_JOYSTICK_AXES)
                    {
                        // Atomically store the new value
                        axis_states_[event.number].store(event.value);
                    }
                }
            }
            // Add a small delay to prevent this thread from consuming 100% CPU
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        RCLCPP_INFO(this->get_logger(), "Joystick polling thread stopped.");
    }

    // *** UPDATED: The control loop is now much simpler. ***
    void control_loop() 
    {
        // 1. Prepare Packet (No joystick reading here; it's done in the background)
        RtPacket packet{};
        packet.sequenceId = sequence_id_++;
        
        memset(packet.deltaRad, 0x00, sizeof(packet.deltaRad));

        // Atomically load the latest values from the state array
        double velocity_scale_s = static_cast<double>(axis_states_[JOYSTICK_AXIS_ID_S].load()) / MAX_JOYSTICK_AXIS_VALUE;
        packet.deltaRad[0][0] = -static_cast<float>(RADIANS_PER_SECOND * velocity_scale_s * CONTROL_INTERVAL_S);

        double velocity_scale_l = static_cast<double>(axis_states_[JOYSTICK_AXIS_ID_L].load()) / MAX_JOYSTICK_AXIS_VALUE;
        packet.deltaRad[0][1] = -static_cast<float>(RADIANS_PER_SECOND * velocity_scale_l * CONTROL_INTERVAL_S);
        
        // 2. Send UDP Packet
        sendto(udp_socket_fd_, &packet, sizeof(packet), 0, (struct sockaddr*)&robot_addr_, sizeof(robot_addr_));

        // 3. Listen for Reply
        RtReply reply{};
        socklen_t addr_len = sizeof(robot_addr_);
        recvfrom(udp_socket_fd_, &reply, sizeof(reply), 0, (struct sockaddr*)&robot_addr_, &addr_len);
        // Error/mismatch check omitted for brevity
    }

    bool setup_joystick()
    {
        joystick_fd_ = open(JOYSTICK_DEVICE_PATH, O_RDONLY | O_NONBLOCK);
        if (joystick_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open joystick at %s", JOYSTICK_DEVICE_PATH);
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Joystick opened successfully.");
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

    rclcpp::Client<motoros2_interfaces::srv::StartRtMode>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;

    uint32_t sequence_id_;
    int joystick_fd_;
    int udp_socket_fd_;
    struct sockaddr_in robot_addr_;
    std::string robot_ip_;

    // *** NEW/MODIFIED MEMBERS for threading ***
    std::thread joystick_thread_;
    std::atomic<bool> running_;
    // Use std::atomic for thread-safe access to the joystick state
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