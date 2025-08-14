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
};
#pragma pack(pop)

// Constants
constexpr double DEGREES_PER_SECOND = 30.0;
constexpr double RADIANS_PER_SECOND = DEGREES_PER_SECOND * (M_PI / 180.0);
constexpr int CONTROL_INTERVAL_MS = 4;
constexpr double CONTROL_INTERVAL_S = CONTROL_INTERVAL_MS / 1000.0;
constexpr double MAX_JOYSTICK_AXIS = 32767.0;
const int JOYSTICK_AXIS_ID = 0; // Left/Right on most gamepads
const char* JOYSTICK_DEVICE_PATH = "/dev/input/js0";
const int ROBOT_UDP_PORT = 8889;

class JoystickRtController : public rclcpp::Node 
{
public:
    JoystickRtController() : Node("rt_joystick_controller_node"), sequence_id_(0), joystick_fd_(-1), udp_socket_fd_(-1) 
    {
        // Declare robot_ip parameter with a default value
        this->declare_parameter<std::string>("robot_ip", "192.168.1.31");
        robot_ip_ = this->get_parameter("robot_ip").as_string();

        client_ = this->create_client<motoros2_interfaces::srv::StartRtMode>("start_rt_mode");
    }

    ~JoystickRtController() 
    {
        if (joystick_fd_ != -1) 
        {
            close(joystick_fd_);
        }
        if (udp_socket_fd_ != -1) 
        {
            close(udp_socket_fd_);
        }
        RCLCPP_INFO(this->get_logger(), "Resources cleaned up. Shutting down.");
    }

    void initialize() 
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for 'start_rt_mode' service...");
        while (!client_->wait_for_service(std::chrono::seconds(1))) 
        {
            if (!rclcpp::ok()) 
            {
                RCLCPP_ERROR(this->get_logger(), "Client interrupted while waiting for service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }

        auto request = std::make_shared<motoros2_interfaces::srv::StartRtMode::Request>();
        auto result_future = client_->async_send_request(request);

        RCLCPP_INFO(this->get_logger(), "Calling StartRtMode service...");
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS) 
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service start_rt_mode");
            return;
        }

        auto result = result_future.get();

        RCLCPP_INFO(this->get_logger(), "Successfully started real-time mode.");

        // Setup Joystick and UDP
        if (!setup_joystick() || !setup_udp_socket()) 
        {
            return;
        }

        // Start the main control loop timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(CONTROL_INTERVAL_MS),
            std::bind(&JoystickRtController::control_loop, this));
    }

private:
    void control_loop() 
    {
        // 1. Read Joystick
        js_event event;
        if (read(joystick_fd_, &event, sizeof(event)) > 0) 
        {
            if (event.type == JS_EVENT_AXIS && event.number == JOYSTICK_AXIS_ID) 
            {
                joystick_axis_value_ = event.value;
            }
        } // We don't block; use the last known value if no new event

        // 2. Prepare Packet
        RtPacket packet{};
        packet.sequenceId = sequence_id_++;
        
        memset(packet.deltaRad, 0x00, sizeof(packet.deltaRad));

        // Calculate incremental motion for axis[0]
        double velocity_scale = static_cast<double>(joystick_axis_value_) / MAX_JOYSTICK_AXIS;
        packet.deltaRad[0][0] = static_cast<float>(RADIANS_PER_SECOND * velocity_scale * CONTROL_INTERVAL_S);
        
        RCLCPP_WARN(this->get_logger(), "Seq: %u, Joy: %d, dRad: %f", packet.sequenceId, joystick_axis_value_, packet.deltaRad[0][0]);

        // 3. Send UDP Packet
        sendto(udp_socket_fd_, &packet, sizeof(packet), 0, (struct sockaddr*)&robot_addr_, sizeof(robot_addr_));

        // 4. Listen for Reply
        RtReply reply{};
        socklen_t addr_len = sizeof(robot_addr_);
        ssize_t len = recvfrom(udp_socket_fd_, &reply, sizeof(reply), 0, (struct sockaddr*)&robot_addr_, &addr_len);

        if (len > 0) 
        {
            if (reply.sequenceId != packet.sequenceId) 
            {
                RCLCPP_WARN(this->get_logger(), "Received mismatched sequence ID. Got: %u, Expected: %u", reply.sequenceId, packet.sequenceId);
            }
        } 
        else 
        {
            RCLCPP_WARN(this->get_logger(), "Failed to receive UDP reply from robot.");
        }
    }

    bool setup_joystick()
    {
        joystick_fd_ = open(JOYSTICK_DEVICE_PATH, O_RDONLY | O_NONBLOCK);
        if (joystick_fd_ < 0) 
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open joystick at %s", JOYSTICK_DEVICE_PATH);
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Joystick opened successfully.");
        return true;
    }

    bool setup_udp_socket() 
    {
        udp_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_socket_fd_ < 0) 
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to create UDP socket.");
            return false;
        }

        // Set a receive timeout
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 2000; // 2ms timeout
        //setsockopt(udp_socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));


        memset(&robot_addr_, 0, sizeof(robot_addr_));
        robot_addr_.sin_family = AF_INET;
        robot_addr_.sin_port = htons(ROBOT_UDP_PORT);
        if (inet_pton(AF_INET, robot_ip_.c_str(), &robot_addr_.sin_addr) <= 0) 
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid robot IP address: %s", robot_ip_.c_str());
            close(udp_socket_fd_);
            udp_socket_fd_ = -1;
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "UDP socket created for robot at %s:%d", robot_ip_.c_str(), ROBOT_UDP_PORT);
        return true;
    }

    rclcpp::Client<motoros2_interfaces::srv::StartRtMode>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;

    uint32_t sequence_id_;
    int joystick_fd_;
    int16_t joystick_axis_value_ = 0;

    int udp_socket_fd_;
    struct sockaddr_in robot_addr_;
    std::string robot_ip_;
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