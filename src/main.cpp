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
// REMOVED: fcntl.h and linux/joystick.h are no longer needed
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>

// The packet structure now sends 6 cartesian increments.
// X, Y, Z in meters; Rx, Ry, Rz in radians.
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

    MAX_AXES //maxies
} CartesianIndeces;

#define MP_GRP_AXES_NUM 8

//##########################################################################
//                  !All data is little-endian!
//##########################################################################

struct RtPacket
{
    unsigned int sequenceId;
    
    //The order of the joints must be in the order of [S L U R B T E 8].
    //Please note that for seven axis robots, the 'E' joint is phyically
    //mounted in the middle of the arm. But it must be sent at the end
    //of the joint array. See JointIndeces enum.
    //
    //For joint-space, this will be radians of each joint.
    //
    //For cartesian, this will be meters and radians of the TCP.
    //The order of the joints must be in the order of [X Y Z Rx Ry Rz Re 8].
    //See CartesianIndeces enum.
    //Rotations are applied in the order of ZYX.
    double delta[MAX_GROUPS][MP_GRP_AXES_NUM];
    
    //Set tool that will be used by motion API (ie: passed by us to mpExRcsIncrementMove(..))
    //NOTE: this will change the 'motion tool' ONLY for those increments which
    //      haven't yet been added to the increment queue. See also the ROS 2
    //      'select_tool' service definition file in motoros2_interfaces.
    int toolIndex[MAX_GROUPS]; //TOOL 0 - 63

} ;


//##########################################################################
//                  !All data is little-endian!
//##########################################################################

struct RtReply
{
    unsigned int sequenceEcho;

    //This is indicative of where the robot is physically located.
    //Please note that this will trail behind the commanded position.
    //The joint ordering will match that of the original command
    //packet. See JointIndeces and CartesianIndeces enums.
    double feedbackPositionJoints[MAX_GROUPS][MP_GRP_AXES_NUM];
    double feedbackPositionCartesian[MAX_GROUPS][MP_GRP_AXES_NUM];

    //The command position is the target destination you are instructing
    //the robot to reach. It's the calculated endpoint based on the sum
    //of all position increments received from the user.
    //
    //This does NOT include the commanded delta from the most recent
    //command packet.
    //
    //This is used to track if the robot's speed is being limited
    //by the Functional Safety Unit (FSU). It can also be used to
    //monitor the latency between command and feedback.
    double previousCommandPositionJoints[MAX_GROUPS][MP_GRP_AXES_NUM];
    double previousCommandPositionCartesian[MAX_GROUPS][MP_GRP_AXES_NUM];

    bool fsuInterferenceDetected;
} ;
#pragma pack(pop)

// Constants
constexpr int CONTROL_INTERVAL_MS = 4;
constexpr double CONTROL_INTERVAL_S = CONTROL_INTERVAL_MS / 1000.0;
const int ROBOT_UDP_PORT = 8889; // Standard RT motion port

// CHANGED: Renamed class to reflect its new purpose
class FixedRotationController : public rclcpp::Node 
{
public:
    FixedRotationController() : Node("fixed_rotation_controller_node"), sequence_id_(0), udp_socket_fd_(-1), running_(true)
    {
        this->declare_parameter<std::string>("robot_ip", "192.168.1.31");
        robot_ip_ = this->get_parameter("robot_ip").as_string();
    }

    ~FixedRotationController() 
    {
        running_ = false;
        // REMOVED: joystick_thread_ is gone
        if (control_thread_.joinable()) {
            control_thread_.join();
        }

        // REMOVED: joystick_fd_ is gone
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
        
        // REMOVED: call to setup_joystick()
        if (!setup_udp_socket()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to setup hardware. Shutting down.");
            rclcpp::shutdown();
            return;
        }

        // REMOVED: joystick_thread_ creation
        control_thread_ = std::thread(&FixedRotationController::control_loop_thread, this);
    }

private:
    // REMOVED: The joystick_poll_thread is no longer needed.

    // CHANGED: This thread now sends a pre-defined motion instead of reading from a joystick.
    void control_loop_thread() 
    {
        RCLCPP_INFO(this->get_logger(), "Control loop thread started. Executing fixed rotation...");

        // --- Define the motion ---
        const double total_angle_deg = 45.0; // The total rotation angle in degrees
        const double duration_s = 2.0;       // The time over which to perform the rotation

        // --- Calculate motion parameters ---
        const double total_angle_rad = total_angle_deg * (M_PI / 180.0);
        const int num_steps = static_cast<int>(duration_s / CONTROL_INTERVAL_S);
        const double rotation_per_step_rad = total_angle_rad / num_steps;

        RCLCPP_INFO(this->get_logger(), "Rotating %.1f deg over %.1f s in %d steps.", total_angle_deg, duration_s, num_steps);

        for (int i = 0; i < num_steps && running_; ++i)
        {
            RtPacket packet{};
            packet.sequenceId = sequence_id_++;
            
            // Initialize all deltas to zero
            memset(packet.delta, 0x00, sizeof(packet.delta));

            // Set the rotation for the X-axis (Roll)
            // The index for TCP_Rx is 3 as per the CartesianIndeces enum.
            packet.delta[Group_1][TCP_Rx] = rotation_per_step_rad;
            
            // Send UDP Packet
            sendto(udp_socket_fd_, &packet, sizeof(packet), 0, (struct sockaddr*)&robot_addr_, sizeof(robot_addr_));

            // Listen for Reply
            RtReply reply{};
            socklen_t addr_len = sizeof(robot_addr_);
            recvfrom(udp_socket_fd_, &reply, sizeof(reply), 0, (struct sockaddr*)&robot_addr_, &addr_len);

            // Error/mismatch check would go here
            if (reply.fsuInterferenceDetected)
            {
                RCLCPP_WARN(this->get_logger(), "FSU interference detected. Robot may be slowed down.");
            }

            // Wait for the next control interval
            std::this_thread::sleep_for(std::chrono::milliseconds(CONTROL_INTERVAL_MS));
        }
        
        // --- Motion Complete ---
        if (running_)
        {
            RCLCPP_INFO(this->get_logger(), "Fixed rotation complete. Shutting down.");
            running_ = false;
            rclcpp::shutdown(); // Request shutdown of the ROS 2 node
        }

        RCLCPP_INFO(this->get_logger(), "Control loop thread stopped.");
    }

    // REMOVED: The setup_joystick function is no longer needed.

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

    // Configuration Members
    std::string robot_ip_;
    
    // State & Networking Members
    uint32_t sequence_id_;
    int udp_socket_fd_;
    struct sockaddr_in robot_addr_;
    
    // Threading Members
    std::thread control_thread_;
    std::atomic<bool> running_;
};

int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    // CHANGED: Instantiated the new class
    auto node = std::make_shared<FixedRotationController>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}