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
#include <sys/time.h> // For timeval

#pragma pack(push, 1)

typedef enum
{
    PacketType_Joint_Increments = 0,
    PacketType_Cart_Increments
} PacketType;

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

    TCP_Rx,         //0.0001 degrees
    TCP_Ry,
    TCP_Rz,
    TCP_Re,

    TCP_8,          //pulse

    MAX_AXES //maxies
} CartesianIndeces;

typedef struct
{
    int drives_powered;
    int e_stopped;
    int in_motion;
    int play_mode;
    int motion_possible;
    int error;
    int error_code;
} RobotState;

#define MP_GRP_AXES_NUM 8

//##########################################################################
//                  !All data is little-endian!
//##########################################################################

struct RtPacket
{
    //The version of the command packet must match the value expected
    //by MotoROS2.
    int version;

    //The packet type must match the control_mode which was specified
    //in when invoking the start_rt_mode service.
    PacketType packetType;

    //Must increment sequentially with each new command packet.
    unsigned int sequenceId;
    
    //The order of the joints must be in the order of [S L U R B T E 8].
    //Please note that for seven axis robots, the 'E' joint is phyically
    //mounted in the middle of the arm. But it must be sent at the end
    //of the joint array. See JointIndices enum.
    //
    //For joint-space, this will be radians of each joint.
    //
    //For cartesian, this will be meters and radians of the TCP.
    //The order of the joints must be in the order of [X Y Z Rx Ry Rz Re 8].
    //See CartesianIndices enum.
    //Rotations are applied in the order of ZYX.
    double delta[MAX_GROUPS][MP_GRP_AXES_NUM];
    
    //Set tool that will be used by motion API (ie: passed by us to mpExRcsIncrementMove(..))
    //NOTE: this will change the 'motion tool' ONLY for those increments which
    //      haven't yet been added to the increment queue. See also the ROS 2
    //      'select_tool' service definition file in motoros2_interfaces.
    int toolIndex[MAX_GROUPS]; //TOOL 0 - 63

    //Reserved for future expansion
    char reserved[64];

} ;


//##########################################################################
//                  !All data is little-endian!
//##########################################################################

struct RtReply
{
    unsigned int sequenceEcho;

    //Essentially a clone of the /robot_status topic. But decoupled
    //from the industrial_msgs/RobotStatus type.
    RobotState state;

    //This is indicative of where the robot is physically located.
    //Please note that this will trail behind the commanded position.
    //The joint ordering will match that of the original command
    //packet. See JointIndices and CartesianIndices enums.
    double feedbackPositionJoints[MAX_GROUPS][MP_GRP_AXES_NUM];
    double feedbackPositionCartesian[MAX_GROUPS][MP_GRP_AXES_NUM];

    //The command position is the target destination you are instructing
    //the robot to reach. It's the calculated endpoint based on the sum
    //of all position increments received from the user.
    //
    //This is used to track if the robot's speed is being limited
    //by the Functional Safety Unit (FSU). It can also be used to
    //monitor the latency between command and feedback.
    double previousCommandPositionJoints[MAX_GROUPS][MP_GRP_AXES_NUM];
    double previousCommandPositionCartesian[MAX_GROUPS][MP_GRP_AXES_NUM];

    //If the FSU speed limit is enabled, it can truncate the commanded
    //delta increments. This flag is an indicator that the *previous*
    //command cycle was truncated. It does NOT indicate that this most
    //recent command packet was truncated.
    bool fsuInterferenceDetected;
} ;
#pragma pack(pop)

// Constants
constexpr int CONTROL_INTERVAL_MS = 4;
constexpr double CONTROL_INTERVAL_S = CONTROL_INTERVAL_MS / 1000.0;
constexpr double MAX_JOYSTICK_AXIS_VALUE = 32767.0;
const int MAX_JOYSTICK_AXES = 8;
const int MAX_JOYSTICK_BUTTONS = 12; // Max buttons to consider
const int ROBOT_UDP_PORT = 22000; // Standard RT motion port
const int ROBOT_STATE_UDP_PORT = 22001; // Port for RobotState broadcasts

class JoystickRtController : public rclcpp::Node 
{
public:
    JoystickRtController() : Node("rt_joystick_controller_node"), sequence_id_(0), joystick_fd_(-1), udp_socket_fd_(-1), state_udp_socket_fd_(-1), running_(true), trigger_pressed_(false)
    {
        this->declare_parameter<std::string>("robot_ip", "192.168.1.31");
        
        // NEW: Parameters for Cartesian control
        this->declare_parameter<std::string>("joystick_device", "/dev/input/js0");
        this->declare_parameter<double>("speed_limit_s", 1);
        this->declare_parameter<double>("speed_limit_l", 1);
        this->declare_parameter<double>("speed_limit_u", 1);
        this->declare_parameter<double>("speed_limit_r", 1);
        this->declare_parameter<double>("speed_limit_b", 1);
        this->declare_parameter<double>("speed_limit_t", 1);

        // NEW: Joystick axis and button mapping parameters
        this->declare_parameter<int>("axis_s_r", 1); // Fwd/Back on left stick
        this->declare_parameter<int>("axis_l_b", 0); // Left/Right on left stick
        this->declare_parameter<int>("axis_u_t", 5); // Up/Down on D-pad/hat
        this->declare_parameter<int>("button_trigger", 0); // Main trigger button

        // Read parameters
        robot_ip_ = this->get_parameter("robot_ip").as_string();
        joystick_device_ = this->get_parameter("joystick_device").as_string();
        speed_limit_s_ = this->get_parameter("speed_limit_s").as_double();
        speed_limit_l_ = this->get_parameter("speed_limit_l").as_double();
        speed_limit_u_ = this->get_parameter("speed_limit_u").as_double();
        speed_limit_r_ = this->get_parameter("speed_limit_r").as_double();
        speed_limit_b_ = this->get_parameter("speed_limit_b").as_double();
        speed_limit_t_ = this->get_parameter("speed_limit_t").as_double();
        axis_s_r_ = this->get_parameter("axis_s_r").as_int();
        axis_l_b_ = this->get_parameter("axis_l_b").as_int();
        axis_u_t_ = this->get_parameter("axis_u_t").as_int();
        button_trigger_ = this->get_parameter("button_trigger").as_int();
        
        // Initialize atomic state arrays 
        for(auto& state : axis_states_) { state.store(0); }
    }

    ~JoystickRtController() 
    {
        running_ = false;
        if (joystick_thread_.joinable()) {
            joystick_thread_.join();
        }
        if (control_thread_.joinable()) {
            control_thread_.join();
        }
        if (state_listener_thread_.joinable())
        {
            state_listener_thread_.join();
        }

        if (joystick_fd_ != -1) { close(joystick_fd_); }
        if (udp_socket_fd_ != -1) { close(udp_socket_fd_); }
        if (state_udp_socket_fd_ != -1)
        {
            close(state_udp_socket_fd_);
        }
        RCLCPP_INFO(this->get_logger(), "Resources cleaned up. Shutting down.");
    }

    bool initialize() 
    {
        auto reset_client = this->create_client<motoros2_interfaces::srv::ResetError>("reset_error");
        RCLCPP_INFO(this->get_logger(), "Waiting for 'reset_error' service...");
        if (!reset_client->wait_for_service(std::chrono::seconds(5))) {
             RCLCPP_ERROR(this->get_logger(), "Service 'reset_error' not available. Exiting.");
             rclcpp::shutdown();
             return false;
        }
        auto reset_request = std::make_shared<motoros2_interfaces::srv::ResetError::Request>();
        auto reset_result_future = reset_client->async_send_request(reset_request);
        RCLCPP_INFO(this->get_logger(), "Calling ResetError service...");
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), reset_result_future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service reset_error");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Successfully reset errors.");

        auto stop_traj_client = this->create_client<std_srvs::srv::Trigger>("stop_traj_mode");
        RCLCPP_INFO(this->get_logger(), "Waiting for 'stop_traj_mode' service...");
        if (!stop_traj_client->wait_for_service(std::chrono::seconds(5))) {
             RCLCPP_ERROR(this->get_logger(), "Service 'stop_traj_mode' not available. Exiting.");
             return false;
        }
        auto stop_traj_request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto stop_traj_future = stop_traj_client->async_send_request(stop_traj_request);
        RCLCPP_INFO(this->get_logger(), "Calling StopTrajMode service...");
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), stop_traj_future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service stop_traj_mode");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Successfully stopped trajectory mode.");

        client_ = this->create_client<motoros2_interfaces::srv::StartRtMode>("start_rt_mode");
        RCLCPP_INFO(this->get_logger(), "Waiting for 'start_rt_mode' service...");
        if (!client_->wait_for_service(std::chrono::seconds(5))) {
             RCLCPP_ERROR(this->get_logger(), "Service 'start_rt_mode' not available. Exiting.");
             rclcpp::shutdown();
             return false;
        }

        auto request = std::make_shared<motoros2_interfaces::srv::StartRtMode::Request>();
        request->control_mode.value = motoros2_interfaces::msg::ControlModeEnum::JOINT_ANGLES;
        auto result_future = client_->async_send_request(request);

        RCLCPP_INFO(this->get_logger(), "Calling StartRtMode service...");
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service start_rt_mode");
            return false;
        }

        auto start_rt_mode_result = result_future.get();
        if (start_rt_mode_result->result_code.value != motoros2_interfaces::msg::MotionReadyEnum::READY)
        {
            RCLCPP_ERROR(this->get_logger(), "start_rt_mode returned code %d: %s", start_rt_mode_result->result_code.value, start_rt_mode_result->message.c_str());
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Successfully started real-time mode.");
        
        if (!setup_joystick() || !setup_udp_socket() || !setup_state_udp_socket())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to setup hardware or network. Shutting down.");
            return false;
        }

        joystick_thread_ = std::thread(&JoystickRtController::joystick_poll_thread, this);
        control_thread_ = std::thread(&JoystickRtController::control_loop_thread, this);
        state_listener_thread_ = std::thread(&JoystickRtController::state_listener_loop_thread, this);
        return true;
    }

private:
    void state_listener_loop_thread()
    {
        RCLCPP_INFO(this->get_logger(), "State listener thread started.");
        
        while (running_)
        {
            RobotState incoming_state{};
            struct sockaddr_in sender_addr;
            socklen_t addr_len = sizeof(sender_addr);
            
            ssize_t bytes_read = recvfrom(state_udp_socket_fd_, &incoming_state, sizeof(incoming_state), 0, (struct sockaddr*)&sender_addr, &addr_len);
            
            if (bytes_read == sizeof(RobotState))
            {
                RCLCPP_INFO(this->get_logger(), "in_motion = %d", incoming_state.in_motion);
                RCLCPP_INFO(this->get_logger(), "e_stopped = %d", incoming_state.e_stopped);
                RCLCPP_INFO(this->get_logger(), "drives_powered = %d", incoming_state.drives_powered);
                RCLCPP_INFO(this->get_logger(), "play_mode = %d", incoming_state.play_mode);
                RCLCPP_INFO(this->get_logger(), "motion_possible = %d", incoming_state.motion_possible);
                RCLCPP_INFO(this->get_logger(), "error = %d", incoming_state.error);
                RCLCPP_INFO(this->get_logger(), "error_code = %d", incoming_state.error_code);
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "State listener thread stopped.");
    }

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

    void control_loop_thread() 
    {
        RCLCPP_INFO(this->get_logger(), "Control loop thread started.");
        while(running_)
        {
            RtPacket packet{};

            packet.version = 1;
            packet.packetType = PacketType_Joint_Increments;
            packet.toolIndex[0] = 0;

            packet.sequenceId = sequence_id_++;
            
            memset(packet.delta, 0x00, sizeof(packet.delta));

            // Get current joystick state
            double s_r_val = static_cast<double>(axis_states_[axis_s_r_].load());
            double l_b_val = static_cast<double>(axis_states_[axis_l_b_].load());
            double u_t_val = static_cast<double>(axis_states_[axis_u_t_].load());
            bool SLU = !trigger_pressed_.load();

            if (SLU)
            {
                packet.delta[0][0] = speed_limit_s_ * (s_r_val / MAX_JOYSTICK_AXIS_VALUE) * CONTROL_INTERVAL_S;
                packet.delta[0][1] = speed_limit_l_ * (l_b_val / MAX_JOYSTICK_AXIS_VALUE) * CONTROL_INTERVAL_S;
                packet.delta[0][2] = speed_limit_u_ * (u_t_val / MAX_JOYSTICK_AXIS_VALUE) * CONTROL_INTERVAL_S;
            }
            else
            {
                packet.delta[0][3] = speed_limit_r_ * (s_r_val / MAX_JOYSTICK_AXIS_VALUE) * CONTROL_INTERVAL_S;
                packet.delta[0][4] = speed_limit_b_ * (l_b_val / MAX_JOYSTICK_AXIS_VALUE) * CONTROL_INTERVAL_S;
                packet.delta[0][5] = speed_limit_t_ * (u_t_val / MAX_JOYSTICK_AXIS_VALUE) * CONTROL_INTERVAL_S;
            }
            // Send UDP Packet
            sendto(udp_socket_fd_, &packet, sizeof(packet), 0, (struct sockaddr*)&robot_addr_, sizeof(robot_addr_));

            // Listen for Reply
            RtReply reply{};
            socklen_t addr_len = sizeof(robot_addr_);
            recvfrom(udp_socket_fd_, &reply, sizeof(reply), 0, (struct sockaddr*)&robot_addr_, &addr_len);


            // Error/mismatch check would go here
            if (reply.fsuInterferenceDetected)
                RCLCPP_ERROR(this->get_logger(), "You are being slowed down");
        }
        RCLCPP_INFO(this->get_logger(), "Control loop thread stopped.");
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

    bool setup_state_udp_socket()
    {
        state_udp_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (state_udp_socket_fd_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to create state UDP socket.");
            return false;
        }

        // Add timeout so recvfrom doesn't block infinitely during shutdown
        //struct timeval tv;
        //tv.tv_sec = 0;
        //tv.tv_usec = 100000; // 100ms
        //setsockopt(state_udp_socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        struct sockaddr_in bind_addr;
        memset(&bind_addr, 0, sizeof(bind_addr));
        bind_addr.sin_family = AF_INET;
        bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        bind_addr.sin_port = htons(ROBOT_STATE_UDP_PORT);

        if (bind(state_udp_socket_fd_, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind state UDP socket on port %d.", ROBOT_STATE_UDP_PORT);
            close(state_udp_socket_fd_);
            state_udp_socket_fd_ = -1;
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "State UDP socket listening on port %d", ROBOT_STATE_UDP_PORT);
        return true;
    }

    // ROS2 Members
    rclcpp::Client<motoros2_interfaces::srv::StartRtMode>::SharedPtr client_;

    // Configuration Members
    std::string robot_ip_;
    std::string joystick_device_;
    double speed_limit_s_, speed_limit_l_, speed_limit_u_, speed_limit_r_, speed_limit_b_, speed_limit_t_;
    
    int axis_s_r_, axis_l_b_, axis_u_t_, button_trigger_;
    
    // State & Networking Members
    uint32_t sequence_id_;
    int joystick_fd_;
    int udp_socket_fd_;
    int state_udp_socket_fd_;
    struct sockaddr_in robot_addr_;
    
    // Threading Members
    std::thread joystick_thread_;
    std::thread control_thread_;
    std::thread state_listener_thread_;
    std::atomic<bool> running_;
    std::atomic<bool> trigger_pressed_;
    std::array<std::atomic<int16_t>, MAX_JOYSTICK_AXES> axis_states_;
};

int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoystickRtController>();
    if(node->initialize())
        rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}