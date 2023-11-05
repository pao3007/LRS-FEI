#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include<unistd.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>

using namespace std::chrono_literals;

class TemplateDroneControl : public rclcpp::Node
{
public:
    TemplateDroneControl() : Node("template_drone_control_node")
    {
        // Set up ROS publishers, subscribers and service clients
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "mavros/state", 10, std::bind(&TemplateDroneControl::state_cb, this, std::placeholders::_1));
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
        takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/takeoff");
        land_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/land");

        rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
        custom_qos.depth = 1;
        custom_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(custom_qos.history, 1), custom_qos);
        local_pos_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", qos, std::bind(&TemplateDroneControl::local_pos_cb, this, std::placeholders::_1));

        // Wait for MAVROS SITL connection
        while (rclcpp::ok() && !current_state_.connected)
        {
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(100ms);
        }

        mavros_msgs::srv::SetMode::Request guided_set_mode_req;
        guided_set_mode_req.custom_mode = "GUIDED";
        while (!set_mode_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the set_mode service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for set_mode service...");
        }

        auto result = set_mode_client_->async_send_request(std::make_shared<mavros_msgs::srv::SetMode::Request>(guided_set_mode_req));

        while (!guided_mode_set){
            sleep(0.5);
            RCLCPP_INFO(this->get_logger(), "Waiting for guided mode");
        }
        RCLCPP_INFO(this->get_logger(), "GUIDED mode set");
        sleep(0.5);
        send_arming_request(true);
        RCLCPP_INFO(this->get_logger(), "Arming set");
        sleep(0.5);
   
        RCLCPP_INFO(this->get_logger(), "Takeoff set");
        send_takeoff_request(1.0);

        RCLCPP_INFO(this->get_logger(), "Sending position command");

        while (!terminate) {
            c = getch(); // Non-blocking key press detection

            if (c == 'a') {
                RCLCPP_INFO(this->get_logger(), "Sending position command");
            } else if (c == 'q') {
                terminate = true;
        }
    }   
    }

private:
    char getch() 
    {
        char buf = 0;
        struct termios old;
        tcgetattr(0, &old);
        if (tcgetattr(0, &old) < 0)
            perror("tcsetattr()");
        old.c_lflag &= ~ICANON;
        old.c_lflag &= ~ECHO;
        old.c_cc[VMIN] = 1;
        old.c_cc[VTIME] = 0;
        if (tcsetattr(0, TCSANOW, &old) < 0)
            perror("tcsetattr ICANON");
        if (read(0, &buf, 1) < 0)
            perror("read()");
        old.c_lflag |= ICANON;
        old.c_lflag |= ECHO;
        if (tcsetattr(0, TCSADRAIN, &old) < 0)
            perror("tcsetattr ~ICANON");
        return buf;
    }

    void local_pos_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        geometry_msgs::msg::PoseStamped current_local_pos_ = *msg;

        // To obtain the position of the drone use this data fields withing the message, please note, that this is the local position of the drone in the NED frame so it is different to the map frame
        // current_local_pos_.pose.position.x
        // current_local_pos_.pose.position.y
        // current_local_pos_.pose.position.z
        // you can do the same for orientation, but you will not need it for this seminar


        RCLCPP_INFO(this->get_logger(), "Current Local Position: %f, %f, %f", current_local_pos_.pose.position.x, current_local_pos_.pose.position.y, current_local_pos_.pose.position.z);
    }

    void state_cb(const mavros_msgs::msg::State::SharedPtr msg)
    {
        current_state_ = *msg;
        state_ = current_state_.mode.c_str();
        RCLCPP_INFO(this->get_logger(), "Current State: %s", current_state_.mode.c_str());
        if (state_ == "GUIDED"){
            guided_mode_set = true;
        }
    }

    void send_arming_request(bool arm)
    {
        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = arm;

        while (!arming_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Service client interrupted while waiting for service to appear.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for mavros/cmd/arming service to be available...");
        }

        auto result_future = arming_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == 
            rclcpp::FutureReturnCode::SUCCESS)
        {
            bool success = result_future.get()->success;
            if (success) {
                RCLCPP_INFO(this->get_logger(), "Successfully sent arming request!");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Arming request denied. Result code: %d", static_cast<int>(result_future.get()->result));
            }
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service mavros/cmd/arming");
        }
    }

    void send_takeoff_request(float altitude)
    {    
      
        auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
        request->altitude = altitude; // Takeoff to 10 meters, for example.
        auto future = takeoff_client_->async_send_request(request);
    
        // Use future to wait for the result.
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::executor::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "Takeoff command sent successfully.");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to send takeoff command: %s", response->result_str.c_str());
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service mavros/cmd/takeoff");
        }
    }


    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_sub_;
    mavros_msgs::msg::State current_state_;
    std::string state_;
    bool guided_mode_set;
    char c;
    bool terminate = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TemplateDroneControl>());
    rclcpp::shutdown();
    return 0;
}
