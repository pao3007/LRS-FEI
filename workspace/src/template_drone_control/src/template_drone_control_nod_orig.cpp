#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <unistd.h>
#include <fcntl.h>
#include <cstdio>
#include <cstdlib>
#include <list>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <math.h>
#include <tuple>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <Python.h>

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
        local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("mavros/setpoint_position/local", 10);

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
        RCLCPP_INFO(this->get_logger(), "COMMAND FINISHED press: \n q > to quit \n t > to takeoff \n l > to land \n m > to move drone to specified position \n n > to navigate auto.");
        while (!terminate) {
            if (isCharAvailable()) {
                char c;
                read(STDIN_FILENO, &c, 1);
                action_menu(c, false);
                RCLCPP_INFO(this->get_logger(), "COMMAND FINISHED press: \n q > to quit \n t > to takeoff \n l > to land \n m > to move drone to specified position \n n > to navigate auto.");
            } 
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(20ms);
        }
    }

private:
    void action_menu(char c, bool block)
    {
        if (c == 'l') {
            RCLCPP_INFO(this->get_logger(), "Sending land command");
            send_land_request(0.0);
        } else if (c == 't') {
            RCLCPP_INFO(this->get_logger(), "Sending takeoff command");
            send_takeoff_request(1.0);
        } else if (c == 'q') {
            terminate = true;
            rclcpp::shutdown();
        } else if (c == 'm') {
            send_manual_coords();
        } else if (c == 'n' && !block) {
            create_points();
            move_to_position();
        } 
    }

    void send_manual_coords()
    {
        double x, y, z;

        std::cout << "Enter X coordinate: ";
        while(!(std::cin >> x)){
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }

        std::cout << "Enter Y coordinate: ";
        while(!(std::cin >> y)){
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }

        std::cout << "Enter Z coordinate: ";
        while(!(std::cin >> z)){
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }

        std::cout << "You entered the coordinates: (" << x << ", " << y << ", " << z << ")" << std::endl;
        send_position_command(x, y, z, hard_precision);
    }

    void move_to_position()
    {
        RCLCPP_INFO(this->get_logger(), "Start moving to desired position");
        for (const auto& point : list_of_points) {
            RCLCPP_INFO(this->get_logger(), "Moving to the new position");
            float precision;
            if (point.precision ==  "soft"){
                precision = soft_precision;
            }else {
                precision = hard_precision;
            }
            send_position_command(point.x, point.y, point.z, precision);
            if (point.action == "land")
            {
                send_land_request(0);
            }
            else if (point.action == "landtakeoff")
            {
                land_takeoff();
            }
            else if (point.action == "yaw180")
            {
                rotate_drone_action(180, soft_precision);
            }
            else if (point.action == "yaw90")
            {
                rotate_drone_action(90, soft_precision);
            }
        }
    }

    void land_takeoff()
    {
        send_land_request(0);
        while(armed_check){
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(20ms);
        }
        my_wait(25);
        send_takeoff_request(1.0);
    }

    std::tuple<float, float> transform_global_to_local(float x, float y)
    {
        float local_x, local_y;
        local_x = -(start_pose.x - x);
        local_y = start_pose.y - y;
        RCLCPP_INFO(this->get_logger(), "Moving to the local position %f, %f", local_x, local_y);
        return std::make_tuple(local_y, local_x); 
    }

    void make_list_of_circle_points(float droneX, float droneY, float radius, int numPoints) {
    Point point;
    list_of_circle_points.clear();
    for (int i = 0; i < numPoints; ++i) {
        float angle = 2 * M_PI * i / numPoints;
        float x = droneX + radius * cos(angle);
        float y = droneY + radius * sin(angle);
        point.x = x;
        point.y = y;
        list_of_circle_points.push_back(point);
    }
    point.x = droneX;
    point.y = droneY;
    list_of_circle_points.push_back(list_of_circle_points.front());
    list_of_circle_points.push_back(point);
    }

    void action_do_circle()
    {
        float max_e = 0.2;
        std::list<Point> waypoints;
        make_list_of_circle_points(current_local_pos_.pose.position.x, current_local_pos_.pose.position.y, 1, 8);
        geometry_msgs::msg::PoseStamped msg;
        double target_yaw;
        tf2::Quaternion q;
        float e_x;
        float e_y;
        float z_current = current_local_pos_.pose.position.z;
        int i = 0;
        for (const auto& waypoint : list_of_circle_points)
        {
            target_yaw = atan2(waypoint.y - current_local_pos_.pose.position.y, waypoint.x - current_local_pos_.pose.position.x);
            q.setRPY(0, 0, target_yaw);
            msg.pose.position.z = z_current;
            msg.pose.orientation.x = q.x();
            msg.pose.orientation.y = q.y();
            msg.pose.orientation.z = q.z();
            msg.pose.orientation.w = q.w();
            msg.header.stamp = this->now();
            msg.header.frame_id = "map";

            e_x = abs(current_local_pos_.pose.position.x - waypoint.x);
            e_y = abs(current_local_pos_.pose.position.y - waypoint.y);
            tf2::Quaternion q_current(
            current_local_pos_.pose.orientation.x,
            current_local_pos_.pose.orientation.y,
            current_local_pos_.pose.orientation.z,
            current_local_pos_.pose.orientation.w
            );
            double roll, pitch, yaw;

            tf2::Matrix3x3 m(q_current);
            m.getRPY(roll, pitch, yaw);
            float current_yaw = yaw;
            float yaw_err = abs(current_yaw - target_yaw);
            if (i == 0 || i == 1) {
                msg.pose.position.x = current_local_pos_.pose.position.x;
                msg.pose.position.y = current_local_pos_.pose.position.y;
                while(yaw_err > 0.2){
                    q_current.setX(current_local_pos_.pose.orientation.x);
                    q_current.setY(current_local_pos_.pose.orientation.y);
                    q_current.setZ(current_local_pos_.pose.orientation.z);
                    q_current.setW(current_local_pos_.pose.orientation.w);

                    m.setRotation(q_current);
                    m.getRPY(roll, pitch, yaw);
                    current_yaw = yaw;
                    yaw_err = abs(current_yaw - target_yaw);
                    local_pos_pub_->publish(msg);
                    rclcpp::spin_some(this->get_node_base_interface());
                    std::this_thread::sleep_for(20ms);
                    RCLCPP_INFO(this->get_logger(), "Error %f", yaw_err);

                }
                i++;
            }
            msg.pose.position.x = waypoint.x;
            msg.pose.position.y = waypoint.y;
            while (
                (e_x > max_e) || (e_y > max_e)
            )
            {
                local_pos_pub_->publish(msg);
                rclcpp::spin_some(this->get_node_base_interface());
                std::this_thread::sleep_for(20ms);
                e_x = abs(current_local_pos_.pose.position.x - waypoint.x);
                e_y = abs(current_local_pos_.pose.position.y - waypoint.y);
            }
        }

    }

    void send_position_command(float x, float y, float z, float max_e)
    {
        if (!armed_check){
            send_arming_request(true);
        }
        if (!takeoff_check && global_pose.z < hard_precision){
            send_takeoff_request(1.0);
        }

        tf2::Quaternion q = rotate_drone(x, y, global_pose.x, global_pose.y, 0.3, true);

        std::tie(x, y) = transform_global_to_local(x, y);
        RCLCPP_INFO(this->get_logger(), "Start moving to the point %f, %f, %f", x, y, z);
        float e_x, e_z, e_y;
        e_x = abs(current_local_pos_.pose.position.x - x);
        e_z = abs(current_local_pos_.pose.position.z - z);
        e_y = abs(current_local_pos_.pose.position.y - y);

        geometry_msgs::msg::PoseStamped msg;
        geometry_msgs::msg::PoseStamped stop;
        geometry_msgs::msg::PoseStamped position;
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";
        msg.pose.position.x = x;
        msg.pose.position.y = y;
        msg.pose.position.z = z;
        msg.pose.orientation.x = q.x();
        msg.pose.orientation.y = q.y();
        msg.pose.orientation.z = q.z();
        msg.pose.orientation.w = q.w();
        std::string info_msg = "COMMAND FINISHED press :\n s > to stop \n c > to continue \n t > to takeoff \n l > to land \n m > to move drone to specified position \n k > to kruh kruh.";

        RCLCPP_INFO(this->get_logger(), info_msg);
        bool continue_drone = true;
        position = msg;
        while (
            (e_x > max_e) || (e_z > max_e) || (e_y > max_e)
        )
        {
            if (isCharAvailable()) {
                char c;
                read(STDIN_FILENO, &c, 1);
                if (!continue_drone)
                {
                    action_menu(c, true);
                    if (c == 'k')
                    {
                        RCLCPP_INFO(this->get_logger(), "START DOING CIRCLE");
                        action_do_circle();
                        RCLCPP_INFO(this->get_logger(), info_msg);
                     }
                    RCLCPP_INFO(this->get_logger(), info_msg);
                }
                if (c == 'c') {
                    continue_drone = true;
                    RCLCPP_INFO(this->get_logger(), "RESUMING DRONE NAVIGATION");
                    position = msg;
                }
                else if (c == 's')
                {
                    stop.pose.position.x = current_local_pos_.pose.position.x;
                    stop.pose.position.y = current_local_pos_.pose.position.y;
                    stop.pose.position.z = current_local_pos_.pose.position.z;
                    stop.pose.orientation.x = msg.pose.orientation.x;
                    stop.pose.orientation.y = msg.pose.orientation.y;
                    stop.pose.orientation.z = msg.pose.orientation.z;
                    stop.pose.orientation.w = msg.pose.orientation.w;
                    continue_drone = false;
                    RCLCPP_INFO(this->get_logger(), "STOPPING DRONE NAVIGATION");
                    RCLCPP_INFO(this->get_logger(), info_msg);
                    position = stop;
                }               
                
            } 
            e_x = abs(current_local_pos_.pose.position.x - x);
            e_z = abs(current_local_pos_.pose.position.z - z);
            e_y = abs(current_local_pos_.pose.position.y - y);

            local_pos_pub_->publish(position);
            rclcpp::spin_some(this->get_node_base_interface());


            std::this_thread::sleep_for(20ms);
        }
    }

    void rotate_drone_action(float yaw_degree, float precision) {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "map";  // set your frame_id
        float yaw_radian = yaw_degree * (M_PI / 180.0);
        tf2::Quaternion q_current(
            current_local_pos_.pose.orientation.x,
            current_local_pos_.pose.orientation.y,
            current_local_pos_.pose.orientation.z,
            current_local_pos_.pose.orientation.w
        );
        double roll, pitch, yaw;
        tf2::Matrix3x3 m(q_current);
        m.getRPY(roll, pitch, yaw);
        float current_yaw = yaw;
        float target_yaw = current_yaw + yaw_radian;
        if (target_yaw > M_PI){
            target_yaw -= M_PI*2;
        }else if (target_yaw < -M_PI)
        {
            target_yaw += M_PI*2;
        }
        tf2::Quaternion q;  
        q.setRPY(0, 0, target_yaw);
        msg.pose.orientation.x = q.x();
        msg.pose.orientation.y = q.y();
        msg.pose.orientation.z = q.z();
        msg.pose.orientation.w = q.w();
        msg.pose.position.x = current_local_pos_.pose.position.x;
        msg.pose.position.y = current_local_pos_.pose.position.y;
        msg.pose.position.z = current_local_pos_.pose.position.z;
        float yaw_err = abs(current_yaw - target_yaw);
        while(yaw_err > precision)
        {
            q_current.setX(current_local_pos_.pose.orientation.x);
            q_current.setY(current_local_pos_.pose.orientation.y);
            q_current.setZ(current_local_pos_.pose.orientation.z);
            q_current.setW(current_local_pos_.pose.orientation.w);

            m.setRotation(q_current);
            m.getRPY(roll, pitch, yaw);
            current_yaw = yaw;
            yaw_err = abs(current_yaw - target_yaw);

            local_pos_pub_->publish(msg);
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(20ms);
        }

        my_wait(100);
        
    }

    tf2::Quaternion rotate_drone(float target_x, float target_y, float current_x, float current_y, float precision, bool rotate) {
        double target_yaw = atan2(target_y - current_y, target_x - current_x) ;
        //RCLCPP_INFO(this->get_logger(), "Target yaw %f", target_yaw);
        target_yaw += start_pose.yaw;        
        //RCLCPP_INFO(this->get_logger(), "Target yaw %f", target_yaw);
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "map";  // set your frame_id

        tf2::Quaternion q;  
        q.setRPY(0, 0, target_yaw);
        msg.pose.orientation.x = q.x();
        msg.pose.orientation.y = q.y();
        msg.pose.orientation.z = q.z();
        msg.pose.orientation.w = q.w();
        msg.pose.position.x = current_local_pos_.pose.position.x;
        msg.pose.position.y = current_local_pos_.pose.position.y;
        msg.pose.position.z = current_local_pos_.pose.position.z;

        tf2::Quaternion q_current(
            current_local_pos_.pose.orientation.x,
            current_local_pos_.pose.orientation.y,
            current_local_pos_.pose.orientation.z,
            current_local_pos_.pose.orientation.w
            );

        double roll, pitch, yaw;
        tf2::Matrix3x3 m(q_current);
        m.getRPY(roll, pitch, yaw);
        float current_yaw = yaw;
        if (target_yaw > M_PI){
            target_yaw -= M_PI*2;
        }else if (target_yaw < -M_PI)
        {
            target_yaw += M_PI*2;
        }

        float yaw_err = abs(current_yaw - target_yaw);
        //RCLCPP_INFO(this->get_logger(), "Target yaw %f - %f = %f", current_yaw, target_yaw, yaw_err);
        //while(((target_yaw-precision) < current_yaw) && (current_yaw < (target_yaw+precision)))
        if (rotate)
        {
            while(yaw_err > precision)
            {
                //RCLCPP_INFO(this->get_logger(), "Target yaw %f - %f = %f", current_yaw, target_yaw, yaw_err);
                q_current.setX(current_local_pos_.pose.orientation.x);
                q_current.setY(current_local_pos_.pose.orientation.y);
                q_current.setZ(current_local_pos_.pose.orientation.z);
                q_current.setW(current_local_pos_.pose.orientation.w);

                m.setRotation(q_current);
                m.getRPY(roll, pitch, yaw);
                current_yaw = yaw;
                yaw_err = abs(current_yaw - target_yaw);

                local_pos_pub_->publish(msg);
                rclcpp::spin_some(this->get_node_base_interface());
                std::this_thread::sleep_for(20ms);
            }
        }
        return q;
    }

    void my_wait(int slp){
        int i = 0;
        while(i < slp)
        {
            i++;
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(20ms);
        }
    }

    void local_pos_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        pob_each++;
        current_local_pos_ = *msg;
        global_pose.x = start_pose.x + current_local_pos_.pose.position.y;
        global_pose.y = start_pose.y - current_local_pos_.pose.position.x;
        global_pose.z = start_pose.z + current_local_pos_.pose.position.z;
        tf2::Quaternion q_current(
            current_local_pos_.pose.orientation.x,
            current_local_pos_.pose.orientation.y,
            current_local_pos_.pose.orientation.z,
            current_local_pos_.pose.orientation.w
            );

        double roll, pitch, yaw;
        tf2::Matrix3x3 m(q_current);
        m.getRPY(roll, pitch, yaw);
        yaw = yaw - start_pose.yaw;
        if (pob_each > 10)
        {
            //RCLCPP_INFO(this->get_logger(), "Current Local Position: %f, %f, %f, %f", global_pose.x, global_pose.y, global_pose.z, yaw);
            pob_each = 0;
        }

    }

    void state_cb(const mavros_msgs::msg::State::SharedPtr msg)
    {
        current_state_ = *msg;
        state_ = current_state_.mode.c_str();
        // RCLCPP_INFO(this->get_logger(), "Current State: %s", current_state_.mode.c_str());

        guided_mode_set = false;
        takeoff_check = false;
        land_check = false;

        if (state_ == "GUIDED"){
            guided_mode_set = true;
            takeoff_check = false;
            land_check = false;
        }
        else if (state_ == "TAKEOFF")
        {
            takeoff_check = true;
            guided_mode_set = false;
            land_check = false;
        }
        else if (state_ == "LAND")
        {
            land_check = true;
            guided_mode_set = false;
            takeoff_check = false;
        }
        armed_check = msg->armed;
    }

    void set_mode(std::string mode){
        RCLCPP_INFO(this->get_logger(), "SEND SET MODE REQUEST");
        mavros_msgs::srv::SetMode::Request guided_set_mode_req;
        guided_set_mode_req.custom_mode = mode;
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
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(20ms);
        }
        RCLCPP_INFO(this->get_logger(), "Drone is set to ");
    }
   
    void send_arming_request(bool arm)
    {
        RCLCPP_INFO(this->get_logger(), "SEND ARMING REQUEST");
        if (!guided_mode_set){
            set_mode("GUIDED");
        }
        
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
                my_wait(50);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Arming request denied. Result code: %d", static_cast<int>(result_future.get()->result));
            }
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service mavros/cmd/arming");
        }

        while (!armed_check){
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(20ms);
        }
        RCLCPP_INFO(this->get_logger(), "Drone is armed");
    }

    void send_land_request(float alt)
    {    
        RCLCPP_INFO(this->get_logger(), "SEND LAND REQUEST");
        auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
        request->altitude = alt; // Takeoff to 10 meters, for example.

        while (!arming_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Service client interrupted while waiting for service to appear.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for mavros/cmd/arming service to be available...");
        }

        auto result_future = land_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == 
            rclcpp::FutureReturnCode::SUCCESS)
        {
            bool success = result_future.get()->success;
            if (success) {
                RCLCPP_INFO(this->get_logger(), "Successfully sent land request!");
                while (global_pose.z > 0.15)
                {
                    rclcpp::spin_some(this->get_node_base_interface());
                    std::this_thread::sleep_for(20ms);
                }
                my_wait(10);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Landing request denied. Result code: %d", static_cast<int>(result_future.get()->result));
            }
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service mavros/cmd/arming");
        }
    }

    void send_takeoff_request(float alt)
    {    
        RCLCPP_INFO(this->get_logger(), "SEND TAKEOFF REQUEST");
        if (!armed_check){
            send_arming_request(true);
        }

        auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
        request->altitude = alt; // Takeoff to 10 meters, for example.
    
        while (!arming_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Service client interrupted while waiting for service to appear.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for mavros/cmd/arming service to be available...");
        }

        auto result_future = takeoff_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == 
            rclcpp::FutureReturnCode::SUCCESS)
        {
            bool success = result_future.get()->success;
            if (success) {
                RCLCPP_INFO(this->get_logger(), "Successfully sent takeoff request!");
                while (global_pose.z < alt/2)
                {
                    rclcpp::spin_some(this->get_node_base_interface());
                    std::this_thread::sleep_for(20ms);
                    //err = abs(global_pose.z - alt);
                }
                my_wait(50);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Takeoff request denied. Result code: %d", static_cast<int>(result_future.get()->result));
            }
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service mavros/cmd/arming");
        }
    }
    // tu nacitavam body do listu
    void create_points(){
        /*list_of_points.push_back({13.599, 2.49, 1.0, "", "soft"});
        list_of_points.push_back({12.599, 2.49, 1.0, "yaw180", "soft"});
        list_of_points.push_back({12.599, 1.49, 1.0, "", "soft"});
        list_of_points.push_back({13.599, 1.49, 1.0, "land", "soft"});*/
        //Py_Initialize();
        //PyRun_SimpleString("exec(open('./script.py').read())");
        //Py_Finalize();
        load_points_from_csv("/home/lrs-ubuntu/Desktop/LRS/LRS-FEI/workspace/src/template_drone_control/src/mission_1_all.csv");
        
    }
    void load_points_from_csv(const std::string& file_path) {
        std::ifstream file(file_path);
        
        if (!file.is_open()) {
            std::cerr << "Error opening file: " << file_path << std::endl;
            return;
        }
    
        std::string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string token;
            Point point;
            int tokenIndex = 0;
    
            while (std::getline(iss, token, ',')) {
    
                switch (tokenIndex) {
                    case 0: point.x = std::stod(token); break;
                    case 1: point.y = std::stod(token); break;
                    case 2: point.z = std::stod(token); break;
                    //case 3: point.action = token; break;
                    //case 4: point.precision = token; break;
                    case 3: point.precision = token; break;
                    case 4: point.action = token; break;
                    default: break; // Extra tokens, could be an error if not expected
                }
                ++tokenIndex;
            }
    
            if (tokenIndex != 5) {
                std::cerr << "Error: Incorrect number of values in line: " << line << std::endl;
                continue; // Skip malformed line
            }
    
            list_of_points.push_back(point);
        }
        
        file.close();
    }

    bool isCharAvailable() {
        fd_set rfds;
        struct timeval tv;

        FD_ZERO(&rfds);
        FD_SET(STDIN_FILENO, &rfds);

        tv.tv_sec = 0;
        tv.tv_usec = 0;

        int retval = select(1, &rfds, NULL, NULL, &tv);

        return retval > 0;
    }
    
    struct Point {
    double x;
    double y;
    double z;
    std::string action;
    std::string precision;
    };
    struct GlobalPose {
        double x;
        double y;
        double z;
        double yaw;
    };
    struct StartPose {
        double x = 13.599900;
        double y = 1.494780;
        double z = 0.003019;
        double yaw = 1.57;
    };
    GlobalPose global_pose;
    StartPose start_pose;
    std::list<Point> list_of_points;
    std::list<Point> list_of_circle_points;
    geometry_msgs::msg::PoseStamped current_local_pos_;
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
    bool armed_check;
    bool land_check;
    bool takeoff_check;
    float hard_precision = 0.02;
    float soft_precision = 0.05;
    char c;
    bool terminate = false;
    float current_yaw;
    int pob_each = 0;
    std::array<double, 6> positionCoefficientsX;
    std::array<double, 6> positionCoefficientsY;
    std::array<double, 6> positionCoefficientsZ;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TemplateDroneControl>());
    rclcpp::shutdown();
    return 0;
}
