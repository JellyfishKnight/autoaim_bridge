// created by liuhan on 2023/11/18
// Submodule of HeliosCV
/*
 * ██   ██ ███████ ██      ██  ██████  ███████
 * ██   ██ ██      ██      ██ ██    ██ ██
 * ███████ █████   ██      ██ ██    ██ ███████
 * ██   ██ ██      ██      ██ ██    ██      ██
 * ██   ██ ███████ ███████ ██  ██████  ███████
 *
 */


#include "AutoaimBridge.hpp"
#include "Packets.hpp"
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_client.hpp>


namespace helios_cv {

AutoaimBridge::AutoaimBridge(const rclcpp::NodeOptions &options) : rclcpp::Node("autoaim_bridge", options) {
    // create params
    try {
        param_listener_ = std::make_shared<ParamsListener>(this->get_node_parameters_interface());
        params_ = param_listener_->get_params();
    } catch (const std::exception &e) {
        RCLCPP_ERROR(logger_, "on_init: %s", e.what());
        exit(0);
    }
    // create serial
    serial_port_ = std::make_shared<serial::Serial>();
    if (!serial_port_) {
        RCLCPP_ERROR(logger_, "Unable to create a serial");
        exit(0);
    }
    try {
        serial_port_->setPort(params_.serial_name);
        serial_port_->setBaudrate(params_.serial_baudrate);
        serial_port_->setFlowcontrol(serial::flowcontrol_none);
        serial_port_->setParity(serial::parity_none); // default is parity_none
        serial_port_->setStopbits(serial::stopbits_one);
        serial_port_->setBytesize(serial::eightbits);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(params_.serial_timeout);
        serial_port_->setTimeout(timeout);
    } catch (serial::SerialException& e) {
        RCLCPP_ERROR(logger_, "throwed an exception while declare a serial : %s", e.what());
        exit(0);
    }
    try {
        serial_port_->open();
    } catch (serial::IOException& e) {
        RCLCPP_ERROR(logger_, "Got exception while open port: \n %s", e.what());
        exit(0);
    }
    if (!serial_port_->isOpen()) {
        RCLCPP_ERROR(logger_, "Unable to open serial");
        exit(0);
    }
    RCLCPP_INFO(logger_, "Successfully activated!");
    // create publisher
    recv_pub_ = this->create_publisher<autoaim_interfaces::msg::ReceiveData>(
        params_.serial_topic_name, rclcpp::SystemDefaultsQoS()
    );
    realtime_recv_pub_ = std::make_shared<realtime_tools::RealtimePublisher<autoaim_interfaces::msg::ReceiveData>>(
        recv_pub_
    );
    static_pub_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
    dynamic_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    // create subcriber
    target_sub_ = this->create_subscription<autoaim_interfaces::msg::Target>(
        params_.target_topic_name, rclcpp::SystemDefaultsQoS(),
        std::bind(&AutoaimBridge::send_callback, this, std::placeholders::_1)
    );
    // Detect parameter client
    detector_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_detector");
    predictor_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_predictor");
    // Reset predictor client
    reset_predictor_client_ = this->create_client<std_srvs::srv::Trigger>("/predictor/reset");
    // set buffers to zero
    std::memset(read_buffer_, 0, sizeof(read_buffer_));
    std::memset(write_buffer_, 0, sizeof(write_buffer_));
    // start to read serial
    std::thread([this]()->void {
        RCLCPP_INFO_ONCE(logger_, "Start to read serial");
        while(rclcpp::ok()) {
            receive_loop();
        }
    }).detach();

}

void AutoaimBridge::receive_loop() {
    auto time = this->now();
    // update parameters if they have changed
    if (param_listener_->is_old(params_)) {
        params_ = param_listener_->get_params();
        RCLCPP_INFO(logger_, "Parameters were updated");
    }
    serial_port_->read(read_buffer_, 1);
    while (read_buffer_[0] != 0x5A && rclcpp::ok()) {
        serial_port_->read(read_buffer_, 1);
    }
    if (read_buffer_[0] == 0x5A) {
        serial_port_->read(read_buffer_ + 1, 18);
        if (ReceivePacket::verify_check_sum(read_buffer_) && read_buffer_[18] == 0x6A) {
            ReceivePacket::convert_read_buffer_to_recv_packet(read_buffer_, recv_packet_);
        } else {
            RCLCPP_INFO(logger_, "packed end: %x", read_buffer_[18]);
            RCLCPP_WARN(logger_, "Checksum Failed");
        }
    }
    // publish gimbal states
    if (realtime_recv_pub_->trylock()) {
        auto & state_msg = realtime_recv_pub_->msg_;
        geometry_msgs::msg::TransformStamped transform_stamped;
        state_msg.header.frame_id = params_.frame_id;
        state_msg.header.stamp = time;
        state_msg.yaw = recv_packet_.yaw;
        state_msg.pitch = recv_packet_.pitch;
        // RCLCPP_INFO(logger_, "yaw: %f", recv_packet_.yaw);
        // RCLCPP_INFO(logger_, "pitch: %f", recv_packet_.pitch);
        state_msg.bullet_speed = recv_packet_.bullet_speed;
        state_msg.target_color = recv_packet_.target_color;
        state_msg.autoaim_mode = recv_packet_.autoaim_mode;
        realtime_recv_pub_->unlockAndPublish();
        // transform from imu to yaw
        transform_stamped.header.frame_id = "odom";
        transform_stamped.child_frame_id = "gimbal_link";
        transform_stamped.header.stamp = time;
        tf2::Quaternion q;
        q.setRPY(0, -recv_packet_.pitch * M_PI / 180.0, recv_packet_.yaw * M_PI / 180.0);
        transform_stamped.transform.rotation = tf2::toMsg(q);
        dynamic_pub_->sendTransform(transform_stamped);
    }
    // check if need to change state
    check_and_set_param();
    // check if need to reset predictor
    if (recv_packet_.reset_predictor == 1 || !reset_predictor_flag_) {
        reset_predictor_flag_ = false;
        if (!reset_predictor_client_->service_is_ready()) {
            RCLCPP_WARN(get_logger(), "Service not ready, skipping tracker reset");
            return;
        }
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        reset_predictor_client_->async_send_request(request);
        RCLCPP_INFO(get_logger(), "Reset tracker!");
        reset_predictor_flag_ = true;
    }
}

void AutoaimBridge::send_callback(autoaim_interfaces::msg::Target::SharedPtr msg) {
    if (msg->tracking) {
        send_packet_.tracking = 1;
    } else {
        send_packet_.tracking = 0;
    }
    if (msg->id[0] <= '9' && msg->id[0] >= '0') {
        send_packet_.id = msg->id[0] - '0';
    } else if (msg->id[0] == 'g') {
        send_packet_.id = 7;
    } else if (msg->id[0] == 'o') {
        send_packet_.id = 8;
    } else if (msg->id[0] == 'b') {
        send_packet_.id = 9;
    } else {
        RCLCPP_WARN_ONCE(logger_, "unknown id! %s",  msg->id.c_str());
    }
    send_packet_.armors_num = msg->armors_num;
    send_packet_.x = msg->position.x;
    send_packet_.y = msg->position.y;
    send_packet_.z = msg->position.z;
    send_packet_.yaw = msg->yaw;
    send_packet_.vx = msg->velocity.x;
    send_packet_.vy = msg->velocity.y;
    send_packet_.vz = msg->velocity.z;
    send_packet_.v_yaw = msg->v_yaw;
    send_packet_.r1 = msg->radius_1;
    send_packet_.r2 = msg->radius_2;
    send_packet_.dz = msg->dz;
    SendPacket::convert_send_packet_to_write_buffer(send_packet_, write_buffer_);
    serial_port_->write(write_buffer_, 51);
}

void AutoaimBridge::check_and_set_param() {
    if (!detector_param_client_->service_is_ready() || !predictor_param_client_->service_is_ready()) {
        RCLCPP_WARN_ONCE(get_logger(), "Service not ready, skipping parameter set");
        return;
    }
    if (!mode_change_flag_ || last_autoaim_state_ != recv_packet_.autoaim_mode) {
        RCLCPP_INFO_ONCE(logger_, "change autoaim mode");
        mode_change_flag_ = false;
        auto param = rclcpp::Parameter("autoaim_mode", recv_packet_.autoaim_mode);
        if (!set_detector_param_future_.valid() ||
            set_detector_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            RCLCPP_INFO_ONCE(get_logger(), "detector is setting autoaim mode to %ld...", param.as_int());
            set_detector_param_future_ = detector_param_client_->set_parameters(
            {param}, [this, param](const ResultFuturePtr & results) {
                    for (const auto & result : results.get()) {
                        if (!result.successful) {
                            RCLCPP_ERROR(get_logger(), "detector has failed to set parameter: %s", result.reason.c_str());
                            return;
                        }
                    }
                    RCLCPP_INFO_ONCE(get_logger(), "detector has successfully set autoaim to %ld!", param.as_int());
                    detector_res_ = true;
                }   
            );
        }
        if (!set_predictor_param_future_.valid() ||
            set_predictor_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            RCLCPP_INFO_ONCE(get_logger(), "predictor is setting autoaim mode to %ld...", param.as_int());
            set_predictor_param_future_ = predictor_param_client_->set_parameters(
                {param}, [this, param](const ResultFuturePtr & results) {
                    for (const auto & result : results.get()) {
                        if (!result.successful) {
                            RCLCPP_ERROR(get_logger(), "predictor has failed to set parameter: %s", result.reason.c_str());
                            return;
                        }
                    }
                    RCLCPP_INFO_ONCE(get_logger(), "predictor has successfully set autoaim to %ld!", param.as_int());
                    predictor_res_ = true;
                }
            );
        }
        mode_change_flag_ = detector_res_ && predictor_res_;
        last_autoaim_state_ = recv_packet_.autoaim_mode;
    }
    if (!color_change_flag_ || previous_receive_color_ != recv_packet_.target_color) {
        RCLCPP_INFO_ONCE(logger_, "change detector color");
        auto param = rclcpp::Parameter("is_blue", recv_packet_.target_color);
        color_change_flag_ = false;
        if (!set_detector_param_future_.valid() ||
            set_detector_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            RCLCPP_INFO_ONCE(get_logger(), "detector is setting color to %ld...", param.as_int());
            set_detector_param_future_ = detector_param_client_->set_parameters(
            {param}, [this, param](const ResultFuturePtr & results) {
                    for (const auto & result : results.get()) {
                        if (!result.successful) {
                            RCLCPP_ERROR(get_logger(), "detector has failed to set parameter: %s", result.reason.c_str());
                            return;
                        }
                    }
                    RCLCPP_INFO_ONCE(get_logger(), "detector has successfully set color to %ld!", param.as_int());
                    color_change_flag_ = true;
                }
            );
        }
        previous_receive_color_ = recv_packet_.target_color;
    }
}

AutoaimBridge::~AutoaimBridge() {
    serial_port_->close();
    recv_pub_.reset();
    realtime_recv_pub_.reset();
}



} // namespace helios_cv

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(helios_cv::AutoaimBridge)