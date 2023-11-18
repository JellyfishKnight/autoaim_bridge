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
        params_.serial_name, rclcpp::SystemDefaultsQoS()
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
    double time_gap = (time.seconds() - last_time_.seconds());
    serial_port_->read(read_buffer_, 1);
    if (read_buffer_[0] == 0xAA) {
        serial_->read(read_buffer_ + 1, 1);
        if (read_buffer_[1] == 0xAA) {
            serial_->read(read_buffer_ + 2, 17);
            if (Resolver::verify_check_sum(read_buffer_)) {
                if (params_.imu_mode == IMU_RVC_MODE) {
                    Resolver::read_packet_to_imu_packet(read_buffer_, rvc_raw_packet_);
                    Resolver::raw_to_imu_packet(imu_packet_, rvc_raw_packet_, time_gap);
                } else if (params_.imu_mode == IMU_SHTP_MODE) {
                    Resolver::read_packet_to_imu_packet(read_buffer_, shtp_raw_packet_);
                    Resolver::raw_to_imu_packet(imu_packet_, shtp_raw_packet_, time_gap);
                } else {
                    RCLCPP_ERROR(logger_, "Unkown imu mode!");
                    return ;
                }
                last_time_ = time;  
            }
        }
    }
    if (!is_inited_) {
        init_yaw_ = imu_packet_.yaw;
        is_inited_ = true;
    }
    // publish imu in euler format
    sensor_interfaces::msg::ImuEuler imu_euler_msg;
    imu_euler_msg.header.stamp = this->now();
    imu_euler_msg.header.frame_id = "imu";
    // caculate total yaw
    imu_euler_msg.yaw = imu_packet_.yaw;
    imu_euler_msg.pitch = imu_packet_.pitch;
    imu_euler_msg.roll = imu_packet_.roll;
    imu_euler_msg.init_yaw = init_yaw_;
    imu_euler_pub_->publish(imu_euler_msg);
    // publish gimbal states
    if (realtime_imu_pub_->trylock()) {
        auto & state_msg = realtime_imu_pub_->msg_;
        geometry_msgs::msg::TransformStamped transform_stamped;
        ///TODO:

        // publish imu sensor data
        realtime_imu_pub_->unlockAndPublish();
        // transform from imu to yaw
        transform_stamped.header.frame_id = params_.imu_frame_id;
        transform_stamped.child_frame_id = params_.yaw_frame_id;
        transform_stamped.header.stamp = realtime_imu_pub_->msg_.header.stamp;
        tf2::Quaternion q;
        q.setRPY(0, pitch_, -yaw_ * M_PI / 180.0);
        transform_stamped.transform.translation.x = params_.imu_to_yaw_joint_tvec[0];
        transform_stamped.transform.translation.y = params_.imu_to_yaw_joint_tvec[1];
        transform_stamped.transform.translation.z = params_.imu_to_yaw_joint_tvec[2];
        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();
        dynamic_pub_->sendTransform(transform_stamped);
    }
    last_imu_packet_ = imu_packet_;
    RCLCPP_DEBUG(logger_, "yaw: %f", yaw_);
    RCLCPP_DEBUG(logger_, "pitch: %f", pitch_);
    RCLCPP_DEBUG(logger_, "roll: %f", roll_);

}

void AutoaimBridge::send_callback(autoaim_interfaces::msg::Target::SharedPtr msg) {

}

AutoaimBridge::~AutoaimBridge() {

}

} // namespace helios_cv