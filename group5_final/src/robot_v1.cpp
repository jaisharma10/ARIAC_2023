#include "robot_v1.hpp"

Robot::Robot()
    : Node("robot_control_node"),
      floor_robot_(std::shared_ptr<rclcpp::Node>(std::move(this)), "floor_robot"),
      ceiling_robot_(std::shared_ptr<rclcpp::Node>(std::move(this)), "ceiling_robot"),
      planning_scene_()
{
    // Use upper joint velocity and acceleration limits
    floor_robot_.setMaxAccelerationScalingFactor(1.0);
    floor_robot_.setMaxVelocityScalingFactor(1.0);

    ceiling_robot_.setMaxAccelerationScalingFactor(1.0);
    ceiling_robot_.setMaxVelocityScalingFactor(1.0);

    // Subscribe to topics
    rclcpp::SubscriptionOptions options;

    topic_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    options.callback_group = topic_cb_group_;

    competition_state_sub_ = this->create_subscription<ariac_msgs::msg::CompetitionState>(
        "/ariac/competition_state", 1,
        std::bind(&Robot::competition_state_cb, this, std::placeholders::_1), options);

    kts1_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
        "/ariac/sensors/kts1_camera/image", rclcpp::SensorDataQoS(),
        std::bind(&Robot::kts1_camera_cb, this, std::placeholders::_1), options);

    kts2_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
        "/ariac/sensors/kts2_camera/image", rclcpp::SensorDataQoS(),
        std::bind(&Robot::kts2_camera_cb, this, std::placeholders::_1), options);

    left_bins_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
        "/ariac/sensors/left_bins_camera/image", rclcpp::SensorDataQoS(),
        std::bind(&Robot::left_bins_camera_cb, this, std::placeholders::_1), options);

    right_bins_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
        "/ariac/sensors/right_bins_camera/image", rclcpp::SensorDataQoS(),
        std::bind(&Robot::right_bins_camera_cb, this, std::placeholders::_1), options);

    floor_gripper_state_sub_ = this->create_subscription<ariac_msgs::msg::VacuumGripperState>(
        "/ariac/floor_robot_gripper_state", rclcpp::SensorDataQoS(),
        std::bind(&Robot::floor_gripper_state_cb, this, std::placeholders::_1), options);

    ceiling_gripper_state_sub_ = this->create_subscription<ariac_msgs::msg::VacuumGripperState>(
        "/ariac/ceiling_robot_gripper_state", rclcpp::SensorDataQoS(),
        std::bind(&Robot::ceiling_gripper_state_cb, this, std::placeholders::_1), options);

    as1_state_sub_ = this->create_subscription<ariac_msgs::msg::AssemblyState>(
        "/ariac/assembly_insert_1_assembly_state", rclcpp::SensorDataQoS(),
        std::bind(&Robot::as1_state_cb, this, std::placeholders::_1), options);

    as2_state_sub_ = this->create_subscription<ariac_msgs::msg::AssemblyState>(
        "/ariac/assembly_insert_2_assembly_state", rclcpp::SensorDataQoS(),
        std::bind(&Robot::as2_state_cb, this, std::placeholders::_1), options);

    as3_state_sub_ = this->create_subscription<ariac_msgs::msg::AssemblyState>(
        "/ariac/assembly_insert_3_assembly_state", rclcpp::SensorDataQoS(),
        std::bind(&Robot::as3_state_cb, this, std::placeholders::_1), options);

    as4_state_sub_ = this->create_subscription<ariac_msgs::msg::AssemblyState>(
        "/ariac/assembly_insert_4_assembly_state", rclcpp::SensorDataQoS(),
        std::bind(&Robot::as4_state_cb, this, std::placeholders::_1), options);

    robot_task_sub_ = this->create_subscription<group5_msgs::msg::FloorRobotTask>(
        "/competitor/robot_task", 1,
        std::bind(&Robot::robot_task_cb, this, std::placeholders::_1), options);

    robot_task_priority_sub_ = this->create_subscription<group5_msgs::msg::FloorRobotTask>(
        "/competitor/robot_task_priority", 1,
        std::bind(&Robot::robot_task_priority_cb, this, std::placeholders::_1), options);

    conveyor_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
        "/ariac/sensors/alc_conveyor/image", rclcpp::SensorDataQoS(),
        std::bind(&Robot::conveyor_camera_cb, this, std::placeholders::_1), options);

    break_beam_sub_ = this->create_subscription<ariac_msgs::msg::BreakBeamStatus>(
        "/ariac/sensors/breakbeam_conveyor/status", rclcpp::SensorDataQoS(),
        std::bind(&Robot::break_beam_cb, this, std::placeholders::_1), options);

    step_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/competitor/step", 1,
        std::bind(&Robot::step_task_cb, this, std::placeholders::_1), options);

    agv_1_camera_sub = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
        "/ariac/sensors/agv_1_camera/image", rclcpp::SensorDataQoS(),
        std::bind(&Robot::agv_1_camera_cb, this, std::placeholders::_1), options);

    agv_2_camera_sub = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
        "/ariac/sensors/agv_2_camera/image", rclcpp::SensorDataQoS(),
        std::bind(&Robot::agv_2_camera_cb, this, std::placeholders::_1), options);

    agv_3_camera_sub = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
        "/ariac/sensors/agv_3_camera/image", rclcpp::SensorDataQoS(),
        std::bind(&Robot::agv_3_camera_cb, this, std::placeholders::_1), options);

    agv_4_camera_sub = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
        "/ariac/sensors/agv_4_camera/image", rclcpp::SensorDataQoS(),
        std::bind(&Robot::agv_4_camera_cb, this, std::placeholders::_1), options);

    // Initialize publishers
    completed_order_pub_ = this->create_publisher<group5_msgs::msg::CompletedOrder>("/competitor/completed_order", 10);

    // Initialize service clients

    quality_checker_ = this->create_client<ariac_msgs::srv::PerformQualityCheck>("/ariac/perform_quality_check");
    pre_assembly_poses_getter_ = this->create_client<ariac_msgs::srv::GetPreAssemblyPoses>("/ariac/get_pre_assembly_poses");
    floor_robot_tool_changer_ = this->create_client<ariac_msgs::srv::ChangeGripper>("/ariac/floor_robot_change_gripper");
    floor_robot_gripper_enable_ = this->create_client<ariac_msgs::srv::VacuumGripperControl>("/ariac/floor_robot_enable_gripper");
    ceiling_robot_gripper_enable_ = this->create_client<ariac_msgs::srv::VacuumGripperControl>("/ariac/ceiling_robot_enable_gripper");

    AddModelsToPlanningScene();

    RCLCPP_INFO(this->get_logger(), "Initialization successful.");
}

Robot::~Robot()
{
    floor_robot_.~MoveGroupInterface();
    ceiling_robot_.~MoveGroupInterface();
}

// ---------------------------------------------------------------------------- //
// | ------------------------ | Callback Functions | ------------------------ | //
// ---------------------------------------------------------------------------- //

void Robot::competition_state_cb(
    const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg)
{
    competition_state_ = msg->competition_state;
}

void Robot::kts1_camera_cb(
    const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
    if (!kts1_camera_received_data)
    {
        RCLCPP_INFO(get_logger(), "Received data from kts1 camera");
        kts1_camera_received_data = true;
    }

    kts1_trays_ = msg->tray_poses;
    kts1_camera_pose_ = msg->sensor_pose;
}

void Robot::kts2_camera_cb(
    const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
    if (!kts2_camera_received_data)
    {
        RCLCPP_INFO(get_logger(), "Received data from kts2 camera");
        kts2_camera_received_data = true;
    }

    kts2_trays_ = msg->tray_poses;
    kts2_camera_pose_ = msg->sensor_pose;
}

void Robot::left_bins_camera_cb(
    const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
    left_bins_parts_ = msg->part_poses;
    left_bins_camera_pose_ = msg->sensor_pose;

    // update bin tracking
    int part_num = (msg->part_poses).size();
    std::vector<std::pair<double, double>> part_locs; 

    for (int i = 0; i < part_num; i++) 
    {
        auto current_part = msg->part_poses.at(i);
        // Update slot status based on part locations
        auto current_part_pose_world = MultiplyPose(left_bins_camera_pose_, current_part.pose);
        part_locs.push_back(std::pair<double, double> {current_part_pose_world.position.x, current_part_pose_world.position.y});
    }

    if (!left_bins_camera_received_data)
    {
        RCLCPP_INFO(get_logger(), "Received data from left bins camera");
        left_bins_camera_received_data = true;
    }
}

void Robot::right_bins_camera_cb(
    const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{

    right_bins_parts_ = msg->part_poses;
    right_bins_camera_pose_ = msg->sensor_pose;

    // update bin tracking
    int part_num = (msg->part_poses).size();
    std::vector<std::pair<double, double>> part_locs; 

    for (int i = 0; i < part_num; i++) 
    {
        auto current_part = msg->part_poses.at(i);
        // Update slot status based on part locations
        auto current_part_pose_world = MultiplyPose(right_bins_camera_pose_, current_part.pose);
        part_locs.push_back(std::pair<double, double> {current_part_pose_world.position.x, current_part_pose_world.position.y});
    }
    track_bin.update_right_slot_status(part_locs); 

    if (!right_bins_camera_received_data)
    {
        RCLCPP_INFO(get_logger(), "Received data from right bins camera");
        right_bins_camera_received_data = true;
    } 
}

void Robot::conveyor_camera_cb(
    const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
    if (!conveyor_camera_received_data_)
    {
        RCLCPP_INFO(get_logger(), "Received data from conveyor camera");
        conveyor_camera_received_data_ = true;
    }

    conveyor_camera_parts_ = msg->part_poses;
    conveyor_camera_pose_ = msg->sensor_pose;

    if (conveyor_camera_parts_.size() > 0)
    {
        conveyor_part_detected = true;
        current_conveyor_part = conveyor_camera_parts_.at(0); // can use vector.front() or vecotr.begin() ?
    }
    else
    {
        conveyor_part_detected = false;
    }
}

void Robot::break_beam_cb(
    const ariac_msgs::msg::BreakBeamStatus::ConstSharedPtr msg)
{
    break_beam_detected = msg->object_detected;
}

void Robot::floor_gripper_state_cb(
    const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg)
{
    floor_gripper_state_ = *msg;
}

void Robot::ceiling_gripper_state_cb(
    const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg)
{
    ceiling_gripper_state_ = *msg;
}

void Robot::agv_1_camera_cb(
    const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
    if (!agv_1_camera_received_data)
    {
        RCLCPP_INFO(get_logger(), "Received data from AGV 1 camera");
        agv_1_camera_received_data = true;
    }

    agv_1_parts_ = msg->part_poses;

    agv_1_camera_pose_ = msg->sensor_pose;
}

void Robot::agv_2_camera_cb(
    const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
    if (!agv_2_camera_received_data)
    {
        RCLCPP_INFO(get_logger(), "Received data from AGV 2 camera");
        agv_2_camera_received_data = true;
    }

    agv_2_parts_ = msg->part_poses;
    agv_2_camera_pose_ = msg->sensor_pose;
}

void Robot::agv_3_camera_cb(
    const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
    if (!agv_3_camera_received_data)
    {
        RCLCPP_INFO(get_logger(), "Received data from AGV 3 camera");
        agv_3_camera_received_data = true;
    }

    agv_3_parts_ = msg->part_poses;
    agv_3_camera_pose_ = msg->sensor_pose;
}

void Robot::agv_4_camera_cb(
    const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg)
{
    if (!agv_4_camera_received_data)
    {
        RCLCPP_INFO(get_logger(), "Received data from AGV 4 camera");
        agv_4_camera_received_data = true;
    }

    agv_4_parts_ = msg->part_poses;
    agv_4_camera_pose_ = msg->sensor_pose;
}

void Robot::as1_state_cb(
    const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg)
{
    assembly_station_states_.insert_or_assign(ariac_msgs::msg::AssemblyTask::AS1, *msg);
}

void Robot::as2_state_cb(
    const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg)
{
    assembly_station_states_.insert_or_assign(ariac_msgs::msg::AssemblyTask::AS2, *msg);
}

void Robot::as3_state_cb(
    const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg)
{
    assembly_station_states_.insert_or_assign(ariac_msgs::msg::AssemblyTask::AS3, *msg);
}

void Robot::as4_state_cb(
    const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg)
{
    assembly_station_states_.insert_or_assign(ariac_msgs::msg::AssemblyTask::AS4, *msg);
}

void Robot::robot_task_cb(
    const group5_msgs::msg::FloorRobotTask::ConstSharedPtr msg)
{
    RCLCPP_INFO(get_logger(), " ----------------------------- ");
    RCLCPP_INFO(get_logger(), "Received Order");
    orders_.push_back(*msg);
}

void Robot::robot_task_priority_cb(
    const group5_msgs::msg::FloorRobotTask::ConstSharedPtr msg)

{
    RCLCPP_INFO(get_logger(), " ----------------------------- ");
    RCLCPP_INFO(get_logger(), "Received priority Order");
    priority_orders_.push_back(*msg);
}

void Robot::step_task_cb(const std_msgs::msg::String::ConstSharedPtr msg)
{
    std::string receivedMsg = msg->data;
    step = receivedMsg;
}

// ---------------------------------------------------------------------------- //
// | ------------------------- | Helper Functions | ------------------------- | //
// ---------------------------------------------------------------------------- //

geometry_msgs::msg::Pose Robot::MultiplyPose(
    geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2)
{
    KDL::Frame f1;
    KDL::Frame f2;

    // convert input poses to KDL frames f1 and f2
    tf2::fromMsg(p1, f1);
    tf2::fromMsg(p2, f2);

    // perform KDL multiplication
    KDL::Frame f3 = f1 * f2;

    // return pose value that coresponds to f3 KDL value
    return tf2::toMsg(f3);
}

void Robot::LogPose(geometry_msgs::msg::Pose p)
{
    tf2::Quaternion q(
        p.orientation.x,
        p.orientation.y,
        p.orientation.z,
        p.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // pi value can be accessed using 'M_PI', from math.h
    roll *= 180 / M_PI;
    pitch *= 180 / M_PI;
    yaw *= 180 / M_PI;

    RCLCPP_INFO(get_logger(), "(X: %.2f, Y: %.2f, Z: %.2f, R: %.2f, P: %.2f, Y: %.2f)",
                p.position.x, p.position.y, p.position.z,
                roll, pitch, yaw);
}

geometry_msgs::msg::Pose Robot::BuildPose(
    double x, double y, double z, geometry_msgs::msg::Quaternion orientation)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation = orientation;

    return pose;
}

geometry_msgs::msg::Pose Robot::FrameWorldPose(std::string frame_id)
{
    geometry_msgs::msg::TransformStamped t;
    geometry_msgs::msg::Pose pose;

    try
    {
        t = tf_buffer->lookupTransform("world", frame_id, tf2::TimePointZero);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR(get_logger(), "Could not get transform");
    }

    pose.position.x = t.transform.translation.x;
    pose.position.y = t.transform.translation.y;
    pose.position.z = t.transform.translation.z;
    pose.orientation = t.transform.rotation;

    return pose;
}

double Robot::GetYaw(geometry_msgs::msg::Pose pose)
{
    tf2::Quaternion q(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return yaw;
}

geometry_msgs::msg::Quaternion Robot::QuaternionFromRPY(double r, double p, double y)
{
    tf2::Quaternion q;
    geometry_msgs::msg::Quaternion q_msg;

    // tf2 method
    q.setRPY(r, p, y);

    q_msg.x = q.x();
    q_msg.y = q.y();
    q_msg.z = q.z();
    q_msg.w = q.w();

    return q_msg;
}

geometry_msgs::msg::Quaternion Robot::SetRobotOrientation(double rotation)
{
    tf2::Quaternion tf_q;
    tf_q.setRPY(0, 3.14159, rotation);

    geometry_msgs::msg::Quaternion q;

    q.x = tf_q.x();
    q.y = tf_q.y();
    q.z = tf_q.z();
    q.w = tf_q.w();

    return q;
}

// ---------------------------------------------------------------------------- //
// | --------------------- | Planning Scene Functions | --------------------- | //
// ---------------------------------------------------------------------------- //

void Robot::AddModelToPlanningScene(
    std::string name, std::string mesh_file, geometry_msgs::msg::Pose model_pose)
{
    moveit_msgs::msg::CollisionObject collision;

    collision.id = name;
    collision.header.frame_id = "world";

    shape_msgs::msg::Mesh mesh;
    shapes::ShapeMsg mesh_msg;

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("test_competitor");
    std::stringstream path;
    path << "file://" << package_share_directory << "/meshes/" << mesh_file;
    std::string model_path = path.str();

    shapes::Mesh *m = shapes::createMeshFromResource(model_path);
    shapes::constructMsgFromShape(m, mesh_msg);

    mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);

    collision.meshes.push_back(mesh);
    collision.mesh_poses.push_back(model_pose);

    collision.operation = collision.ADD;

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision);

    planning_scene_.addCollisionObjects(collision_objects);
}

void Robot::AddModelsToPlanningScene()
{
    // Add bins
    std::map<std::string, std::pair<double, double>> bin_positions = {
        {"bin1", std::pair<double, double>(-1.9, 3.375)},
        {"bin2", std::pair<double, double>(-1.9, 2.625)},
        {"bin3", std::pair<double, double>(-2.65, 2.625)},
        {"bin4", std::pair<double, double>(-2.65, 3.375)},
        {"bin5", std::pair<double, double>(-1.9, -3.375)},
        {"bin6", std::pair<double, double>(-1.9, -2.625)},
        {"bin7", std::pair<double, double>(-2.65, -2.625)},
        {"bin8", std::pair<double, double>(-2.65, -3.375)}};

    geometry_msgs::msg::Pose bin_pose;
    for (auto const &bin : bin_positions)
    {
        bin_pose.position.x = bin.second.first;
        bin_pose.position.y = bin.second.second;
        bin_pose.position.z = 0;
        bin_pose.orientation = QuaternionFromRPY(0, 0, 3.14159);

        AddModelToPlanningScene(bin.first, "bin.stl", bin_pose);
    }

    // Add assembly stations
    std::map<std::string, std::pair<double, double>> assembly_station_positions = {
        {"as1", std::pair<double, double>(-7.3, 3)},
        {"as2", std::pair<double, double>(-12.3, 3)},
        {"as3", std::pair<double, double>(-7.3, -3)},
        {"as4", std::pair<double, double>(-12.3, -3)},
    };

    geometry_msgs::msg::Pose assembly_station_pose;
    for (auto const &station : assembly_station_positions)
    {
        assembly_station_pose.position.x = station.second.first;
        assembly_station_pose.position.y = station.second.second;
        assembly_station_pose.position.z = 0;
        assembly_station_pose.orientation = QuaternionFromRPY(0, 0, 0);

        AddModelToPlanningScene(station.first, "assembly_station.stl", assembly_station_pose);
    }

    // Add assembly briefcases
    std::map<std::string, std::pair<double, double>> assembly_insert_positions = {
        {"as1_insert", std::pair<double, double>(-7.7, 3)},
        {"as2_insert", std::pair<double, double>(-12.7, 3)},
        {"as3_insert", std::pair<double, double>(-7.7, -3)},
        {"as4_insert", std::pair<double, double>(-12.7, -3)},
    };

    geometry_msgs::msg::Pose assembly_insert_pose;
    for (auto const &insert : assembly_insert_positions)
    {
        assembly_insert_pose.position.x = insert.second.first;
        assembly_insert_pose.position.y = insert.second.second;
        assembly_insert_pose.position.z = 1.011;
        assembly_insert_pose.orientation = QuaternionFromRPY(0, 0, 0);

        AddModelToPlanningScene(insert.first, "assembly_insert.stl", assembly_insert_pose);
    }

    geometry_msgs::msg::Pose conveyor_pose;
    conveyor_pose.position.x = -0.6;
    conveyor_pose.position.y = 0;
    conveyor_pose.position.z = 0;
    conveyor_pose.orientation = QuaternionFromRPY(0, 0, 0);

    AddModelToPlanningScene("conveyor", "conveyor.stl", conveyor_pose);

    geometry_msgs::msg::Pose kts1_table_pose;
    kts1_table_pose.position.x = -1.3;
    kts1_table_pose.position.y = -5.84;
    kts1_table_pose.position.z = 0;
    kts1_table_pose.orientation = QuaternionFromRPY(0, 0, 3.14159);

    AddModelToPlanningScene("kts1_table", "kit_tray_table.stl", kts1_table_pose);

    geometry_msgs::msg::Pose kts2_table_pose;
    kts2_table_pose.position.x = -1.3;
    kts2_table_pose.position.y = 5.84;
    kts2_table_pose.position.z = 0;
    kts2_table_pose.orientation = QuaternionFromRPY(0, 0, 0);

    AddModelToPlanningScene("kts2_table", "kit_tray_table.stl", kts2_table_pose);
}

// ---------------------------------------------------------------------------- //
// | ---------------------- | Quality Check Functions | --------------------- | //
// ---------------------------------------------------------------------------- //

std::vector<bool> Robot::QualityCheckService()
{
    // Retrieve Quality Status from Sensor
    auto request_1 = std::make_shared<ariac_msgs::srv::PerformQualityCheck::Request>();
    request_1->order_id = current_order_.id;
    auto quality_result_1 = quality_checker_->async_send_request(request_1);
    quality_result_1.wait();

    // Store Quality Status Result as a vector
    bool all_passed_bool = quality_result_1.get()->all_passed;
    bool q1_faulty_part = quality_result_1.get()->quadrant1.faulty_part;
    bool q2_faulty_part = quality_result_1.get()->quadrant2.faulty_part;
    bool q3_faulty_part = quality_result_1.get()->quadrant3.faulty_part;
    bool q4_faulty_part = quality_result_1.get()->quadrant4.faulty_part;
    std::vector<bool> qv_1{all_passed_bool, q1_faulty_part, q2_faulty_part, q3_faulty_part, q4_faulty_part};

    RCLCPP_INFO(get_logger(), " | ------------ | Waiting for 3 seconds | ------------ | ");
    rclcpp::sleep_for(std::chrono::seconds(3));

    auto request_2 = std::make_shared<ariac_msgs::srv::PerformQualityCheck::Request>();
    request_2->order_id = current_order_.id;
    auto quality_result_2 = quality_checker_->async_send_request(request_2);
    quality_result_2.wait();

    // Store Quality Status Result as a vector
    all_passed_bool = quality_result_2.get()->all_passed;
    q1_faulty_part = quality_result_2.get()->quadrant1.faulty_part;
    q2_faulty_part = quality_result_2.get()->quadrant2.faulty_part;
    q3_faulty_part = quality_result_2.get()->quadrant3.faulty_part;
    q4_faulty_part = quality_result_2.get()->quadrant4.faulty_part;

    // Define a vector to store final values
    std::vector<bool> quality_check_vector{all_passed_bool, q1_faulty_part, q2_faulty_part, q3_faulty_part, q4_faulty_part};

    // Print Check 1 Results
    std::string output_string = "Check 1 elements --> ";
    for (auto element : qv_1)
    {
        output_string += (element ? "true ||" : "false ||");
    }
    RCLCPP_INFO_STREAM(get_logger(), "" << output_string);

    // Print Check 2 Results
    output_string = "Check 2 elements --> ";
    for (auto element : quality_check_vector)
    {
        output_string += (element ? "true ||" : "false ||");
    }
    RCLCPP_INFO_STREAM(get_logger(), "" << output_string);

    // Display Quality Status Result
    RCLCPP_INFO(get_logger(), " ---------------------------------------------------------------------------- ");
    RCLCPP_INFO(get_logger(), "All Passed: %s", quality_check_vector[0] ? "true" : "false");
    RCLCPP_INFO(get_logger(), " ---------------------------------------------------------------------------- ");
    RCLCPP_INFO(get_logger(), "Final Request --> Faulty Parts Check:");
    RCLCPP_INFO(get_logger(), "     - Quadrant 1: %s", quality_check_vector[1] ? "true" : "false");
    RCLCPP_INFO(get_logger(), "     - Quadrant 2: %s", quality_check_vector[2] ? "true" : "false");
    RCLCPP_INFO(get_logger(), "     - Quadrant 3: %s", quality_check_vector[3] ? "true" : "false");
    RCLCPP_INFO(get_logger(), "     - Quadrant 4: %s", quality_check_vector[4] ? "true" : "false");
    RCLCPP_INFO(get_logger(), " ---------------------------------------------------------------------------- ");

    return quality_check_vector;
}

bool Robot::KittingPartQualityCheck(ariac_msgs::msg::KittingTask task)
{
    // FloorRobotSendHome();

    // Call Quality Sensor Service
    auto faulty_parts = QualityCheckService();

    // Process the Response - Identify Issues
    if (faulty_parts[0] == false)
    {
        RCLCPP_INFO_STREAM(get_logger(), "quality_check failed");

        // Get AGV specific information
        int agv_num = task.agv_number;

        switch (agv_num)
        {
        case 1:
            agv_camera_parts_ = agv_1_parts_;
            agv_camera_poses_ = agv_1_camera_pose_;
            break;
        case 2:
            agv_camera_parts_ = agv_2_parts_;
            agv_camera_poses_ = agv_2_camera_pose_;
            break;
        case 3:
            agv_camera_parts_ = agv_3_parts_;
            agv_camera_poses_ = agv_3_camera_pose_;
            break;
        case 4:
            agv_camera_parts_ = agv_4_parts_;
            agv_camera_poses_ = agv_4_camera_pose_;
            break;
        default:
            break;
        }

        // Check Each Quadrant x 4 ---------------
        int counter = 1;
        faulty_parts.erase(faulty_parts.begin()); // go through list of bool_faulty [q1, q2, q3, q4]

        for (bool faulty_quadrant : faulty_parts)
        {
            if (faulty_quadrant == true)
            {
                RCLCPP_INFO_STREAM(get_logger(), "Quadrant " << counter << ": Faulty Part Detected --> " << faulty_quadrant);
                for (auto kit_part : task.parts) // loop through Order Required Parts
                {
                    if (kit_part.quadrant == counter) // identify Part on Error Quadrant
                    {
                        RCLCPP_INFO_STREAM(get_logger(), "Replace Part on Quadrant " << counter);
                        ReplaceFaultyPart(kit_part.part, agv_camera_parts_, agv_camera_poses_, agv_num, kit_part.quadrant);
                    }
                }
            }
            else
            {
                RCLCPP_INFO_STREAM(get_logger(), "No Faulty Part on Quadrant " << counter);
            }
            counter++;
        }
    }

    else
    {
        RCLCPP_INFO_STREAM(get_logger(), "quality_check passed");
    }

    RCLCPP_INFO(get_logger(), " - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");
    RCLCPP_INFO(get_logger(), " Quality Check Complete ");
    RCLCPP_INFO(get_logger(), " - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");

    FloorRobotSendHome();

    return false;
}

// function to pick up from AGV Tray, Place in Disposable Bin
bool Robot::FloorRobotPickAGVTrayPart(ariac_msgs::msg::Part part_to_pick,
                                      std::vector<ariac_msgs::msg::PartPose> agv_parts,
                                      geometry_msgs::msg::Pose agv_camera_pose,
                                      int agv_num,
                                      int quadrant)
{

    RCLCPP_INFO(get_logger(), " ---------------------------------------------------------------------------- ");
    RCLCPP_INFO_STREAM(get_logger(), "Need to pick up a Faulty " << part_colors_[part_to_pick.color] << " " << part_types_[part_to_pick.type]);
    RCLCPP_INFO(get_logger(), " ---------------------------------------------------------------------------- ");

    auto agv_tray_pose = FrameWorldPose("agv" + std::to_string(agv_num) + "_tray");
    auto part_pick_offset = BuildPose(quad_offsets_[quadrant].first, quad_offsets_[quadrant].second, 0.0,
                                      geometry_msgs::msg::Quaternion());
    auto part_pick_pose = MultiplyPose(agv_tray_pose, part_pick_offset);

    geometry_msgs::msg::Pose part_pose;

    RCLCPP_INFO(get_logger(), "Quadrant Number --> %d", quadrant);
    RCLCPP_INFO(get_logger(), "Quadrant pose x --> %f", part_pick_pose.position.x);
    RCLCPP_INFO(get_logger(), "Quadrant pose y --> %f", part_pick_pose.position.y);
    RCLCPP_INFO(get_logger(), " ---------------------------------------------------------------------------- ");

    // Check AGV Parts
    for (auto part : agv_parts)
    {
        if (part.part.color == part_to_pick.color && part.part.type == part_to_pick.type)
        {
            RCLCPP_INFO(get_logger(), " ---------------------------------------------------------------------------- ");
            RCLCPP_INFO_STREAM(get_logger(), "Identified a --> " << part_colors_[part.part.color] << " " << part_types_[part.part.type] << " on the AGV");
            part_pose = MultiplyPose(agv_camera_pose, part.pose);

            // check if it is the same quadrant
            double x_diff = part_pick_pose.position.x - part_pose.position.x;
            double y_diff = part_pick_pose.position.y - part_pose.position.y;
            double distance = std::sqrt(x_diff * x_diff + y_diff * y_diff);
            if (distance < 0.005)
            {
                part_pose = MultiplyPose(agv_camera_pose, part.pose);
                RCLCPP_INFO(get_logger(), " Pick up Part at pose x --> %f", part_pose.position.x);
                RCLCPP_INFO(get_logger(), " Pick up Part at pose y --> %f", part_pose.position.y);
                break;
            }
        }
    }

    RCLCPP_INFO(get_logger(), " ---------------------------------------------------------------------------- ");

    // move to agv
    floor_robot_.setJointValueTarget("linear_actuator_joint", rail_positions_["agv" + std::to_string(agv_num)]);
    floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
    FloorRobotMovetoTarget();

    // get Yaw angle of end effector
    geometry_msgs::msg::Pose current_pose = floor_robot_.getCurrentPose().pose;
    double robot_rotation = GetYaw(current_pose);

    RCLCPP_INFO(get_logger(), "Move closer to AGV Part");
    std::vector<geometry_msgs::msg::Pose> waypoints;

    waypoints.push_back(BuildPose(part_pose.position.x,
                                  part_pose.position.y,
                                  part_pose.position.z + 0.5,
                                  SetRobotOrientation(robot_rotation)));

    waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                  part_pose.position.z + part_heights_[part_to_pick.type] + pick_offset_,
                                  SetRobotOrientation(robot_rotation)));

    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

    waypoints.clear();

    // Enable Gripper to Pick Up Part
    RCLCPP_INFO(get_logger(), "Enable Gripper State");
    FloorRobotSetGripperState(true);

    RCLCPP_INFO(get_logger(), "Pick up the part");
    if (!FloorRobotWaitForAttach(7.0))
    {
        RCLCPP_INFO(get_logger(), " x - x - x - x - x - x - x - x - x - x - x - x - x - x  ");
        RCLCPP_INFO(get_logger(), " | ------------ | Failed to Pick Up | ------------ | ");
        RCLCPP_INFO(get_logger(), " x - x - x - x - x - x - x - x - x - x - x - x - x - x  ");
        return false;
    }

    // Add part to planning scene
    part_number_++;
    std::string part_name = part_colors_[part_to_pick.color] +
                            "_" + part_types_[part_to_pick.type] +
                            "_" + std::to_string(part_number_);

    RCLCPP_INFO(get_logger(), " - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");
    RCLCPP_INFO_STREAM(get_logger(), "Add to Scene --> " << part_colors_[part_to_pick.color] << " "
                                                         << part_types_[part_to_pick.type] << " #"
                                                         << std::to_string(part_number_));
    RCLCPP_INFO(get_logger(), " - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");

    AddModelToPlanningScene(part_name, part_types_[part_to_pick.type] + ".stl", part_pose);
    floor_robot_.attachObject(part_name);
    floor_robot_attached_part_ = part_to_pick;

    // Move up slightly
    RCLCPP_INFO(get_logger(), "Move up slightly");
    waypoints.push_back(BuildPose(part_pose.position.x,
                                  part_pose.position.y,
                                  part_pose.position.z + 0.5,
                                  SetRobotOrientation(robot_rotation)));

    FloorRobotMoveCartesian(waypoints, 0.6, 0.6);

    // Move to Bin Position (FK)
    RCLCPP_INFO(get_logger(), "Return to Bin Prep Position");
    FloorRobotMovetoTarget();

    if (!floor_gripper_state_.attached)
    {
        RCLCPP_WARN(get_logger(), " Nothing is Attached to Gripper !!!");
        return false;
    }

    return true;
}

// function to dispose a part
bool Robot::FloorRobotDisposePart(int agv_num)
{
    RCLCPP_INFO(get_logger(), " ---------------------------------------------------------------------------- ");
    RCLCPP_INFO_STREAM(get_logger(), " Attempting to Dispose Faulty Part at nearest Disposal Bin ");
    RCLCPP_INFO(get_logger(), " ---------------------------------------------------------------------------- ");

    // Go to Closest Disposal Bin Position (FK)
    RCLCPP_INFO(get_logger(), "Move to Center Disposal Bin Position");
    floor_robot_.setJointValueTarget(center_disposal_bin_js);
    FloorRobotMovetoTarget();

    // Disable Gripper to Drop Part
    RCLCPP_INFO(get_logger(), "Disable Gripper to Drop Part");
    FloorRobotSetGripperState(false);

    // Remove part from Planning Scene
    std::string part_name = part_colors_[floor_robot_attached_part_.color] +
                            "_" + part_types_[floor_robot_attached_part_.type] +
                            "_" + std::to_string(part_number_);

    RCLCPP_INFO(get_logger(), " - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");
    RCLCPP_INFO_STREAM(get_logger(), "Remove From Scene --> " << part_colors_[floor_robot_attached_part_.color] << " "
                                                              << part_types_[floor_robot_attached_part_.type] << " #"
                                                              << std::to_string(part_number_));
    RCLCPP_INFO(get_logger(), " - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");
    floor_robot_.detachObject(part_name);
    planning_scene_.removeCollisionObjects(std::vector<std::string>{part_name});

    RCLCPP_INFO(get_logger(), "Faulty Part Disposed");
    FloorRobotMovetoTarget();

    return true;
}

bool Robot::ReplaceFaultyPart(ariac_msgs::msg::Part part_to_pick,
                              std::vector<ariac_msgs::msg::PartPose> agv_parts_,
                              geometry_msgs::msg::Pose camera_pose_,
                              int agv_num,
                              int quadrant)
{
    // Pick Up from AGV --> Dispose --> Pick Replacement Part --> Place on AGV Tray
    if (FloorRobotPickAGVTrayPart(part_to_pick, agv_parts_, camera_pose_, agv_num, quadrant) == true)
    {
        if (FloorRobotDisposePart(agv_num) == true)
        {
            if (FloorRobotPickBinPart(part_to_pick) == true)
            {
                FloorRobotPlacePartOnKitTray(agv_num, quadrant);
            }
            else
            {
                RCLCPP_WARN(get_logger(), " Part Bin Pick Up Failed. Skip Current Part Replacement");
            }
        }
        else
        {
            RCLCPP_WARN(get_logger(), " Part Disposal Failed. Skip Current Part Replacement");
        }
    }
    else
    {
        RCLCPP_WARN(get_logger(), " AGV Part Pick Up Failed. Skip Current Part Replacement");
    }

    return true;
}


// ---------------------------------------------------------------------------- //
// | ----------------------- | Floor Robot Functions | ---------------------- | //
// ---------------------------------------------------------------------------- //

bool Robot::FloorRobotMovetoTarget()
{
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(floor_robot_.plan(plan));

    if (success)
    {
        return static_cast<bool>(floor_robot_.execute(plan));
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Unable to generate plan");
        return false;
    }
}

bool Robot::FloorRobotMoveCartesian(
    std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf)
{
    moveit_msgs::msg::RobotTrajectory trajectory;

    double path_fraction = floor_robot_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

    if (path_fraction < 0.9)
    {
        RCLCPP_ERROR(get_logger(), "Unable to generate trajectory through waypoints");
        return false;
    }

    // Retime trajectory
    robot_trajectory::RobotTrajectory rt(floor_robot_.getCurrentState()->getRobotModel(), "floor_robot");
    rt.setRobotTrajectoryMsg(*floor_robot_.getCurrentState(), trajectory);
    totg_.computeTimeStamps(rt, vsf, asf);
    rt.getRobotTrajectoryMsg(trajectory);

    return static_cast<bool>(floor_robot_.execute(trajectory));
}

bool Robot::FloorRobotWaitForAttach(double timeout)
{
    // Wait for part to be attached
    rclcpp::Time start = now();
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose starting_pose = floor_robot_.getCurrentPose().pose;

    while (!floor_gripper_state_.attached)
    {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");

        waypoints.clear();
        starting_pose.position.z -= 0.001;
        waypoints.push_back(starting_pose);

        FloorRobotMoveCartesian(waypoints, 0.1, 0.1);

        usleep(200);

        if (now() - start > rclcpp::Duration::from_seconds(timeout))
        {
            RCLCPP_ERROR(get_logger(), "x - x - x - x - x - x - x - x - x - x ");
            RCLCPP_ERROR(get_logger(), "x      Unable to pick up object     x ");
            RCLCPP_ERROR(get_logger(), "x - x - x - x - x - x - x - x - x - x ");
            return false;
        }
    }
    RCLCPP_INFO_STREAM(get_logger(), "Gripper Status: " << floor_gripper_state_.attached);

    return true;
}

void Robot::FloorRobotWaitForAttachConveyor(double timeout)
{
    // Wait for part to be attached
    rclcpp::Time start = now();
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose starting_pose = floor_robot_.getCurrentPose().pose;

    while (!floor_gripper_state_.attached)
    {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");

        waypoints.clear();
        starting_pose.position.z -= 0.001;
        starting_pose.position.y -= 0.15;
        waypoints.push_back(starting_pose);

        FloorRobotMoveCartesian(waypoints, 1.0, 1.0);
        floor_robot_.getCurrentPose().pose;
        usleep(200);

        if (now() - start > rclcpp::Duration::from_seconds(timeout))
        {
            RCLCPP_ERROR(get_logger(), "Unable to pick up object");
            return;
        }
    }
    RCLCPP_ERROR(get_logger(), "Pick up success");
}

void Robot::FloorRobotSendHome()
{
    RCLCPP_INFO(get_logger(), " - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");
    RCLCPP_INFO(get_logger(), " Floor Robot Going Home ");
    RCLCPP_INFO(get_logger(), " - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");

    // Move floor robot to home joint state
    floor_robot_.setNamedTarget("home");
    FloorRobotMovetoTarget();
}

bool Robot::FloorRobotSetGripperState(bool enable)
{
    if (floor_gripper_state_.enabled == enable)
    {
        if (floor_gripper_state_.enabled)
            RCLCPP_INFO(get_logger(), "Already enabled");
        else
            RCLCPP_INFO(get_logger(), "Already disabled");

        return false;
    }

    // Call enable service
    auto request = std::make_shared<ariac_msgs::srv::VacuumGripperControl::Request>();
    request->enable = enable;

    auto result = floor_robot_gripper_enable_->async_send_request(request);
    result.wait();

    if (!result.get()->success)
    {
        RCLCPP_ERROR(get_logger(), "Error calling gripper enable service");
        return false;
    }

    return true;
}

bool Robot::FloorRobotChangeGripper(std::string station, std::string gripper_type)
{

    RCLCPP_INFO(get_logger(), " - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");
    RCLCPP_INFO(get_logger(), " Changing Gripper ");
    RCLCPP_INFO(get_logger(), " - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");

    // Move gripper into tool changer
    auto tc_pose = FrameWorldPose(station + "_tool_changer_" + gripper_type + "_frame");

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(BuildPose(tc_pose.position.x, tc_pose.position.y,
                                  tc_pose.position.z + 0.4, SetRobotOrientation(0.0)));

    waypoints.push_back(BuildPose(tc_pose.position.x, tc_pose.position.y,
                                  tc_pose.position.z, SetRobotOrientation(0.0)));

    if (!FloorRobotMoveCartesian(waypoints, 0.2, 0.1))
        return false;

    // Call service to change gripper
    auto request = std::make_shared<ariac_msgs::srv::ChangeGripper::Request>();

    if (gripper_type == "trays")
    {
        request->gripper_type = ariac_msgs::srv::ChangeGripper::Request::TRAY_GRIPPER;
    }
    else if (gripper_type == "parts")
    {
        request->gripper_type = ariac_msgs::srv::ChangeGripper::Request::PART_GRIPPER;
    }

    auto result = floor_robot_tool_changer_->async_send_request(request);
    result.wait();
    if (!result.get()->success)
    {
        RCLCPP_ERROR(get_logger(), "Error calling gripper change service");
        return false;
    }

    waypoints.clear();
    waypoints.push_back(BuildPose(tc_pose.position.x, tc_pose.position.y,
                                  tc_pose.position.z + 0.4, SetRobotOrientation(0.0)));

    if (!FloorRobotMoveCartesian(waypoints, 0.2, 0.1))
        return false;

    return true;
}

bool Robot::FloorRobotPickandPlaceTray(int tray_id, int agv_num)
{
    // Check if kit tray is on one of the two tables
    geometry_msgs::msg::Pose tray_pose;
    std::string station;
    bool found_tray = false;

    RCLCPP_INFO(get_logger(), " - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");
    RCLCPP_INFO(get_logger(), " Pick up Tray: %i", tray_id);
    RCLCPP_INFO(get_logger(), " - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");

    // Check table 1
    for (auto tray : kts1_trays_)
    {
        // RCLCPP_INFO(get_logger(), "Table 1 Tray ID: %i", tray.id);
        if (tray.id == tray_id)
        {
            station = "kts1";
            RCLCPP_INFO(get_logger(), "Tray on Table 1");
            tray_pose = MultiplyPose(kts1_camera_pose_, tray.pose);
            found_tray = true;
            break;
        }
    }
    // Check table 2
    if (!found_tray)
    {
        for (auto tray : kts2_trays_)
        {
            // RCLCPP_INFO(get_logger(), "Table 2 Tray ID: %i", tray.id);
            if (tray.id == tray_id)
            {
                station = "kts2";
                RCLCPP_INFO(get_logger(), "Tray on Table 2");
                tray_pose = MultiplyPose(kts2_camera_pose_, tray.pose);
                found_tray = true;
                break;
            }
        }
    }
    // Not on Table 1 or Table 2
    if (!found_tray)
    {
        RCLCPP_WARN(get_logger(), "Tray not Found");
        return false;
    }

    double tray_rotation = GetYaw(tray_pose);

    // Move floor robot to the corresponding kit tray table
    if (station == "kts1")
    {
        RCLCPP_INFO(get_logger(), "Moving to Table 1");
        floor_robot_.setJointValueTarget(floor_kts1_js_);
    }
    else
    {
        RCLCPP_INFO(get_logger(), "Moving to Table 2");
        floor_robot_.setJointValueTarget(floor_kts2_js_);
    }

    RCLCPP_INFO(get_logger(), "Go to Kitting Table Prep Position");
    FloorRobotMovetoTarget();

    // Change gripper to tray gripper
    if (floor_gripper_state_.type != "tray_gripper")
    {
        RCLCPP_INFO(get_logger(), "| --------------------- | Change Gripper | --------------------- |");
        FloorRobotChangeGripper(station, "trays");
        // Return to Kitting Tray Prep Position
        RCLCPP_INFO(get_logger(), "Return to Kitting Table Prep Position");
        FloorRobotMovetoTarget();
    }

    // Move closer to Table to pcik up tray (IK)
    RCLCPP_INFO(get_logger(), "Go closer to Pick Up Tray");
    std::vector<geometry_msgs::msg::Pose> waypoints;

    waypoints.push_back(BuildPose(tray_pose.position.x, tray_pose.position.y,
                                  tray_pose.position.z + 0.2, SetRobotOrientation(tray_rotation)));
    waypoints.push_back(BuildPose(tray_pose.position.x, tray_pose.position.y,
                                  tray_pose.position.z + pick_offset_, SetRobotOrientation(tray_rotation)));
    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

    // Enable Gripper to Pick up Tray
    FloorRobotSetGripperState(true);

    // Attach Part to Gripper
    FloorRobotWaitForAttach(2.0);

    // Add kit tray to planning scene
    RCLCPP_INFO(get_logger(), "Adding Kitting Tray to Planning Scene");
    std::string tray_name = "kit_tray_" + std::to_string(tray_id);
    AddModelToPlanningScene(tray_name, "kit_tray.stl", tray_pose);
    floor_robot_.attachObject(tray_name);

    // Move up slightly (IK)
    RCLCPP_INFO(get_logger(), "Move Up slightly");
    waypoints.clear();
    waypoints.push_back(BuildPose(tray_pose.position.x, tray_pose.position.y,
                                  tray_pose.position.z + 0.2, SetRobotOrientation(tray_rotation)));
    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

    // Move to Table Prep Position (FK)
    RCLCPP_INFO(get_logger(), "Return to Kitting Table Prep Position");
    FloorRobotMovetoTarget();

    // goes to AGV Station (FK)
    RCLCPP_INFO(get_logger(), "Go to AGV Station Prep Position");
    floor_robot_.setJointValueTarget("linear_actuator_joint", rail_positions_["agv" + std::to_string(agv_num)]);
    floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
    FloorRobotMovetoTarget();

    // Go closer to AGV to drop tray (IK)
    RCLCPP_INFO(get_logger(), "Go closer to Drop Tray");
    auto agv_tray_pose = FrameWorldPose("agv" + std::to_string(agv_num) + "_tray");
    auto agv_rotation = GetYaw(agv_tray_pose);

    waypoints.clear();
    waypoints.push_back(BuildPose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                  agv_tray_pose.position.z + 0.3, SetRobotOrientation(agv_rotation)));

    waypoints.push_back(BuildPose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                  agv_tray_pose.position.z + kit_tray_thickness_ + drop_height_, SetRobotOrientation(agv_rotation)));

    FloorRobotMoveCartesian(waypoints, 0.2, 0.1);

    // Disable Gripper to Drop Tray
    RCLCPP_INFO(get_logger(), "Disable Gripper to Drop Tray");
    FloorRobotSetGripperState(false);
    floor_robot_.detachObject(tray_name);

    // Move up slightly (IK)
    RCLCPP_INFO(get_logger(), "Move Up Slightly");
    waypoints.clear();
    waypoints.push_back(BuildPose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                  agv_tray_pose.position.z + 0.2, SetRobotOrientation(agv_rotation)));
    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

    // AGV Prep Position (FK)
    RCLCPP_INFO(get_logger(), "Return to AGV Station Prep Position");
    FloorRobotMovetoTarget();

    waypoints.clear();

    return true;
}

bool Robot::FloorRobotPickConveyorPart(ariac_msgs::msg::Part part_to_pick)
{
    
    RCLCPP_INFO(get_logger(), " - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");
    RCLCPP_INFO_STREAM(get_logger(), " Attempting to pick a " << part_colors_[part_to_pick.color] << " " << part_types_[part_to_pick.type]);
    RCLCPP_INFO(get_logger(), " - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");

    // get part pose in world frame
    auto part = conveyor_camera_parts_.at(0);
    geometry_msgs::msg::Pose part_pose = MultiplyPose(conveyor_camera_pose_, part.pose);
    double part_rotation = GetYaw(part_pose);

    double pick_up_location = -1;

    // adjust robot configuration
    floor_robot_.setJointValueTarget("linear_actuator_joint", pick_up_location);
    floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 3.14);
    FloorRobotMovetoTarget();

    // move to pick up location
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(BuildPose(part_pose.position.x, -pick_up_location - conveyer_pick_offset[part_to_pick.type],
                                  part_pose.position.z + part_heights_[part_to_pick.type] + 0.01, SetRobotOrientation(part_rotation)));
    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);
    waypoints.clear();

    // pick up conveyer object
    waypoints.push_back(BuildPose(part_pose.position.x, -pick_up_location - conveyer_pick_offset[part_to_pick.type],
                                  part_pose.position.z + part_heights_[part_to_pick.type] + 0.003, SetRobotOrientation(part_rotation)));
    waypoints.push_back(BuildPose(part_pose.position.x, -pick_up_location - conveyer_pick_offset[part_to_pick.type] - 0.3,
                                  part_pose.position.z + part_heights_[part_to_pick.type] - 0.00005, SetRobotOrientation(part_rotation)));
    waypoints.push_back(BuildPose(part_pose.position.x, -pick_up_location - conveyer_pick_offset[part_to_pick.type] - 0.5,
                                  part_pose.position.z + part_heights_[part_to_pick.type] + 0.01, SetRobotOrientation(part_rotation)));
    while (true)
    {
        if (break_beam_detected == true)
        {
            FloorRobotSetGripperState(true);
            FloorRobotMoveCartesian(waypoints, 0.041, 0.1); // velocity factor was 0.023
            waypoints.clear();
            break;
        }
    }

    // Pick Up Part Pose
    geometry_msgs::msg::Pose pick_part_pose = floor_robot_.getCurrentPose().pose;

    // Add part to planning scene
    part_number_++;
    std::string part_name = part_colors_[part_to_pick.color] +
                            "_" + part_types_[part_to_pick.type] +
                            "_" + std::to_string(part_number_);

    RCLCPP_INFO(get_logger(), " - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");
    RCLCPP_INFO_STREAM(get_logger(), "Add to Scene --> " << part_colors_[part_to_pick.color] << " "
                                                         << part_types_[part_to_pick.type] << " #"
                                                         << std::to_string(part_number_));
    RCLCPP_INFO(get_logger(), " - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");
    AddModelToPlanningScene(part_name, part_types_[part_to_pick.type] + ".stl", pick_part_pose);
    floor_robot_.attachObject(part_name);
    floor_robot_attached_part_ = part_to_pick;

    // Move up slightly
    waypoints.clear();
    geometry_msgs::msg::Pose current_pose = floor_robot_.getCurrentPose().pose;
    current_pose.position.z = part_pose.position.z + part_heights_[part_to_pick.type] + 0.05;
    waypoints.push_back(current_pose);
    FloorRobotMoveCartesian(waypoints, 0.3, 0.1);
    waypoints.clear();

    RCLCPP_INFO(get_logger(), " - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");
    track_bin.print_slot_status();
    RCLCPP_INFO(get_logger(), " - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");
    RCLCPP_INFO(get_logger(), " - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");
    track_bin.bin_full_check();
    RCLCPP_INFO(get_logger(), " - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");

    // adjust robot configuration
    floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
    FloorRobotMovetoTarget();

    // Place part on empty bin
    double bin_drop_height = 0.005;
    auto bin_pose = FrameWorldPose("bin" + std::to_string(part_types_bin_[part_to_pick.type]) + "_frame");
    auto place_xy = track_bin.get_place_pose(part_types_bin_[part_to_pick.type]);

    waypoints.push_back(BuildPose(place_xy.first - conveyer_pick_offset[part_to_pick.type], 
                                    place_xy.second,
                                    bin_pose.position.z + 0.3, 
                                    SetRobotOrientation(0)));
    
    waypoints.push_back(BuildPose(place_xy.first - conveyer_pick_offset[part_to_pick.type], 
                                    place_xy.second,
                                    bin_pose.position.z + part_heights_[floor_robot_attached_part_.type] + bin_drop_height,
                                    SetRobotOrientation(0)));

    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);
    waypoints.clear();

    // Detach Gripper and Remove from Scene
    FloorRobotSetGripperState(false);
    floor_robot_.detachObject(part_name);
    planning_scene_.removeCollisionObjects(std::vector<std::string>{part_name});

    RCLCPP_INFO(get_logger(), " - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");
    RCLCPP_INFO_STREAM(get_logger(), "Remove From Scene --> " << part_colors_[floor_robot_attached_part_.color] << " "
                                                              << part_types_[floor_robot_attached_part_.type] << " #"
                                                              << std::to_string(part_number_));
    RCLCPP_INFO(get_logger(), " - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");

    // go back to home position
    FloorRobotSendHome();

    return true;
}

bool Robot::FloorRobotPickBinPart(ariac_msgs::msg::Part part_to_pick)
{
    RCLCPP_INFO(get_logger(), " - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");
    RCLCPP_INFO_STREAM(get_logger(), " Attempting to pick a " << part_colors_[part_to_pick.color] << " " << part_types_[part_to_pick.type]);
    RCLCPP_INFO(get_logger(), " - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");

    // Check if part is in one of the bins
    geometry_msgs::msg::Pose part_pose;
    bool found_part = false;
    std::string bin_side;

    // Check left bins
    for (auto part : left_bins_parts_)
    {
        if (part.part.type == part_to_pick.type && part.part.color == part_to_pick.color)
        {
            part_pose = MultiplyPose(left_bins_camera_pose_, part.pose);
            found_part = true;
            bin_side = "left_bins";
            RCLCPP_INFO(get_logger(), "Part Located in Left Bin!");
            break;
        }
    }
    // Check right bins
    if (!found_part)
    {
        for (auto part : right_bins_parts_)
        {
            if (part.part.type == part_to_pick.type && part.part.color == part_to_pick.color)
            {
                part_pose = MultiplyPose(right_bins_camera_pose_, part.pose);
                found_part = true;
                bin_side = "right_bins";
                RCLCPP_INFO(get_logger(), "Part Located in Right Bin!");
                break;
            }
        }
    }
    // If not on left or right bin
    if (!found_part)
    {
        RCLCPP_ERROR(get_logger(), "Unable to locate part");
        return false;
    }

    double part_rotation = GetYaw(part_pose);
    // Change gripper at location closest to part
    if (floor_gripper_state_.type != "part_gripper")
    {
        RCLCPP_INFO(get_logger(), "| --------- Change Gripper to Part Gripper --------- |");
        std::string station;
        if (part_pose.position.y < 0)
        {
            station = "kts1";
        }
        else
        {
            station = "kts2";
        }

        // Move floor robot to the corresponding kit tray table
        if (station == "kts1")
        {
            floor_robot_.setJointValueTarget(floor_kts1_js_);
        }
        else
        {
            floor_robot_.setJointValueTarget(floor_kts2_js_);
        }
        FloorRobotMovetoTarget();
        FloorRobotChangeGripper(station, "parts");
        FloorRobotMovetoTarget();
    }

    // Move to Bin Position (FK)
    RCLCPP_INFO(get_logger(), "Move to Bin Prep Position");
    floor_robot_.setJointValueTarget("linear_actuator_joint", rail_positions_[bin_side]);
    floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
    FloorRobotMovetoTarget();

    // Move closer to Part (IK)
    RCLCPP_INFO(get_logger(), "Move closer to the part in bin");
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                  part_pose.position.z + 0.5, SetRobotOrientation(part_rotation)));

    waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                  part_pose.position.z + part_heights_[part_to_pick.type] + pick_offset_, SetRobotOrientation(part_rotation)));

    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

    // Enable Gripper to Pick Up Part
    FloorRobotSetGripperState(true);
    // Wait 3s to attach part
    RCLCPP_INFO(get_logger(), "Pick up the part");

    if (!FloorRobotWaitForAttach(3.0))
    {
        RCLCPP_WARN(get_logger(), "Issue in Picking up Part !!!");
        return false;
    }

    // Add part to planning scene
    RCLCPP_INFO(get_logger(), "Add part to Planning Scene");
    part_number_++;
    std::string part_name = part_colors_[part_to_pick.color] +
                            "_" + part_types_[part_to_pick.type] +
                            "_" + std::to_string(part_number_);

    RCLCPP_INFO(get_logger(), " - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");
    RCLCPP_INFO_STREAM(get_logger(), "Add to Scene --> " << part_colors_[part_to_pick.color] << " "
                                                         << part_types_[part_to_pick.type] << " #"
                                                         << std::to_string(part_number_));
    RCLCPP_INFO(get_logger(), " - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");

    AddModelToPlanningScene(part_name, part_types_[part_to_pick.type] + ".stl", part_pose);
    floor_robot_.attachObject(part_name);
    floor_robot_attached_part_ = part_to_pick;

    // Move up slightly
    RCLCPP_INFO(get_logger(), "Move up slightly");
    waypoints.clear();
    waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y,
                                  part_pose.position.z + 0.3, SetRobotOrientation(0)));

    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

    // Move to Bin Position (FK)
    RCLCPP_INFO(get_logger(), "Return to Bin Prep Position");
    FloorRobotMovetoTarget();

    if (!floor_gripper_state_.attached)
    {
        RCLCPP_WARN(get_logger(), "Issue in Picking up Part !!!");
        return false;
    }

    return true;
}

bool Robot::FloorRobotPlacePartOnKitTray(int agv_num, int quadrant)
{
    RCLCPP_INFO(get_logger(), " - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");
    RCLCPP_INFO(get_logger(), " Place Part on AGV ");
    RCLCPP_INFO(get_logger(), " - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");
    if (!floor_gripper_state_.attached)
    {
        RCLCPP_ERROR(get_logger(), "No part attached");
        return false;
    }

    // Move to AGV Prep Position (FK)
    RCLCPP_INFO(get_logger(), "Go to AGV Station Prep Position");
    floor_robot_.setJointValueTarget("linear_actuator_joint", rail_positions_["agv" + std::to_string(agv_num)]);
    floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
    FloorRobotMovetoTarget();

    // Determine target pose for part based on agv_tray pose
    auto agv_tray_pose = FrameWorldPose("agv" + std::to_string(agv_num) + "_tray");
    auto part_drop_offset = BuildPose(quad_offsets_[quadrant].first, quad_offsets_[quadrant].second, 0.0,
                                      geometry_msgs::msg::Quaternion());
    auto part_drop_pose = MultiplyPose(agv_tray_pose, part_drop_offset);

    // Move closer to AGV to place part
    RCLCPP_INFO(get_logger(), "Go closer to Drop Part");
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                  part_drop_pose.position.z + 0.3, SetRobotOrientation(0)));

    waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                  part_drop_pose.position.z + part_heights_[floor_robot_attached_part_.type] + drop_height_,
                                  SetRobotOrientation(0)));
    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

    // Disable Gripper to Drop Part
    RCLCPP_INFO(get_logger(), "Disable Gripper to Drop Part");
    FloorRobotSetGripperState(false);

    // Remove part from Planning Scene
    std::string part_name = part_colors_[floor_robot_attached_part_.color] +
                            "_" + part_types_[floor_robot_attached_part_.type] +
                            "_" + std::to_string(part_number_);

    RCLCPP_INFO(get_logger(), " - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");
    RCLCPP_INFO_STREAM(get_logger(), "Remove From Scene --> " << part_colors_[floor_robot_attached_part_.color] << " "
                                                              << part_types_[floor_robot_attached_part_.type] << " #"
                                                              << std::to_string(part_number_));
    RCLCPP_INFO(get_logger(), " - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");
    floor_robot_.detachObject(part_name);
    planning_scene_.removeCollisionObjects(std::vector<std::string>{part_name});

    // Move slightly up
    RCLCPP_INFO(get_logger(), "Move Up Slightly");
    waypoints.clear();
    waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
                                  part_drop_pose.position.z + 0.3,
                                  SetRobotOrientation(0)));
    FloorRobotMoveCartesian(waypoints, 0.2, 0.1);

    // Return to AGV Prep Position
    RCLCPP_INFO(get_logger(), "Return to AGV Station Prep Position");
    FloorRobotMovetoTarget();

    return true;
}

// ----------------------------------------------------------------------------- //
// | ---------------------- | Ceiling Robot Functions | ---------------------- | //
// ----------------------------------------------------------------------------- //

void Robot::CeilingRobotSendHome()
{
    // Move ceiling robot to home joint state
    ceiling_robot_.setNamedTarget("home");
    CeilingRobotMovetoTarget();
}

bool Robot::CeilingRobotSetGripperState(bool enable)
{
    if (ceiling_gripper_state_.enabled == enable)
    {
        if (ceiling_gripper_state_.enabled)
            RCLCPP_INFO(get_logger(), "Already enabled");
        else
            RCLCPP_INFO(get_logger(), "Already disabled");

        return false;
    }

    // Call enable service
    auto request = std::make_shared<ariac_msgs::srv::VacuumGripperControl::Request>();
    request->enable = enable;

    auto result = ceiling_robot_gripper_enable_->async_send_request(request);
    result.wait();

    if (!result.get()->success)
    {
        RCLCPP_ERROR(get_logger(), "Error calling gripper enable service");
        return false;
    }

    return true;
}

void Robot::CeilingRobotWaitForAttach(double timeout)
{
    // Wait for part to be attached
    rclcpp::Time start = now();
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose starting_pose = ceiling_robot_.getCurrentPose().pose;

    while (!ceiling_gripper_state_.attached)
    {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");

        waypoints.clear();
        starting_pose.position.z -= 0.001;
        waypoints.push_back(starting_pose);

        CeilingRobotMoveCartesian(waypoints, 0.01, 0.01, false);

        usleep(200);

        if (now() - start > rclcpp::Duration::from_seconds(timeout))
        {
            RCLCPP_ERROR(get_logger(), "Unable to pick up object");
            return;
        }
    }
}

bool Robot::CeilingRobotWaitForAssemble(int station, ariac_msgs::msg::AssemblyPart part)
{
    // Wait for part to be attached
    rclcpp::Time start = now();
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose starting_pose = ceiling_robot_.getCurrentPose().pose;

    bool assembled = false;
    while (!assembled)
    {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for part to be assembled");

        // Check if part is assembled
        switch (part.part.type)
        {
        case ariac_msgs::msg::Part::BATTERY:
            assembled = assembly_station_states_[station].battery_attached;
            break;
        case ariac_msgs::msg::Part::PUMP:
            assembled = assembly_station_states_[station].pump_attached;
            break;
        case ariac_msgs::msg::Part::SENSOR:
            assembled = assembly_station_states_[station].sensor_attached;
            break;
        case ariac_msgs::msg::Part::REGULATOR:
            assembled = assembly_station_states_[station].regulator_attached;
            break;
        default:
            RCLCPP_WARN(get_logger(), "Not a valid part type");
            return false;
        }

        double step = 0.0005;

        waypoints.clear();
        starting_pose.position.x += step * part.install_direction.x;
        starting_pose.position.y += step * part.install_direction.y;
        starting_pose.position.z += step * part.install_direction.z;
        waypoints.push_back(starting_pose);

        CeilingRobotMoveCartesian(waypoints, 0.01, 0.01, false);

        usleep(500);

        if (now() - start > rclcpp::Duration::from_seconds(5))
        {
            RCLCPP_ERROR(get_logger(), "Unable to assemble object");
            ceiling_robot_.stop();
            return false;
        }
    }

    RCLCPP_INFO(get_logger(), "Part is assembled");

    return true;
}

bool Robot::CeilingRobotMovetoTarget()
{
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(ceiling_robot_.plan(plan));

    if (success)
    {
        return static_cast<bool>(ceiling_robot_.execute(plan));
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Unable to generate plan");
        return false;
    }
}

bool Robot::CeilingRobotMoveCartesian(
    std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf, bool avoid_collisions)
{
    moveit_msgs::msg::RobotTrajectory trajectory;

    double path_fraction = ceiling_robot_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory, avoid_collisions);

    if (path_fraction < 0.9)
    {
        RCLCPP_ERROR(get_logger(), "Unable to generate trajectory through waypoints");
        return false;
    }

    // Retime trajectory
    robot_trajectory::RobotTrajectory rt(ceiling_robot_.getCurrentState()->getRobotModel(), "ceiling_robot");
    rt.setRobotTrajectoryMsg(*ceiling_robot_.getCurrentState(), trajectory);
    totg_.computeTimeStamps(rt, vsf, asf);
    rt.getRobotTrajectoryMsg(trajectory);

    return static_cast<bool>(ceiling_robot_.execute(trajectory));
}

bool Robot::CeilingRobotMoveToAssemblyStation(int station)
{
    switch (station)
    {
    case 1:
        ceiling_robot_.setJointValueTarget(ceiling_as1_js_);
        break;
    case 2:
        ceiling_robot_.setJointValueTarget(ceiling_as2_js_);
        break;
    case 3:
        ceiling_robot_.setJointValueTarget(ceiling_as3_js_);
        break;
    case 4:
        ceiling_robot_.setJointValueTarget(ceiling_as4_js_);
        break;
    default:
        RCLCPP_WARN(get_logger(), "Not a valid assembly station");
        return false;
    }

    return CeilingRobotMovetoTarget();
}

bool Robot::CeilingRobotPickAGVPart(ariac_msgs::msg::PartPose part)
{
    double part_rotation = GetYaw(part.pose);
    std::vector<geometry_msgs::msg::Pose> waypoints;

    double dx = 0;
    double dy = 0;

    if (part.part.type == ariac_msgs::msg::Part::BATTERY)
    {
        dx = battery_grip_offset_ * cos(part_rotation);
        dy = battery_grip_offset_ * sin(part_rotation);
    }

    waypoints.push_back(BuildPose(part.pose.position.x + dx, part.pose.position.y + dy,
                                  part.pose.position.z + 0.4, SetRobotOrientation(part_rotation)));

    waypoints.push_back(BuildPose(part.pose.position.x + dx, part.pose.position.y + dy,
                                  part.pose.position.z + part_heights_[part.part.type] + pick_offset_, SetRobotOrientation(part_rotation)));

    CeilingRobotMoveCartesian(waypoints, 0.7, 0.7, true);

    CeilingRobotSetGripperState(true);

    CeilingRobotWaitForAttach(3.0);

    // Add part to planning scene
    std::string part_name = part_colors_[part.part.color] + "_" + part_types_[part.part.type];
    AddModelToPlanningScene(part_name, part_types_[part.part.type] + ".stl", part.pose);
    ceiling_robot_.attachObject(part_name);
    ceiling_robot_attached_part_ = part.part;

    // Move up slightly
    auto current_pose = ceiling_robot_.getCurrentPose().pose;
    current_pose.position.z += 0.2;

    waypoints.clear();
    waypoints.push_back(current_pose);

    CeilingRobotMoveCartesian(waypoints, 0.7, 0.7, true);

    return true;
}

bool Robot::CeilingRobotAssemblePart(int station, ariac_msgs::msg::AssemblyPart part)
{
    // Check that part is attached and matches part to assemble
    if (!ceiling_gripper_state_.attached)
    {
        RCLCPP_WARN(get_logger(), "No part attached");
        return false;
    }

    if (part.part != ceiling_robot_attached_part_)
    {
        RCLCPP_WARN(get_logger(), "Incorrect part attached for this assembly");
        return false;
    }

    // Calculate assembled pose in world frame
    std::string insert_frame_name;
    switch (station)
    {
    case 1:
        insert_frame_name = "as1_insert_frame";
        break;
    case 2:
        insert_frame_name = "as2_insert_frame";
        break;
    case 3:
        insert_frame_name = "as3_insert_frame";
        break;
    case 4:
        insert_frame_name = "as4_insert_frame";
        break;
    default:
        RCLCPP_WARN(get_logger(), "Not a valid assembly station");
        return false;
    }

    // Calculate robot positions at assembly and approach
    KDL::Vector install(part.install_direction.x, part.install_direction.y, part.install_direction.z);

    KDL::Frame insert;
    tf2::fromMsg(FrameWorldPose(insert_frame_name), insert);

    KDL::Frame part_assemble;
    tf2::fromMsg(part.assembled_pose.pose, part_assemble);

    KDL::Frame part_to_gripper;

    // Build approach waypoints
    std::vector<geometry_msgs::msg::Pose> waypoints;
    if (part.part.type == ariac_msgs::msg::Part::BATTERY)
    {
        tf2::fromMsg(BuildPose(battery_grip_offset_, 0, part_heights_[part.part.type], QuaternionFromRPY(M_PI, 0, M_PI)), part_to_gripper);

        KDL::Vector up(0, 0, 0.1);
        waypoints.push_back(tf2::toMsg(insert * KDL::Frame(up) * KDL::Frame(install * -0.06) * part_assemble * part_to_gripper));
        waypoints.push_back(tf2::toMsg(insert * KDL::Frame(install * -0.06) * part_assemble * part_to_gripper));
    }
    else
    {
        tf2::fromMsg(BuildPose(0, 0, part_heights_[part.part.type], QuaternionFromRPY(M_PI, 0, M_PI)), part_to_gripper);

        waypoints.push_back(tf2::toMsg(insert * KDL::Frame(install * -0.1) * part_assemble * part_to_gripper));
    }

    // Move to approach position
    CeilingRobotMoveCartesian(waypoints, 0.3, 0.3, true);

    // Move to just before assembly position
    waypoints.clear();
    waypoints.push_back(tf2::toMsg(insert * KDL::Frame(install * -0.003) * part_assemble * part_to_gripper));
    CeilingRobotMoveCartesian(waypoints, 0.1, 0.1, true);

    CeilingRobotWaitForAssemble(station, part);

    CeilingRobotSetGripperState(false);

    std::string part_name = part_colors_[ceiling_robot_attached_part_.color] +
                            "_" + part_types_[ceiling_robot_attached_part_.type];
    ceiling_robot_.detachObject(part_name);

    // Move away slightly
    auto current_pose = ceiling_robot_.getCurrentPose().pose;

    if (part.part.type == ariac_msgs::msg::Part::REGULATOR)
    {
        current_pose.position.x -= 0.05;
    }
    else
    {
        current_pose.position.z += 0.1;
    }

    waypoints.clear();
    waypoints.push_back(current_pose);

    CeilingRobotMoveCartesian(waypoints, 0.3, 0.3, true);

    return true;
}

// ---------------------------------------------------------------------------- //
// | --------------------------- | Perform Task | --------------------------- | //
// ---------------------------------------------------------------------------- //

// bool Robot::CompleteOrders()
// {
//     // Wait for first order to be published
//     while (priority_orders_.size() == 0){
//     }
//     bool success;
//     while (true)
//     {
//         if (competition_state_ == ariac_msgs::msg::CompetitionState::ENDED)
//         {
//             success = false;
//             break;
//         }
//         // complete each order from the queue
//         if (priority_orders_.size() == 0)
//         {
//             if (competition_state_ != ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE)
//             {
//                 // wait for more orders
//                 RCLCPP_INFO(get_logger(), "Waiting for orders...");
//                 while (priority_orders_.size() == 0)
//                 {
//                 }
//             }
//             else
//             {
//                 RCLCPP_INFO(get_logger(), "Completed all orders");
//                 success = true;
//                 break;
//             }
//         }
//         current_order_ = priority_orders_.front();
//         priority_orders_.erase(priority_orders_.begin());
//         if (current_order_.type == ariac_msgs::msg::Order::KITTING)
//         {
//           RCLCPP_INFO(get_logger(), " ------------------------------------------- ");
//           RCLCPP_INFO(get_logger(), " Kitting Task ");
//           RCLCPP_INFO(get_logger(), " ------------------------------------------- ");
//           if (!conveyor_parts_picked){
//             Robot::PickUpConveyor(current_order_.kitting_task);
//             Robot::CompleteKittingTask(current_order_.kitting_task);
//             conveyor_parts_picked= true;
//             }
//           else{
//             Robot::CompleteKittingTask(current_order_.kitting_task);
//           }
//         }
//         if (current_order_.type == ariac_msgs::msg::Order::COMBINED)
//         {
//           RCLCPP_INFO(get_logger(), " ------------------------------------------- ");
//           RCLCPP_INFO(get_logger(), " Combined Task ");
//           RCLCPP_INFO(get_logger(), " ------------------------------------------- ");
//           if (!conveyor_parts_picked){
//             Robot::PickUpConveyor(current_order_.combined_task);
//             Robot::CompleteCombinedTask(current_order_.combined_task);
//             conveyor_parts_picked= true;
//             }
//           else{
//             Robot::CompleteCombinedTask(current_order_.combined_task);
//           }
//         }
//         // publish status
//         auto completed_order = group5_msgs::msg::CompletedOrder();
//         completed_order.order_id = current_order_.id;
//         completed_order_pub_->publish(completed_order);
//         Robot::FloorRobotSendHome();
//     }
//     RCLCPP_INFO(get_logger(), " - Publishing Order to Topic: /competitor/floor_robot_task");
//     RCLCPP_INFO(get_logger(), " - Going Back to Home Position");
//     return success;
// }

bool Robot::CompleteOrders()
{
    // Wait for first order to be published
    while (priority_orders_.empty() && orders_.empty())
    {
        // Add any additional logic here if needed
    }

    bool success = true;
    while (true)
    {
        if (competition_state_ == ariac_msgs::msg::CompetitionState::ENDED)
        {
            RCLCPP_INFO(get_logger(), "Shutdown Robot Executioner Node");
            rclcpp::sleep_for(std::chrono::seconds(2));
            success = false;
            break;
        }

        // Complete each order from the priority queue first
        if (!priority_orders_.empty())
        {
            current_order_ = priority_orders_.front();
            priority_orders_.erase(priority_orders_.begin());

            if (current_order_.type == ariac_msgs::msg::Order::KITTING)
            {
                RCLCPP_INFO(get_logger(), " ------------------------------------------- ");
                RCLCPP_INFO(get_logger(), " Kitting Task ");
                RCLCPP_INFO(get_logger(), " ------------------------------------------- ");
                if (!conveyor_parts_picked)
                {
                    Robot::PickUpConveyor(current_order_.kitting_task);
                    conveyor_parts_picked = true;
                    bool kitting_completed = Robot::CompleteKittingTask(current_order_.kitting_task);
                    if (kitting_completed)
                    {

                        auto completed_order = group5_msgs::msg::CompletedOrder();
                        completed_order.order_id = current_order_.id;
                        completed_order_pub_->publish(completed_order);
                        Robot::FloorRobotSendHome();
                    }
                }
                else
                {
                    bool kitting_completed = Robot::CompleteKittingTask(current_order_.kitting_task);

                    if (kitting_completed)
                    {
                        auto completed_order = group5_msgs::msg::CompletedOrder();
                        completed_order.order_id = current_order_.id;
                        completed_order_pub_->publish(completed_order);
                        Robot::FloorRobotSendHome();
                    }
                }
            }
            else if (current_order_.type == ariac_msgs::msg::Order::COMBINED)
            {
                RCLCPP_INFO(get_logger(), " ------------------------------------------- ");
                RCLCPP_INFO(get_logger(), " Combined Task ");
                RCLCPP_INFO(get_logger(), " ------------------------------------------- ");
                if (!conveyor_parts_picked)
                {
                    Robot::PickUpConveyor(current_order_.combined_task);
                    conveyor_parts_picked = true;
                    bool completed_combined = Robot::CompleteCombinedTask(current_order_.combined_task);

                    if (completed_combined)
                    {

                        auto completed_order = group5_msgs::msg::CompletedOrder();
                        completed_order.order_id = current_order_.id;
                        completed_order_pub_->publish(completed_order);
                        Robot::FloorRobotSendHome();
                    }
                }
                else
                {
                    bool completed_combined = Robot::CompleteCombinedTask(current_order_.combined_task);

                    if (completed_combined)
                    {

                        auto completed_order = group5_msgs::msg::CompletedOrder();
                        completed_order.order_id = current_order_.id;
                        completed_order_pub_->publish(completed_order);
                        Robot::FloorRobotSendHome();
                    }
                }
            }

            // Publish status and send robot home
        }
        else if (!orders_.empty())
        {
            current_order_ = orders_.front();
            orders_.erase(orders_.begin());

            // Process regular order logic here

            if (current_order_.type == ariac_msgs::msg::Order::KITTING)
            {
                RCLCPP_INFO(get_logger(), " ------------------------------------------- ");
                RCLCPP_INFO(get_logger(), " Kitting Task ");
                RCLCPP_INFO(get_logger(), " ------------------------------------------- ");
                if (!conveyor_parts_picked)
                {
                    Robot::PickUpConveyor(current_order_.kitting_task);

                    conveyor_parts_picked = true;
                    bool kitting_completed = Robot::CompleteKittingTask(current_order_.kitting_task);

                    if (kitting_completed)
                    {
                        auto completed_order = group5_msgs::msg::CompletedOrder();
                        completed_order.order_id = current_order_.id;
                        completed_order_pub_->publish(completed_order);
                        Robot::FloorRobotSendHome();
                    }
                }
                else
                {
                    bool kitting_completed = Robot::CompleteKittingTask(current_order_.kitting_task);

                    if (kitting_completed)
                    {
                        auto completed_order = group5_msgs::msg::CompletedOrder();
                        completed_order.order_id = current_order_.id;
                        completed_order_pub_->publish(completed_order);
                        Robot::FloorRobotSendHome();
                    }
                }
            }
            else if (current_order_.type == ariac_msgs::msg::Order::COMBINED)
            {
                RCLCPP_INFO(get_logger(), " ------------------------------------------- ");
                RCLCPP_INFO(get_logger(), " Combined Task ");
                RCLCPP_INFO(get_logger(), " ------------------------------------------- ");
                if (!conveyor_parts_picked)
                {
                    Robot::PickUpConveyor(current_order_.combined_task);
                    conveyor_parts_picked = true;
                    bool completed_combined = Robot::CompleteCombinedTask(current_order_.combined_task);

                    if (completed_combined == true)
                    {
                        auto completed_order = group5_msgs::msg::CompletedOrder();
                        completed_order.order_id = current_order_.id;
                        completed_order_pub_->publish(completed_order);
                        Robot::FloorRobotSendHome();
                    }
                }
                else
                {
                    bool completed_combined = Robot::CompleteCombinedTask(current_order_.combined_task);

                    if (completed_combined)
                    {

                        auto completed_order = group5_msgs::msg::CompletedOrder();
                        completed_order.order_id = current_order_.id;
                        completed_order_pub_->publish(completed_order);
                        Robot::FloorRobotSendHome();
                    }
                }
            }

            // Publish status and send robot home for regular orders
        }
        else if (competition_state_ == ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE)
        {
            RCLCPP_INFO(get_logger(), "Completed all orders");
            break;
        }
        else
        {
            // Wait for more orders
            RCLCPP_INFO(get_logger(), "Waiting for orders...");
            while (priority_orders_.empty() && orders_.empty())
            {
            }
        }
    }

    return success;
}

template <typename T>
void Robot::PickUpConveyor(T task)
{
    FloorRobotSendHome();
    // check parts not on bins
    std::vector<ariac_msgs::msg::Part> conveyor_parts = {};
    std::vector<ariac_msgs::msg::Part> bin_parts = {};
    // store bin parts into 2D array
    int bin_parts_array[5][4] = {};
    for (auto part : left_bins_parts_)
    {
        bin_parts_array[part.part.color][part.part.type - 10] += 1;
    }
    for (auto part : right_bins_parts_)
    {
        bin_parts_array[part.part.color][part.part.type - 10] += 1;
    }

    // store task parts into 2D array
    int task_parts_array[5][4] = {};
    for (auto part : task.parts)
    {
        task_parts_array[part.part.color][part.part.type - 10] += 1;
    }

    // push bin parts and conveyer parts
    int remaining_parts_array[5][4] = {};
    for (int i = 0; i < 5; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            if (bin_parts_array[i][j] > 0)
            {
                ariac_msgs::msg::Part part_bin;
                part_bin.color = i;
                part_bin.type = j + 10;
                bin_parts.push_back(part_bin);
            }
            remaining_parts_array[i][j] = task_parts_array[i][j] - bin_parts_array[i][j];
            if (remaining_parts_array[i][j] > 0)
            {
                ariac_msgs::msg::Part part_conveyer;
                part_conveyer.color = i;
                part_conveyer.type = j + 10;
                conveyor_parts.push_back(part_conveyer);
            }
        }
    }

    // pick up parts that are not on the bin from the conveyer and place on the bin

    while (conveyor_parts.size() > 0)
    {
        if (conveyor_part_detected)
        {
            // Change gripper to part gripper
            if (floor_gripper_state_.type != "part_gripper")
            {
                std::string station;
                // Move floor robot to the kit tray table 1
                floor_robot_.setJointValueTarget(floor_kts1_js_);
                FloorRobotMovetoTarget();
                FloorRobotChangeGripper("kts1", "parts");
            }
            for (int i = 0; i < (int)conveyor_parts.size(); i++)
            {
                auto part = conveyor_parts.at(i);
                if (current_conveyor_part.part.color == part.color && current_conveyor_part.part.type == part.type)
                {
                    FloorRobotPickConveyorPart(part);
                    conveyor_parts.erase(conveyor_parts.begin() + i);
                    goto cnt;
                }
            }
        }
    cnt:;
    }
}

bool Robot::CompleteKittingTask(ariac_msgs::msg::KittingTask task)
{

    // If a Priority Order
    if (current_order_.priority == true)
    {
        RCLCPP_INFO(get_logger(), " ========================================================= ");
        RCLCPP_INFO(get_logger(), " ||               Kitting Order - Priority              || ");
        RCLCPP_INFO(get_logger(), " ========================================================= ");

        FloorRobotSendHome();
        FloorRobotPickandPlaceTray(task.tray_id, task.agv_number);

        for (auto kit_part : task.parts)
        {
            FloorRobotPickBinPart(kit_part.part);
            FloorRobotPlacePartOnKitTray(task.agv_number, kit_part.quadrant);
        }

        KittingPartQualityCheck(task);
        MoveAGV(task.agv_number, task.destination);

        return true;
    }

    // If a Regualar Order
    else
    {
        RCLCPP_INFO(get_logger(), " ========================================================= ");
        RCLCPP_INFO(get_logger(), " ||                Kitting Order - Regular              || ");
        RCLCPP_INFO(get_logger(), " ========================================================= ");

        auto resumeFromMethod = current_order_.step;
        RCLCPP_INFO(get_logger(), " resume from step %s", resumeFromMethod.c_str());

        if (priority_orders_.size() == 0) // priority check
        {
            RCLCPP_INFO(get_logger(), " ---------------------------------------------------------------------------- ");
            RCLCPP_INFO(get_logger(), "     --> Execute Robot to home position");
            RCLCPP_INFO(get_logger(), " ---------------------------------------------------------------------------- ");

            FloorRobotSendHome();
            if (priority_orders_.size() != 0) // priority check
            {
                current_order_.step = "FloorRobotPickandPlaceTray";
                orders_.insert(orders_.begin(), current_order_);
                return false;
            }
            else if (current_order_.step.empty())
            {
                resumeFromMethod = "FloorRobotPickandPlaceTray";
            }
        }

        if (resumeFromMethod == "FloorRobotPickandPlaceTray")
        {
            RCLCPP_INFO(get_logger(), " ---------------------------------------------------------------------------- ");
            RCLCPP_INFO(get_logger(), "      --> Pick and Place Tray non priority order");
            RCLCPP_INFO(get_logger(), " ---------------------------------------------------------------------------- ");
            FloorRobotPickandPlaceTray(task.tray_id, task.agv_number);

            if (priority_orders_.size() != 0) // priority check
            {
                RCLCPP_INFO(get_logger(), " ---------------------------------------------------------------------------- ");
                RCLCPP_INFO(get_logger(), "     -->  Priority order recieved while Pick and place tray");
                RCLCPP_INFO(get_logger(), " ---------------------------------------------------------------------------- ");
                current_order_.step = "FloorRobotPickandPlacePart";
                orders_.insert(orders_.begin(), current_order_);
                return false;
            }
            else
            {
                resumeFromMethod = "FloorRobotPickandPlacePart";
            }
        }

        if (resumeFromMethod == "FloorRobotPickandPlacePart")
        {
            RCLCPP_INFO(get_logger(), " ---------------------------------------------------------------------------- ");
            RCLCPP_INFO(get_logger(), "     -->  No priority Pick and place parts");
            RCLCPP_INFO(get_logger(), " ---------------------------------------------------------------------------- ");

            for (auto kit_part : task.parts)
            {
                if (partTrack[kit_part.part.type] == current_order_.id)
                {
                    RCLCPP_INFO(get_logger(), " Continue ");
                    continue;
                }

                RCLCPP_INFO(get_logger(), " Pick and Place parts on tray ");
                FloorRobotPickBinPart(kit_part.part);
                FloorRobotPlacePartOnKitTray(task.agv_number, kit_part.quadrant);
                partTrack[kit_part.part.type] = current_order_.id;

                if (priority_orders_.size() != 0) // priority check
                {
                    RCLCPP_INFO(get_logger(), " Still no priority orders ");
                    if (partTrack.size() != task.parts.size())
                    {
                        current_order_.step = "FloorRobotPickandPlacePart";
                        RCLCPP_INFO(get_logger(), " ---------------------------------------------------------------------------- ");
                        RCLCPP_INFO(get_logger(), "     -->  Priority order recieved while picking and placing bin part");
                        RCLCPP_INFO(get_logger(), " ---------------------------------------------------------------------------- ");
                        orders_.insert(orders_.begin(), current_order_);
                        return false;
                    }
                    else
                    {
                        partTrack.clear();
                        current_order_.step = "PerformQualityCheck";
                        RCLCPP_INFO(get_logger(), " ---------------------------------------------------------------------------- ");
                        RCLCPP_INFO(get_logger(), "     -->  Priority order recieved while picking and placing bin part after every part");
                        RCLCPP_INFO(get_logger(), " ---------------------------------------------------------------------------- ");
                        orders_.insert(orders_.begin(), current_order_);
                        return false; // needed here??
                    }
                }
            }
        }
        if (resumeFromMethod == "PerformQualityCheck")
        {
            KittingPartQualityCheck(task);

            if (priority_orders_.size() != 0) // priority check
            {
                RCLCPP_INFO(get_logger(), " ---------------------------------------------------------------------------- ");
                RCLCPP_INFO(get_logger(), "     -->  Priority order recieved while Quality check");
                RCLCPP_INFO(get_logger(), " ---------------------------------------------------------------------------- ");
                current_order_.step = "MoveAgv";
                orders_.insert(orders_.begin(), current_order_);
                return false;
            }
            else
            {
                resumeFromMethod = "MoveAGV";
            }
        }

        if (resumeFromMethod == "MoveAGV")
        {
            RCLCPP_INFO(get_logger(), " ---------------------------------------------------------------------------- ");
            RCLCPP_INFO(get_logger(), "     -->  No priority order MoveAGV");
            RCLCPP_INFO(get_logger(), " ---------------------------------------------------------------------------- ");
            MoveAGV(task.agv_number, task.destination);
        }

        RCLCPP_INFO(get_logger(), " ---------------------------------------------------------------------------- ");
        RCLCPP_INFO(get_logger(), "     -->  Moving AGV to Destination ");
        RCLCPP_INFO(get_logger(), " ---------------------------------------------------------------------------- ");
        agv_availability[std::to_string(task.agv_number)] = false;
        MoveAGV(task.agv_number, task.destination);

        return true;
    }
}

bool Robot::CompleteCombinedTask(ariac_msgs::msg::CombinedTask task)
{
    // Decide on a tray to use
    // int id;

    RCLCPP_INFO(get_logger(), " ========================================================= ");
    RCLCPP_INFO(get_logger(), " ||                   Combined Order                    || ");
    RCLCPP_INFO(get_logger(), " ========================================================= ");

    if (current_order_.priority == true)
    {
        if (kts1_trays_.size() != 0)
        {
            RCLCPP_INFO(get_logger(), "Trays on Table 1.");
            // id = kts1_trays_[0].id;
        }
        else if (kts2_trays_.size() != 0)
        {
            RCLCPP_INFO(get_logger(), "Trays on Table 2.");
            // id = kts2_trays_[0].id;
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "No trays available.");
            return false;
        }

        RCLCPP_INFO(get_logger(), " ========================================================= ");
        for (const auto &entry : agv_availability)
        {
            RCLCPP_INFO(get_logger(), "|| %s: %d", entry.first.c_str(), entry.second);
        }
        RCLCPP_INFO(get_logger(), " ========================================================= ");

        // Decide which AGV to use
        int agv_number;
        if (task.station == ariac_msgs::msg::CombinedTask::AS1 or task.station == ariac_msgs::msg::CombinedTask::AS2)
        {
            if (agv_availability["1"] == 1)
            {
                agv_number = 1;
            }
            else
            {
                agv_number = 2;
            }
        }
        else
        {
            if (agv_availability["4"] == 1)
            {
                agv_number = 4;
            }
            else
            {
                agv_number = 3;
            }
        }

        // Bring AGV back to AGV
        MoveAGV(agv_number, ariac_msgs::srv::MoveAGV::Request::KITTING);

        // Pick and Place Tray
        FloorRobotPickandPlaceTray(0, agv_number);

        int count = 1;
        // Pick and Place Parts on Tray
        for (auto assembly_part : task.parts)
        {
            FloorRobotPickBinPart(assembly_part.part);
            FloorRobotPlacePartOnKitTray(agv_number, count);
            count++;
        }

        int destination;
        // Pick Destination based on Assembly Station
        if (task.station == ariac_msgs::msg::CombinedTask::AS1 or task.station == ariac_msgs::msg::CombinedTask::AS3)
        {
            destination = ariac_msgs::srv::MoveAGV::Request::ASSEMBLY_FRONT;
        }
        else
        {
            destination = ariac_msgs::srv::MoveAGV::Request::ASSEMBLY_BACK;
        }

        //  Move AGV to assembly station
        MoveAGV(agv_number, destination);

        CeilingRobotMoveToAssemblyStation(task.station);

        // Get Assembly Poses
        auto request = std::make_shared<ariac_msgs::srv::GetPreAssemblyPoses::Request>();
        request->order_id = current_order_.id;
        auto result = pre_assembly_poses_getter_->async_send_request(request);

        result.wait();

        std::vector<ariac_msgs::msg::PartPose> agv_part_poses;
        if (result.get()->valid_id)
        {
            agv_part_poses = result.get()->parts;
            if (agv_part_poses.size() == 0)
            {
                RCLCPP_WARN(get_logger(), "No part poses recieved");
                return false;
            }
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Not a valid order ID");
            return false;
        }

        for (auto const &part_to_assemble : task.parts)
        {
            // Check if matching part exists in agv_parts
            bool part_exists = false;
            ariac_msgs::msg::PartPose part_to_pick;
            part_to_pick.part = part_to_assemble.part;
            for (auto const &agv_part : agv_part_poses)
            {
                if (agv_part.part.type == part_to_assemble.part.type && agv_part.part.color == part_to_assemble.part.color)
                {
                    part_exists = true;
                    part_to_pick.pose = agv_part.pose;
                    break;
                }
            }

            if (!part_exists)
            {
                RCLCPP_WARN_STREAM(get_logger(), "Part with type: " << part_to_assemble.part.type << " and color: " << part_to_assemble.part.color << " not found on tray");
                continue;
            }

            // Pick up part
            CeilingRobotPickAGVPart(part_to_pick);
            CeilingRobotMoveToAssemblyStation(task.station);

            // Assemble Part to insert
            CeilingRobotAssemblePart(task.station, part_to_assemble);
            CeilingRobotMoveToAssemblyStation(task.station);
        }

        return true;
    }

    else
    {

        // Decide which AGV to use
        RCLCPP_ERROR(get_logger(), "                      Not Priority                         ");
        RCLCPP_ERROR(get_logger(), " ========================================================= ");
        for (const auto &entry : agv_availability)
        {
            RCLCPP_INFO(get_logger(), "|| %s: %d", entry.first.c_str(), entry.second);
        }
        RCLCPP_ERROR(get_logger(), " ========================================================= ");

        // Decide which AGV to use
        int agv_number;
        if (task.station == ariac_msgs::msg::CombinedTask::AS1 or task.station == ariac_msgs::msg::CombinedTask::AS2)
        {
            if (agv_availability["1"] == true)
            {
                agv_number = 1;
            }
            else
            {
                agv_number = 2;
            }
        }
        else
        {
            if (agv_availability["4"] == true)
            {
                agv_number = 4;
            }
            else
            {
                agv_number = 3;
            }
        }

        auto resumeFromMethod = current_order_.step;
        if (priority_orders_.size() == 0 && resumeFromMethod.empty())
        {
            MoveAGV(agv_number, ariac_msgs::srv::MoveAGV::Request::KITTING);
            RCLCPP_INFO(get_logger(), " - Moved Agv");

            if (priority_orders_.size() != 0)
            {
                RCLCPP_INFO(get_logger(), " - Priority order recieved after moving agv");
                current_order_.step = "FloorRobotPickandPlaceTray";
                orders_.insert(orders_.begin(), current_order_);
                return false;
            }
            else
            {
                resumeFromMethod = "FloorRobotPickandPlaceTray";
            }
        }

        if (resumeFromMethod == "FloorRobotPickandPlaceTray")
        {
            FloorRobotPickandPlaceTray(0, agv_number);
            RCLCPP_INFO(get_logger(), " - Placed tray for the order combined");
            if (priority_orders_.size() != 0)
            {
                current_order_.step = "FloorRobotPickandPlacePart";
                orders_.insert(orders_.begin(), current_order_);

                RCLCPP_INFO(get_logger(), " - Priority order recieved after placing the tray");
                return false;
            }

            else
            {
                resumeFromMethod = "FloorRobotPickandPlacePart";
            }
        }

        if (resumeFromMethod == "FloorRobotPickandPlacePart")
        {
            for (auto assembly_part : task.parts)
            {
                if (partTrack[assembly_part.part.type] == current_order_.id)
                {
                    continue;
                }
                // int count=partTrack.size()+1;
                // FloorRobotPickBinPart(assembly_part.part);
                // FloorRobotPlacePartOnKitTray(agv_number,count);
                //  RCLCPP_INFO(get_logger(), count);
                int count = 14 - assembly_part.part.type;
                FloorRobotPickBinPart(assembly_part.part);
                FloorRobotPlacePartOnKitTray(agv_number, count);
                RCLCPP_INFO(get_logger(), "%d", count);

                partTrack[assembly_part.part.type] = current_order_.id;

                if (priority_orders_.size() != 0)
                {

                    if (partTrack.size() != task.parts.size())
                    {

                        current_order_.step = "FloorRobotPickandPlacePart";
                        RCLCPP_INFO(get_logger(), " - Priority order recieved while picking  and placing bin part in combined");
                        orders_.insert(orders_.begin(), current_order_);
                        return false;
                    }
                    else
                    {
                        partTrack.clear();

                        current_order_.step = "MoveAgv";
                        RCLCPP_INFO(get_logger(), " - Priority order recieved while picking  and placing bin part after every part in combined order");
                        orders_.insert(orders_.begin(), current_order_);
                        return false;
                    }
                }

                // count++;
            }
        }

        int destination;
        if (task.station == ariac_msgs::msg::CombinedTask::AS1 or task.station == ariac_msgs::msg::CombinedTask::AS3)
        {
            destination = ariac_msgs::srv::MoveAGV::Request::ASSEMBLY_FRONT;
        }
        else
        {
            destination = ariac_msgs::srv::MoveAGV::Request::ASSEMBLY_BACK;
        }
        if (current_order_.step.empty() || resumeFromMethod == "MoveAgv")
        {
            MoveAGV(agv_number, destination);

            CeilingRobotMoveToAssemblyStation(task.station);

            if (priority_orders_.size() != 0)
            {
                current_order_.step = "CeilingRobotAssemblePart";
                orders_.insert(orders_.begin(), current_order_);
                return false;
            }

            // Get Assembly Poses
            auto request = std::make_shared<ariac_msgs::srv::GetPreAssemblyPoses::Request>();
            request->order_id = current_order_.id;
            auto result = pre_assembly_poses_getter_->async_send_request(request);

            result.wait();

            std::vector<ariac_msgs::msg::PartPose> agv_part_poses;
            if (result.get()->valid_id)
            {
                agv_part_poses = result.get()->parts;

                if (agv_part_poses.size() == 0)
                {
                    RCLCPP_WARN(get_logger(), "No part poses recieved");
                    return false;
                }
            }
            else
            {
                RCLCPP_WARN(get_logger(), "Not a valid order ID");
                return false;
            }
            partCount = 0;
            if (current_order_.step.empty() || resumeFromMethod == "CeilingRobotAssemblePart")
            {

                for (auto const &part_to_assemble : task.parts)
                {
                    // Check if matching part exists in agv_parts
                    if (priority_orders_.size() != 0)
                    {
                        current_order_.step = "CeilingRobotAssemblePart";
                        orders_.insert(orders_.begin(), current_order_);
                    }
                    bool part_exists = false;
                    ariac_msgs::msg::PartPose part_to_pick;
                    part_to_pick.part = part_to_assemble.part;
                    for (auto const &agv_part : agv_part_poses)
                    {
                        if (agv_part.part.type == part_to_assemble.part.type && agv_part.part.color == part_to_assemble.part.color)
                        {
                            part_exists = true;
                            part_to_pick.pose = agv_part.pose;
                            break;
                        }
                    }

                    if (!part_exists)
                    {
                        RCLCPP_WARN_STREAM(get_logger(), "Part with type: " << part_to_assemble.part.type << " and color: " << part_to_assemble.part.color << " not found on tray");
                        continue;
                    }
                    RCLCPP_INFO(get_logger(), " - Assembly in progress");
                    // Pick up part
                    CeilingRobotPickAGVPart(part_to_pick);

                    CeilingRobotMoveToAssemblyStation(task.station);

                    // Assemble Part to insert
                    CeilingRobotAssemblePart(task.station, part_to_assemble);

                    CeilingRobotMoveToAssemblyStation(task.station);
                }
            }
        }

        return true;
    }
}

// ---------------------------------------------------------------------------- //
// | --------------------------- | AGV Functions | -------------------------- | //
// ---------------------------------------------------------------------------- //

bool Robot::LockAGVTray(int agv_num)
{
    RCLCPP_INFO(get_logger(), " - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");
    RCLCPP_INFO(get_logger(), " Lock AGV ");
    RCLCPP_INFO(get_logger(), " - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;
    std::string srv_name = "/ariac/agv" + std::to_string(agv_num) + "_lock_tray";
    client = this->create_client<std_srvs::srv::Trigger>(srv_name);
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result = client->async_send_request(request);
    result.wait();
    return result.get()->success;
}

bool Robot::MoveAGV(int agv_num, int destination)
{
    RCLCPP_INFO(get_logger(), " - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");
    RCLCPP_INFO(get_logger(), " Moving AGV to Destination ");
    RCLCPP_INFO(get_logger(), " - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");
    rclcpp::Client<ariac_msgs::srv::MoveAGV>::SharedPtr client;
    std::string srv_name = "/ariac/move_agv" + std::to_string(agv_num);
    client = this->create_client<ariac_msgs::srv::MoveAGV>(srv_name);
    auto request = std::make_shared<ariac_msgs::srv::MoveAGV::Request>();
    request->location = destination;
    auto result = client->async_send_request(request);
    result.wait();
    return result.get()->success;
}

// ---------------------------------------------------------------------------- //
// | --------------------------- | Main Function | -------------------------- | //
// ---------------------------------------------------------------------------- //

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto test_competitor = std::make_shared<Robot>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(test_competitor);
    std::thread([&executor]()
                { executor.spin(); })
        .detach();
    test_competitor->CompleteOrders();
    rclcpp::shutdown();
}