#pragma once

#define ANSI_COLOR_YELLOW  "\033[33m"

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <unistd.h>

#include <cmath>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/msg/collision_object.hpp>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/msg/mesh.h>

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <kdl/frames.hpp>
#include <tf2_kdl/tf2_kdl.h>


#include <std_srvs/srv/trigger.hpp>

#include <ariac_msgs/msg/order.hpp>
#include <ariac_msgs/msg/advanced_logical_camera_image.hpp>
#include <ariac_msgs/msg/break_beam_status.hpp>
#include <ariac_msgs/msg/kitting_task.hpp>
#include <ariac_msgs/msg/kit_tray_pose.hpp>
#include <ariac_msgs/msg/vacuum_gripper_state.hpp>
#include <ariac_msgs/msg/assembly_state.hpp>
#include <ariac_msgs/msg/competition_state.hpp>

#include <ariac_msgs/msg/conveyor_parts.hpp>
#include <ariac_msgs/msg/bin_parts.hpp>

#include <ariac_msgs/srv/change_gripper.hpp>
#include <ariac_msgs/srv/vacuum_gripper_control.hpp>
#include <ariac_msgs/srv/perform_quality_check.hpp>
#include <ariac_msgs/srv/get_pre_assembly_poses.hpp>
#include <ariac_msgs/srv/move_agv.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

#include <group5_msgs/msg/floor_robot_task.hpp>
#include <group5_msgs/msg/completed_order.hpp>
#include <group5_msgs/msg/floor_robot_task.hpp>

// #include <competitor_msgs/msg/robots_status.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <track_bin.hpp>

class Robot : public rclcpp::Node
{
public:
    /// Constructor
    Robot();

    ~Robot();

    // Floor Robot Public Functions
    /**
    * @brief Sends the floor robot back to its home position.
    */
    void FloorRobotSendHome();

    /**
     * @brief Sets the gripper state of the floor robot.
     * 
     * @param enable Activate or deactivate the gripper
     * @return true if the gripper state was successfully set, false otherwise.
     */

    bool FloorRobotSetGripperState(bool enable);

     /**
     * @brief Changes the gripper type of the floor robot.
     * 
     * @param station the availabel kitting station
     * @param gripper_type select part gripper or tray gripper
     * @return true if the gripper type was changed
     */
    bool FloorRobotChangeGripper(std::string station, std::string gripper_type);

    /**
     * @brief Floor robot will Pick and place tray.
     * 
     * @param tray_id required tray id for the picking up tray
     * @param  agv_num the agv where the tray should be placed
     * @return true if the tray was placed
     */
    bool FloorRobotPickandPlaceTray(int tray_id, int agv_num);
      /**
     * @brief Pick and place bin part
     * 
     * @param part_to_pick required part to be picked from bin
     * @return true if the part was picked and placed
     */
    bool FloorRobotPickBinPart(ariac_msgs::msg::Part part_to_pick);
        /**
     * @brief Pick and place conveyor part
     * 
     * @param part_to_pick required part to be picked from conveyor
     * @return true if the part was picked and placed
     */
    bool FloorRobotPickConveyorPart(ariac_msgs::msg::Part part_to_pick);
    /**
     * @brief Pick up conveyor method
     * 
     * @param task required task type 
     * 
     */
    template<typename T>
    void PickUpConveyor(T task);
       /**
     * @brief Pick and place part on kit tray
     * 
     * @param agv_num agv number for placing part
     * @param quadrant quadrant for placing the part
     * @return true if the part was picked and placed
     */
    bool FloorRobotPlacePartOnKitTray(int agv_num, int quadrant);
    // Ceiling Robot Public functions
    
      // Floor Robot Public Functions
    /**
    * @brief Sends the ceiling robot back to its home position.
    */
   
    void CeilingRobotSendHome();
      /**
     * @brief Sets the gripper state of the ceiling robot.
     * 
     * @param enable Activate or deactivate the gripper
     * @return true if the gripper state was successfully set, false otherwise.
     */

    bool CeilingRobotSetGripperState(bool enable);
           /**
     * @brief Lock the tray on agv
     * 
     * @param agv_num agv number for locking the tray 
     * @return true if the tray is locked
     */
    bool LockAGVTray(int agv_num);
           /**
     * @brief move agv to a destination
     * 
     * @param agv_num agv number for moving
     * @param destination sending agv to this loaction
     * @return true if agv was moved
     */
    bool MoveAGV(int agv_num, int destination);
           /**
     * @brief complete orders
     * @return true orders are executed
     */
    bool CompleteOrders();
      /**
     * @brief complete Kitting task
     * @param task kitting msg
     * @return true if the task was complted
     */
    bool CompleteKittingTask(ariac_msgs::msg::KittingTask task);
   /**
     * @brief complete Assembly task
     * @param task AssemblyTask msg
     * @return true if the task was complted
     */
    bool CompleteAssemblyTask(ariac_msgs::msg::AssemblyTask task);
    /**
     * @brief complete combined task
     * 
     * @param task CombinedTask msg
     * @return true if the task was complted
     */
    bool CompleteCombinedTask(ariac_msgs::msg::CombinedTask task);
   
    /**
     * @brief track the availabel bins
     */
    TrackBin track_bin;    

    void PickAllConveyorParts();

private:

    // FloorRobotMoveFunctions
    bool FloorRobotMovetoTarget();
    bool FloorRobotMoveCartesian(std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf);
    bool FloorRobotWaitForAttach(double timeout);
    void FloorRobotWaitForAttachConveyor(double timeout);

    // CeilingRobotMoveFunctions
    bool CeilingRobotMovetoTarget();
    bool CeilingRobotMoveCartesian(std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf, bool avoid_collisions);
    void CeilingRobotWaitForAttach(double timeout);
    bool CeilingRobotWaitForAssemble(int station, ariac_msgs::msg::AssemblyPart part);
    bool CeilingRobotMoveToAssemblyStation(int station);
    bool CeilingRobotPickAGVPart(ariac_msgs::msg::PartPose part);
    bool CeilingRobotAssemblePart(int station, ariac_msgs::msg::AssemblyPart part);
    



    // Quality Check Functions
    bool ReplaceFaultyPart(ariac_msgs::msg::Part part_to_pick, 
                            std::vector<ariac_msgs::msg::PartPose> agv_parts, 
                            geometry_msgs::msg::Pose agv_camera_pose, 
                            int agv_num,
                            int quadrant);
    bool KittingPartQualityCheck(ariac_msgs::msg::KittingTask task);
    bool FloorRobotPickAGVTrayPart(ariac_msgs::msg::Part part_to_pick, 
                                    std::vector<ariac_msgs::msg::PartPose> agv_parts, 
                                    geometry_msgs::msg::Pose agv_camera_pose, 
                                    int agv_num,
                                    int quadrant);
    bool FloorRobotDisposePart(int agv_num);
    std::vector<bool> QualityCheckService();

    // Helper Functions
    geometry_msgs::msg::Quaternion SetRobotOrientation(double rotation);

    void LogPose(geometry_msgs::msg::Pose p);
    geometry_msgs::msg::Pose MultiplyPose(geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2);
    geometry_msgs::msg::Pose BuildPose(double x, double y, double z, geometry_msgs::msg::Quaternion orientation);
    geometry_msgs::msg::Pose FrameWorldPose(std::string frame_id);
    double GetYaw(geometry_msgs::msg::Pose pose);
    geometry_msgs::msg::Quaternion QuaternionFromRPY(double r, double p, double y);

    // Planning Scene Functions
    void AddModelToPlanningScene(std::string name, std::string mesh_file, geometry_msgs::msg::Pose model_pose);
    void AddModelsToPlanningScene();
    
    //Callback Groups
    rclcpp::CallbackGroup::SharedPtr client_cb_group_;
    rclcpp::CallbackGroup::SharedPtr topic_cb_group_;

    // MoveIt Interfaces
    moveit::planning_interface::MoveGroupInterface floor_robot_;
    moveit::planning_interface::MoveGroupInterface ceiling_robot_;
    
    moveit::planning_interface::PlanningSceneInterface planning_scene_;

    trajectory_processing::TimeOptimalTrajectoryGeneration totg_;

    // TF
    std::unique_ptr<tf2_ros::Buffer> tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
    std::shared_ptr<tf2_ros::TransformListener> tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // Publishers
    /*!< Publisher to the topic /ariac/orders */
    rclcpp::Publisher<group5_msgs::msg::CompletedOrder>::SharedPtr completed_order_pub_;

    // Subscriptions
    rclcpp::Subscription<ariac_msgs::msg::VacuumGripperState>::SharedPtr floor_gripper_state_sub_;
    rclcpp::Subscription<ariac_msgs::msg::VacuumGripperState>::SharedPtr ceiling_gripper_state_sub_;
    rclcpp::Subscription<ariac_msgs::msg::CompetitionState>::SharedPtr competition_state_sub_;

    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr kts1_camera_sub_;
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr kts2_camera_sub_;
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr left_bins_camera_sub_;
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr right_bins_camera_sub_;
    rclcpp::Subscription<group5_msgs::msg::FloorRobotTask>::SharedPtr  robot_task_sub_;
    rclcpp::Subscription<group5_msgs::msg::FloorRobotTask>::SharedPtr  robot_task_priority_sub_;

    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr conveyor_camera_sub_;
    rclcpp::Subscription<ariac_msgs::msg::BreakBeamStatus>::SharedPtr break_beam_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr step_sub_;

     
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr agv_1_camera_sub;
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr agv_2_camera_sub;
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr agv_3_camera_sub;
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr agv_4_camera_sub;


    rclcpp::Subscription<ariac_msgs::msg::AssemblyState>::SharedPtr as1_state_sub_;
    rclcpp::Subscription<ariac_msgs::msg::AssemblyState>::SharedPtr as2_state_sub_;
    rclcpp::Subscription<ariac_msgs::msg::AssemblyState>::SharedPtr as3_state_sub_;
    rclcpp::Subscription<ariac_msgs::msg::AssemblyState>::SharedPtr as4_state_sub_;

    rclcpp::Subscription<ariac_msgs::msg::ConveyorParts>::SharedPtr init_conv_parts_sub_;
    rclcpp::Subscription<ariac_msgs::msg::BinParts>::SharedPtr init_bin_parts_sub_;
    
    //Assembly States 
    std::map<int, ariac_msgs::msg::AssemblyState> assembly_station_states_;


    // Orders List
    group5_msgs::msg::FloorRobotTask current_order_;
    std::vector<group5_msgs::msg::FloorRobotTask> orders_;
    std::vector<group5_msgs::msg::FloorRobotTask> priority_orders_;
    std::string step;
    unsigned int competition_state_;

    // Gripper State
    ariac_msgs::msg::VacuumGripperState floor_gripper_state_;
    ariac_msgs::msg::Part floor_robot_attached_part_;
    ariac_msgs::msg::VacuumGripperState ceiling_gripper_state_;
    ariac_msgs::msg::Part ceiling_robot_attached_part_;

    // Sensor poses
    geometry_msgs::msg::Pose kts1_camera_pose_;
    geometry_msgs::msg::Pose kts2_camera_pose_;
    geometry_msgs::msg::Pose left_bins_camera_pose_;
    geometry_msgs::msg::Pose right_bins_camera_pose_;
    geometry_msgs::msg::Pose conveyor_camera_pose_;

    geometry_msgs::msg::Pose agv_1_camera_pose_;
    geometry_msgs::msg::Pose agv_2_camera_pose_;
    geometry_msgs::msg::Pose agv_3_camera_pose_;
    geometry_msgs::msg::Pose agv_4_camera_pose_;

    // Trays
    std::vector<ariac_msgs::msg::KitTrayPose> kts1_trays_;
    std::vector<ariac_msgs::msg::KitTrayPose> kts2_trays_;

    // Bins
    std::vector<ariac_msgs::msg::PartPose> left_bins_parts_;
    std::vector<ariac_msgs::msg::PartPose> right_bins_parts_;

    // Empty Bin Calculation
    std::vector<int> all_bins = {1, 2, 5, 6};
    std::vector<int> occupied_bins = {};        // read from bin topic
    std::vector<int> empty_bins;                // difference of All Bins - Occupied bins

    struct Slot_XY {
        double x;
        double y;
    };

    std::vector<Slot_XY> empty_bin_poses;

    std::map<int, std::pair<double, double>> bin_centers = {
        {2, std::pair<double, double>(-1.9, 2.625)},
        {1, std::pair<double, double>(-1.9, 3.375)},
        {5, std::pair<double, double>(-1.9, -3.375)},
        {6, std::pair<double, double>(-1.9, -2.625)},
    };
        
    std::vector<std::pair<double, double>> slot_offsets = {
        {-0.18, -0.18},
        {-0.18, 0.0},
        {-0.18, 0.18},
        {0.0, -0.18},
        {0.0, 0.0},
        {0.0, 0.18},
        {0.18, -0.18},
        {0.18, 0.0},
        {0.18, 0.18},            
    };

    // Conveyers
    std::vector<ariac_msgs::msg::PartPose> conveyor_camera_parts_;
    ariac_msgs::msg::PartPose current_conveyor_part;
    int remaining_conveyor_parts[5][4] = {};
    int conveyor_pick_up_counter = 0;

    // AGVs
    geometry_msgs::msg::Pose agv_camera_poses_;
    std::vector<ariac_msgs::msg::PartPose> agv_camera_parts_;

    std::vector<ariac_msgs::msg::PartPose> agv_1_parts_;
    std::vector<ariac_msgs::msg::PartPose> agv_2_parts_;
    std::vector<ariac_msgs::msg::PartPose> agv_3_parts_;
    std::vector<ariac_msgs::msg::PartPose> agv_4_parts_;

    std::map<int8_t, std::string> partTrack;

    std::map <int8_t,std::string>partTrackCombined;

    int partCount;

    std::vector<bool> faulty_part_vector;
    int part_number_ = 0;

    std::map<int, std::vector<ariac_msgs::msg::PartPose>> agv_parts_map;

    std::map<int, bool> agv_track {
        {1, false},
        {2, false},
        {3, false},
        {4, false}
    };

    // Sensor Callbacks
    bool kts1_camera_received_data = false;
    bool kts2_camera_received_data = false;
    bool left_bins_camera_received_data = false;
    bool right_bins_camera_received_data = false;
    bool floor_robot_task_received_data_ = false;
    bool conveyor_camera_received_data_ = false;

    bool agv_1_camera_received_data = false;
    bool agv_2_camera_received_data = false;
    bool agv_3_camera_received_data = false;
    bool agv_4_camera_received_data = false;

    // Conveyor/Bins Callback
    bool initial_bin_parts_received  = false;
    bool initial_conveyer_parts_received = false;

    // check if conveyor is detecting parts
    bool conveyor_part_detected = false;

    // check if break beam is detected
    bool break_beam_detected = false;

    // check if conveyor parts are picked
    bool conveyor_parts_picked= false;

    void kts1_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
    void kts2_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
    void left_bins_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
    void right_bins_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
    void conveyor_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
    void break_beam_cb(const ariac_msgs::msg::BreakBeamStatus::ConstSharedPtr msg);
    
    // Competition state callback
    void competition_state_cb(const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg);
    
    // Gripper State Callback
    void floor_gripper_state_cb(const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg);
    void ceiling_gripper_state_cb(const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg);

    void agv_1_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
    void agv_2_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
    void agv_3_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
    void agv_4_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);

    void as1_state_cb(const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg);
    void as2_state_cb(const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg);
    void as3_state_cb(const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg);
    void as4_state_cb(const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg);
    
    // Floor Robot Task Callback
    void robot_task_cb(const group5_msgs::msg::FloorRobotTask::ConstSharedPtr msg);
    void robot_task_priority_cb(const group5_msgs::msg::FloorRobotTask::ConstSharedPtr msg);
    
    //Step callback
    void step_task_cb(const std_msgs::msg::String::ConstSharedPtr msg);
    
    //  Function for checking insufficient part
    bool check_insufficient_part(group5_msgs::msg::FloorRobotTask msg);

    // callback fucntion for initial conveyer/bin parts
    void init_conv_parts_cb(ariac_msgs::msg::ConveyorParts msg);
    void init_bin_parts_cb(ariac_msgs::msg::BinParts msg);
   
    // ARIAC Services
    rclcpp::Client<ariac_msgs::srv::PerformQualityCheck>::SharedPtr quality_checker_;
    rclcpp::Client<ariac_msgs::srv::GetPreAssemblyPoses>::SharedPtr pre_assembly_poses_getter_;
    rclcpp::Client<ariac_msgs::srv::ChangeGripper>::SharedPtr floor_robot_tool_changer_;
    rclcpp::Client<ariac_msgs::srv::VacuumGripperControl>::SharedPtr floor_robot_gripper_enable_;
    rclcpp::Client<ariac_msgs::srv::VacuumGripperControl>::SharedPtr ceiling_robot_gripper_enable_;

    // Constants
    double kit_tray_thickness_ = 0.01;

    // double drop_height_ = 0.002;
    double drop_height_ = 0.01;
    double pick_offset_ = 0.003;
    double battery_grip_offset_ = -0.05;
    
    std::map<int, std::string> part_types_ = {
        {ariac_msgs::msg::Part::BATTERY, "battery"},
        {ariac_msgs::msg::Part::PUMP, "pump"},
        {ariac_msgs::msg::Part::REGULATOR, "regulator"},
        {ariac_msgs::msg::Part::SENSOR, "sensor"}};

    std::map<int, std::string> part_colors_ = {
        {ariac_msgs::msg::Part::RED, "red"},
        {ariac_msgs::msg::Part::BLUE, "blue"},
        {ariac_msgs::msg::Part::GREEN, "green"},
        {ariac_msgs::msg::Part::ORANGE, "orange"},
        {ariac_msgs::msg::Part::PURPLE, "purple"},
    };

    // Part heights
    std::map<int, double> part_heights_ = {
        {ariac_msgs::msg::Part::BATTERY, 0.04},
        {ariac_msgs::msg::Part::PUMP, 0.12},
        {ariac_msgs::msg::Part::REGULATOR, 0.07},
        {ariac_msgs::msg::Part::SENSOR, 0.07}};

    // Part types to bin
    std::map<int, int> part_types_bin_ = {
        {ariac_msgs::msg::Part::BATTERY,2},
        {ariac_msgs::msg::Part::PUMP, 5},
        {ariac_msgs::msg::Part::REGULATOR, 6},
        {ariac_msgs::msg::Part::SENSOR, 1}};

    // conveyor offset
    std::map<int, double> conveyer_pick_offset = {
        {ariac_msgs::msg::Part::PUMP, 0.03},
        {ariac_msgs::msg::Part::REGULATOR, 0.01}
    };

    // Quadrant Offsets
    std::map<int, std::pair<double, double>> quad_offsets_ = {
        {ariac_msgs::msg::KittingPart::QUADRANT1, std::pair<double, double>(-0.08, 0.12)},
        {ariac_msgs::msg::KittingPart::QUADRANT2, std::pair<double, double>(0.08, 0.12)},
        {ariac_msgs::msg::KittingPart::QUADRANT3, std::pair<double, double>(-0.08, -0.12)},
        {ariac_msgs::msg::KittingPart::QUADRANT4, std::pair<double, double>(0.08, -0.12)},
    };

    std::map<std::string, double> rail_positions_ = {
        {"agv1", -4.5},
        {"agv2", -1.2},
        {"agv3", 1.2},
        {"agv4", 4.5},
        {"left_bins", 3},
        {"right_bins", -3}};

    // Joint value targets for kitting stations
    std::map<std::string, double> floor_kts1_js_ = {
        {"linear_actuator_joint", 4.0},
        {"floor_shoulder_pan_joint", 1.57},
        {"floor_shoulder_lift_joint", -1.57},
        {"floor_elbow_joint", 1.57},
        {"floor_wrist_1_joint", -1.57},
        {"floor_wrist_2_joint", -1.57},
        {"floor_wrist_3_joint", 0.0}};

    std::map<std::string, double> floor_kts2_js_ = {
        {"linear_actuator_joint", -4.0},
        {"floor_shoulder_pan_joint", -1.57},
        {"floor_shoulder_lift_joint", -1.57},
        {"floor_elbow_joint", 1.57},
        {"floor_wrist_1_joint", -1.57},
        {"floor_wrist_2_joint", -1.57},
        {"floor_wrist_3_joint", 0.0}};

    std::map<std::string, double> ceiling_as1_js_ = {
        {"gantry_x_axis_joint", 1},
        {"gantry_y_axis_joint", -3},
        {"gantry_rotation_joint", 1.571},
        {"ceiling_shoulder_pan_joint", 0},
        {"ceiling_shoulder_lift_joint", -2.37},
        {"ceiling_elbow_joint", 2.37},
        {"ceiling_wrist_1_joint", 3.14},
        {"ceiling_wrist_2_joint", -1.57},
        {"ceiling_wrist_3_joint", 0}
    };

    std::map<std::string, double> ceiling_as2_js_ = {
        {"gantry_x_axis_joint", -4},
        {"gantry_y_axis_joint", -3},
        {"gantry_rotation_joint", 1.571},
        {"ceiling_shoulder_pan_joint", 0},
        {"ceiling_shoulder_lift_joint", -2.37},
        {"ceiling_elbow_joint", 2.37},
        {"ceiling_wrist_1_joint", 3.14},
        {"ceiling_wrist_2_joint", -1.57},
        {"ceiling_wrist_3_joint", 0}
    };

    std::map<std::string, double> ceiling_as3_js_ = {
        {"gantry_x_axis_joint", 1},
        {"gantry_y_axis_joint", 3},
        {"gantry_rotation_joint", 1.571},
        {"ceiling_shoulder_pan_joint", 0},
        {"ceiling_shoulder_lift_joint", -2.37},
        {"ceiling_elbow_joint", 2.37},
        {"ceiling_wrist_1_joint", 3.14},
        {"ceiling_wrist_2_joint", -1.57},
        {"ceiling_wrist_3_joint", 0}
    };

    std::map<std::string, double> ceiling_as4_js_ = {
        {"gantry_x_axis_joint", -4},
        {"gantry_y_axis_joint", 3},
        {"gantry_rotation_joint", 1.571},
        {"ceiling_shoulder_pan_joint", 0},
        {"ceiling_shoulder_lift_joint", -2.37},
        {"ceiling_elbow_joint", 2.37},
        {"ceiling_wrist_1_joint", 3.14},
        {"ceiling_wrist_2_joint", -1.57},
        {"ceiling_wrist_3_joint", 0}
    };

    // Joint value targets for Conveyor Base Position
    std::map<std::string, double> conveyor_prep_js = {
        {"linear_actuator_joint", -1.5},
        {"floor_shoulder_pan_joint", 3.15},
        {"floor_shoulder_lift_joint", -1.57},
        {"floor_elbow_joint", 1.57},
        {"floor_wrist_1_joint", -1.57},
        {"floor_wrist_2_joint", -1.57},
        {"floor_wrist_3_joint", 0.0}
    };
    
    // Joint value targets for Part Disposal
    std::map<std::string, double> center_disposal_bin_js = {
        {"linear_actuator_joint", -0.25},
        {"floor_shoulder_pan_joint", 0.0},
        {"floor_shoulder_lift_joint", -1.17},
        {"floor_elbow_joint", 1.57},
        {"floor_wrist_1_joint", -1.95},
        {"floor_wrist_2_joint", -1.57},
        {"floor_wrist_3_joint", 0.0}
    };

    std::map<std::string, double> left_disposal_bin_js = {
        {"linear_actuator_joint", 4.40},
        {"floor_shoulder_pan_joint", 2.55},
        {"floor_shoulder_lift_joint", -1.45},
        {"floor_elbow_joint", 1.57},
        {"floor_wrist_1_joint", -1.65},
        {"floor_wrist_2_joint", -1.57},
        {"floor_wrist_3_joint", 0.0}
    };

    std::map<std::string, double> right_disposal_bin_js = {
        {"linear_actuator_joint", -4.40},
        {"floor_shoulder_pan_joint", -2.95},
        {"floor_shoulder_lift_joint", -1.45},
        {"floor_elbow_joint", 1.57},
        {"floor_wrist_1_joint", -1.65},
        {"floor_wrist_2_joint", -1.57},
        {"floor_wrist_3_joint", 0.0}
    };


};