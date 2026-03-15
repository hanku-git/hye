#ifndef TASK_PLANNER_HPP
#define TASK_PLANNER_HPP

#include "task_planner_default.hpp"

class TaskPlanner : public TaskPlannerDefault {
public:
    TaskPlanner();
    ~TaskPlanner();

    virtual bool makeTaskDefaultSetup();

    bool makeTaskList();
    bool makeGraphyTaskList();
    //////////////////// LLM task
    void LLMMoveHome();
    void LLMMoveMid();
    void LLMMoveGripperHome();
    void LLMAttachGripper(int current_gripper, int vel_level);
    void LLMDetachGripper(int current_gripper, int vel_level);
    void LLMAttachTip(int current_tip, int vel_level);
    void LLMDetachTip(int current_tip, int vel_level);
    void LLMScanMatchingGrasping(int target_id, int vel_level);
    void LLMTestTask(int target_id, int vel_level);

    void UpdateBoxPose(double x, double y, double rz);
    void LLMMoveBox(int boxNumber, int boxStack, int current_obj, int vel_level, int palette_num);
    void LLMMoveWayPoints(std::vector<vector<double>> waypoints, int vel_level);
    //////////////////// Robot calibration task
    void CalGenerationJSTask(std::vector<vector<double>> pose_set, double qd, double qdd, std::string folder_name);
    void CalGenerationCSTask(std::vector<vector<double>> pose_set, double qd, double qdd, std::string folder_name);
    //////////////////// Bin picking task
    void makeRecogOnlyGraspingTaskList(bool is_slow_mode);
    void makeTestRecogOnlyGraspingTaskList(bool is_slow_mode);
    void makeToolChangeMotionTaskList();
    void makeTipChangeMotionTaskList();
    void makeSetTipIndexTask(size_t current_tip_idx);
    void makeTipChangeMotionTaskListKR0509();
    void makeRecogOnlyPoseEstimationTaskList();
    void makeInitialScanMatchingWithToolChangingMotionTaskList();
    void makeCurrentToolChangingMotionTaskList();
    void makeCurrentTipChangingMotionTaskList();
    // KORAS GRIPPER INITIALIZE
    void makeKORASGripperSetInitialValueTask();

    void makeBinPickingTestTaskList();

    void makeBinPickingCylinderDualGripperTaskList();


    void rightGenDrumTask();
    void leftGenDrumTask();

    void genKUAISTask();

public:
    std::vector<UnitTask> void_task_list_;
    std::vector<UnitTask> module_task_test1_;
    std::vector<UnitTask> module_task_test2_;
    std::vector<UnitTask> module_task_test3_;
    std::vector<UnitTask> module_task_test4_;
    std::vector<UnitTask> module_task_test5_;
    std::vector<UnitTask> cal_task_list_;

    std::string current_directory_;
    std::string fname_;
    std::string cal_mode_;

    //// LLM Task
    CsDouble current_box_pose_;

    std::vector<UnitTask> llm_task_test1_;
    std::vector<UnitTask> llm_task_list1_;
    std::vector<UnitTask> llm_task_list2_;
    std::vector<UnitTask> llm_task_list3_;
    std::vector<UnitTask> llm_task_box_;
    std::vector<UnitTask> llm_task_attach_;
    std::vector<UnitTask> llm_task_detach_;
    std::vector<UnitTask> llm_task_attach_tip_;
    std::vector<UnitTask> llm_task_detach_tip_;
    std::vector<UnitTask> llm_task_waypoints_;
    std::vector<UnitTask> llm_task_zivid_;

    std::vector<UnitTask> llm_task_test_;

    std::vector<UnitTask> drum_total_task_;
    std::vector<UnitTask> drum_scan_sub_task_;


    std::vector<UnitTask> drum_grp_lid_cap_close_;
    std::vector<UnitTask> drum_grp_lid_cap_open_;
    std::vector<UnitTask> drum_grp_initialize_task_;
    std::vector<UnitTask> drum_grp_grasping_pose_task_;
    std::vector<UnitTask> drum_grp_screwing_pose_task_;
    std::vector<UnitTask> drum_grp_unscrewing_pose_task_;
    std::vector<UnitTask> drum_grp_exit_pose_task_;


    std::vector<UnitTask> kuais_total_task_;


    //// Bin picking task
    std::vector<UnitTask> module_task_bin_picking_single_machine_tending_lathe_;
    std::vector<UnitTask> module_task_bin_picking_single_machine_tending_milling_;
    std::vector<UnitTask> module_task_bin_picking_dual_machine_tending_;
    std::vector<UnitTask> module_task_bin_picking_machine_tending_bolt_bush_;
    std::vector<UnitTask> module_task_bin_picking_pick_and_place_;
    std::vector<UnitTask> module_task_bin_picking_pick_and_place_eb_joint_;
    std::vector<UnitTask> module_task_bin_picking_pick_and_place_t_joint_with_2f_gripper_;
    std::vector<UnitTask> module_task_bin_picking_pick_and_place_t_joint_with_vacuum_gripper_;
    std::vector<UnitTask> module_task_bin_picking_pick_and_place_bolt_bush_;
    std::vector<UnitTask> module_task_bin_picking_pick_and_place_cylinder_;
    std::vector<UnitTask> module_task_bin_picking_pick_and_place_square_peg_;

    //// Graphy Test Task
    std::vector<UnitTask> module_task_graphy_stage5_grasping_test_1_;
    std::vector<UnitTask> module_task_graphy_stage5_grasping_test_2_;
    std::vector<UnitTask> module_task_graphy_stage5_only_grasping_test_;
    std::vector<UnitTask> module_task_graphy_stage5_plate_rotation_test_1_;
    std::vector<UnitTask> module_task_graphy_stage5_plate_rotation_test_2_;

    std::vector<UnitTask> module_task_tool_changing_attach_motion_;
    std::vector<UnitTask> module_task_tool_changing_detach_motion_;
    std::vector<UnitTask> module_task_dual_tool_initialize_;


    std::vector<UnitTask> module_ui_task_tip_changing_attach_motion_;
    std::vector<UnitTask> module_ui_task_tip_changing_detach_motion_;
    std::vector<UnitTask> module_task_tip_changing_attach_motion_;
    std::vector<UnitTask> module_task_tip_changing_detach_motion_;
    std::vector<UnitTask> module_task_tip_changing_set_current_tip_idx_;

    std::vector<UnitTask> module_task_initial_scan_matching_;
    std::vector<UnitTask> module_task_initial_scan_matching_with_tool_changing_motion_;


    //// TODO: code here 아래 추가한 후, sequential demo에 추가
    //// TODO: code here 아래 추가한 후, sequential demo에 추가
    //// TODO: code here 아래 추가한 후, sequential demo에 추가
    //// TODO: code here 아래 추가한 후, sequential demo에 추가
    //// TODO: code here 아래 추가한 후, sequential demo에 추가
    std::vector<UnitTask> module_task_tool_changing_current_index_detach_and_attach_motion_;
    std::vector<UnitTask> module_task_tip_changing_current_index_detach_and_attach_motion_;

    std::vector<UnitTask> module_task_load_id_;
    std::vector<UnitTask> module_recog_only_grasping_pose_estimation_;
    std::vector<UnitTask> module_koras_gripper_set_initial_value_task_;
    std::vector<UnitTask> module_bin_picking_test_;

    std::vector<std::vector<UnitTask>> module_task_graphy_list_;
    std::vector<UnitTask> module_task_graphy_;
    std::vector<UnitTask> module_task_graphy_2;
    std::vector<UnitTask> module_task_graphy_3;
    std::vector<UnitTask> module_task_graphy_4;
    std::vector<UnitTask> module_task_graphy_5;
    std::vector<UnitTask> module_task_graphy_6;
    std::vector<UnitTask> module_task_graphy_7;
    std::vector<UnitTask> module_task_graphy_8;
    std::vector<UnitTask> module_task_graphy_9;
    std::vector<UnitTask> module_task_graphy_10;
    std::vector<UnitTask> module_task_graphy_11;
    std::vector<UnitTask> module_task_graphy_12;
    std::vector<UnitTask> module_task_graphy_13;
    std::vector<UnitTask> module_task_graphy_14;
    std::vector<UnitTask> module_task_graphy_15;
    std::vector<UnitTask> module_task_graphy_16;
    std::vector<UnitTask> module_task_graphy_17;
    std::vector<UnitTask> module_task_graphy_18;
    std::vector<UnitTask> module_task_graphy_19;
    std::vector<UnitTask> module_task_graphy_20;
    std::vector<UnitTask> module_task_graphy_21;
    std::vector<UnitTask> module_task_graphy_22;
    std::vector<UnitTask> module_task_graphy_23;
    std::vector<UnitTask> module_task_graphy_24;
    std::vector<UnitTask> module_task_graphy_25;
    std::vector<UnitTask> module_task_graphy_26;
    std::vector<UnitTask> module_task_graphy_27;
    std::vector<UnitTask> module_task_graphy_28;
    std::vector<UnitTask> module_task_graphy_29;

private:
    void setPosePosition();

    // Read parameters
    void readAssemParam();
};

#endif // TASK_PLANNER_HPP
