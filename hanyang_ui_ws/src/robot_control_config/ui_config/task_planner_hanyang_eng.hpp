#ifndef TASK_PLANNER_HANYANG_ENG_HPP
#define TASK_PLANNER_HANYANG_ENG_HPP

#include "task_planner.hpp"

class TaskPlannerHanyangEng : public TaskPlanner {
public:
    TaskPlannerHanyangEng();
    ~TaskPlannerHanyangEng();

    bool makeTaskDefaultSetup() override;

    // Hanyang 특정 작업 메서드 추가
    void HanyangEngCustomTask();
    void generateHanyangEngTask();

    void genTaskSetting();
    void genGripperTask();
    void genScanTask();
    void rightGenDrumTask();
    void leftGenDrumTask();

    //// PLC 연동 작업
    void genAMRReadyPosePLCCommTask(bool is_plc_test_mode);
    void genLeftDrumPLCCommTask(bool is_plc_test_mode);
    void genRightDrumPLCCommTask(bool is_plc_test_mode);
    void genPLCCommTask();

    std::vector<UnitTask> rightGenSubTask1Task();
    std::vector<UnitTask> rightGenSubTask2Task();
    std::vector<UnitTask> rightGenSubTask3Task();
    std::vector<UnitTask> rightGenSubTask4Task();
    std::vector<UnitTask> rightGenSubTask5Task();
    std::vector<UnitTask> rightGenSubTask6Task();
    std::vector<UnitTask> rightGenSubTask7Task();
    std::vector<UnitTask> rightGenSubTask8Task();
    std::vector<UnitTask> rightGenSubTask9Task();
    std::vector<UnitTask> rightGenSubTask10Task();
    std::vector<UnitTask> rightGenSubTask11Task();
    std::vector<UnitTask> rightGenSubTask12Task();
    std::vector<UnitTask> rightGenSubTask13Task();
    std::vector<UnitTask> rightGenSubTask14Task();
    std::vector<UnitTask> rightGenSubTask15Task();
    std::vector<UnitTask> rightGenSubTask16Task();
    std::vector<UnitTask> rightGenSubTask17Task();
    std::vector<UnitTask> rightGenSubTask18Task();
    std::vector<UnitTask> rightGenSubTask19Task();
    std::vector<UnitTask> rightGenSubTask20Task();
    std::vector<UnitTask> rightGenSubTask21Task();
    std::vector<UnitTask> rightGenSubTask22Task();
    std::vector<UnitTask> rightGenSubTask23Task();
    std::vector<UnitTask> rightGenSubTask24Task();


    std::vector<UnitTask> leftGenSubTask1Task();
    std::vector<UnitTask> leftGenSubTask2Task();
    std::vector<UnitTask> leftGenSubTask3Task();
    std::vector<UnitTask> leftGenSubTask4Task();
    std::vector<UnitTask> leftGenSubTask5Task();
    std::vector<UnitTask> leftGenSubTask6Task();
    std::vector<UnitTask> leftGenSubTask7Task();
    std::vector<UnitTask> leftGenSubTask8Task();


    std::vector<UnitTask> rightHomePoseTask();

public:
    std::vector<UnitTask> doosan_module_task_;

    //// Task for AMR Setup JS Position (PLC read/write)

    // 1) JS Position PLC Comm. Task
    std::vector<UnitTask> amr_ready_pose_plc_comm_task_;
    std::vector<UnitTask> right_drum_home_pose_plc_comm_task_;
    std::vector<UnitTask> left_drum_home_pose_plc_comm_task_;

    // 2) 캡 분리, 커플러 체결 작업의 PLC Comm. Task
    std::vector<UnitTask> right_drum_cap_detach_coupler_screwing_plc_comm_task_;
    std::vector<UnitTask> right_drum_cap_detach_plc_comm_task_;
    std::vector<UnitTask> right_drum_coupler_screwing_plc_comm_task_;

    std::vector<UnitTask> left_drum_cap_detach_coupler_screwing_plc_comm_task_;
    std::vector<UnitTask> left_drum_cap_detach_plc_comm_task_;
    std::vector<UnitTask> left_drum_coupler_screwing_plc_comm_task_;


    // 3) 커플러 해제, 캡 체결 작업의 PLC Comm. Task
    std::vector<UnitTask> right_drum_coupler_detach_cap_screwing_plc_comm_task_;
    std::vector<UnitTask> right_drum_cap_screwing_plc_comm_task_;
    std::vector<UnitTask> right_drum_coupler_detach_plc_comm_task_;

    std::vector<UnitTask> left_drum_coupler_detach_cap_screwing_plc_comm_task_;
    std::vector<UnitTask> left_drum_cap_screwing_plc_comm_task_;
    std::vector<UnitTask> left_drum_coupler_detach_plc_comm_task_;


    // 4) 드럼 회전각도 인식
    std::vector<UnitTask> drum_rotating_angle_detection_plc_comm_task_;
    // 5) 바코드 인식
    std::vector<UnitTask> drum_barcode_read_plc_comm_task_;




    //// Total Task
    std::vector<UnitTask> right_drum_total_task_;
    std::vector<UnitTask> left_drum_total_task_;
    std::vector<UnitTask> drum_scan_sub_task_;


    std::vector<UnitTask> right_drum_sub_task_1_;
    std::vector<UnitTask> right_drum_sub_task_2_;
    std::vector<UnitTask> right_drum_sub_task_3_;
    std::vector<UnitTask> right_drum_sub_task_4_;
    std::vector<UnitTask> right_drum_sub_task_5_;
    std::vector<UnitTask> right_drum_sub_task_6_;
    std::vector<UnitTask> right_drum_sub_task_7_;
    std::vector<UnitTask> right_drum_sub_task_8_;
    std::vector<UnitTask> right_drum_sub_task_9_;
    std::vector<UnitTask> right_drum_sub_task_10_;
    std::vector<UnitTask> right_drum_sub_task_11_;
    std::vector<UnitTask> right_drum_sub_task_12_;
    std::vector<UnitTask> right_drum_sub_task_13_;
    std::vector<UnitTask> right_drum_sub_task_14_;
    std::vector<UnitTask> right_drum_sub_task_15_;
    std::vector<UnitTask> right_drum_sub_task_16_;
    std::vector<UnitTask> right_drum_sub_task_17_;
    std::vector<UnitTask> right_drum_sub_task_18_;
    std::vector<UnitTask> right_drum_sub_task_19_;
    std::vector<UnitTask> right_drum_sub_task_20_;
    std::vector<UnitTask> right_drum_sub_task_21_;
    std::vector<UnitTask> right_drum_sub_task_22_;
    std::vector<UnitTask> right_drum_sub_task_23_;
    std::vector<UnitTask> right_drum_sub_task_24_;


    std::vector<UnitTask> left_drum_sub_task_1_;
    std::vector<UnitTask> left_drum_sub_task_2_;
    std::vector<UnitTask> left_drum_sub_task_3_;
    std::vector<UnitTask> left_drum_sub_task_4_;
    std::vector<UnitTask> left_drum_sub_task_5_;
    std::vector<UnitTask> left_drum_sub_task_6_;
    std::vector<UnitTask> left_drum_sub_task_7_;
    std::vector<UnitTask> left_drum_sub_task_8_;

    std::vector<UnitTask> drum_scan_task_1_;
    std::vector<UnitTask> drum_scan_task_2_;
    std::vector<UnitTask> drum_scan_task_3_;
    std::vector<UnitTask> drum_scan_task_4_;
    std::vector<UnitTask> drum_scan_task_5_;
    std::vector<UnitTask> drum_scan_task_6_;
    std::vector<UnitTask> drum_scan_task_7_;
    std::vector<UnitTask> drum_scan_task_8_;
    std::vector<UnitTask> drum_scan_task_9_;


    std::vector<UnitTask> drum_grp_lid_cap_close_;
    std::vector<UnitTask> drum_grp_lid_cap_open_;
    std::vector<UnitTask> drum_grp_initialize_task_;
    std::vector<UnitTask> drum_grp_grasping_pose_task_;
    std::vector<UnitTask> drum_grp_screwing_pose_task_;
    std::vector<UnitTask> drum_grp_unscrewing_pose_task_;
    std::vector<UnitTask> drum_grp_exit_pose_task_;
    std::vector<UnitTask> drum_grp_lid_cap_close_tight;
    std::vector<UnitTask> drum_grp_lid_cap_open_tight;

    std::vector<UnitTask> drum_grp_lid_cap_close_for_screwing_;


    std::vector<UnitTask> drum_grp_lid_cap_screwing_open_;



    std::vector<UnitTask> drum_grp_plus_motor_ctrl_task_;
    std::vector<UnitTask> drum_grp_minus_motor_ctrl_task_;
    std::vector<UnitTask> drum_grp_plus_5000_task_;
    std::vector<UnitTask> drum_grp_plus_4000_task_;
    std::vector<UnitTask> drum_grp_plus_3000_task_;
    std::vector<UnitTask> drum_grp_plus_2000_task_;
    std::vector<UnitTask> drum_grp_minus_5000_task_;
    std::vector<UnitTask> drum_grp_minus_4000_task_;
    std::vector<UnitTask> drum_grp_minus_3000_task_;
    std::vector<UnitTask> drum_grp_minus_2000_task_;



private:
    JsDouble amr_ready_pose_;

    JsDouble right_home_pose_;
    JsDouble right_hole_scan_pose_;
    JsDouble middle_pose1_;
    JsDouble middle_pose2_;
    JsDouble middle_pose3_;
    JsDouble middle_pose4_;
    JsDouble hose_grapping_pose_top_;
    JsDouble hole_pose_top_;
    JsDouble hole_pose_top_top_;
    JsDouble star_top_;
    JsDouble star_off_;
    JsDouble star_bottom_;
    JsDouble star_off_off_;

    JsDouble left_home_pose_;
    JsDouble left_hole_scan_pose_;

    JsDouble drum_rotating_angle_detection_scan_pose_;


};

#endif // TASK_PLANNER_HANYANG_ENG_HPP