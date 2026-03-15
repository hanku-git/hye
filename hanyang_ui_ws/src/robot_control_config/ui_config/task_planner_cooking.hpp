#ifndef TASK_PLANNER_COOKING_HPP
#define TASK_PLANNER_COOKING_HPP

#include "task_planner.hpp"

class TaskPlannerCooking : public TaskPlanner {
public:
    TaskPlannerCooking();
    ~TaskPlannerCooking();

    bool makeTaskDefaultSetup() override;

    void CookingCustomTask();
    void generateCookingTask();


    /////////////////////////////////////////Cooking robot/////////////////////////////////////////////////
    void setGeneralSettingsForCookingTask();
    std::vector<UnitTask> CookToolPointTask(const std::string &tool_name, double x, double y, double z, double roll, double pitch, double yaw);
    std::vector<UnitTask> LangSAMMoveTask(double x, double y, double z, double roll, double pitch, double yaw);




public:
    std::vector<UnitTask> doosan_module_task_;

    //// Cooking
    JsDouble js_home_pose_;
    JsDouble js_mid_pose_;
    JsDouble js_mid_2f_pose_;
    JsDouble js_mid_tag_pose_;

    CsDouble cs_position_gripper_change_init_1_;
    CsDouble cs_position_gripper_change_init_2_;
    CsDouble cs_position_gripper_change_init_3_;
    CsDouble cs_position_gripper_change_init_4_;
    CsDouble cs_position_gripper_change_init_5_;

    CsDouble tcp_default_;

};

#endif // TASK_PLANNER_COOKING_HPP