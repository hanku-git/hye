#ifndef TASK_PLANNER_HYUNDAI_LLM_HPP
#define TASK_PLANNER_HYUNDAI_LLM_HPP

#include "task_planner.hpp"

class TaskPlannerHyundaiLlm : public TaskPlanner {
public:
    TaskPlannerHyundaiLlm();
    ~TaskPlannerHyundaiLlm();

    bool makeTaskDefaultSetup() override;

    // Hanyang 특정 작업 메서드 추가
    void HyundaiLlmCustomTask();
};

#endif // TASK_PLANNER_HYUNDAI_LLM_HPP