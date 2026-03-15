#include "task_planner_hyundai_llm.hpp"
#include <iostream>

TaskPlannerHyundaiLlm::TaskPlannerHyundaiLlm() {
    std::cout << "TaskPlannerHyundaiLlm initialized." << std::endl;
}

TaskPlannerHyundaiLlm::~TaskPlannerHyundaiLlm() {
    std::cout << "TaskPlannerHyundaiLlm destroyed." << std::endl;
}

bool TaskPlannerHyundaiLlm::makeTaskDefaultSetup() {
    std::cout << "Creating Hyundai task list..." << std::endl;
    // Hyundai 전용 작업 리스트 로직 구현
    return true;
}

void TaskPlannerHyundaiLlm::HyundaiLlmCustomTask() {
    std::cout << "Executing Hyundai-specific task..." << std::endl;
    // 특정 작업 로직 구현
}
