#pragma once

#include <mr_planner/core/instance.h>

#include <memory>

struct SmoothnessMetrics
{
    double normalized_jerk_score{0.0};
    double directional_consistency{0.0};
};

SmoothnessMetrics calculate_smoothness(const MRTrajectory &synced_plan, std::shared_ptr<PlanInstance> instance);

