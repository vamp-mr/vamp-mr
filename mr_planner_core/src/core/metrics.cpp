#include <mr_planner/core/metrics.h>

#include <cmath>

SmoothnessMetrics calculate_smoothness(const MRTrajectory &synced_plan, std::shared_ptr<PlanInstance> instance)
{
    const int num_robots = instance ? instance->getNumberOfRobots() : 0;
    double total_squared_jerk = 0;
    double total_directional_consistency = 0;
    int segment_count = 0;

    for (int i = 0; i < num_robots; i++)
    {
        const auto &trajectory = synced_plan[i].trajectory;
        const auto &times = synced_plan[i].times;
        const int dof = static_cast<int>(instance->getRobotDOF(i));

        for (int k = 0; k < dof; k++)
        {
            double robot_squared_jerk = 0;
            const double robot_duration = times.back() - times.front();
            const double robot_distance = trajectory.back().joint_values[k] - trajectory.front().joint_values[k];

            for (size_t j = 2; j < trajectory.size() - 1; j++)
            {
                const double dt1 = times[j] - times[j - 1];
                const double dt2 = times[j + 1] - times[j];

                if (dt1 > 1e-5 && dt2 > 1e-5)
                {
                    const double v1 = (trajectory[j].joint_values[k] - trajectory[j - 1].joint_values[k]) / dt1;
                    const double v2 = (trajectory[j + 1].joint_values[k] - trajectory[j].joint_values[k]) / dt2;

                    const double a1 =
                        (v1 - (trajectory[j - 1].joint_values[k] - trajectory[j - 2].joint_values[k]) /
                                   (times[j - 1] - times[j - 2])) /
                        dt1;
                    const double a2 = (v2 - v1) / dt2;

                    const double jerk = (a2 - a1) / ((dt1 + dt2) / 2);
                    robot_squared_jerk += jerk * jerk * dt2;
                }
            }

            segment_count++;
            if (robot_duration > 0 && robot_distance > 0)
            {
                total_squared_jerk += std::sqrt(robot_squared_jerk /
                                                (std::pow(robot_duration, 5) * std::pow(robot_distance, 2)));
            }
        }

        double robot_directional_consistency = 0;
        int robot_segment_count = 0;
        for (size_t j = 1; j < trajectory.size() - 1; j++)
        {
            const double dt1 = times[j] - times[j - 1];
            const double dt2 = times[j + 1] - times[j];

            if (dt1 > 1e-5 && dt2 > 1e-5)
            {
                double dot_product = 0;
                double norm_v1 = 0;
                double norm_v2 = 0;

                for (int k = 0; k < dof; k++)
                {
                    const double v1 = (trajectory[j].joint_values[k] - trajectory[j - 1].joint_values[k]) / dt1;
                    const double v2 = (trajectory[j + 1].joint_values[k] - trajectory[j].joint_values[k]) / dt2;

                    dot_product += v1 * v2;
                    norm_v1 += v1 * v1;
                    norm_v2 += v2 * v2;
                }

                if (norm_v1 < 1e-5 || norm_v2 < 1e-5)
                {
                    continue;
                }
                norm_v1 = std::sqrt(norm_v1);
                norm_v2 = std::sqrt(norm_v2);
                const double cos_theta = dot_product / (norm_v1 * norm_v2);
                robot_directional_consistency += 1 - cos_theta;
                robot_segment_count++;
            }
        }

        if (robot_segment_count > 0)
        {
            total_directional_consistency += robot_directional_consistency / robot_segment_count;
        }
    }

    SmoothnessMetrics metrics;
    metrics.normalized_jerk_score = (segment_count > 0) ? (total_squared_jerk / segment_count) : 0.0;
    metrics.directional_consistency = (num_robots > 0) ? (total_directional_consistency / num_robots) : 0.0;

    return metrics;
}

