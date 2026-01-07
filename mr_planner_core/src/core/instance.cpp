#include <mr_planner/core/instance.h>
#include <mr_planner/core/logger.h>
#include <algorithm>
#include <limits>

#if MR_PLANNER_WITH_ROS
shape_msgs::SolidPrimitive PlanInstance::getPrimitive(const Object &obj)
{
    shape_msgs::SolidPrimitive primitive;
    switch (obj.shape) {
        case Object::Shape::Box:
            primitive.type = shape_msgs::SolidPrimitive::BOX;
            primitive.dimensions = {obj.length, obj.width, obj.height};
            break;
        case Object::Shape::Sphere:
            primitive.type = shape_msgs::SolidPrimitive::SPHERE;
            primitive.dimensions = {obj.radius};
            break;
        case Object::Shape::Cylinder:
            primitive.type = shape_msgs::SolidPrimitive::CYLINDER;
            primitive.dimensions = {obj.height, obj.radius};
            break;
        case Object::Shape::Mesh:
        default:
            primitive.type = shape_msgs::SolidPrimitive::BOX;
            primitive.dimensions = {obj.length, obj.width, obj.height};
            break;
    }
    return primitive;
}

geometry_msgs::Pose PlanInstance::getPose(const Object &obj)
{
    geometry_msgs::Pose pose;
    pose.position.x = obj.x;
    pose.position.y = obj.y;
    pose.position.z = obj.z;
    pose.orientation.x = obj.qx;
    pose.orientation.y = obj.qy;
    pose.orientation.z = obj.qz;
    pose.orientation.w = obj.qw;
    return pose;
}
#endif

void PlanInstance::setNumberOfRobots(int num_robots) {
    num_robots_ = num_robots;
    start_poses_.resize(num_robots);
    goal_poses_.resize(num_robots);
    robot_dof_.resize(num_robots);
    hand_dof_.resize(num_robots, 0);
}

void PlanInstance::setStartPose(int robot_id, const std::vector<double> &pose) {
    start_poses_[robot_id].robot_id = robot_id;
    start_poses_[robot_id].robot_name = robot_names_[robot_id];
    start_poses_[robot_id].joint_values = pose;
}

void PlanInstance::setGoalPose(int robot_id, const std::vector<double> &pose) {
    goal_poses_[robot_id].robot_id = robot_id;
    goal_poses_[robot_id].robot_name = robot_names_[robot_id];
    goal_poses_[robot_id].joint_values = pose;
}

void PlanInstance::setRobotDOF(int robot_id, size_t dof) {
    if (robot_id >= robot_dof_.size()) {
        robot_dof_.resize(robot_id + 1);
    }
    robot_dof_[robot_id] = dof;
}

void PlanInstance::setHandDof(int robot_id, size_t dof) {
    if (robot_id >= hand_dof_.size()) {
        hand_dof_.resize(robot_id + 1);
    }
    hand_dof_[robot_id] = dof;
}


size_t PlanInstance::getRobotDOF(int robot_id) const {
    return robot_dof_[robot_id];
}

size_t PlanInstance::getHandDOF(int robot_id) const {
    return hand_dof_[robot_id];
}

RobotPose PlanInstance::initRobotPose(int robot_id) const {
    RobotPose pose;
    pose.robot_id = robot_id;
    pose.robot_name = robot_names_[robot_id];
    pose.joint_values.resize(robot_dof_[robot_id]);
    pose.hand_values.resize(hand_dof_[robot_id], 0);
    return pose;
}

double PlanInstance::getVMax(int robot_id) {
    return v_max_;
}

void PlanInstance::setVmax(double vmax) {
    v_max_ = vmax;
}

int PlanInstance::numCollisionChecks() {
    int ans = num_collision_checks_;
    num_collision_checks_ = 0;
    return ans;
}

double PlanInstance::computeMotionStepSize(const std::vector<RobotPose> &start,
                                           const std::vector<RobotPose> &goal,
                                           int num_samples) const
{
    if (num_samples <= 0 || start.empty() || goal.empty()) {
        return 0.0;
    }

    double max_distance = 0.0;
    for (const auto &start_pose : start) {
        auto it = std::find_if(goal.begin(), goal.end(), [&](const RobotPose &pose) {
            return pose.robot_id == start_pose.robot_id;
        });
        if (it != goal.end()) {
            max_distance = std::max(max_distance, computeDistance(start_pose, *it));
        }
    }

    if (max_distance <= std::numeric_limits<double>::epsilon()) {
        return 0.0;
    }

    return max_distance / static_cast<double>(num_samples);
}

// Returns true if any waypoint along the interpolated motion collides; false otherwise.
bool PlanInstance::checkMultiRobotMotion(const std::vector<RobotPose> &start,
                                         const std::vector<RobotPose> &goal,
                                         double step_size,
                                         bool self)
{
    if (start.size() != goal.size()) {
        throw std::invalid_argument("Start and goal pose sets must have the same size");
    }

    if (start.empty()) {
        // Nothing to validate; treat as collision free.
        return false;
    }

    const double effective_step = (step_size > 0.0) ? step_size : 0.1;

    std::unordered_map<int, RobotPose> start_map;
    start_map.reserve(start.size());
    for (const auto &pose : start) {
        start_map[pose.robot_id] = pose;
    }

    struct PosePair {
        RobotPose start;
        RobotPose goal;
        double distance{0.0};
    };

    std::vector<PosePair> pairs;
    pairs.reserve(goal.size());

    double max_distance = 0.0;
    for (const auto &goal_pose : goal) {
        auto it = start_map.find(goal_pose.robot_id);
        if (it == start_map.end()) {
            throw std::invalid_argument("Goal pose set is missing a matching start pose");
        }

        PosePair pair{it->second, goal_pose, computeDistance(it->second, goal_pose)};
        max_distance = std::max(max_distance, pair.distance);
        pairs.emplace_back(std::move(pair));
    }

    if (max_distance <= std::numeric_limits<double>::epsilon()) {
        // Degenerate case; bubble up whether the provided configuration is already in collision.
        return checkCollision(goal, self);
    }

    const auto step_count = static_cast<std::size_t>(
        std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(max_distance / effective_step))));

    // Ensure the starting configuration itself is valid before stepping.
    if (checkCollision(start, self)) {
        return true;
    }

    std::vector<RobotPose> interpolated;
    interpolated.reserve(pairs.size());

    for (std::size_t step = 1; step <= step_count; ++step) {
        const double t = static_cast<double>(step) / static_cast<double>(step_count);
        interpolated.clear();
        for (const auto &pair : pairs) {
            interpolated.emplace_back(interpolate(pair.start, pair.goal, t));
        }

        if (checkCollision(interpolated, self)) {
            return true;
        }
    }

    // No collisions observed along the motion.
    return false;
}

bool PlanInstance::checkMultiRobotTrajectory(const MRTrajectory &trajectory,
                                             bool self)
{
    if (trajectory.empty())
    {
        throw std::invalid_argument("Multi-robot trajectory is empty");
    }

    const std::size_t robot_count = trajectory.size();

    const std::size_t step_count = trajectory.front().trajectory.size();
    if (step_count == 0)
    {
        return false;
    }

    for (const auto &robot_traj : trajectory)
    {
        if (robot_traj.trajectory.size() != step_count)
        {
            throw std::invalid_argument("Multi-robot trajectories must all share the same waypoint count");
        }
    }

    std::vector<RobotPose> snapshot;
    snapshot.reserve(robot_count);

    for (std::size_t step = 0; step < step_count; ++step)
    {
        snapshot.clear();
        for (const auto &robot_traj : trajectory)
        {
            snapshot.push_back(robot_traj.trajectory[step]);
        }

        if (checkCollision(snapshot, self))
        {
            return true;
        }
    }

    return false;
}
