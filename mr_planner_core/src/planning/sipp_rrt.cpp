#include "mr_planner/planning/sipp_rrt.h"
#include "mr_planner/core/logger.h"


SIPP_RRT::SIPP_RRT(std::shared_ptr<PlanInstance> instance, int robot_id)
    : SingleAgentPlanner(instance, robot_id) {
        vMax_ = instance_->getVMax(robot_id);
    }

bool SIPP_RRT::findSafeIntervals(const RobotPose &pose, std::vector<SafeInterval> &safe_interval) {
    safe_interval.clear();

    double si_start = -1;

    double t = 0.0;
    int ind = 0;
    while (t <= moving_obs_span_) {
        bool has_collision = false;
        for (int i = 0; i < obs_poses_.size(); i++) {
            if (instance_->checkCollision({pose, obs_poses_[i][ind]}, true) == true) {
                has_collision = true;
                break;
            }
        }

        if (has_collision) {
            if (si_start >= 0 && si_start < (t-col_dt_)) {
                // add a safe interval if there is a collision-free interval
                SafeInterval si;
                si.pose = pose;
                si.t_start = si_start;
                si.t_end = t - col_dt_;
                safe_interval.push_back(si);
            }
            si_start = -1;
        }
        else {
            if (si_start == -1) {
                si_start = t;
            }
        }

        t += col_dt_;
        ind ++;
    }

    if (si_start >= 0) {
        SafeInterval si;
        si.pose = pose;
        si.t_start = si_start;
        si.t_end = 1000000;
        safe_interval.push_back(si);
    }
    return true;
}


bool SIPP_RRT::init(const PlannerOptions &options) { 
    // initialize two trees, one for the start and one for the goal
    start_pose_ = instance_->getStartPose(robot_id_);
    goal_pose_ = instance_->getGoalPose(robot_id_);

    // check if goal pose is collision free with the static environment
    if (instance_->checkCollision({goal_pose_}, false) == true) {
        log("Goal pose is in collision", LogLevel::ERROR);
        instance_->checkCollision({goal_pose_}, false, true); // print debug info about contact
        return false;
    }
    // check if the start pose is collision free with the static environment
    if (instance_->checkCollision({start_pose_}, false) == true) {
        log("Start pose is in collision", LogLevel::ERROR);
        instance_->checkCollision({start_pose_}, false, true); // print debug info about contact
        return false;
    }

    // precompute obstacle poses at discretized time steps
    obstacles_ = options.obstacles;
    precompute_obs_pose();
    
    // find safe intervals for start and goal poses
    std::vector<SafeInterval> start_si;
    if (!findSafeIntervals(start_pose_, start_si)) {
        log("No safe intervals found for start pose", LogLevel::ERROR);
        return false;
    }

    std::vector<SafeInterval> goal_si;
    if (!findSafeIntervals(goal_pose_, goal_si)) {
        log("No safe intervals found for goal pose", LogLevel::ERROR);
        return false;
    }
    
    min_time_ = instance_->computeDistance(start_pose_, goal_pose_) / vMax_;
    // find the last time when obstacles pass thru the goal poses
    for (const RobotTrajectory &obs : obstacles_) {
        double last_time = obs.times.back();
        int num_points = last_time / col_dt_ + 1;
        int ind = 0;
        RobotPose obs_i_pose;
        for (int i = 0; i < num_points; i++) {  
            double t = i * col_dt_;
            while (ind + 1 < obs.times.size() && obs.times[ind + 1] <= t) {
                ind++;
            }
            if (ind + 1 == obs.times.size()) {
                // assuming obstacle stays at the end of the trajectory
                obs_i_pose = obs.trajectory[ind];
            } else {
                double alpha = (t - obs.times[ind]) / (obs.times[ind + 1] - obs.times[ind]);
                obs_i_pose = instance_->interpolate(obs.trajectory[ind], obs.trajectory[ind + 1], alpha);
            }
            if (instance_->checkCollision({goal_pose_, obs_i_pose}, true) == true) {
                // has collision
                min_time_ = std::max(min_time_, t + col_dt_);
            }
        }
    }

    if (min_time_ < 1e-6) {
        log("No need to plan for static agent", LogLevel::INFO);
        return false;
    }
    log("Minimum time: " + std::to_string(min_time_), LogLevel::DEBUG);

    // initialize the root nodes in the trees

    auto startVertex = std::make_shared<Vertex>(start_pose_);
    startVertex->setTime(0.0);
    startVertex->setSafeInterval(start_si[0]);
    start_tree_.addRoot(startVertex);
    totalTreeSize++;

    for (int i = 0; i < goal_si.size(); i++) {
        if (goal_si[i].t_end < min_time_) {
            continue;
        }
        if (goal_si[i].t_start < min_time_) {
            goal_si[i].t_start = min_time_;
        }
        // add vertex to the goal tree
        auto goalVertex = std::make_shared<Vertex>(goal_pose_);
        goalVertex->setOtherTime(goal_si[i].t_end);
        goalVertex->setSafeInterval(goal_si[i]);
        goal_tree_.addRoot(goalVertex);
        totalTreeSize++;
    }

    return true;
}
 
bool SIPP_RRT::plan(const PlannerOptions &options) {

    start_time_ = std::chrono::high_resolution_clock::now();

    if (!init(options)) {
        best_cost_ = 0.0;
        solution_.robot_id = robot_id_;
        solution_.times.clear();
        solution_.trajectory.clear();
        solution_.times.push_back(0.0);
        solution_.trajectory.push_back(goal_pose_);
        solution_.cost = 0.0;
        log("Plan is just the start/goal pose", LogLevel::INFO);
        return true;
    }

   
    VertexPtr goalVertex = goal_tree_.roots[0]; 

    // try to connect start and goal in minimum time
    RobotTrajectory interpolate_traj;
    if (interpolate(start_pose_, goal_pose_, interpolate_traj)) {
        log("Connected start and goal in minimum time", LogLevel::INFO);
        solution_ = interpolate_traj;
        best_cost_ = solution_.cost;

        for (int i = 0; i < solution_.times.size(); i++) {
            log(solution_.trajectory[i], LogLevel::DEBUG);
            log("solution times: " + std::to_string(solution_.times[i]), LogLevel::DEBUG);
        }

        return true;
    }

    
    while (!terminate(options)) {
        log("iteration: " + std::to_string(numIterations_) 
            + ", start tree size " + std::to_string(start_tree_.vertices.size())
            + ", goal tree size " + std::to_string(goal_tree_.vertices.size()), LogLevel::DEBUG);
        numIterations_++;

        // 1. sample new state
        RobotPose new_pose;
        bool found_sample = sample(new_pose);
        if (!found_sample) {
            log("Failed to sample new pose", LogLevel::DEBUG);
            swap_trees();
            continue;
        }
    
        numValidSamples_++;
        log("Sampled new pose", LogLevel::DEBUG);
        log(new_pose, LogLevel::DEBUG);
        
        // 4. connect the safe intervals to the other tree
        std::vector<VertexPtr> added_vertices;
        GrowState extended_state = step(new_pose, added_vertices, *current_tree_, goal_tree, false);
        if (extended_state == TRAPPED) {
            swap_trees();
            continue;
        }

        // 5. connect to the other tree
        assert(added_vertices.size() > 0);
        RobotPose delta_pose = added_vertices[0]->pose;
        std::vector<VertexPtr> connected_vertex; // vertex in current tree, which are also connected to other tree
        if (connect(delta_pose, connected_vertex, *other_tree_, !goal_tree) == REACHED) {
            update_solution(connected_vertex, goal_tree);
        }
        

        // 5. swap_trees
        swap_trees();

    }

    log("Final iteration: " + std::to_string(numIterations_) 
            + ", " + std::to_string(numValidSamples_) + " valid samples"
            + ", start tree size " + std::to_string(start_tree_.vertices.size())
            + ", goal tree size " + std::to_string(goal_tree_.vertices.size()), LogLevel::INFO);

    if (best_cost_ < std::numeric_limits<double>::max()) {
        return true;
    }
    return false;
}

GrowState SIPP_RRT::step(const RobotPose &new_pose, std::vector<VertexPtr> &added_vertices, Tree &tree, bool goal_tree, bool connect)
{
    if (connect) {
        if (goal_tree) {
            log("Connecting to goal tree", LogLevel::DEBUG);
        }
        else {
            log("Connecting to start tree", LogLevel::DEBUG);
        }
    }
    else {
        if (goal_tree) {
            log("Extending from goal tree", LogLevel::DEBUG);
        }
        else {
            log("Extending from start tree", LogLevel::DEBUG);
        }
    }

    added_vertices.clear();

    // 2. extend this to the nearest tree
    RobotPose delta_pose;
    RobotPose nearest_pose;
    GrowState extended = extend(new_pose, tree, delta_pose, nearest_pose, goal_tree);
    if (extended == TRAPPED) {
        log("Trapped", LogLevel::DEBUG);
        return TRAPPED;
    }

    if (!connect || extended != REACHED) {
        // add to the current tree
        // 3. compute safe interval

        std::vector<SafeInterval> safe_intervals;
        findSafeIntervals(delta_pose, safe_intervals);
        log("Found " + std::to_string(safe_intervals.size()) + " safe intervals", LogLevel::DEBUG);
        
        // 4. connect the safe intervals to the other tree
        for (int i = 0; i < safe_intervals.size(); i++) {
            auto newVertex = std::make_shared<Vertex>(delta_pose);
            newVertex->setSafeInterval(safe_intervals[i]);
            if (setSIParent(newVertex, nearest_pose, tree, goal_tree, false) == REACHED) {
                log("Connected safe interval " + std::to_string(newVertex->time) + " to parent t=" + std::to_string(newVertex->parent->time), LogLevel::DEBUG);
                added_vertices.push_back(newVertex);
            }
        }
    }
    else {
        // add to the opposite tree, si already computed
        Tree* other_tree_ = (goal_tree) ? &start_tree_ : &goal_tree_;
        std::vector<VertexPtr> new_vertices_other_tree = other_tree_->vertex_map[new_pose];
        for (auto &new_vertex : new_vertices_other_tree) {
            if (setSIParent(new_vertex, nearest_pose, tree, goal_tree, true) == REACHED) {
                log("Connected to other tree " + std::to_string(new_vertex->time) + " to parent t=" + std::to_string(new_vertex->parent->time)
                     + ", other parent t=" + std::to_string(new_vertex->otherParent->time), LogLevel::DEBUG);
                added_vertices.push_back(new_vertex);
            }
        }
    }
    if (added_vertices.size() == 0) {
        log("Safe interval not connected", LogLevel::DEBUG);
        return TRAPPED;
    }
    
    return extended;
}

bool SIPP_RRT::interpolate(const RobotPose &pose_1, const RobotPose &pose_2, RobotTrajectory &solution) {
    double distance = instance_->computeDistance(pose_1, pose_2);
    double travel_t = distance / vMax_;
    travel_t = std::max(travel_t, min_time_);
    travel_t = std::ceil(travel_t / col_dt_) * col_dt_;

    int num_steps = std::round(travel_t / col_dt_) + 1;
    double t = 0.0;
    for (int i = 0; i < num_steps; i++) {
        double alpha = t / travel_t;
        RobotPose pose = instance_->interpolate(pose_1, pose_2, alpha);

        // check collision statically
        if (i > 0) {
            if (instance_->checkCollision({pose}, false) == true) {
                return false;
            }
            if (validateMotion(solution.trajectory.back(), pose, solution.times.back(), t) == false) {
                return false;
            }
        }

        solution.trajectory.push_back(pose);
        solution.times.push_back(t);
        t += col_dt_;
    }
    solution.cost = travel_t;
    return true;
}


bool SIPP_RRT::terminate(const PlannerOptions &options) {
    log("num iterations: " + std::to_string(numIterations_) + " max iterations: " + std::to_string(options.max_planning_iterations), LogLevel::DEBUG);
    if (numIterations_ > options.max_planning_iterations) {
        return true;
    }
    auto elapsed = std::chrono::high_resolution_clock::now() - start_time_;
    double elapsed_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(elapsed).count();
    log("elapsed time: " + std::to_string(elapsed_seconds) + " max time: " + std::to_string(options.max_planning_time), LogLevel::DEBUG);
    if (elapsed_seconds > options.max_planning_time) {
        return true;
    }
    if (options.terminate_on_first_sol && best_cost_ < std::numeric_limits<double>::max()) {
        return true;
    }

    return false;
}

bool SIPP_RRT::sample(RobotPose &new_pose) {
    new_pose = instance_->initRobotPose(robot_id_);
    bool found_sample = false;
    int tries = 0;
    int max_tries = 10;

    while (!found_sample && tries < max_tries) {
        tries++;
        //log("sample tries: " + std::to_string(tries), LogLevel::DEBUG);
        found_sample = instance_->sample(new_pose);
        if (!found_sample) {
            continue;
        }
        // // calculate time bound
        // double min_time = instance_->computeDistance(start_pose_, newpose) / vMax_;
        // double max_time = max_time_ - instance_->computeDistance(newpose, goal_pose_) / vMax_;
        // if (min_time > max_time) {
        //     found_sample = false;
        //     log("sample pose not reachable", LogLevel::DEBUG);
        //     continue;
        // }
    
    }

    return found_sample;
}


GrowState SIPP_RRT::setSIParent(VertexPtr &new_sample, const RobotPose &nearest_pose, Tree &tree, 
        bool goalTree, bool otherParent) {
    std::vector<VertexPtr> nearest_vertices = tree.vertex_map[nearest_pose];

    auto new_si = new_sample->si;
    log("new sample safe interval [" + std::to_string(new_si.t_start) + " " + std::to_string(new_si.t_end) + "]", LogLevel::DEBUG);
    log("nearest vertices size: " + std::to_string(nearest_vertices.size()), LogLevel::DEBUG);

    double dist = instance_->computeDistance(nearest_pose, new_sample->pose);
    double travel_t = dist / vMax_;
    travel_t = std::ceil(travel_t / col_dt_) * col_dt_;

    if (!goalTree) {
        // arrive as early as possible
        // sort the nearest vertices by time ascending
        std::sort(nearest_vertices.begin(), nearest_vertices.end(), 
            [](const VertexPtr &a, const VertexPtr &b) {
                return a->time < b->time;
            });

        for (auto &nearest_vertex : nearest_vertices) {
            log("nearest vertex in start tree:", LogLevel::DEBUG);
            log(nearest_vertex->pose, LogLevel::DEBUG);
            auto nearest_si = nearest_vertex->si;
            double nearest_t = std::ceil(nearest_vertex->time / col_dt_) * col_dt_;
            double t_earliest = nearest_t + travel_t;
            double nearest_t_upperb = std::ceil(nearest_si.t_end / col_dt_) * col_dt_;
            double t_latest = nearest_t_upperb + travel_t;
            log("arrive si: [" + std::to_string(t_earliest) + ", " + std::to_string(t_latest) + "]", LogLevel::DEBUG);
            if (t_earliest > new_si.t_end || t_latest < new_si.t_start) {
                continue;
            }

            // iterate over all discrete timestep from t_earliest to t_latest
            double t = std::max(t_earliest, new_si.t_start);
            double until = std::min(t_latest, new_si.t_end);
            // if until is infinity, then set it to obs_span + t_travel
            if (until > 999999) {
                until = moving_obs_span_ + travel_t;
            }
            while (t <= until) {
                if (validateMotion(nearest_pose, new_sample->pose, t - travel_t, t)) {
                    if (otherParent) {
                        if (t <= new_sample->other_time) {
                            new_sample->setTime(t);
                            new_sample->addOtherParent(nearest_vertex);
                            new_sample->otherParent->setOtherTime(t - travel_t);
                            tree.addVertex(new_sample);
                            return REACHED;
                        }
                    }
                    else {
                        new_sample->setTime(t);
                        new_sample->addParent(nearest_vertex);
                        new_sample->parent->setOtherTime(t - travel_t);
                        tree.addVertex(new_sample);
                        return REACHED;
                    }
                }
                t += col_dt_;
            }
        }
    }
    else {
        // goal tree, arrive as late as possible
        // sort the nearest vertices by other_time descending
        std::sort(nearest_vertices.begin(), nearest_vertices.end(), 
            [](const VertexPtr &a, const VertexPtr &b) {
                return a->other_time > b->other_time;
            });
        
        for (auto &nearest_vertex : nearest_vertices) {
            
            auto nearest_si = nearest_vertex->si;
            double nearest_t = std::ceil(nearest_vertex->other_time / col_dt_) * col_dt_;
            double t_latest = nearest_t - travel_t;
            double t_earliest = std::ceil(nearest_si.t_start / col_dt_) * col_dt_ - travel_t;
            if (t_earliest > new_si.t_end || t_latest < new_si.t_start) {
                continue;
            }

            // iterate over all discrete timestep from t_latest to t_earliest
            double t = std::min(t_latest, new_si.t_end);
            double until = std::max(t_earliest, new_si.t_start);

            log("nearest vertex in goal tree:", LogLevel::DEBUG);
            log(nearest_vertex->pose, LogLevel::DEBUG);
            log("leave si: [" + std::to_string(until) + ", " + std::to_string(t_latest) + "]", LogLevel::DEBUG);

            while (t >= until) {
                if (validateMotion(nearest_pose, new_sample->pose, t, t + travel_t)) {
                    if (otherParent) {
                        if (t >= new_sample->time) {
                            new_sample->setOtherTime(t);
                            new_sample->addOtherParent(nearest_vertex);
                            new_sample->otherParent->setTime(t + travel_t);
                            tree.addVertex(new_sample);
                            return REACHED;
                        }
                    }
                    else {
                        new_sample->setOtherTime(t);
                        new_sample->addParent(nearest_vertex);
                        new_sample->parent->setTime(t + travel_t);
                        tree.addVertex(new_sample);
                        return REACHED;
                    }
                }
                if (t > 900000) {
                    // if it is impossible to connect to goal tree with inf t, meaning no moving robots
                    // move down to moving_obs_span
                    t = moving_obs_span_;
                }
                t -= col_dt_;
            }
        }
    }
    return TRAPPED;

}


double SIPP_RRT::nearest(const RobotPose &sample, VertexPtr &nearest_vertex, const Tree &tree, bool goalTree) {
    assert(tree.vertices.size() > 0);

    double min_distance = std::numeric_limits<double>::max();
    for (auto &vertex : tree.vertices) {
        double distance = instance_->computeDistance(vertex->pose, sample);
        
        if (distance < min_distance) {
            min_distance = distance;
            nearest_vertex = vertex;
        }
    }
    return min_distance;
}

GrowState SIPP_RRT::extend(const RobotPose &new_sample, Tree &tree, RobotPose &delta_pose, RobotPose &nearest_pose, bool goalTree) {
    // find the nearest vertex in the tree
    VertexPtr nearest_vertex;
    double distanceSpace = nearest(new_sample, nearest_vertex, tree, goalTree);
    if (nearest_vertex == nullptr) {
        log("nearest vertex is null", LogLevel::DEBUG);
        return TRAPPED;
    }
    nearest_pose = nearest_vertex->pose;
    log("nearest vertex:", LogLevel::DEBUG);
    log(nearest_pose, LogLevel::DEBUG);
 
    bool reached = true;
    if (distanceSpace <= max_delta_)
    {
        // check if can directly reach the new sample
        if (std::abs(distanceSpace) < 1e-6) {
            log("Reached the new sample", LogLevel::DEBUG);
            delta_pose = new_sample;
            return REACHED;
        }
        else if (instance_->connect(nearest_pose, new_sample)) {
            log("Reach the new sample", LogLevel::DEBUG);
            delta_pose = new_sample;
            return REACHED;
        }
        else {
            log("Trapped on the way to new sample", LogLevel::DEBUG);
            return TRAPPED;
        }
    }
    else {
        reached = false;
        // steer from the nearest vertex to the new sample
        delta_pose = instance_->initRobotPose(robot_id_);
        if (instance_->steer(nearest_vertex->pose, new_sample, max_delta_, delta_pose)) {
            log("Steer to the new sample", LogLevel::DEBUG);
            return ADVANCED;
        }
        else {
            log("Trapped on the way to new sample", LogLevel::DEBUG);
            return TRAPPED;
        }
    } 

    return TRAPPED;
}

void SIPP_RRT::precompute_obs_pose() {
    moving_obs_span_ = 0.0;
    for (int i = 0; i < obstacles_.size(); i++) {
        RobotTrajectory obs = obstacles_[i];
        double tend = obs.times.back();
        moving_obs_span_ = std::max(moving_obs_span_, tend);
    }
    
    moving_obs_steps_ = std::ceil(moving_obs_span_ / col_dt_) + 1;
    
    // precompute all the obstacle poses for all the obstacle robots
    obs_poses_.clear();
    obs_poses_.reserve(obstacles_.size());
    for (const RobotTrajectory &obs : obstacles_) {
        std::vector<RobotPose> obs_i_pose;
        obs_i_pose.reserve(moving_obs_steps_);

        int ind = 0;
        // compile this dynamic obstacle's poses at each time step
        for (int s = 0; s < moving_obs_steps_; s++) {
            double t = s * col_dt_;
            while (ind + 1 < obs.times.size() && obs.times[ind + 1] <= t) {
                ind++;
            }
            if (ind + 1 == obs.times.size()) {
                // assuming obstacle stays at the end of the trajectory
                obs_i_pose.push_back(obs.trajectory[ind]);
            } else {
                double alpha = (t - obs.times[ind]) / (obs.times[ind + 1] - obs.times[ind]);
                RobotPose obs_i_pose_s = instance_->interpolate(obs.trajectory[ind], obs.trajectory[ind + 1], alpha);
                obs_i_pose.push_back(obs_i_pose_s);
            }
        }
        obs_poses_.push_back(obs_i_pose);
    }
}

bool SIPP_RRT::validateMotion(const RobotPose &pose_1, const RobotPose &pose_2, double t1, double t2) {
    // check inter agent collision
    assert (t1 <= t2);
    int num_steps = (t2 - t1) / col_dt_ + 1;

    int s_t1 = t1 / col_dt_;
    // check for inter-agent collision
    for (int s = 0; s < num_steps; s++) {
        double t = t1 + s * col_dt_;
        double alpha = (t - t1) / (t2 - t1);
        RobotPose a_pose = instance_->interpolate(pose_1, pose_2, alpha);
        int s_t = s + s_t1;
        if (s_t >= moving_obs_steps_) {
            s_t = moving_obs_steps_ - 1;
        }
        for (int i = 0; i < obs_poses_.size(); i++) {
            if (instance_->checkCollision({a_pose, obs_poses_[i][s_t]}, true) == true) {
                return false;
            }

        }
    }
    return true;
}

GrowState SIPP_RRT::connect(const RobotPose &delta_pose, std::vector<VertexPtr> &connected_vertex, Tree &tree, bool goalTree) {
    GrowState gsc = ADVANCED;
    while (gsc == ADVANCED) {
        // repeat the steps of extending, finding safe intervals, and adding parents
        std::vector<VertexPtr> added_vertex;
        gsc = step(delta_pose, added_vertex, tree, goalTree, true);
        if (gsc == REACHED) {
            connected_vertex = added_vertex;
            return REACHED;
        }
    }
    return gsc;
}

void SIPP_RRT::shortenSITime(VertexPtr connected_vertex, bool goal_tree) {
    auto shortenTime = [&](VertexPtr &current, VertexPtr &other) {
        SafeInterval other_si = other->si;
        double dist = instance_->computeDistance(current->pose, other->pose);
        double travel_t = dist / vMax_;
        travel_t = std::ceil(travel_t / col_dt_) * col_dt_;

        auto nearest_si = current->si;
        double nearest_t = std::ceil(current->time / col_dt_) * col_dt_;
        double t_earliest = nearest_t + travel_t;
        double nearest_t_upperb = std::ceil(nearest_si.t_end / col_dt_) * col_dt_;
        double t_latest = nearest_t_upperb + travel_t;
        log("arrive si: [" + std::to_string(t_earliest) + ", " + std::to_string(t_latest) + "]", LogLevel::DEBUG);

        // iterate over all discrete timestep from t_earliest to t_latest
        double t = std::max(t_earliest, other_si.t_start);
        double until = std::min(t_latest, other_si.t_end);
        while (t <= until) {
            if (validateMotion(current->pose, other->pose, t - travel_t, t)) {
                other->setTime(t);
                current->setOtherTime(t - travel_t);
                break;
            }
            t += col_dt_;
        }
        current = other;
    };

    VertexPtr current = connected_vertex;
    if (!goal_tree) {
        VertexPtr other = current->otherParent;
        shortenTime(current, other);
    }

    while (current->parent != nullptr) {
        VertexPtr other = current->parent;
        shortenTime(current, other);
    }
}

void SIPP_RRT::update_solution(std::vector<VertexPtr> &connected_vertex, bool goalTree) {
    VertexPtr new_sample = nullptr;
    for (auto &vertex : connected_vertex) {
        if (new_sample == nullptr) {
            new_sample = vertex;
        }
        else if (goalTree) {
            if (vertex->other_time < new_sample->other_time) {
                new_sample = vertex;
            }
        }
        else {
            if (vertex->time < new_sample->time) {
                new_sample = vertex;
            }
        }
    }
    // connect the new sample to the 
    assert (new_sample->parent != nullptr && new_sample->otherParent != nullptr);

    shortenSITime(new_sample, goalTree);

    // trace back the path from the new sample to the root
    std::vector<VertexPtr> path, back_path;
    VertexPtr current = new_sample;
    while (current != nullptr) {
        path.push_back(current);
        current = current->parent;
    }
    // reverse the stack
    std::reverse(path.begin(), path.end());

    // trace to the other parent root
    current = new_sample->otherParent;
    while (current != nullptr) {
        back_path.push_back(current);
        current = current->parent;
    }

    // concatenate the two paths
    path.insert(path.end(), back_path.begin(), back_path.end());

    if (goalTree) {
        std::reverse(path.begin(), path.end());
    }

    double solutionCost = path.back()->time;
    if (solutionCost < best_cost_) {
        best_cost_ = solutionCost;
        // set the path to the solution
        solution_.robot_id = robot_id_;
        solution_.times.clear();
        solution_.trajectory.clear();
        solution_.cost = best_cost_;
        for (int i = 0; i < path.size(); i++) {
            auto vertex = path[i];
            solution_.times.push_back(vertex->time);
            solution_.trajectory.push_back(vertex->pose);
            if (i < path.size() - 1 && vertex->time < (vertex->other_time - 1e-5)) {
                solution_.times.push_back(vertex->other_time);
                solution_.trajectory.push_back(vertex->pose);
            }
        }
        log("Found a better solution with cost: " + std::to_string(best_cost_), LogLevel::INFO);
        log(solution_, LogLevel::DEBUG);
    }

    return;
}

void SIPP_RRT::swap_trees() {
    current_tree_ = (current_tree_ == &start_tree_) ? &goal_tree_ : &start_tree_;
    other_tree_ = (other_tree_ == &start_tree_) ? &goal_tree_ : &start_tree_;
    goal_tree = !goal_tree;
}


bool SIPP_RRT::getPlan(RobotTrajectory &solution) const {
    if (best_cost_ < std::numeric_limits<double>::max()) {
        solution = solution_;
        return true;
    }
    return false;
}

double SIPP_RRT::getPlanCost() const {
    return best_cost_;
}