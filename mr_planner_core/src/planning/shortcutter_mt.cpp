#include "mr_planner/planning/shortcutter.h"

ShortcutSamplerMT::ShortcutSamplerMT(const ShortcutOptions &options, std::shared_ptr<ActivityGraph> act_graph)
    : ShortcutSampler(options), act_graph_(act_graph)
{
}

bool ShortcutSamplerMT::init(const MRTrajectory &sync_plan, const std::vector<int> &num_points, const std::vector<std::vector<int>> &num_task_points)
{
    num_robots_ = sync_plan.size();
    num_points_ = num_points;
    num_task_points_ = num_task_points;
    act_ids_.resize(num_robots_);
    for (int i = 0; i < num_robots_; i++) {
        act_ids_[i] = sync_plan[i].act_ids;
    }
    
    return true;
}

bool ShortcutSamplerMT::sampleShortcut(Shortcut &shortcut, double time_progress)
{
    return sampleUniform(shortcut);
}

bool ShortcutSamplerMT::sampleUniform(Shortcut &shortcut)
{
    int r_id = rand() % num_robots_;
    int a = rand() % num_points_[r_id];
    
    // find which task does a fall into
    int act_id = act_ids_[r_id][a];

    int act_length = 0;
    for (int t = 0; t < act_id; t++) {
        act_length += num_task_points_[r_id][t];
    }

    if (num_task_points_[r_id][act_id] == 0) {
        log("Sampled Robot " + std::to_string(r_id) + " Task " + std::to_string(act_id) + " has no points", LogLevel::ERROR);
    }
    int b = act_length + rand() % num_task_points_[r_id][act_id];

    if (a > b) {
        std::swap(a, b);
    }
    if ((a+1)>=b) {
        return false;
    }

    shortcut.robot_id = r_id;
    shortcut.a = a;
    shortcut.b = b;
    shortcut.act = act_graph_->get(r_id, act_id);

    return true;
}

ShortcutterMT::ShortcutterMT(std::shared_ptr<PlanInstance> instance,
                std::shared_ptr<ActivityGraph> act_graph,
                const ShortcutOptions &options)
{
    instance_ = instance;
    act_graph_ = act_graph;
    options_ = options;
    srand(options.seed);

    
}

void ShortcutterMT::calculate_numpoints() {
    int num_robots = instance_->getNumberOfRobots();
    num_points_.resize(num_robots);
    num_task_points_.resize(num_robots);
    for (int i = 0; i < num_robots; i++) {
        num_points_[i] = 0;
        num_task_points_[i] = std::vector<int>(act_graph_->num_activities(i), 0);
    }

    for (size_t i = 0; i < synced_plan_.size(); i++) {
        num_points_[i] = synced_plan_[i].trajectory.size();
        if (num_points_[i] == 0) {
            log("Robot " + std::to_string(i) + " has no points", LogLevel::WARN);
            continue;
        }
        int ind = synced_plan_[i].act_ids[0];
        int counter = 0;
        for (size_t j = 0; j < num_points_[i] - 1; j++) {
            counter ++;
            if (synced_plan_[i].act_ids[j] != synced_plan_[i].act_ids[j+1]) {
                num_task_points_[i][ind] = counter;
                counter = 0;
                ind += (synced_plan_[i].act_ids[j+1] - synced_plan_[i].act_ids[j]);
            }
        }
        counter++;
        num_task_points_[i][ind] = counter;
    }
}

bool ShortcutterMT::shortcutSolution(const MRTrajectory &solution,
                                MRTrajectory &smoothed_solution)
{
    sampler_ = std::make_shared<ShortcutSamplerMT>(options_, act_graph_);
    synced_plan_ = solution;
    
    prempt_home_act();
    update_timed_index();
    pre_makespan_ = makespan_;
    calculate_numpoints();
    if (!checkTaskDep()) {
        log("Initial solution violates task dep", LogLevel::ERROR);
        return false;
    }

    if (options_.t_limit > 0) {
        calculate_path_length_and_wait_time(instance_, synced_plan_, path_length_, wait_time_);
        logProgress(options_.progress_file, 0.0);
    }
    else {
        smoothed_solution = solution;
        return true;
    }

    sampler_->init(synced_plan_, num_points_, num_task_points_);

    double t_limit = options_.t_limit;
    double log_interval = options_.log_interval;
    double log_limit = options_.log_interval;
    double elapsed = 0.0;
    double dt = options_.dt;

    while (elapsed < t_limit) {
        auto tic = std::chrono::high_resolution_clock::now();
        
        Shortcut shortcut;
        if (!sampler_->sampleShortcut(shortcut, elapsed / t_limit)) {
            auto toc = std::chrono::high_resolution_clock::now();
            elapsed += std::chrono::duration_cast<std::chrono::nanoseconds>(toc - tic).count() * 1e-9;
            continue;
        }

        log("Sampled shortcut from robot " + std::to_string(shortcut.robot_id) + " activity " + shortcut.act->type_string() + 
            " timestep " + std::to_string(shortcut.a) +
            " to timestep " + std::to_string(shortcut.b), LogLevel::DEBUG);

        preCheckShortcut(shortcut);
        if (shortcut.col_type == CollisionType::NONE) {
            auto tic_inner = std::chrono::high_resolution_clock::now();
            checkShortcut(shortcut);
            auto inner = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - tic_inner).count();
            t_check_ += (inner * 1e-9);
            n_check_++;
            n_colcheck_ += instance_->numCollisionChecks();
            if (shortcut.col_type == CollisionType::NONE) {
                updatePlan(shortcut);

                if (options_.prioritized_shortcut) {
                    update_timed_index();
                    calculate_numpoints();
                }
                sampler_->init(synced_plan_, num_points_, num_task_points_);

                n_valid_ ++;
                
                log("found shortcut for robot " + std::to_string(shortcut.robot_id) + " of length " + std::to_string(shortcut.path.size() + 1), LogLevel::DEBUG);
                log("from " + std::to_string(shortcut.a) + " to " + std::to_string(shortcut.b), LogLevel::DEBUG);

                if (!checkTaskDep()) {
                    log("Shortcut violates task dep", LogLevel::ERROR);
                    return false;
                }
            }
        }

        auto toc = std::chrono::high_resolution_clock::now();
        elapsed += std::chrono::duration_cast<std::chrono::nanoseconds>(toc - tic).count() * 1e-9;
        while (elapsed > log_limit) {
            log_limit += log_interval;
            if (options_.progress_file != "") {
                calculate_path_length_and_wait_time(instance_, synced_plan_, path_length_, wait_time_);
                logProgress(options_.progress_file, elapsed);
            }
        }
    }

    if (options_.path_shortcut) {
        // change the velocity of the trajectory
        MRTrajectory speedup_traj;
        speedup_traj.resize(synced_plan_.size());
        // keep the same number of points, just change the time if robot is not all moving at max speed
        int step = 0;
        int makespan_count = std::ceil(makespan_ / options_.dt) + 1;
        while (step < makespan_count) {
            double timeDilation = 1;
            if (step > 0) {
                for (int i = 0; i < instance_->getNumberOfRobots(); i++) {
                    int ind_prev = timed_index_[i][step-1];
                    int ind = timed_index_[i][step];
                    double dist = std::abs(instance_->computeDistance(synced_plan_[i].trajectory[ind], synced_plan_[i].trajectory[ind_prev]));
                    double speed = (dist / options_.dt);
                    timeDilation = std::max(timeDilation, speed / instance_->getVMax(i));
                }
            }

            // append the point to speedup_traj
            for (int i = 0; i < instance_->getNumberOfRobots(); i++) {
                int ind = timed_index_[i][step];
                speedup_traj[i].trajectory.push_back(synced_plan_[i].trajectory[ind]);
                if (step > 0) {
                    speedup_traj[i].times.push_back(speedup_traj[i].times.back() + options_.dt * timeDilation);
                }
                else {
                    speedup_traj[i].times.push_back(synced_plan_[i].times[ind]);
                }
                speedup_traj[i].act_ids.push_back(synced_plan_[i].act_ids[ind]);
            }
        
            step ++;
        }
        double speedup_makespan = 0;
        for (int i = 0; i < instance_->getNumberOfRobots(); i++) {
            speedup_traj[i].robot_id = i;
            speedup_traj[i].cost = speedup_traj[i].times.back();
            speedup_makespan = std::max(speedup_makespan, speedup_traj[i].times.back());
        }
        smoothed_solution = speedup_traj;
        log ("Shortcutting done, makespan before " + std::to_string(pre_makespan_) 
            +  " makespan " + std::to_string(speedup_makespan) + ", elapsed " + std::to_string(elapsed), LogLevel::INFO);
    }
    else {
        smoothed_solution = synced_plan_;
        log ("Shortcutting done, makespan before " + std::to_string(pre_makespan_) 
            +  " makespan " + std::to_string(makespan_) + ", elapsed " + std::to_string(elapsed), LogLevel::INFO);
    }
    return true;
}

void ShortcutterMT::preCheckShortcut(Shortcut &shortcut)
{
    int robot_id = shortcut.robot_id;
    int a = shortcut.a;
    int b = shortcut.b;
    RobotPose pose_a = synced_plan_[robot_id].trajectory[a];
    RobotPose pose_b = synced_plan_[robot_id].trajectory[b];

    double t_a = synced_plan_[robot_id].times[a];
    double t_b = synced_plan_[robot_id].times[b];

    double dist_ab = instance_->computeDistance(pose_a, pose_b);
    double dist_ab_old = 0;
    RobotPose pose_i = pose_a;
    RobotPose pose_j = pose_a;
    for (int i = a + 1; i <= b; i++) {
        dist_ab_old += instance_->computeDistance(synced_plan_[robot_id].trajectory[i-1], synced_plan_[robot_id].trajectory[i]);
    }

    double t_ab = dist_ab / instance_->getVMax(robot_id);
    int num_steps = std::max(1, int(std::ceil(t_ab / options_.dt)));
    if (options_.prioritized_shortcut) {
        if (num_steps < (b - a)) {
            // interpolate and get the path
            shortcut.col_type = CollisionType::NONE;
            shortcut.path.clear();
            shortcut.path.reserve(num_steps - 1);
            for (int i = 1; i < num_steps; i++) {
                double alpha = i * 1.0 / num_steps;
                RobotPose pose_i = instance_->interpolate(pose_a, pose_b, alpha);
                shortcut.path.push_back(pose_i);
            }
        }
        else {
            shortcut.col_type = CollisionType::NO_NEED;
            return;
        }
    }
    else if (options_.path_shortcut) {
        if (dist_ab < dist_ab_old) {
            shortcut.col_type = CollisionType::NONE;
            shortcut.path.clear();
            shortcut.path.reserve(b - a - 1);
            for (int i = a + 1; i < b; i++) {
                double alpha = (i - a) * 1.0 / (b - a);
                RobotPose pose_i = instance_->interpolate(pose_a, pose_b, alpha);
                shortcut.path.push_back(pose_i);
            }
            shortcut.new_t_afterb.clear();
            shortcut.new_t_afterb = std::vector<double>(synced_plan_[robot_id].times.begin() + b, synced_plan_[robot_id].times.end());
            return;
        }
        else {
            shortcut.col_type = CollisionType::NO_NEED;
            return;
        }
    }
    else {
        log("Unknown shortcut type", LogLevel::ERROR);
        return;
    }

    // find where does the collision check need to end
    int step = std::round(t_a / options_.dt) + num_steps;
    double t_diff = (t_b - t_a) - num_steps * options_.dt;
    int c = b;
    int cur_act_id = synced_plan_[robot_id].act_ids[c];
    shortcut.new_t_afterb.clear();
    shortcut.timed_index_afterb.clear();

    shortcut.new_t_afterb.push_back(step * options_.dt);
    
    int makespan_count = std::ceil(makespan_ / options_.dt) + 1;

    while (step < makespan_count) {

        shortcut.timed_index_afterb[step] = c;         
        step++;

        if (c == synced_plan_[robot_id].trajectory.size() - 1) {
            continue;
        }

        double t_next = synced_plan_[robot_id].times[c + 1];
        if ((step * options_.dt + t_diff) < t_next) {
            continue;
        }

        int act_id_next = synced_plan_[robot_id].act_ids[c + 1];
        bool has_task_dep = false;
        int dep_act_id = -1, dep_robot_id, dep_ind, dep_act_id_now;
        double t_dep;

        if (act_id_next != cur_act_id) {
            // end of task, check the incoming type-2 acts
            auto act = act_graph_->get(robot_id, act_id_next);
            if (act == nullptr) {
                log("Activity not found", LogLevel::ERROR);
            }
            for (auto dep_act: act->type2_prev) {
                dep_act_id = dep_act->act_id;
                dep_robot_id = dep_act->robot_id;
                dep_ind = timed_index_[dep_robot_id][step-1];
                t_dep = synced_plan_[dep_robot_id].times[dep_ind];
                dep_act_id_now = synced_plan_[dep_robot_id].act_ids[dep_ind];
                if (dep_act_id_now <= dep_act_id) {
                    has_task_dep = true;
                    break;
                }
            }
        }
        if (has_task_dep) {
            t_diff -= options_.dt;
            log("diff " + std::to_string(t_diff) + " step " + std::to_string(step) + " t_dep " + std::to_string(t_dep) 
                + " dep_act_id_now " + std::to_string(dep_act_id_now) + " dep_act_id " + std::to_string(dep_act_id), LogLevel::DEBUG);
            if (options_.tight_shortcut && t_diff <= 0) {
                shortcut.col_type = CollisionType::UNTIGHT;
                log("Shortcut is not tight", LogLevel::DEBUG);
                return;
            }
            else if (t_diff < 0) {
                log("Shortcut is not supposed to increase the makespan", LogLevel::ERROR);
                shortcut.col_type = CollisionType::UNTIGHT;
                return;
            }
        }
        else {
            if (dep_act_id != -1) {
                log("cleared task dep", LogLevel::DEBUG);
            }
            c++;
            shortcut.new_t_afterb.push_back(step * options_.dt);
            cur_act_id = act_id_next;
        }
    }

    if (shortcut.new_t_afterb.size() != (synced_plan_[robot_id].trajectory.size() - b)) {
        log("Shortcut new_t_afterb size " + std::to_string(shortcut.new_t_afterb.size()) + " trajectory size " + std::to_string(synced_plan_[robot_id].trajectory.size()), LogLevel::ERROR);
    }

    return;
}

bool ShortcutterMT::checkTaskDep()
{
    int num_robot = instance_->getNumberOfRobots();

    
    for (int i = 0; i < instance_->getNumberOfRobots(); i++) {
        int num_act = act_graph_->num_activities(i);
        int act_start_ind = 0;
        for (int act_id = 0; act_id < num_act; act_id++) {
            // check if this task has incoming type-2 task dependency
            auto act = act_graph_->get(i, act_id);
            double t_act_start = synced_plan_[i].times[act_start_ind];

            for (auto dep_act: act->type2_prev) {
                int dep_robot_id = dep_act->robot_id;
                int dep_act_id = dep_act->act_id;
                if (dep_robot_id == i) {
                    log("Self dependency is not allowed", LogLevel::ERROR);
                }
                int dep_act_end_ind = 0;
                for (int dep_act_j = 0; dep_act_j <= dep_act_id; dep_act_j++) {
                    dep_act_end_ind += num_task_points_[dep_robot_id][dep_act_j];
                }
                double t_dep_finish = synced_plan_[dep_robot_id].times[dep_act_end_ind-1];
                if ((t_dep_finish - t_act_start) > 0.001) {
                    log("Robot " + std::to_string(i) + " Task " + std::to_string(act_id) + " time " + std::to_string(t_act_start) + " ind " + std::to_string(act_start_ind) +
                        " has incoming dependency from Robot " + std::to_string(dep_robot_id) + " Task " + std::to_string(dep_act_id) + " time " + std::to_string(t_dep_finish) + " ind " + std::to_string(dep_act_end_ind - 1),
                         LogLevel::ERROR);
                    return false;
                }
            }
            act_start_ind += num_task_points_[i][act_id];
        }
         
    }
    return true;
}

void ShortcutterMT::checkShortcut(Shortcut &shortcut)
{
    int robot_id = shortcut.robot_id;
    int a = shortcut.a;
    int b = shortcut.b;
    int shortcut_length = shortcut.path.size() + 1;
    RobotPose pose_a = synced_plan_[robot_id].trajectory[a];
    RobotPose pose_b = synced_plan_[robot_id].trajectory[b];

    double t_a = synced_plan_[robot_id].times[a];
    double t_b = synced_plan_[robot_id].times[b];

    // build the collision scene
    std::shared_ptr<Activity> cur_act = shortcut.act;

    instance_->resetScene(false);
    
    // add all static objects that needs to be collision checked
    std::vector<ObjPtr> indep_objs = act_graph_->find_indep_obj(cur_act);
    for (auto obj : indep_objs) {
        instance_->addMoveableObject(obj->obj);
        //instance->updateScene();
    }

    for (int act_id = 0; act_id <= cur_act->act_id; act_id++) {
        updateScene(robot_id, act_id);
    }

    for (int i = 0; i < shortcut.path.size(); i++) {
        RobotPose pose_i = shortcut.path[i];

        // check environment collision
        if (instance_->checkCollision({pose_i}, false) == true) {
            if (options_.print_contact) {
                instance_->checkCollision({pose_i}, false, true);
            }
            shortcut.col_type = CollisionType::STATIC; // collide with the evnrionment
            return;
        }
    }

    // check robot-robot collision
    // from timestep a+1 till the end of task with incoming type-2 edge
    // for all other robot
    // update their collision geometry
    // check collision
    
    
    int makespan_count = std::ceil(makespan_ / options_.dt) + 1;
    int step_a = std::round(t_a / options_.dt);
    int step = step_a + 1;

    // update the scene of other robots
    std::vector<int> last_act(instance_->getNumberOfRobots(), 0);
    for (int j = 0; j < instance_->getNumberOfRobots(); j++) {
        if (j != robot_id) {
            int ind = timed_index_[j][step_a];
            int act_id_j = synced_plan_[j].act_ids[ind];
            for (int act_id = 0; act_id <= act_id_j; act_id++) {
                updateScene(j, act_id);
            }
            last_act[j] = act_id_j;
        }
        else {
            last_act[j] = cur_act->act_id;
        }
    }

    // iterate over discretized time stepm, until the end or b if path shortcut
    while (step < makespan_count) {
        // check collision
        RobotPose pose_i;
        if (step < step_a + shortcut_length) {
            pose_i = shortcut.path[step - step_a - 1];
        }
        else if (options_.prioritized_shortcut) {
            int ind = shortcut.timed_index_afterb[step];
            int cur_act_id = synced_plan_[robot_id].act_ids[ind];
            if (last_act[robot_id] != cur_act_id) {
                // check if this is the start of a new task which has a type-2 task dependency, stop if that's the case
                auto act = act_graph_->get(robot_id, cur_act_id);
                updateScene(robot_id, cur_act_id);
                last_act[robot_id] = cur_act_id;
            }
            // set the pose to the corresponding index to current time stamp
            pose_i = synced_plan_[robot_id].trajectory[ind];
        }
        else if (options_.path_shortcut) {
            // no need to check collision for the rest of the path
            break;
        }
        else {
            log("Unknown shortcut type", LogLevel::ERROR);
        }

        for (int j = 0; j < instance_->getNumberOfRobots(); j++) {
            if (j != robot_id) {
                RobotPose pose_j;
                int ind_j = timed_index_[j][step];
                if (ind_j >= synced_plan_[j].trajectory.size()) {
                    log("Timed Index out of bound", LogLevel::ERROR);
                }
                int cur_act_id_j = synced_plan_[j].act_ids[ind_j];
                if (last_act[j] != cur_act_id_j) {
                    updateScene(j, cur_act_id_j);
                    last_act[j] = cur_act_id_j;
                }
                pose_j = synced_plan_[j].trajectory[ind_j];
                if (instance_->checkCollision({pose_i, pose_j}, true)) {
                    if (options_.print_contact) {
                        instance_->checkCollision({pose_i, pose_j}, true, true);
                    }
                    shortcut.col_type = CollisionType::ROBOT; // collide with other robots
                    return;
                }
            }

        }
        step++;
    }
     

    shortcut.col_type = CollisionType::NONE;
    return;
}

void ShortcutterMT::updateScene(int robot_id, int act_id) {
    // updated attached / detached object
    std::shared_ptr<const Activity> act_j = act_graph_->get(robot_id, act_id);
    for (auto obj : act_j->obj_attached) {
        instance_->moveRobot(robot_id, act_j->start_pose);
        if (obj->vanish) {
            instance_->addMoveableObject(obj->obj);
            //instance->updateScene();
        }
        else {
            instance_->moveObject(obj->obj);
            //instance->updateScene();
        }
        instance_->attachObjectToRobot(obj->obj.name, robot_id, obj->next_attach_link, act_j->start_pose);
        //instance->updateScene();
    }
    for (auto obj : act_j->obj_detached) {
        instance_->detachObjectFromRobot(obj->obj.name, act_j->start_pose);
        //instance->updateScene();
        if (obj->vanish) {
            instance_->removeObject(obj->obj.name);
            //instance->updateScene();
        }
    }
    for (auto col_node : act_j->collision_nodes) {
        instance_->setCollision(col_node.obj_name, col_node.link_name, col_node.allow);
    }
}

double ShortcutterMT::calculate_makespan() {
    double makespan = 0;
    for (int i = 0; i < instance_->getNumberOfRobots(); i++) {
        makespan = std::max(makespan, synced_plan_[i].times.back());
    }
    return makespan;
}

void ShortcutterMT::prempt_home_act() {
    for (int i = 0; i < instance_->getNumberOfRobots(); i++) {

        std::map<int, int> update_act_id;
        
        int non_home_act_id = -1;
        for (int act_id = act_graph_->num_activities(i) - 1; act_id >= 0; act_id--) {
            auto act = act_graph_->get(i, act_id);
            if (act->is_skippable() && non_home_act_id != -1) {
                update_act_id[act_id] = non_home_act_id;
            }
            else {
                non_home_act_id = act_id;
            }
        }
        
        for (int j = 0; j < synced_plan_[i].act_ids.size(); j++) {
            int act_id = synced_plan_[i].act_ids[j];
            if (update_act_id.find(act_id) != update_act_id.end()) {
                synced_plan_[i].act_ids[j] = update_act_id[act_id];
            }
        }
    }
}
void ShortcutterMT::update_timed_index() {
    timed_index_.clear();
    timed_index_.resize(instance_->getNumberOfRobots());
   
    makespan_ = calculate_makespan();
    int makespan_count = std::ceil(makespan_ / options_.dt) + 1;

    for (int i = 0; i < instance_->getNumberOfRobots(); i++) {
        int step = 0; 
        int j = 0;
        while (step < makespan_count) {
            while ((j+1) < synced_plan_[i].times.size() && int(synced_plan_[i].times[j+1]/options_.dt) <= step) {
                j++;
            } 
            timed_index_[i].push_back(j);
            step++;
        }
    }
}


void ShortcutterMT::updatePlan(const Shortcut &shortcut)
{
    int robot_id = shortcut.robot_id;
    int a = shortcut.a;
    int b = shortcut.b;
    int shortcut_length = shortcut.path.size() + 1;

    // update the plan
    // delete the points between a and b (not inclusive)
    synced_plan_[robot_id].trajectory.erase(synced_plan_[robot_id].trajectory.begin() + a + 1, synced_plan_[robot_id].trajectory.begin() + b);
    synced_plan_[robot_id].times.erase(synced_plan_[robot_id].times.begin() + a + 1, synced_plan_[robot_id].times.end());
    synced_plan_[robot_id].act_ids.erase(synced_plan_[robot_id].act_ids.begin() + a + 1, synced_plan_[robot_id].act_ids.begin() + b);

    // insert the new points
    
    synced_plan_[robot_id].trajectory.insert(synced_plan_[robot_id].trajectory.begin() + a + 1, shortcut.path.begin(), shortcut.path.end());
    for (int c = a + 1; c < a + shortcut_length; c++) {
        synced_plan_[robot_id].times.push_back(synced_plan_[robot_id].times[c-1] + options_.dt);
    }
    synced_plan_[robot_id].times.insert(synced_plan_[robot_id].times.begin() + a + shortcut_length, shortcut.new_t_afterb.begin(), shortcut.new_t_afterb.end());
    

    // update action id
    synced_plan_[robot_id].act_ids.insert(synced_plan_[robot_id].act_ids.begin() + a + 1, shortcut_length - 1, shortcut.act->act_id);
    synced_plan_[robot_id].cost = synced_plan_[robot_id].times.back();
    return;
}
 

bool ShortcutterMT::logProgress(const std::string &filename, double elapsed)
{
    std::ofstream file(filename, std::ios::app);
    file << "," << "," 
        << "," << pre_makespan_ << "," << "," << makespan_ 
        << "," << ",,," << elapsed << "," << "," << t_check_ << "," << n_check_
        << "," << n_valid_ << "," << "," << ","
        << ", " << n_colcheck_ << "," << path_length_ << "," << wait_time_ << std::endl;
    file.close();
    return true;
}