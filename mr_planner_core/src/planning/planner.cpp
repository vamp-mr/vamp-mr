#include "mr_planner/planning/planner.h"
#include "mr_planner/planning/sipp_rrt.h"
#include "mr_planner/planning/strrt.h"
#include "mr_planner/core/instance.h"
#include "mr_planner/core/logger.h"
#include <boost/algorithm/string.hpp>
#include <algorithm>
#include <limits>
#include <random>
#if MR_PLANNER_WITH_ROS
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/kinematic_constraints/utils.h>
#endif

/*---------------------------------Priority Planner---------------------------------------*/

PriorityPlanner::PriorityPlanner(std::shared_ptr<PlanInstance> instance) : AbstractPlanner(instance) {
}

bool PriorityPlanner::plan(const PlannerOptions &options) {
    // Implement the planning algorithm using problem_instance_

    // set a randomized order
    std::vector<int> order;
    for (int i = 0; i < num_robots_; i++) {
        order.push_back(i);
    }
    if (options.pp_random_order) {
        std::random_device rd;
        std::mt19937 g(rd());
        std::shuffle(order.begin(), order.end(), g);
    }

    // iterate through the robots in the randomized order
    MRTrajectory solution;
    planning_time_ = 0;
    for (int i = 0; i < num_robots_; i++) {
        int robot_id = order[i];
        
        std::shared_ptr<SingleAgentPlanner> planner;
        if (options.single_agent_planner == "STRRT") {
            planner = std::make_shared<STRRT>(instance_, robot_id);
        } else if (options.single_agent_planner == "SIPP_RRT") {
            planner = std::make_shared<SIPP_RRT>(instance_, robot_id);
        }
        else {
            log("Unknown single agent planner: " + options.single_agent_planner, LogLevel::ERROR);
            return false;
        }
        PlannerOptions option_i = options;
        option_i.max_planning_time = options.max_planning_time / num_robots_;
        option_i.obstacles = solution;
        
        auto tic = std::chrono::high_resolution_clock::now();
        bool solved = planner->plan(option_i);
        auto toc = std::chrono::high_resolution_clock::now();
        planning_time_ += std::chrono::duration_cast<std::chrono::nanoseconds>(toc - tic).count() * 1e-9;
        log("Planning time for robot " + std::to_string(robot_id) + ": " + std::to_string(planning_time_), LogLevel::DEBUG);

        if (options.log_fname != "") {
            logProgressFileAppend(options.log_fname, "r" + std::to_string(robot_id),
                                    "r" + std::to_string(robot_id), planning_time_, 
                                    planner->getPlanCost(),  planner->getPlanCost());
        }

        if (solved) {
            RobotTrajectory traj;
            planner->getPlan(traj);
            solution.push_back(traj);
            log("Found plan for robot " + std::to_string(robot_id), LogLevel::HLINFO);
        } else {
            log("Failed to plan for robot " + std::to_string(robot_id) + "!", LogLevel::ERROR);
            return false; // Return false if planning fails
        }
    }

    solution_ = solution;
    solved = true;
    return true; // Return true if planning succeeds
}

bool PriorityPlanner::getPlan(MRTrajectory &solution) const {
    // Retrieve the plan.
    if (!solved) {
        return false;
    }
    solution = solution_;
    return true;
}

/*-------------------------------End of Priority Planner------------------------------------*/

CBSPlanner::CBSPlanner(std::shared_ptr<PlanInstance> instance) : AbstractPlanner(instance) {
}

CBSPlanner::CBSPlanner(std::shared_ptr<PlanInstance> instance, std::vector<std::shared_ptr<RoadMap>> roadmaps) : AbstractPlanner(instance), roadmaps_(roadmaps) {
    num_robots_ = roadmaps.size();
    //std::cout << "Size of the 1st input roadmap: " << roadmaps[0]->getRoadmap()->size << std::endl;
    for (int i = 0; i < num_robots_; i++) {
        auto planner = std::make_shared<PRM>(instance_, i, roadmaps_[i]);
        agent_planners_.push_back(planner);
    }
}

CBSPlanner::CBSPlanner(std::shared_ptr<PlanInstance> instance, std::vector<std::shared_ptr<RoadMap>> roadmaps, std::shared_ptr<VoxelGrid> voxel_grid) 
    : AbstractPlanner(instance), roadmaps_(roadmaps), voxel_grid_(voxel_grid) {
    num_robots_ = roadmaps.size();
    for (int i = 0; i < num_robots_; i++) {
        auto planner = std::make_shared<PRM>(instance_, i, roadmaps_[i], voxel_grid_);
        agent_planners_.push_back(planner);
    }
}

void CBSPlanner::seedNodeSaltRng(const PlannerOptions &options) {
    std::uint64_t seed_value = 0;
    if (options.rrt_seed >= 0) {
        seed_value = (static_cast<std::uint64_t>(options.rrt_seed) << 32) ^ 0xC0B5C0DEULL;
    } else {
        std::random_device rd;
        seed_value = (static_cast<std::uint64_t>(rd()) << 32) ^ static_cast<std::uint64_t>(rd());
    }
    node_salt_rng_.seed(seed_value);
    node_salt_rng_seeded_ = true;
}

std::uint64_t CBSPlanner::nextNodeSalt() {
    if (!node_salt_rng_seeded_) {
        std::random_device rd;
        std::uint64_t fallback_seed = (static_cast<std::uint64_t>(rd()) << 32) ^ static_cast<std::uint64_t>(rd());
        node_salt_rng_.seed(fallback_seed);
        node_salt_rng_seeded_ = true;
    }
    return node_salt_rng_();
}

void CBSPlanner::swapStartGoal() {
    // swap start and goal
    for (int i = 0; i < num_robots_; i++) {
        auto planner = agent_planners_[i];
        planner->swapStartGoal();
    }
    // reset the solution
    solution_.clear();
    solved = false;
    col_count = 0;
    log("Swapped start and goal", LogLevel::DEBUG);
}

void CBSPlanner::revertSolution() {
    // First, align the solution of different robots, making them the same length. Use the last position for matching.
    auto original_solution = solution_;
    int max_length = 0;
    std::vector<double> longest_time_sequence;
    for (int i = 0; i < num_robots_; i++) {
        if (solution_[i].trajectory.size() > max_length) {
            max_length = solution_[i].trajectory.size();
            longest_time_sequence = solution_[i].times;
        }
    }
    for (int i = 0; i < num_robots_; i++) {
        if (solution_[i].trajectory.size() < max_length) {
            for(int j = solution_[i].trajectory.size(); j < max_length; j++) {
                // add the last position to the end of the trajectory
                solution_[i].trajectory.push_back(solution_[i].trajectory.back());
                // Align the time sequence to the longest one
                solution_[i].times.push_back(longest_time_sequence[j]);
            }
        }
    }
    // revert solution_ to the original start and goal
    for(int i = 0; i < num_robots_; i++) {
        for (int j = 0; j < solution_[i].times.size(); j++) {
            solution_[i].times[j] = solution_[i].times.back() - solution_[i].times[j];
        }
        std::reverse(solution_[i].trajectory.begin(), solution_[i].trajectory.end());
        std::reverse(solution_[i].times.begin(), solution_[i].times.end());
        solution_[i].cost = solution_[i].times.back();
    }
}

bool CBSPlanner::plan(const struct PlannerOptions &options) {
    time_limit_ = options.max_planning_time;
    planning_time_ = 0;
    auto tic = std::chrono::system_clock::now();
    seedNodeSaltRng(options);
    //For each robot, initialize a planner
    std::vector<int> order;
    for (int i = 0; i < num_robots_; i++) {
        order.push_back(i);
    }

    // iterate through the robots in the randomized order
    MRTrajectory solution;

    //Conflict Based Search implementation
    //initialize the search
    std::priority_queue<CBSNode*, std::vector<CBSNode*>, CompareNode> open;
    CBSNode* root = new CBSNode();
    root->salt = nextNodeSalt();
    root->robots = order;

    //Find a solution for root regardless of constraints
    auto root_start = std::chrono::high_resolution_clock::now();
    for(int i = 0; i < num_robots_; i++) {
        auto planner = agent_planners_[i];
        PlannerOptions option_i = options;
        option_i.isRoot = true;

        double robot_lower_bound = 0;
        bool solved = planner->plan(option_i, solution, robot_lower_bound);

        if (solved) {
            RobotTrajectory traj;
            planner->getPlan(traj);
            solution.push_back(traj);
            root->lower_bound = std::max(root->lower_bound, robot_lower_bound);
            //std::cout << "Solution size: " << solution.size() << '\n';
            //log("Found plan for robot " + std::to_string(i), LogLevel::HLINFO);

        } else {
            log("CBSPlanner: Failed to plan for robot " + std::to_string(i) + " at root!", LogLevel::ERROR);
            return false; // Return false if planning fails
        }
    }
    root->solution = solution;
    root->speedup_solution = accelerateSolution(solution);
    root->cost = 0;
    root->makespan = 0;
    int n_pairs, n_conflicts;
    std::tie(n_pairs, n_conflicts) = countNumConflicts(root, options);
    root->num_pairs = n_pairs;
    root->num_conflicts = n_conflicts;
    for(int i = 0; i < num_robots_; i++) {
        root->cost += root->speedup_solution[i].cost;
        if(root->speedup_solution[i].times.back() > root->makespan) {
            root->makespan = root->speedup_solution[i].times.back();
        }
    }
    auto root_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> root_time = (root_end - root_start);
    // std::cout << "Root node time: " << root_time.count() << '\n';

    open.push(root);
    int num_nodes_expanded = 0;
    std::vector<int> conflict_stats(num_robots_, 0);
    int resampling_threshold = 10;
    int resampling_interval = 10;
    int densemap_threshold = num_robots_;
    std::vector<int> replan_failures(num_robots_, 0);
    int total_failures = 0;
    while(!open.empty()) {
        num_nodes_expanded++;
        
        CBSNode* current = open.top();
        open.pop();
        
        // std::cout << "CBS High-level Open size: " << open.size() << '\n';

        // //Print all constraints in the current node, in order of robot id
        // std::cout << "Current node constraints: \n";
        // for (int i = 0; i < num_robots_; i++) {
        //    std::cout << "Robot " << i << ": ";
        //    for(int j = 0; j < current->constraints.size(); j++) {
        //        if(current->constraints[j].robot_id == i) {
        //            // Use posehash and poseedgehash to print the constraints
        //            if(current->constraints[j].type == ConstraintType::VERTEX) {
        //                RobotPoseHash pose_hash;
        //                std::cout << "Vertex, " << pose_hash(current->constraints[j].pose);
        //           } else if(current->constraints[j].type == ConstraintType::EDGE) {
        //                PoseEdgeHash pose_edge_hash;
        //                std::cout << "Edge, " << pose_edge_hash(PoseEdge(current->constraints[j].pose, current->constraints[j].to_pose));
        //            } else {
        //                std::cout << "Unknown constraint type ";
        //            }
        //            std::cout << ", time, " << current->constraints[j].time;
        //            std::cout << "| ";
        //        }
        //    }
        //    std::cout << std::endl;
        // }
        
        //Find all conflicts in the current node
        Conflict conflict_;
        PlannerOptions curr_option = options;
        if(findVertexConflict(current, conflict_) > 0) {
            //std::cout << "Vertex conflict found\n";
            current->conflicts.push_back(conflict_);
        } else if (findEdgeConflict(current, conflict_, options) > 0) {
            //std::cout << "Edge conflict found\n";
            current->conflicts.push_back(conflict_);
        }
        if(current->conflicts.empty()) { // return assuming no conflict, to test if discrete collision check will cause inaccuracy.
            log("CBS found a solution with # of nodes expanded: " + std::to_string(num_nodes_expanded), LogLevel::INFO);
            // delete all nodes in the open list
            while(!open.empty()) {
                delete open.top();
                open.pop();
            }
            solution = current->solution;
            // convert solution time
            solution = accelerateSolution(solution);
            // compute makespan
            double makespan = 0;
            for(int i = 0; i < num_robots_; i++) {
                if(solution[i].times.back() > makespan) {
                    makespan = solution[i].times.back();
                }
            }
            // compute cost
            for(int i = 0; i < num_robots_; i++) {
                solution[i].cost = solution[i].times.back();
                solution[i].num_nodes_expanded = num_nodes_expanded;
                solution[i].num_col_checks = current->num_col_checks / num_nodes_expanded;
            }

            solution_ = solution;
            solved = true;
            delete current;

            return true;
        }
        
        conflict_stats[current->conflicts[0].robots[0]]++;
        conflict_stats[current->conflicts[0].robots[1]]++;
        
        /* re-sample
        if(num_nodes_expanded % resampling_interval == 0) {
            for(int i = 0; i < num_robots_; i++) {
                if(conflict_stats[i] > resampling_threshold) {
                    PlannerOptions option_i = options;
                    option_i.constraints = current->constraints;
                    agent_planners_[i]->reSampling(option_i);
                }
            }
        }
        */
        auto toc = std::chrono::system_clock::now();
        planning_time_ = std::chrono::duration_cast<std::chrono::nanoseconds>(toc - tic).count() * 1e-9;

        if(planning_time_ > time_limit_) {
            log("CBS planning time exceeded the limit of " + std::to_string(time_limit_) + " seconds with # of nodes expanded: " 
                + std::to_string(num_nodes_expanded), LogLevel::WARN);
            // prepare for return
            delete current;
            while(!open.empty()) {
                delete open.top();
                open.pop();
            }
            return false;
        }

        //find the first conflict
        Conflict conflict = current->conflicts[0];
        //For each robot in the conflict, create a new node with the constraint that the robot cannot appear at the conflict in terms of both time and vertex;
        CBSNode *left_child, *right_child;
        std::tie(left_child, right_child) = generateChildNodes(current, conflict, options);
        if(left_child) {
            open.push(left_child);
        } /*else {
            total_failures++;
            if(total_failures == densemap_threshold) {
                // Use densemap
                std::cout << "Applying dense roadmap for all agents" << std::endl;
                for(int i = 0; i < num_robots_; i++) {
                    agent_planners_[i]->applyDenseMap();
                }
            }
        }*/
        if(right_child) {
            open.push(right_child);
        } /*else {
            total_failures++;
            if(total_failures == densemap_threshold) {
                // Use densemap
                std::cout << "Applying dense roadmap for all agents" << std::endl;
                for(int i = 0; i < num_robots_; i++) {
                    agent_planners_[i]->applyDenseMap();
                }
            }
        }*/
        delete current;
    }
    return false;
}

std::pair<CBSNode*, CBSNode*> CBSPlanner::generateChildNodes(CBSNode *current, const Conflict &conflict, const PlannerOptions &options) {
    CBSNode *left_child = nullptr;
    CBSNode *right_child = nullptr;

    // std::thread left_thread([&]() {
    //     left_child = generateChildNode(current, conflict, options, 0);
    // });

    // std::thread right_thread([&]() {
    //     right_child = generateChildNode(current, conflict, options, 1);
    // });

    // left_thread.join();
    // right_thread.join();

    left_child = generateChildNode(current, conflict, options, 0);
    right_child = generateChildNode(current, conflict, options, 1);

    return std::make_pair(left_child, right_child);
}

CBSNode *CBSPlanner::generateChildNode(CBSNode *current, const Conflict &conflict, const PlannerOptions &options, int i) {
    CBSNode* newNode = new CBSNode();
    newNode->salt = nextNodeSalt();
    newNode->robots = current->robots;
    newNode->constraints = current->constraints;
    newNode->solution.resize(num_robots_);
    newNode->speedup_solution.resize(num_robots_);
    for(int j = 0; j < num_robots_; j++) {
        if(j != conflict.robots[i]) {
            newNode->solution[j] = current->solution[j];
            newNode->speedup_solution[j] = current->speedup_solution[j];
        }
    }

    //Add the constraints to the new node
    addConstraint(current, newNode, conflict, i);

    newNode->lower_bound = current->lower_bound;
    //Update the low-level path for robot i, and update the cost of the node
    auto planner = agent_planners_[conflict.robots[i]];
    PlannerOptions option_i = options;
    option_i.constraints = newNode->constraints;

    double robot_lower_bound = 0;
    bool solved = planner->plan(option_i, current->speedup_solution, robot_lower_bound);

    if (solved) {
        RobotTrajectory traj;
        planner->getPlan(traj);
        newNode->solution[conflict.robots[i]] = traj;
        newNode->speedup_solution = accelerateSolution(newNode->solution);
        newNode->cost = 0;
        newNode->makespan = 0;
        int n_pairs, n_conflicts;
        std::tie(n_pairs, n_conflicts) = countNumConflicts(newNode, options);
        newNode->num_pairs = n_pairs;
        newNode->num_conflicts = n_conflicts;
        newNode->num_col_checks = current->num_col_checks + traj.num_col_checks;
        newNode->lower_bound = std::max(robot_lower_bound, newNode->lower_bound);
        for(int j = 0; j < num_robots_; j++) {
            newNode->cost += newNode->speedup_solution[j].cost;
            if(newNode->speedup_solution[j].times.back() > newNode->makespan) {
                newNode->makespan = newNode->speedup_solution[j].times.back();
            }
        }
    } else {
        delete newNode;
        newNode = nullptr;
    }
    return newNode;
}

void CBSPlanner::addConstraint(CBSNode *current, CBSNode *newNode, const Conflict &conflict, int i) {
    Constraint newConstraint;
    // robot and conflict time
    int r0 = conflict.robots[0];
    int r1 = conflict.robots[1];
    int t_idx0 = conflict.time_idx[0];
    int t_idx1 = conflict.time_idx[1];
    double t0, t1;
    RobotPose pose0, pose1;
    if (t_idx0 < current->solution[r0].times.size()) {
        t0 = current->solution[r0].times[t_idx0];
        pose0 = current->solution[r0].trajectory[t_idx0];
    }
    else {
        t0 = current->solution[r0].times.back();
        pose0 = current->solution[r0].trajectory.back();
    }
    if (t_idx1 < current->solution[r1].times.size()) {
        t1 = current->solution[r1].times[t_idx1];
        pose1 =  current->solution[r1].trajectory[t_idx1];
    }
    else {
        t1 = current->solution[r1].times.back();
        pose1 = current->solution[r1].trajectory.back();
    }

    RobotPose goalpose0 = current->solution[r0].trajectory.back();
    RobotPose goalpose1 = current->solution[r1].trajectory.back();

    if(conflict.isTarget) {
        // r0 is at goal
        if (i == 0) {
            // log("adding target (>) constraint to goal", LogLevel::DEBUG);
            // // this agent must arrive after t
            // newConstraint.robot_id = r0;
            // newConstraint.type = ConstraintType::GLENGTH;
            // newConstraint.time = t1;
            // newConstraint.pose = goalpose0;
            newConstraint.type = ConstraintType::VERTEX;
            newConstraint.isAvoidance = true;
            newConstraint.robot_id = r0;
            newConstraint.time = t1;
            newConstraint.pose = pose1;
        }
        else {
            // log ("adding target (<) constraint and avoidance", LogLevel::DEBUG);
            // // this agent must arrive before or at t, other agent need to avoid after t
            // newConstraint.robot_id = r0;
            // newConstraint.type = ConstraintType::LEQLENGTH;
            // newConstraint.isAvoidance = true;
            // newConstraint.time = t1;
            // newConstraint.pose = goalpose0;
            // newConstraint.to_pose = goalpose0;
            newConstraint.type = conflict.type;
            newConstraint.robot_id = r1;
            newConstraint.time = t1;
            newConstraint.pose = pose1;
            if(newConstraint.type == ConstraintType::EDGE) {
                newConstraint.to_pose = current->solution[r1].trajectory[t_idx1 + 1];
            }
        }
    } 
    else {
        newConstraint.type = conflict.type;
        if(i == 0) { // avoidance constraint
            newConstraint.isAvoidance = true;
            newConstraint.robot_id = r0;
            newConstraint.time = t0;
            newConstraint.pose = pose1;
            if (newConstraint.type == ConstraintType::EDGE) {
                newConstraint.to_pose = current->solution[r1].trajectory[t_idx1 + 1];
            }

            // if(conflict.time_idx[i] < current->solution[conflict.robots[i]].times.size()) {
            //     newConstraint.time = current->solution[conflict.robots[i]].times[conflict.time_idx[i]];
            // } else {
            //     newConstraint.time = current->solution[conflict.robots[1-i]].times[conflict.time_idx[i]];
            // }
            // if(conflict.time_idx[i] < current->solution[conflict.robots[1-i]].trajectory.size()) {
            //     newConstraint.pose = current->solution[conflict.robots[1-i]].trajectory[conflict.time_idx[i]];
            // } else {
            //     newConstraint.pose = current->solution[conflict.robots[1-i]].trajectory.back();
            // }
            // if(newConstraint.type == ConstraintType::EDGE) {
            //     if(conflict.time_idx[i] + 1 < current->solution[conflict.robots[1-i]].trajectory.size()) {
            //         newConstraint.to_pose = current->solution[conflict.robots[1-i]].trajectory[conflict.time_idx[i] + 1];
            //     } else {
            //         newConstraint.to_pose = current->solution[conflict.robots[1-i]].trajectory.back();
            //     }
            // }
            // // In case no conflict is recorded in the table, it need to at least know the direct vertex/edge constraint
            // if(newConstraint.type == ConstraintType::VERTEX) {
            //     if(conflict.time_idx[i] < current->solution[conflict.robots[i]].times.size()) {
            //         newConstraint.self_vertex = current->solution[conflict.robots[i]].trajectory[conflict.time_idx[i]];
            //     } else {
            //         newConstraint.self_vertex = current->solution[conflict.robots[i]].trajectory.back();
            //     }
            // } else {
            //     if(conflict.time_idx[i] < current->solution[conflict.robots[i]].times.size()) {
            //         newConstraint.self_edge_start = current->solution[conflict.robots[i]].trajectory[conflict.time_idx[i]];
            //         newConstraint.self_edge_end = current->solution[conflict.robots[i]].trajectory[conflict.time_idx[i] + 1];
            //     } else {
            //         newConstraint.self_edge_start = current->solution[conflict.robots[i]].trajectory.back();
            //         newConstraint.self_edge_end = current->solution[conflict.robots[i]].trajectory.back();
            //     }
            // }
        } else { // vertex or edge constraint
            newConstraint.robot_id = r1;
            newConstraint.time = t1; // t0 should be the same as t1
            newConstraint.pose = pose1;
            // if(conflict.time_idx[i] < current->solution[conflict.robots[i]].times.size()) {
            //     newConstraint.time = current->solution[conflict.robots[i]].times[conflict.time_idx[i]];
            //     newConstraint.pose = current->solution[conflict.robots[i]].trajectory[conflict.time_idx[i]];
            // } else {
            //     newConstraint.time = current->solution[conflict.robots[1-i]].times[conflict.time_idx[i]];
            //     newConstraint.pose = current->solution[conflict.robots[i]].trajectory.back();
            // }
            if(newConstraint.type == ConstraintType::EDGE) {
                newConstraint.to_pose = current->solution[r1].trajectory[t_idx1 + 1];
            }
        }
    }
    newNode->constraints.push_back(newConstraint);
}

bool CBSPlanner::getPlan(MRTrajectory &solution) const {
    if(!solved) {
        return false;
    }
    solution = solution_;
    return true;
}

double CBSPlanner::getPlanTime() const {
    return planning_time_;
}

void CBSPlanner::stop() {
    // stop the planner
    PlannerOptions options;
    for(int i = 0; i < num_robots_; i++) {
        agent_planners_[i]->terminate(options);
    }
}

MRTrajectory CBSPlanner::accelerateSolution(const MRTrajectory &solution) {
    int end_tsp = 0;
    for(int i = 0; i < num_robots_; i++) {
        if(solution[i].times.size() > end_tsp) {
            end_tsp = solution[i].times.size();
        }
    }

    std::vector<double> max_interval_makespan(end_tsp, 0);
    for(int i = 0; i < num_robots_; i++) {
        for(int j = 1; j < solution[i].times.size(); j++) {
            double exec_time = instance_->computeDistance(solution[i].trajectory[j-1], solution[i].trajectory[j]) / instance_->getVMax(i);
            if(exec_time > max_interval_makespan[j]) {
                max_interval_makespan[j] = exec_time;
            }
        }
    }

    // Accelerate the solution
    MRTrajectory speedup_traj(num_robots_);
    for(int i = 0; i < num_robots_; i++) {
        speedup_traj[i] = solution[i];
        speedup_traj[i].times.clear();
        speedup_traj[i].times.push_back(0);
        for(int j = 1; j < solution[i].times.size(); j++) {
            speedup_traj[i].times.push_back(speedup_traj[i].times.back() + max_interval_makespan[j]);
        }
    }

    // compute the cost
    for(int i = 0; i < num_robots_; i++) {
        speedup_traj[i].cost = speedup_traj[i].times.back();
    }
    return speedup_traj;
}

bool CBSPlanner::findVertexConflict(const CBSNode *node, Conflict &conflict) {
    for(int i = 0; i < node->robots.size(); i++) {
        for(int j = i + 1; j < node->robots.size(); j++) {
            // Check target conflicts
            if(node->solution[node->robots[i]].times.size() < node->solution[node->robots[j]].times.size()) {
                for(int k = node->solution[node->robots[j]].times.size() - 1; k >= node->solution[node->robots[i]].times.size(); k--) {
                    // Query the collision map of both robots before checking
                    // auto key = std::make_pair(node->solution[node->robots[i]].trajectory.back(), node->solution[node->robots[j]].trajectory[k]);
                    // int res_i = roadmaps_[node->robots[i]]->queryVertexCollisionMap(key);
                    // int res_j = roadmaps_[node->robots[j]]->queryVertexCollisionMap(key);
                    // if((res_i == 1) || (res_j == 1)) {
                    //     conflict.type = ConstraintType::VERTEX;
                    //     conflict.isTarget = true;
                    //     conflict.robots.push_back(node->robots[i]);
                    //     conflict.robots.push_back(node->robots[j]);
                    //     conflict.time_idx.push_back(k);
                    //     conflict.time_idx.push_back(k);
                    //     return true;
                    // } else if((res_i == 0) || (res_j == 0)) {
                    //     // no collision found, proceed to the next vertex
                    //     continue;
                    // }
                    /*
                    bool voxel_res = instance_->checkVertexCollisionByVoxels({key.first, key.second}, voxel_grid_);
                    if(!voxel_res) {
                        roadmaps_[node->robots[i]]->updateVertexCollisionMap(key, false);
                        roadmaps_[node->robots[j]]->updateVertexCollisionMap(key, false);
                        continue;
                    }
                    */
                    if(instance_->checkCollision({node->solution[node->robots[i]].trajectory.back(), node->solution[node->robots[j]].trajectory[k]}, true)) {
                        conflict.type = ConstraintType::VERTEX;
                        conflict.isTarget = true;
                        conflict.robots.push_back(node->robots[i]);
                        conflict.robots.push_back(node->robots[j]);
                        conflict.time_idx.push_back(k);
                        conflict.time_idx.push_back(k);
                        // Update the collision maps for both robots
                        // roadmaps_[node->robots[i]]->updateVertexCollisionMap(key, true);
                        // roadmaps_[node->robots[j]]->updateVertexCollisionMap(key, true);
                        return true;
                    } else {
                        // Update the collision maps for both robots
                        // roadmaps_[node->robots[i]]->updateVertexCollisionMap(key, false);
                        // roadmaps_[node->robots[j]]->updateVertexCollisionMap(key, false);
                    }
                }
            } else if(node->solution[node->robots[i]].times.size() > node->solution[node->robots[j]].times.size()) {
                for(int k = node->solution[node->robots[i]].times.size() - 1; k >= node->solution[node->robots[j]].times.size(); k--) {
                    // Query the collision map of both robots before checking
                    // auto key = std::make_pair(node->solution[node->robots[i]].trajectory[k], node->solution[node->robots[j]].trajectory.back());
                    // int res_i = roadmaps_[node->robots[i]]->queryVertexCollisionMap(key);
                    // int res_j = roadmaps_[node->robots[j]]->queryVertexCollisionMap(key);
                    // if((res_i == 1) || (res_j == 1)) {
                    //     conflict.type = ConstraintType::VERTEX;
                    //     conflict.isTarget = true;
                    //     conflict.robots.push_back(node->robots[j]);
                    //     conflict.robots.push_back(node->robots[i]);
                    //     conflict.time_idx.push_back(k);
                    //     conflict.time_idx.push_back(k);
                    //     return true;
                    // } else if((res_i == 0) || (res_j == 0)) {
                    //     // no collision found, proceed to the next vertex
                    //     continue;
                    // }
                    /*
                    bool voxel_res = instance_->checkVertexCollisionByVoxels({key.first, key.second}, voxel_grid_);
                    if(!voxel_res) {
                        roadmaps_[node->robots[i]]->updateVertexCollisionMap(key, false);
                        roadmaps_[node->robots[j]]->updateVertexCollisionMap(key, false);
                        continue;
                    }
                    */
                    if(instance_->checkCollision({node->solution[node->robots[i]].trajectory[k], node->solution[node->robots[j]].trajectory.back()}, true)) {
                        conflict.type = ConstraintType::VERTEX;
                        conflict.isTarget = true;
                        conflict.robots.push_back(node->robots[j]);
                        conflict.robots.push_back(node->robots[i]);
                        conflict.time_idx.push_back(k);
                        conflict.time_idx.push_back(k);
                        // Update the collision maps for both robots
                        // roadmaps_[node->robots[i]]->updateVertexCollisionMap(key, true);
                        // roadmaps_[node->robots[j]]->updateVertexCollisionMap(key, true);
                        return true;
                    } else {
                        // Update the collision maps for both robots
                        // roadmaps_[node->robots[i]]->updateVertexCollisionMap(key, false);
                        // roadmaps_[node->robots[j]]->updateVertexCollisionMap(key, false);
                    }
                }
            }
            //Check vertex conflicts
            for(int k = std::min(node->solution[node->robots[i]].times.size(), node->solution[node->robots[j]].times.size()) - 1; k >= 0; k--) {
                // Query the collision map of both robots before checking
                // auto key = std::make_pair(node->solution[node->robots[i]].trajectory[k], node->solution[node->robots[j]].trajectory[k]);
                // int res_i = roadmaps_[node->robots[i]]->queryVertexCollisionMap(key);
                // int res_j = roadmaps_[node->robots[j]]->queryVertexCollisionMap(key);

                // if((res_i == 1) || (res_j == 1)) {
                //     conflict.type = ConstraintType::VERTEX;
                //     conflict.robots.push_back(node->robots[i]);
                //     conflict.robots.push_back(node->robots[j]);
                //     conflict.time_idx.push_back(k);
                //     conflict.time_idx.push_back(k);
                //     return true;
                // } else if((res_i == 0) || (res_j == 0)) {
                //     // no collision found, proceed to the next vertex
                //     continue;
                // }
                /*
                bool voxel_res = instance_->checkVertexCollisionByVoxels({key.first, key.second}, voxel_grid_);
                if(!voxel_res) {
                    roadmaps_[node->robots[i]]->updateVertexCollisionMap(key, false);
                    roadmaps_[node->robots[j]]->updateVertexCollisionMap(key, false);
                    continue;
                }
                */
                if(instance_->checkCollision({node->solution[node->robots[i]].trajectory[k], node->solution[node->robots[j]].trajectory[k]}, true)) {
                    conflict.type = ConstraintType::VERTEX;
                    conflict.robots.push_back(node->robots[i]);
                    conflict.robots.push_back(node->robots[j]);
                    conflict.time_idx.push_back(k);
                    conflict.time_idx.push_back(k);
                    // Update the collision maps for both robots
                    // roadmaps_[node->robots[i]]->updateVertexCollisionMap(key, true);
                    // roadmaps_[node->robots[j]]->updateVertexCollisionMap(key, true);
                    return true;
                } else {
                    // Update the collision maps for both robots
                    // roadmaps_[node->robots[i]]->updateVertexCollisionMap(key, false);
                    // roadmaps_[node->robots[j]]->updateVertexCollisionMap(key, false);
                }
            }
        }
    }
    return false;
}

bool CBSPlanner::findEdgeConflict(const CBSNode *node, Conflict &conflict, const PlannerOptions &options) {
    // Edge conflicts
    int num_interpolations = options.num_interpolations;
    for(int i = 0; i < node->robots.size(); i++) {
        for(int j = i + 1; j < node->robots.size(); j++) {
            // Check target conflicts, i.e., after the first robot reach the target, the following robot must check the collision with the target
            // Note : still use the interpolation method to check the collision
            if(node->solution[node->robots[i]].times.size() < node->solution[node->robots[j]].times.size()) {
                for(int k = node->solution[node->robots[j]].times.size() - 1; k >= node->solution[node->robots[i]].times.size(); k--) {
                    // Before checking, query the collision maps for both robots
                    // PoseEdge edge = PoseEdge(node->solution[node->robots[j]].trajectory[k-1], node->solution[node->robots[j]].trajectory[k]);
                    // auto key = std::make_pair(node->solution[node->robots[i]].trajectory.back(), edge);
                    // int res_i = roadmaps_[node->robots[i]]->queryTargetCollisionMap(key);
                    // int res_j = roadmaps_[node->robots[j]]->queryTargetCollisionMap(key);
                    // if((res_i == 1) || (res_j == 1)) {
                    //     conflict.type = ConstraintType::EDGE;
                    //     conflict.isTarget = true;
                    //     conflict.robots.push_back(node->robots[i]);
                    //     conflict.robots.push_back(node->robots[j]);
                    //     conflict.time_idx.push_back(k-1);
                    //     conflict.time_idx.push_back(k-1);
                    //     return true;
                    // } else if((res_i == 0) || (res_j == 0)) {
                    //     // Indicates no conflict, proceed to the next edge
                    //     continue;
                    // }
                    // Check collision by voxels
                    /*
                    bool voxel_res; std::unordered_set<int> bad_interpolations;
                    std::tie(voxel_res, bad_interpolations) = instance_->checkEdgeCollisionByVoxels({PoseEdge(key.first, key.first), key.second}, voxel_grid_);
                    if(!voxel_res) {
                        roadmaps_[node->robots[i]]->updateTargetCollisionMap(key, false);
                        roadmaps_[node->robots[j]]->updateTargetCollisionMap(key, false);
                        continue;
                    }
                    */
                    const std::vector<RobotPose> motion_start = {
                        node->solution[node->robots[i]].trajectory.back(),
                        node->solution[node->robots[j]].trajectory[k - 1]
                    };
                    const std::vector<RobotPose> motion_goal = {
                        node->solution[node->robots[i]].trajectory.back(),
                        node->solution[node->robots[j]].trajectory[k]
                    };

                    const double step_size = instance_->computeMotionStepSize(motion_start, motion_goal, num_interpolations);
                    const bool collision_found = instance_->checkMultiRobotMotion(motion_start, motion_goal, step_size, true);

                    if(collision_found) {
                        conflict.type = ConstraintType::EDGE;
                        conflict.isTarget = true;
                        conflict.robots.push_back(node->robots[i]);
                        conflict.robots.push_back(node->robots[j]);
                        conflict.time_idx.push_back(k-1);
                        conflict.time_idx.push_back(k-1);
                        // roadmaps_[node->robots[i]]->updateTargetCollisionMap(key, true);
                        // roadmaps_[node->robots[j]]->updateTargetCollisionMap(key, true);
                        return true;
                    }

                    // roadmaps_[node->robots[i]]->updateTargetCollisionMap(key, false);
                    // roadmaps_[node->robots[j]]->updateTargetCollisionMap(key, false);
                }
            } else if(node->solution[node->robots[i]].times.size() > node->solution[node->robots[j]].times.size()) {
                for(int k = node->solution[node->robots[i]].times.size() - 1; k >= node->solution[node->robots[j]].times.size(); k--) {
                    // Before checking, query the collision maps for both robots
                    // PoseEdge edge = PoseEdge(node->solution[node->robots[i]].trajectory[k-1], node->solution[node->robots[i]].trajectory[k]);
                    // auto key = std::make_pair(node->solution[node->robots[j]].trajectory.back(), edge);
                    // int res_i = roadmaps_[node->robots[i]]->queryTargetCollisionMap(key);
                    // int res_j = roadmaps_[node->robots[j]]->queryTargetCollisionMap(key);
                    // if((res_i == 1) || (res_j == 1)) {
                    //     conflict.type = ConstraintType::EDGE;
                    //     conflict.isTarget = true;
                    //     conflict.robots.push_back(node->robots[j]);
                    //     conflict.robots.push_back(node->robots[i]);
                    //     conflict.time_idx.push_back(k-1);
                    //     conflict.time_idx.push_back(k-1);
                    //     return true;
                    // } else if((res_i == 0) || (res_j == 0)) {
                    //     // Indicates no conflict, proceed to the next edge
                    //     continue;
                    // }
                    // Check collision by voxels
                    /*
                    bool voxel_res; std::unordered_set<int> bad_interpolations;
                    std::tie(voxel_res, bad_interpolations) = instance_->checkEdgeCollisionByVoxels({PoseEdge(key.first, key.first), key.second}, voxel_grid_);
                    if(!voxel_res) {
                        roadmaps_[node->robots[i]]->updateTargetCollisionMap(key, false);
                        roadmaps_[node->robots[j]]->updateTargetCollisionMap(key, false);
                        continue;
                    }
                    */
                    const std::vector<RobotPose> motion_start = {
                        node->solution[node->robots[j]].trajectory.back(),
                        node->solution[node->robots[i]].trajectory[k - 1]
                    };
                    const std::vector<RobotPose> motion_goal = {
                        node->solution[node->robots[j]].trajectory.back(),
                        node->solution[node->robots[i]].trajectory[k]
                    };

                    const double step_size = instance_->computeMotionStepSize(motion_start, motion_goal, num_interpolations);
                    const bool collision_found = instance_->checkMultiRobotMotion(motion_start, motion_goal, step_size, true);

                    if(collision_found) {
                        conflict.type = ConstraintType::EDGE;
                        conflict.isTarget = true;
                        conflict.robots.push_back(node->robots[j]);
                        conflict.robots.push_back(node->robots[i]);
                        conflict.time_idx.push_back(k-1);
                        conflict.time_idx.push_back(k-1);
                        // roadmaps_[node->robots[j]]->updateTargetCollisionMap(key, true);
                        // roadmaps_[node->robots[i]]->updateTargetCollisionMap(key, true);
                        return true;
                    }

                    // roadmaps_[node->robots[i]]->updateTargetCollisionMap(key, false);
                    // roadmaps_[node->robots[j]]->updateTargetCollisionMap(key, false);
                }
            }
            // Check conflicts w.r.t. single timestamp inside edges
            for(int k = std::min(node->solution[node->robots[i]].times.size(), node->solution[node->robots[j]].times.size()) - 1; k >= 1; k--) {
                // Query the collision map of both robots before checking
                // PoseEdge edge_i = PoseEdge(node->solution[node->robots[i]].trajectory[k-1], node->solution[node->robots[i]].trajectory[k]);
                // PoseEdge edge_j = PoseEdge(node->solution[node->robots[j]].trajectory[k-1], node->solution[node->robots[j]].trajectory[k]);
                // auto key = std::make_pair(edge_i, edge_j);
                // int res_i = roadmaps_[node->robots[i]]->queryEdgeCollisionMap(key);
                // int res_j = roadmaps_[node->robots[j]]->queryEdgeCollisionMap(key);
                // if((res_i == 1) || (res_j == 1)) {
                //     conflict.type = ConstraintType::EDGE;
                //     conflict.robots.push_back(node->robots[i]);
                //     conflict.robots.push_back(node->robots[j]);
                //     conflict.time_idx.push_back(k-1);
                //     conflict.time_idx.push_back(k-1);
                //     return true;
                // } else if((res_i == 0) || (res_j == 0)) {
                //     // no collision found, proceed to the next edge
                //     continue;
                // }
                // Check collision by voxels
                /*
                bool voxel_res; std::unordered_set<int> bad_interpolations;
                std::tie(voxel_res, bad_interpolations) = instance_->checkEdgeCollisionByVoxels({key.first, key.second}, voxel_grid_);
                if(!voxel_res) {
                    roadmaps_[node->robots[i]]->updateEdgeCollisionMap(key, false);
                    roadmaps_[node->robots[j]]->updateEdgeCollisionMap(key, false);
                    continue;
                }
                */
                const std::vector<RobotPose> motion_start = {
                    node->solution[node->robots[i]].trajectory[k - 1],
                    node->solution[node->robots[j]].trajectory[k - 1]
                };
                const std::vector<RobotPose> motion_goal = {
                    node->solution[node->robots[i]].trajectory[k],
                    node->solution[node->robots[j]].trajectory[k]
                };

                const double step_size = instance_->computeMotionStepSize(motion_start, motion_goal, num_interpolations);
                const bool collision_found = instance_->checkMultiRobotMotion(motion_start, motion_goal, step_size, true);

                if(collision_found) {
                    conflict.type = ConstraintType::EDGE;
                    conflict.robots.push_back(node->robots[i]);
                    conflict.robots.push_back(node->robots[j]);
                    conflict.time_idx.push_back(k-1);
                    conflict.time_idx.push_back(k-1);
                    // roadmaps_[node->robots[i]]->updateEdgeCollisionMap(key, true);
                    // roadmaps_[node->robots[j]]->updateEdgeCollisionMap(key, true);
                    return true;
                }

                // roadmaps_[node->robots[i]]->updateEdgeCollisionMap(key, false);
                // roadmaps_[node->robots[j]]->updateEdgeCollisionMap(key, false);
            }
        }
    }

    return false;
}

std::pair<int, int> CBSPlanner::countNumConflicts(const CBSNode *node, const PlannerOptions &options) {
    // Edge conflicts
    int num_conflicts = 0;
    int num_pairs = 0;
    int num_interpolations = options.num_interpolations;
    for(int i = 0; i < node->robots.size(); i++) {
        for(int j = i + 1; j < node->robots.size(); j++) {
            bool found_in_pair = false;
            // Check conflicts w.r.t. single timestamp inside edges
            for(int k = 1; k < std::min(node->solution[node->robots[i]].times.size(), node->solution[node->robots[j]].times.size()); k++) {
                /* Before checking, first query the collision maps for both robots */
                // auto vertex_key = std::make_pair(node->solution[node->robots[i]].trajectory[k], node->solution[node->robots[j]].trajectory[k]);
                // PoseEdge edge_i = PoseEdge(node->solution[node->robots[i]].trajectory[k-1], node->solution[node->robots[i]].trajectory[k]);
                // PoseEdge edge_j = PoseEdge(node->solution[node->robots[j]].trajectory[k-1], node->solution[node->robots[j]].trajectory[k]);
                // auto edge_key = std::make_pair(edge_i, edge_j);
                // int res_vi = roadmaps_[node->robots[i]]->queryVertexCollisionMap(vertex_key);
                // int res_vj = roadmaps_[node->robots[j]]->queryVertexCollisionMap(vertex_key);
                // int res_ei = roadmaps_[node->robots[i]]->queryEdgeCollisionMap(edge_key);
                // int res_ej = roadmaps_[node->robots[j]]->queryEdgeCollisionMap(edge_key);
                // bool results_found = false;
                // if((res_vi == 1) || (res_vj == 1)) {
                //     num_conflicts++;
                //     results_found = true;
                //     if(!found_in_pair) {
                //         found_in_pair = true;
                //         num_pairs++;
                //     }
                // }
                // if((res_ei == 1) || (res_ej == 1)) {
                //     num_conflicts++;
                //     results_found = true;
                //     if(!found_in_pair) {
                //         found_in_pair = true;
                //         num_pairs++;
                //     }
                // }
                // if(((res_vi == 0) || (res_vj == 0)) && ((res_ei == 0) || (res_ej == 0))) {
                //     // no collision found, proceed to the next edge
                //     results_found = true;
                // }
                // if(results_found) {
                //     continue;
                // }

                // Check collision by voxels
                /*
                bool voxel_vertex_res = instance_->checkVertexCollisionByVoxels({vertex_key.first, vertex_key.second}, voxel_grid_);
                if(!voxel_vertex_res) {
                    roadmaps_[node->robots[i]]->updateVertexCollisionMap(vertex_key, false);
                    roadmaps_[node->robots[j]]->updateVertexCollisionMap(vertex_key, false);
                }
                bool voxel_edge_res; std::unordered_set<int> bad_interpolations;
                std::tie(voxel_edge_res, bad_interpolations) = instance_->checkEdgeCollisionByVoxels({edge_key.first, edge_key.second}, voxel_grid_);
                if(!voxel_edge_res) {
                    roadmaps_[node->robots[i]]->updateEdgeCollisionMap(edge_key, false);
                    roadmaps_[node->robots[j]]->updateEdgeCollisionMap(edge_key, false);
                }
                if(!voxel_vertex_res && !voxel_edge_res) {
                    continue;
                }
                */
                
                const std::vector<RobotPose> motion_start = {
                    node->solution[node->robots[i]].trajectory[k - 1],
                    node->solution[node->robots[j]].trajectory[k - 1]
                };
                const std::vector<RobotPose> motion_goal = {
                    node->solution[node->robots[i]].trajectory[k],
                    node->solution[node->robots[j]].trajectory[k]
                };

                const double step_size = instance_->computeMotionStepSize(motion_start, motion_goal, num_interpolations);
                const bool found_in_edge = instance_->checkMultiRobotMotion(motion_start, motion_goal, step_size, true);

                if(found_in_edge) {
                    num_conflicts++;
                    if(!found_in_pair) {
                        found_in_pair = true;
                        num_pairs++;
                    }
                    // const bool goal_collision = instance_->checkCollision(motion_goal, true);
                    // if(goal_collision) {
                    //     roadmaps_[node->robots[i]]->updateVertexCollisionMap(vertex_key, true);
                    //     roadmaps_[node->robots[j]]->updateVertexCollisionMap(vertex_key, true);
                    // } else {
                    //     roadmaps_[node->robots[i]]->updateEdgeCollisionMap(edge_key, true);
                    //     roadmaps_[node->robots[j]]->updateEdgeCollisionMap(edge_key, true);
                    // }
                } else {
                    // roadmaps_[node->robots[i]]->updateVertexCollisionMap(vertex_key, false);
                    // roadmaps_[node->robots[j]]->updateVertexCollisionMap(vertex_key, false);
                    // roadmaps_[node->robots[i]]->updateEdgeCollisionMap(edge_key, false);
                    // roadmaps_[node->robots[j]]->updateEdgeCollisionMap(edge_key, false);
                }
            }
            // Check target conflicts, i.e., after the first robot reach the target, the following robot must check the collision with the target
            // Note : still use the interpolation method to check the collision
            if(node->solution[node->robots[i]].times.size() < node->solution[node->robots[j]].times.size()) {
                for(int k = node->solution[node->robots[i]].times.size(); k < node->solution[node->robots[j]].times.size(); k++) {
                    // Query the collision map of both robots before checking
                    // auto vertex_key = std::make_pair(node->solution[node->robots[i]].trajectory.back(), node->solution[node->robots[j]].trajectory[k]);
                    // PoseEdge edge = PoseEdge(node->solution[node->robots[j]].trajectory[k-1], node->solution[node->robots[j]].trajectory[k]);
                    // auto target_key = std::make_pair(node->solution[node->robots[i]].trajectory.back(), edge);
                    // int res_vi = roadmaps_[node->robots[i]]->queryVertexCollisionMap(vertex_key);
                    // int res_vj = roadmaps_[node->robots[j]]->queryVertexCollisionMap(vertex_key);
                    // int res_ti = roadmaps_[node->robots[i]]->queryTargetCollisionMap(target_key);
                    // int res_tj = roadmaps_[node->robots[j]]->queryTargetCollisionMap(target_key);
                    // bool results_found = false;
                    // if(res_vi == 1 || res_vj == 1) {
                    //     num_conflicts++;
                    //     results_found = true;
                    //     if(!found_in_pair) {
                    //         found_in_pair = true;
                    //         num_pairs++;
                    //     }
                    // }
                    // if((res_ti == 1 || res_tj == 1)) {
                    //     num_conflicts++;
                    //     results_found = true;
                    //     if(!found_in_pair) {
                    //         found_in_pair = true;
                    //         num_pairs++;
                    //     }
                    // }
                    // if((res_vi == 0 || res_vj == 0) && (res_ti == 0 || res_tj == 0)) {
                    //     // no collision found, proceed to the next edge
                    //     results_found = true;
                    // }
                    // if(results_found) {
                    //     continue;
                    // }
                    // Check collision by voxels
                    /*
                    bool voxel_vertex_res = instance_->checkVertexCollisionByVoxels({vertex_key.first, vertex_key.second}, voxel_grid_);
                    if(!voxel_vertex_res) {
                        roadmaps_[node->robots[i]]->updateVertexCollisionMap(vertex_key, false);
                        roadmaps_[node->robots[j]]->updateVertexCollisionMap(vertex_key, false);
                    }
                    bool voxel_target_res; std::unordered_set<int> bad_interpolations;
                    std::tie(voxel_target_res, bad_interpolations) = instance_->checkEdgeCollisionByVoxels({PoseEdge(target_key.first, target_key.first), target_key.second}, voxel_grid_);
                    if(!voxel_target_res) {
                        roadmaps_[node->robots[i]]->updateTargetCollisionMap(target_key, false);
                        roadmaps_[node->robots[j]]->updateTargetCollisionMap(target_key, false);
                    }
                    if(!voxel_vertex_res && !voxel_target_res) {
                        continue;
                    }
                    */
                    // Check for collision
                    const std::vector<RobotPose> motion_start = {
                        node->solution[node->robots[i]].trajectory.back(),
                        node->solution[node->robots[j]].trajectory[k - 1]
                    };
                    const std::vector<RobotPose> motion_goal = {
                        node->solution[node->robots[i]].trajectory.back(),
                        node->solution[node->robots[j]].trajectory[k]
                    };

                    const double step_size = instance_->computeMotionStepSize(motion_start, motion_goal, num_interpolations);
                    const bool found_in_edge = instance_->checkMultiRobotMotion(motion_start, motion_goal, step_size, true);

                    if(found_in_edge) {
                        num_conflicts++;
                        if(!found_in_pair) {
                            found_in_pair = true;
                            num_pairs++;
                        }
                        // const bool goal_collision = instance_->checkCollision(motion_goal, true);
                        // if(goal_collision) {
                        //     roadmaps_[node->robots[i]]->updateVertexCollisionMap(vertex_key, true);
                        //     roadmaps_[node->robots[j]]->updateVertexCollisionMap(vertex_key, true);
                        // } else {
                        //     roadmaps_[node->robots[i]]->updateTargetCollisionMap(target_key, true);
                        //     roadmaps_[node->robots[j]]->updateTargetCollisionMap(target_key, true);
                        // }
                    } else {
                        // roadmaps_[node->robots[i]]->updateVertexCollisionMap(vertex_key, false);
                        // roadmaps_[node->robots[j]]->updateVertexCollisionMap(vertex_key, false);
                        // roadmaps_[node->robots[i]]->updateTargetCollisionMap(target_key, false);
                        // roadmaps_[node->robots[j]]->updateTargetCollisionMap(target_key, false);
                    }
                }
            } else if(node->solution[node->robots[i]].times.size() > node->solution[node->robots[j]].times.size()) {            
                for(int k = node->solution[node->robots[j]].times.size(); k < node->solution[node->robots[i]].times.size(); k++) {
                    // Query the collision map of both robots before checking
                    // auto vertex_key = std::make_pair(node->solution[node->robots[i]].trajectory[k], node->solution[node->robots[j]].trajectory.back());
                    // PoseEdge edge = PoseEdge(node->solution[node->robots[i]].trajectory[k-1], node->solution[node->robots[i]].trajectory[k]);
                    // auto target_key = std::make_pair(node->solution[node->robots[j]].trajectory.back(), edge);
                    // int res_vi = roadmaps_[node->robots[i]]->queryVertexCollisionMap(vertex_key);
                    // int res_vj = roadmaps_[node->robots[j]]->queryVertexCollisionMap(vertex_key);
                    // int res_ti = roadmaps_[node->robots[i]]->queryTargetCollisionMap(target_key);
                    // int res_tj = roadmaps_[node->robots[j]]->queryTargetCollisionMap(target_key);
                    // bool results_found = false;
                    // if(res_vi == 1 || res_vj == 1) {
                    //     num_conflicts++;
                    //     results_found = true;
                    //     if(!found_in_pair) {
                    //         found_in_pair = true;
                    //         num_pairs++;
                    //     }
                    // }
                    // if(res_ti == 1 || res_tj == 1) {
                    //     num_conflicts++;
                    //     results_found = true;
                    //     if(!found_in_pair) {
                    //         found_in_pair = true;
                    //         num_pairs++;
                    //     }
                    // }
                    // if((res_vi == 0 || res_vj == 0) && (res_ti == 0 || res_tj == 0)) {
                    //     // no collision found, proceed to the next edge
                    //     results_found = true;
                    // }
                    // if(results_found) {
                    //     continue;
                    // }
                    // Check collision by voxels
                    /*
                    bool voxel_vertex_res = instance_->checkVertexCollisionByVoxels({vertex_key.first, vertex_key.second}, voxel_grid_);
                    if(!voxel_vertex_res) {
                        roadmaps_[node->robots[i]]->updateVertexCollisionMap(vertex_key, false);
                        roadmaps_[node->robots[j]]->updateVertexCollisionMap(vertex_key, false);
                    }
                    bool voxel_target_res; std::unordered_set<int> bad_interpolations;
                    std::tie(voxel_target_res, bad_interpolations) = instance_->checkEdgeCollisionByVoxels({PoseEdge(target_key.first, target_key.first), target_key.second}, voxel_grid_);
                    if(!voxel_target_res) {
                        roadmaps_[node->robots[i]]->updateTargetCollisionMap(target_key, false);
                        roadmaps_[node->robots[j]]->updateTargetCollisionMap(target_key, false);
                    }
                    if(!voxel_vertex_res && !voxel_target_res) {
                        continue;
                    }
                    */
                    // Divide the edge into small segments
                    const std::vector<RobotPose> motion_start = {
                        node->solution[node->robots[i]].trajectory[k - 1],
                        node->solution[node->robots[j]].trajectory.back()
                    };
                    const std::vector<RobotPose> motion_goal = {
                        node->solution[node->robots[i]].trajectory[k],
                        node->solution[node->robots[j]].trajectory.back()
                    };

                    const double step_size = instance_->computeMotionStepSize(motion_start, motion_goal, num_interpolations);
                    const bool found_in_edge = instance_->checkMultiRobotMotion(motion_start, motion_goal, step_size, true);

                    if(found_in_edge) {
                        num_conflicts++;
                        if(!found_in_pair) {
                            found_in_pair = true;
                            num_pairs++;
                        }
                        // const bool goal_collision = instance_->checkCollision(motion_goal, true);
                        // if(goal_collision) {
                        //     roadmaps_[node->robots[i]]->updateVertexCollisionMap(vertex_key, true);
                        //     roadmaps_[node->robots[j]]->updateVertexCollisionMap(vertex_key, true);
                        // } else {
                        //     roadmaps_[node->robots[i]]->updateTargetCollisionMap(target_key, true);
                        //     roadmaps_[node->robots[j]]->updateTargetCollisionMap(target_key, true);
                        // }
                    } else {
                        // roadmaps_[node->robots[i]]->updateVertexCollisionMap(vertex_key, false);
                        // roadmaps_[node->robots[j]]->updateVertexCollisionMap(vertex_key, false);
                        // roadmaps_[node->robots[i]]->updateTargetCollisionMap(target_key, false);
                        // roadmaps_[node->robots[j]]->updateTargetCollisionMap(target_key, false);
                    }
                }
            }
        }
    }

    // std::cout << "Number of pairs with conflicts: " << num_pairs << std::endl;
    // return num_conflicts;
    return {num_pairs, num_conflicts};
}

RobotPose myInterpolate(const RobotPose &a, const RobotPose &b, double t) {
    // Interpolate between two poses
    if (a.robot_id != b.robot_id) {
        return RobotPose();
    }
    if (a.joint_values.size() != b.joint_values.size()) {
        return RobotPose();
    }
    RobotPose pose = a;
    for (int i = 0; i < a.joint_values.size(); i++) {
        pose.joint_values[i] = a.joint_values[i] + (b.joint_values[i] - a.joint_values[i]) * t;
    }
    return pose;
}

#if MR_PLANNER_WITH_ROS
bool convertSolution(std::shared_ptr<PlanInstance> instance,
                    const moveit_msgs::RobotTrajectory &plan_traj,
                    MRTrajectory &solution,
                    bool reset_speed) {
    // Convert a MoveIt plan to a RobotTrajectory
    int numRobots = instance->getNumberOfRobots();
    solution.clear();
    solution.resize(numRobots);
    for (int i = 0; i < numRobots; i++) {
        solution[i].robot_id = i;
    }

    for (int i = 0; i < plan_traj.joint_trajectory.points.size(); i++) {
        int st = 0;
        double timeDilation = 0;
        for (int j = 0; j < numRobots; j++) {
            RobotPose pose = instance->initRobotPose(j);
            assert (st + pose.joint_values.size() <= plan_traj.joint_trajectory.points[i].positions.size());
            for (int k = 0; k < pose.joint_values.size(); k++) {
                pose.joint_values[k] = plan_traj.joint_trajectory.points[i].positions[k + pose.joint_values.size()*j];
            }

            if (i > 0) {
                double dt = plan_traj.joint_trajectory.points[i].time_from_start.toSec() - plan_traj.joint_trajectory.points[i-1].time_from_start.toSec();
                double speed = std::abs(instance->computeDistance(solution[j].trajectory.back(), pose)) / dt;
                if (speed > instance->getVMax(j)) {
                    timeDilation = std::max(timeDilation, speed / instance->getVMax(j));
                }
                else if (speed <= instance->getVMax(j)) {
                    timeDilation = std::max(timeDilation, speed / instance->getVMax(j)); 
                }
            }
            solution[j].trajectory.push_back(pose);
            st += pose.joint_values.size();
        }

        for (int j = 0; j < numRobots; j++) {
            if (i > 0) {
                double dt = plan_traj.joint_trajectory.points[i].time_from_start.toSec() - plan_traj.joint_trajectory.points[i-1].time_from_start.toSec();
                if (reset_speed) {
                    dt = dt * timeDilation;
                }
                solution[j].times.push_back(solution[j].times.back() + dt);
            }
            else {
                solution[j].times.push_back(plan_traj.joint_trajectory.points[i].time_from_start.toSec());
            }
        }
    }

    // for each robot, if the robot is not moving at the end, keep removing the last point
    for (int i = 0; i < numRobots; i++) {
       RobotPose last_pose = solution[i].trajectory.back();
       while (solution[i].trajectory.size() > 1 && instance->computeDistance(solution[i].trajectory[solution[i].trajectory.size()-2], last_pose) < 1e-5) {
           solution[i].trajectory.pop_back();
           solution[i].times.pop_back();
       }
    }

    for (int i = 0; i < numRobots; i++) {
        solution[i].cost = solution[i].times.back();
    }

    return true;
}

bool convertSolution(std::shared_ptr<PlanInstance> instance,
                    const moveit_msgs::RobotTrajectory &plan_traj,
                    int robot_id,
                    RobotTrajectory &solution)
{   
    for (int i = 0; i < plan_traj.joint_trajectory.points.size(); i++) {
        double timeDilation = 0;
        RobotPose pose = instance->initRobotPose(robot_id);
        
        assert (pose.joint_values.size() == plan_traj.joint_trajectory.points[i].positions.size());
        for (int k = 0; k < pose.joint_values.size(); k++) {
            pose.joint_values[k] = plan_traj.joint_trajectory.points[i].positions[k];
        }

        if (i > 0) {
            double dt = plan_traj.joint_trajectory.points[i].time_from_start.toSec() - plan_traj.joint_trajectory.points[i-1].time_from_start.toSec();
            double speed = std::abs(instance->computeDistance(solution.trajectory.back(), pose)) / dt;
            timeDilation = speed / instance->getVMax(robot_id);
        }
        solution.trajectory.push_back(pose);

        if (i > 0) {
            double dt = plan_traj.joint_trajectory.points[i].time_from_start.toSec() - plan_traj.joint_trajectory.points[i-1].time_from_start.toSec();
            dt = dt * timeDilation;
            solution.times.push_back(solution.times.back() + dt);
        }
        else {
            solution.times.push_back(plan_traj.joint_trajectory.points[i].time_from_start.toSec());
        }
    }


    solution.cost = solution.times.back();
    return true;
}

bool saveSolution(std::shared_ptr<PlanInstance> instance,
                  const moveit_msgs::RobotTrajectory &plan_traj,
                  const std::string &file_name)
{
    int numRobots = instance->getNumberOfRobots();

    std::ofstream file(file_name);
    if (!file.is_open()) {
        log("Failed to open file " + file_name + " for writing!", LogLevel::ERROR);
        return false;
    }
    int total_dim = 0;
    for (int j = 0; j < numRobots; j++) {
        total_dim += instance->getRobotDOF(j);
    }
    if (plan_traj.joint_trajectory.points[0].positions.size() != total_dim) {
        log("Invalid plan trajectory size!", LogLevel::ERROR);
        return false;
    }

    file << "time" << ",";

    for (int j = 0; j < numRobots; j++) {
        for (int d = 0; d < instance->getRobotDOF(j); d++) {
            file << "q" << std::to_string(j) << "_" << std::to_string(d) << ",";
        }
    }
    file << std::endl;

    int max_size = plan_traj.joint_trajectory.points.size();
    for (int i = 0; i < max_size; i++) {
        file << plan_traj.joint_trajectory.points[i].time_from_start.toSec() << ",";
        for (int d = 0; d < total_dim; d++) {
            file << plan_traj.joint_trajectory.points[i].positions[d] << ",";
        }
        file << std::endl;
    }
    file.close();

    return true;
}
#endif

bool saveSolution(std::shared_ptr<PlanInstance> instance,
                  const MRTrajectory &synced_traj,
                  const std::string &file_name)

{
    int numRobots = instance->getNumberOfRobots();

    std::ofstream file(file_name);
    if (!file.is_open()) {
        log("Failed to open file " + file_name + " for writing!", LogLevel::ERROR);
        return false;
    }
    int total_dim = 0;
    for (int j = 0; j < numRobots; j++) {
        total_dim += instance->getRobotDOF(j);
    }

    file << "time" << ",";

    for (int j = 0; j < numRobots; j++) {
        for (int d = 0; d < instance->getRobotDOF(j); d++) {
            file << "q" << std::to_string(j) << "_" << std::to_string(d) << ",";
        }
    }
    file << std::endl;

    int max_size = 0;
    for (int j = 0; j < numRobots; j++) {
        max_size = std::max(max_size, int(synced_traj[j].times.size()));
    }
    for (int i = 0; i < max_size; i++) {
        double t = -1;
        for (int j = 0; j < numRobots; j++) {
            if (i < synced_traj[j].times.size()) {
                double tj = synced_traj[j].times[i];
                if (t != -1 && (tj != t)) {
                    log("Time is not synched for the plan!", LogLevel::ERROR);
                    return false;
                }
                t = tj;
            }
        }
        file << t << ",";
        for (int j = 0; j < numRobots; j++) {
            for (int d = 0; d < instance->getRobotDOF(j) ; d++) {
                file << synced_traj[j].trajectory[i].joint_values[d] << ",";
            }
        }
        file << std::endl;
    }
    file.close();

    return true;
}

#if MR_PLANNER_WITH_ROS
bool loadSolution(std::shared_ptr<PlanInstance> instance,
                 const std::string &file_name,
                 double dt,
                 moveit_msgs::RobotTrajectory &plan_traj)
{
    // assume joint name is already given
    int numRobots = instance->getNumberOfRobots();

    std::ifstream file(file_name);
    if (!file.is_open()) {
        log("Failed to open file " + file_name + " for reading!", LogLevel::ERROR);
        return false;
    }
    
    
    int total_dim = 0;
    for (int j = 0; j < numRobots; j++) {
        total_dim += instance->getRobotDOF(j);
    }
    std::string line;
    std::getline(file, line);
    std::vector<std::string> tokens;
    boost::split(tokens, line, boost::is_any_of(","));
    if (tokens.size() != total_dim || plan_traj.joint_trajectory.joint_names.size() != total_dim) {
        log("Invalid plan trajectory size!", LogLevel::ERROR);
        return false;
    }

    plan_traj.joint_trajectory.points.clear();    
   
    int t = 0;
    while (std::getline(file, line)) {
        boost::split(tokens, line, boost::is_any_of(","));
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions.resize(total_dim);
        point.velocities.resize(total_dim);
        point.accelerations.resize(total_dim);
        point.time_from_start = ros::Duration(t * dt);
        for (int d = 0; d < total_dim; d++) {
            point.positions[d] = std::stod(tokens[d]);
        }

        plan_traj.joint_trajectory.points.push_back(point);
        t++;
    }
    
    // compute velocities and accelerations with central difference
    auto &points = plan_traj.joint_trajectory.points;
    for (int i = 1; i < points.size() - 1; i++) {
        for (int j = 0; j < total_dim; j++) {
            points[i].velocities[j] = (points[i+1].positions[j] - points[i-1].positions[j]) / (2 * dt);
            points[i].accelerations[j] = (points[i+1].positions[j] - 2 * points[i].positions[j] + points[i-1].positions[j]) / (dt * dt);
        }
    }

    return true;
}


bool loadSolution(std::shared_ptr<PlanInstance> instance,
                 const std::string &file_name,
                 moveit_msgs::RobotTrajectory &plan_traj)
{
    // assume joint name is already given
    int numRobots = instance->getNumberOfRobots();

    std::ifstream file(file_name);
    if (!file.is_open()) {
        log("Failed to open file " + file_name + " for reading!", LogLevel::ERROR);
        return false;
    }
    
    
    int total_dim = 0;
    for (int j = 0; j < numRobots; j++) {
        total_dim += instance->getRobotDOF(j);
    }
    std::string line;
    std::getline(file, line);
    std::vector<std::string> tokens;
    boost::split(tokens, line, boost::is_any_of(","));
    // some csv has a extra comma at the end
    if (tokens.back().empty()) {
        tokens.pop_back();
    }
    if (tokens.size() != (total_dim + 1)|| plan_traj.joint_trajectory.joint_names.size() != total_dim) {
        log("Invalid plan trajectory size!", LogLevel::ERROR);
        return false;
    }

    plan_traj.joint_trajectory.points.clear();    
   
    int t = 0;
    while (std::getline(file, line)) {
        boost::split(tokens, line, boost::is_any_of(","));
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions.resize(total_dim);
        point.velocities.resize(total_dim);
        point.accelerations.resize(total_dim);
        point.time_from_start = ros::Duration(std::stod(tokens[0]));
        for (int d = 0; d < total_dim; d++) {
            point.positions[d] = std::stod(tokens[d+1]);
        }

        plan_traj.joint_trajectory.points.push_back(point);
        t++;
    }
    std::cout << "Loaded " << t << " points" << std::endl;
    
    // compute velocities and accelerations with central difference
    auto &points = plan_traj.joint_trajectory.points;
    for (int i = 1; i < points.size() - 1; i++) {
        for (int j = 0; j < total_dim; j++) {
            points[i].velocities[j] = (points[i+1].positions[j] - points[i-1].positions[j]) / (points[i+1].time_from_start.toSec() - points[i-1].time_from_start.toSec());
            points[i].accelerations[j] = (points[i+1].positions[j] - 2 * points[i].positions[j] + points[i-1].positions[j]) 
                / ((points[i].time_from_start.toSec() - points[i-1].time_from_start.toSec()) * (points[i+1].time_from_start.toSec() - points[i].time_from_start.toSec()));
        }
    }

    return true;
}
#endif

void retimeSolution(std::shared_ptr<PlanInstance> instance,
                    const MRTrajectory &solution,
                    MRTrajectory &retime_solution,
                    double dt)
{
    // retime solution based on maximum velocity, assuming solution has uniform time discretization and the same time for all robots
    // no act id
    int num_robot = instance->getNumberOfRobots();
    retime_solution.clear();

    int count = solution[0].times.size();
    for (int i = 0; i < num_robot; i++) {
        retime_solution.push_back(RobotTrajectory());
        retime_solution[i].robot_id = i;
        if (count != solution[i].times.size()) {
            log("Solution has different number of points for each robot!", LogLevel::ERROR);
        }
        if (solution[i].act_ids.size() > 0) {
            log("Solution is multi-task!", LogLevel::ERROR);
        }
    }

    int step = 0;
    while (step < count) {
        double timeDilation = 0;
        double dt_step;
        if (step > 0) {
            for (int i = 0; i < instance->getNumberOfRobots(); i++) {
                double dist = std::abs(instance->computeDistance(solution[i].trajectory[step], solution[i].trajectory[step - 1]));
                dt_step = solution[i].times[step] - solution[i].times[step - 1];
                double speed = (dist / dt_step);
                timeDilation = std::max(timeDilation, speed / instance->getVMax(i)); 
            }
        }

        // append the point to speedup_traj
        for (int i = 0; i < instance->getNumberOfRobots(); i++) {
            retime_solution[i].trajectory.push_back(solution[i].trajectory[step]);
            if (step > 0) {
                retime_solution[i].times.push_back(retime_solution[i].times.back() + dt_step * timeDilation);
            }
            else {
                retime_solution[i].times.push_back(solution[i].times[step]);
            }
        }
        step ++;
    }

    for (int i = 0; i < num_robot; i++) {
        retime_solution[i].cost = retime_solution[i].times.back();
    }
}

void retimeSolution(std::shared_ptr<PlanInstance> instance,
                    const RobotTrajectory &solution,
                    RobotTrajectory &retime_solution,
                    int robot_id)
{
    // retime solution based on maximum velocity
    if (robot_id != solution.robot_id) {
        log("Robot id does not match!", LogLevel::ERROR);
    }

    retime_solution.robot_id = solution.robot_id;
    retime_solution.trajectory = solution.trajectory;
    retime_solution.times.clear();

    int count = solution.times.size();

    if (solution.act_ids.size() > 0) {
        log("Solution is multi-task!", LogLevel::ERROR);
    }

    int step = 0;
    while (step < count) {
        double timeDilation = 0;
        double dt_step;
        if (step > 0) {
            double dist = std::abs(instance->computeDistance(solution.trajectory[step], solution.trajectory[step - 1]));
            dt_step = solution.times[step] - solution.times[step - 1];
            double speed = (dist / dt_step);
            timeDilation = speed / instance->getVMax(retime_solution.robot_id);
        }

        // append the point to speedup_traj
        if (step > 0) {
            retime_solution.times.push_back(retime_solution.times.back() + dt_step * timeDilation);
        }
        else {
            retime_solution.times.push_back(solution.times[step]);
        }
        
        step ++;
    }
    retime_solution.cost = retime_solution.times.back();
}

#if MR_PLANNER_WITH_ROS
void rediscretizeSolution(std::shared_ptr<PlanInstance> instance,
                    const moveit_msgs::RobotTrajectory &plan_traj,
                    moveit_msgs::RobotTrajectory &retime_traj,
                    double new_dt)
{
    // Convert a MoveIt plan to a RobotTrajectory
    int numRobots = instance->getNumberOfRobots();
    retime_traj.joint_trajectory.joint_names = plan_traj.joint_trajectory.joint_names;
    retime_traj.joint_trajectory.points.clear();
    
    int numPoints = std::ceil(plan_traj.joint_trajectory.points.back().time_from_start.toSec() / new_dt) + 1;
    retime_traj.joint_trajectory.points.resize(numPoints);
    int ind = 0;
    int total_dim = plan_traj.joint_trajectory.joint_names.size();

    for (int i = 0; i < numPoints; i++) {
        double time = i * new_dt;
        while (ind + 1 < plan_traj.joint_trajectory.points.size() && plan_traj.joint_trajectory.points[ind+1].time_from_start.toSec() <= time) {
            ind++;
        }
        int dof_s = 0;
        for (int j = 0; j < numRobots; j++) {
            RobotPose pose = instance->initRobotPose(j);
            assert ((dof_s + pose.joint_values.size()) <= plan_traj.joint_trajectory.points[ind].positions.size());
            if (ind + 1 == plan_traj.joint_trajectory.points.size()) {
                for (int d = 0; d < instance->getRobotDOF(j); d++) {
                    pose.joint_values[d] = plan_traj.joint_trajectory.points[ind].positions[dof_s + d];
                }
            }
            else {
                RobotPose pose_next = instance->initRobotPose(j);
                RobotPose pose_prev = instance->initRobotPose(j);
                for (int d = 0; d < instance->getRobotDOF(j); d++) {
                    pose_next.joint_values[d] = plan_traj.joint_trajectory.points[ind+1].positions[dof_s + d];
                    pose_prev.joint_values[d] = plan_traj.joint_trajectory.points[ind].positions[dof_s + d];
                }
                double alpha = (time - plan_traj.joint_trajectory.points[ind].time_from_start.toSec()) 
                        / (plan_traj.joint_trajectory.points[ind + 1].time_from_start.toSec() - plan_traj.joint_trajectory.points[ind].time_from_start.toSec());
                pose = instance->interpolate(pose_prev, pose_next, alpha);
            }

            for (int d = 0; d < instance->getRobotDOF(j); d++) {
                retime_traj.joint_trajectory.points[i].positions.push_back(pose.joint_values[d]);
            }

            dof_s += instance->getRobotDOF(j);
        }
        retime_traj.joint_trajectory.points[i].time_from_start = ros::Duration(time);
        retime_traj.joint_trajectory.points[i].velocities.resize(total_dim);
        retime_traj.joint_trajectory.points[i].accelerations.resize(total_dim);

    }

    // compute velocities and accelerations with central difference
    auto &points = retime_traj.joint_trajectory.points;
    for (int i = 1; i < points.size() - 1; i++) {
        for (int j = 0; j < total_dim; j++) {
            points[i].velocities[j] = (points[i+1].positions[j] - points[i-1].positions[j]) / (points[i+1].time_from_start.toSec() - points[i-1].time_from_start.toSec());
            points[i].accelerations[j] = (points[i+1].positions[j] - 2 * points[i].positions[j] + points[i-1].positions[j]) 
                / ((points[i].time_from_start.toSec() - points[i-1].time_from_start.toSec()) * (points[i+1].time_from_start.toSec() - points[i].time_from_start.toSec()));
        }
    }

}
#endif

void rediscretizeSolution(std::shared_ptr<PlanInstance> instance,
                        const MRTrajectory &solution,
                        MRTrajectory &retime_solution,
                        double new_dt)
{
    // assuming a single task for each robot
    int numRobots = instance->getNumberOfRobots();
    double maxtime = 0;
    for (int i = 0; i < numRobots; i++) {
        maxtime = std::max(maxtime, solution[i].times.back());
        if (solution[i].act_ids.size() > 0) {
            log("Solution is multi-task!", LogLevel::ERROR);
        }
    }

    retime_solution.clear();
    retime_solution.resize(numRobots);
    for (int i = 0; i < numRobots; i++) {
        retime_solution[i].robot_id = i;
    }

    int numSteps = maxtime / new_dt;
    
    std::vector<int> index(numRobots, 0);
    for (int s = 0; s <= numSteps; s++) {
        double t = s * new_dt;
        for (int i = 0; i < numRobots; i++) {
            while (index[i] + 1 < solution[i].times.size() && solution[i].times[index[i] + 1] <= t) {
                index[i]++;
            }
            if (index[i] + 1 == solution[i].times.size()) {
                // assuming obstacle stays at the end of the trajectory
                retime_solution[i].trajectory.push_back(solution[i].trajectory[index[i]]);
                retime_solution[i].times.push_back(t);
            } else {
                double alpha = (t - solution[i].times[index[i]]) / (solution[i].times[index[i] + 1] - solution[i].times[index[i]]);
                RobotPose pose_i = instance->interpolate(solution[i].trajectory[index[i]], solution[i].trajectory[index[i] + 1], alpha);
                retime_solution[i].trajectory.push_back(pose_i);
                retime_solution[i].times.push_back(t);
            }
        }
    }
    
    for (int i = 0; i < numRobots; i++) {
        retime_solution[i].cost = retime_solution[i].times.back();
    }
    return;
}

void removeWait(std::shared_ptr<PlanInstance> instance,
                        MRTrajectory &solution)
{
    int numRobots = instance->getNumberOfRobots();
    for (int i = 0; i < numRobots; i++)
    {
        int index = solution[i].times.size() - 1;
        while (index >= 1) {
            double dist = instance->computeDistance(solution[i].trajectory[index - 1], solution[i].trajectory[index]);
            if (dist < 1e-5) {
                solution[i].trajectory.erase(solution[i].trajectory.begin() + index);
                solution[i].times.erase(solution[i].times.begin() + index);
            }
            index --;
        }
        solution[i].cost = solution[i].times.back();
    }
}

bool validateSolution(std::shared_ptr<PlanInstance> instance,
                    const MRTrajectory &solution,
                    double col_dt)
{
    const int numRobots = instance->getNumberOfRobots();
    double maxtime = 0.0;
    for (int i = 0; i < numRobots; i++) {
        maxtime = std::max(maxtime, solution[i].times.back());
    }

    const int numSteps = maxtime / col_dt;

    MRTrajectory sampled(numRobots);
    for (int i = 0; i < numRobots; ++i) {
        sampled[i].robot_id = solution[i].robot_id;
    }

    std::vector<int> index(numRobots, 0);
    for (int s = 0; s <= numSteps; ++s) {
        const double t = s * col_dt;
        for (int i = 0; i < numRobots; ++i) {
            while (index[i] + 1 < solution[i].times.size() && solution[i].times[index[i] + 1] <= t) {
                index[i]++;
            }
            if (index[i] + 1 == solution[i].times.size()) {
                sampled[i].trajectory.push_back(solution[i].trajectory[index[i]]);
            } else {
                const double alpha = (t - solution[i].times[index[i]]) /
                                     (solution[i].times[index[i] + 1] - solution[i].times[index[i]]);
                sampled[i].trajectory.push_back(
                    instance->interpolate(solution[i].trajectory[index[i]],
                                          solution[i].trajectory[index[i] + 1], alpha));
            }
        }
    }

    if (!instance->checkMultiRobotTrajectory(sampled, false)) {
        return true;
    }

    // Collision detected; fall back to step-by-step inspection for logging.
    for (int s = 0; s <= numSteps; ++s) {
        std::vector<RobotPose> poses;
        poses.reserve(numRobots);
        for (int i = 0; i < numRobots; ++i) {
            poses.push_back(sampled[i].trajectory[s]);
        }

        if (instance->checkCollision(poses, false)) {
            log("Plan has collision at step " + std::to_string(s) +
                    " / " + std::to_string(numSteps), LogLevel::WARN);
            instance->checkCollision(poses, false, true);
            for (const auto &pose : poses) {
                log(pose, LogLevel::WARN);
            }
            return false;
        }
    }

    return false;
}

bool validateSolution(std::shared_ptr<PlanInstance> instance,
                       const MRTrajectory &solution) 
{
    const int numRobots = instance->getNumberOfRobots();
    int max_count = 0;
    for (int i = 0; i < numRobots; i++) {
        max_count = std::max(max_count, static_cast<int>(solution[i].trajectory.size()));
    }

    MRTrajectory padded(solution);
    for (int i = 0; i < numRobots; ++i) {
        padded[i].robot_id = solution[i].robot_id;
        while (padded[i].trajectory.size() < static_cast<std::size_t>(max_count)) {
            padded[i].trajectory.push_back(padded[i].trajectory.back());
        }
    }

    if (!instance->checkMultiRobotTrajectory(padded, false)) {
        return true;
    }

    for (int s = 0; s < max_count; s++) {
        std::vector<RobotPose> poses;
        poses.reserve(numRobots);
        for (int i = 0; i < numRobots; i++) {
            poses.push_back(padded[i].trajectory[s]);
        }
        if (instance->checkCollision(poses, false)) {
            log("Plan has collision at step " + std::to_string(s) +
                 " / " + std::to_string(max_count), LogLevel::WARN);
            instance->checkCollision(poses, false, true);
            for (const auto &pose : poses) {
                log(pose, LogLevel::WARN);
            }
            return false;
        }
    }

    return false;
}

#if MR_PLANNER_WITH_ROS
bool validateSolution(std::shared_ptr<PlanInstance> instance,
                     const moveit_msgs::RobotTrajectory &plan_traj)
{
    const int numRobots = instance->getNumberOfRobots();
    const std::size_t stepCount = plan_traj.joint_trajectory.points.size();

    MRTrajectory sampled(numRobots);
    for (int i = 0; i < numRobots; ++i) {
        sampled[i].robot_id = i;
    }

    for (std::size_t s = 0; s < stepCount; ++s) {
        int dof_s = 0;
        for (int i = 0; i < numRobots; ++i) {
            RobotPose pose_i = instance->initRobotPose(i);
            for (int d = 0; d < instance->getRobotDOF(i); d++) {
                pose_i.joint_values[d] = plan_traj.joint_trajectory.points[s].positions[dof_s + d];
            }
            sampled[i].trajectory.push_back(pose_i);
            dof_s += instance->getRobotDOF(i);
        }
    }

    if (!instance->checkMultiRobotTrajectory(sampled, false)) {
        return true;
    }

    for (std::size_t s = 0; s < stepCount; ++s) {
        std::vector<RobotPose> poses;
        poses.reserve(numRobots);
        for (int i = 0; i < numRobots; ++i) {
            poses.push_back(sampled[i].trajectory[s]);
        }
        if (instance->checkCollision(poses, false)) {
            log("Plan has collision at step " + std::to_string(s) +
                 " / " + std::to_string(stepCount), LogLevel::WARN);
            instance->checkCollision(poses, false, true);
            for (const auto &pose : poses) {
                log(pose, LogLevel::WARN);
            }
            return false;
        }
    }

    return false;
}

bool optimizeTrajectory(std::shared_ptr<PlanInstance> instance,
                        const moveit_msgs::RobotTrajectory& input_trajectory,
                        const std::string& group_name,
                        robot_model::RobotModelConstPtr robot_model,
                        const ros::NodeHandle& node_handle,
                        moveit_msgs::RobotTrajectory& smoothed_traj
                        )
{
    // Create a planning pipeline instance
    planning_pipeline::PlanningPipelinePtr planning_pipeline(
        new planning_pipeline::PlanningPipeline(robot_model, node_handle, "planning_plugin", "request_adapters"));

    // Set up the planning request
    planning_interface::MotionPlanRequest req;
    req.group_name = group_name;
    req.start_state.is_diff = false;
    req.start_state.joint_state.name = input_trajectory.joint_trajectory.joint_names;
    req.start_state.joint_state.position = input_trajectory.joint_trajectory.points.front().positions;
    req.start_state.joint_state.velocity = input_trajectory.joint_trajectory.points.front().velocities;
    req.start_state.joint_state.effort.resize(input_trajectory.joint_trajectory.joint_names.size(), 0.0);
    req.planner_id = "RRTstar";

    // add attached object
    for (int robot_id = 0; robot_id < instance->getNumberOfRobots(); robot_id++) {
        std::vector<Object> attached_objs = instance->getAttachedObjects(robot_id);
        for (auto &obj : attached_objs) {
            moveit_msgs::AttachedCollisionObject co;
            co.link_name = obj.parent_link;
            co.object.id = obj.name;
            co.object.header.frame_id = obj.parent_link;
            co.object.operation = co.object.ADD;
            req.start_state.attached_collision_objects.push_back(co);
        }
    }

    // set the goal state
    robot_state::RobotState goal_state(robot_model);
    const robot_model::JointModelGroup* joint_model_group = robot_model->getJointModelGroup(group_name);
    goal_state.setJointGroupPositions(joint_model_group, input_trajectory.joint_trajectory.points.back().positions);
    moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group, 0.01, 0.01);

    req.goal_constraints.clear();
    req.goal_constraints.push_back(joint_goal);

    // Set the initial trajectory as a path constraint
    req.reference_trajectories.resize(1);
    req.reference_trajectories[0].joint_trajectory.push_back(input_trajectory.joint_trajectory);

    // Set up the planning context
    planning_interface::MotionPlanResponse res;
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

    // Run the optimization
    bool success = planning_pipeline->generatePlan(planning_scene, req, res);

    if (success)
    {
        // The optimized trajectory is now in res.trajectory_
        res.trajectory_->getRobotTrajectoryMsg(smoothed_traj); 
        smoothed_traj.joint_trajectory.points[0].time_from_start = ros::Duration(0);
        std::vector<RobotPose> last_poses(instance->getNumberOfRobots());
        for (int robot_id = 0; robot_id < instance->getNumberOfRobots(); robot_id++) {
            RobotPose pose = instance->initRobotPose(robot_id);
            for (int d = 0; d < instance->getRobotDOF(robot_id); d++) {
                pose.joint_values[d] = smoothed_traj.joint_trajectory.points[0].positions[d];
            }
            last_poses[robot_id] = pose;
        }
        for (int i = 1; i < smoothed_traj.joint_trajectory.points.size(); i++) {
            int dof_s = 0;
            double max_time = 0;
            for (int robot_id = 0; robot_id < instance->getNumberOfRobots(); robot_id++) {
                RobotPose pose = instance->initRobotPose(robot_id);
                for (int d = 0; d < instance->getRobotDOF(robot_id); d++) {
                    pose.joint_values[d] = smoothed_traj.joint_trajectory.points[i].positions[dof_s + d];
                }

                double dt = instance->computeDistance(pose, last_poses[robot_id]) / instance->getVMax(robot_id);
                max_time = std::max(max_time, dt);

                dof_s += instance->getRobotDOF(robot_id);
                last_poses[robot_id] = pose;
            }
            smoothed_traj.joint_trajectory.points[i].time_from_start = ros::Duration(max_time) + smoothed_traj.joint_trajectory.points[i-1].time_from_start;
        }
        ROS_INFO("Optimized trajectory successfully");
    }
    else
    {
        ROS_ERROR("Failed to optimize trajectory");
    }
    return success;
}
#endif
