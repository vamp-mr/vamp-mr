
#include "mr_planner/planning/resampler.h"
#include "mr_planner/core/logger.h"

// Resampler class
Resampler::Resampler(std::shared_ptr<PlanInstance> instance, int robot_id, std::shared_ptr<Graph> roadmap)
    : instance_(instance), roadmap_(roadmap), robot_id_(robot_id) {
    planner_ = std::make_shared<RRTConnect>(instance, robot_id);
}

bool Resampler::resample(const PlannerOptions &options) {
    /* Plan using RRTConnect and add the trajectory to the roadmap
    bool success = planner_->plan(options);
    if(!success) {
        return false;
    }
    RobotTrajectory solution;
    planner_->getPlan(solution);
    addTrajToRoadmap(solution, options);
    */
    // Plan for 10 times in 10 threads, each thread owns a planner instance
    const bool deterministic = options.rrt_seed >= 0;
    const int attempts = num_threads_;
    std::vector<RobotTrajectory> solutions(attempts);

    if (deterministic) {
        for (int i = 0; i < attempts; ++i) {
            auto planner = std::make_shared<RRTConnect>(instance_, robot_id_);
            PlannerOptions options_copy = options;
            options_copy.rrt_seed = options.rrt_seed + i;
            bool success = planner->plan(options_copy);
            if (success) {
                planner->getPlan(solutions[i]);
            } else {
                solutions[i] = RobotTrajectory();
            }
        }
    } else {
        std::vector<std::thread> threads;
        threads.reserve(attempts);
        for (int i = 0; i < attempts; ++i) {
            threads.emplace_back([this, &options, &solutions, i]() {
                auto planner = std::make_shared<RRTConnect>(instance_, robot_id_);
                bool success = planner->plan(options);
                if (success) {
                    planner->getPlan(solutions[i]);
                } else {
                    solutions[i] = RobotTrajectory(); // empty trajectory on failure
                }
            });
        }
        for (auto &thread : threads) {
            thread.join();
        }
    }
    // Add all successful trajectories to the roadmap
    for (const auto &solution : solutions) {
        addTrajToRoadmap(solution, options);
    }

    /* Enlarge the edge radius of roadmap and add more edges
    std::cout << "Resampling done, enlarging the roadmap edges..." << std::endl;
    for (const auto &vertex : roadmap_->vertices) {
        std::priority_queue<std::pair<double, std::shared_ptr<Vertex>>, std::vector<std::pair<double, std::shared_ptr<Vertex>>>, CompareEdge> nearest;
        for (auto neighbor : roadmap_->vertices) {
            double dist = instance_->computeDistance(vertex->pose, neighbor->pose);
            if(vertex == neighbor || roadmap_->getNeighbors(vertex).find(neighbor) != roadmap_->getNeighbors(vertex).end()) {
                continue;
            }
            nearest.push(std::make_pair(dist, neighbor));
        }

        int k = roadmap_->num_neighbors;
        while (!nearest.empty() && k > 0) {
            auto neighbor = nearest.top().second;
            nearest.pop();
            auto options_copy = options;
            options_copy.max_dist = 2 * options.max_dist;
            if (validateMotion(vertex, neighbor, options_copy) && roadmap_->getNeighbors(neighbor).size() < roadmap_->num_neighbors) {
                roadmap_->addEdge(vertex, neighbor);
                k--;
            }
        }
    }
    std::cout << "Roadmap edges enlarged." << std::endl;
    */
    return true;
}

bool Resampler::resample(const PlannerOptions &options, const MRTrajectory &other_solutions) {
    bool success = planner_->plan(options, other_solutions);
    if(!success) {
        return false;
    }
    RobotTrajectory solution;
    planner_->getPlan(solution);
    addTrajToRoadmap(solution, options);

    return true;
}

void Resampler::addTrajToRoadmap(const RobotTrajectory &solution, const PlannerOptions &options) {
    if (solution.trajectory.empty()) {
        return;
    }
    if(solution.trajectory.size() <= 2) {
        // If the trajectory is too short, we cannot add it to the roadmap
        log("Trajectory too short to add to roadmap for robot " + std::to_string(robot_id_), LogLevel::WARN);
        return;
    }
    if(!(solution.trajectory.front() == instance_->getStartPose(robot_id_)) || !(solution.trajectory.back() == instance_->getGoalPose(robot_id_))) {
        log("Trajectory start or end does not match the instance start or goal pose for robot " + std::to_string(robot_id_), LogLevel::WARN);
        return;
    }
    // add the trajectory to the roadmap
    auto last_vertex = roadmap_->getVertex(solution.trajectory.front());
    for (int j = 1; j < solution.trajectory.size() - 1; j++) {
        // add the pose to the roadmap
        auto new_vertex = roadmap_->addVertex(solution.trajectory[j]);
        // add the edge to the roadmap
        roadmap_->addEdge(last_vertex, new_vertex);
        // add other edges - the nearest 25 neighbors 
        std::priority_queue<std::pair<double, std::shared_ptr<Vertex>>, std::vector<std::pair<double, std::shared_ptr<Vertex>>>, CompareEdge> nearest;
        for (auto vertex : roadmap_->vertices) {
            double dist = instance_->computeDistance(new_vertex->pose, vertex->pose);
            nearest.push(std::make_pair(dist, vertex));
        }
        
        int k = roadmap_->num_neighbors;
        while (!nearest.empty() && k > 0) {
            auto neighbor = nearest.top().second;
            nearest.pop();
            if (validateMotion(new_vertex, neighbor, options)) {
                roadmap_->addEdge(new_vertex, neighbor);
                k--;
            }
        }
        last_vertex = new_vertex;
    }
    roadmap_->addEdge(last_vertex, roadmap_->getVertex(solution.trajectory.back()));
}

bool Resampler::validateMotion(const std::shared_ptr<Vertex> &u, const std::shared_ptr<Vertex> &v, const PlannerOptions &options) {
    return (!(u->pose == v->pose) &&
            instance_->computeDistance(v->pose, u->pose) < options.max_dist && 
            instance_->connect(v->pose, u->pose));
}
