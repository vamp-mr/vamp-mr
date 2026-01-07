#include "mr_planner/planning/SingleAgentPlanner.h"
#include "mr_planner/planning/prm.h"
#include "mr_planner/core/logger.h"
#include <algorithm>
#include <limits>
#include <cmath>
#include <functional>
#include <queue>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <vector>

//PRM Class
PRM::PRM(std::shared_ptr<PlanInstance> instance, int robot_id)
    : SingleAgentPlanner(instance, robot_id) {
}

PRM::PRM(std::shared_ptr<PlanInstance> instance, int robot_id, std::shared_ptr<RoadMap> roadmap)
    : SingleAgentPlanner(instance, robot_id), roadmap_obj_(roadmap) {
        roadmap_ = std::make_shared<Graph>(*roadmap->getRoadmap());
}

PRM::PRM(std::shared_ptr<PlanInstance> instance, int robot_id, std::shared_ptr<RoadMap> roadmap, std::shared_ptr<VoxelGrid> voxel_grid)
    : SingleAgentPlanner(instance, robot_id), roadmap_obj_(roadmap), voxel_grid_(voxel_grid) {
        roadmap_ = std::make_shared<Graph>(*roadmap->getRoadmap());
}

void PRM::seedSaltRng(const PlannerOptions &options) {
    std::uint64_t seed_value = 0;
    if (options.rrt_seed >= 0) {
        seed_value = static_cast<std::uint64_t>(options.rrt_seed);
        seed_value = (seed_value << 32) ^ static_cast<std::uint64_t>(robot_id_);
    } else {
        std::random_device rd;
        seed_value = (static_cast<std::uint64_t>(rd()) << 32) ^ static_cast<std::uint64_t>(rd());
    }
    salt_rng_.seed(seed_value);
    salt_rng_seeded_ = true;
}

std::uint64_t PRM::nextSalt() {
    if (!salt_rng_seeded_) {
        std::random_device rd;
        std::uint64_t fallback_seed = (static_cast<std::uint64_t>(rd()) << 32) ^ static_cast<std::uint64_t>(rd());
        salt_rng_.seed(fallback_seed);
        salt_rng_seeded_ = true;
    }
    return salt_rng_();
}

/**
 * @brief Initialize the roadmap with the given options
 */
bool PRM::init(const PlannerOptions &options) {
    if (robot_id_ < 0 || robot_id_ >= instance_->getNumberOfRobots()) {
        std::cout << "Invalid robot id: " << robot_id_ << std::endl;
    }
    if(!instance_) {
        std::cout << "Instance not set" << std::endl;
    }

    num_samples_ = options.num_samples;
    max_dist_ = options.max_dist;

    start_pose_ = instance_->getStartPose(robot_id_);
    goal_pose_ = instance_->getGoalPose(robot_id_);

    return true;
}

bool PRM::plan(const PlannerOptions &options) {
    seedSaltRng(options);
    start_time_ = std::chrono::system_clock::now();
    bool success = true;
    if(options.isRoot) {
        success = updateRoadmap(options);
    }
    return success ? searchPath(options) : false;
}

bool PRM::plan(const PlannerOptions &options, double &lower_bound) {
    bool success = plan(options);
    lower_bound = lower_bound_;
    return success;
}

bool PRM::plan(const PlannerOptions &options, const MRTrajectory &other_solutions) {
    other_solutions_ = other_solutions;
    return plan(options);
}

bool PRM::plan(const PlannerOptions &options, const MRTrajectory &other_solutions, double &lower_bound) {
    other_solutions_ = other_solutions;
    bool success = plan(options);
    lower_bound = lower_bound_;
    return success;
}

bool PRM::getPlan(RobotTrajectory &solution) const {
    solution = solution_;
    return true;
}

bool PRM::updateRoadmap(const PlannerOptions &options) {
    if (!init(options)) {
        solution_.robot_id = robot_id_;
        solution_.times.clear();
        solution_.trajectory.clear();
        solution_.times.push_back(0.0);
        solution_.trajectory.push_back(goal_pose_);
        solution_.cost = 0.0;
        std::cout << "Failed to initialize PRM" << std::endl;
        return false;
    }
    if(swapped_) {
        auto temp = start_pose_;
        start_pose_ = goal_pose_;
        goal_pose_ = temp;
    }


    auto sample_begin = std::chrono::high_resolution_clock::now();
    auto start = roadmap_->addVertex(start_pose_);
    sampleStart(start);
    start_id_ = start->id;

    auto goal = roadmap_->addVertex(goal_pose_);
    sampleGoal(goal);
    goal_id_ = goal->id;
    auto sample_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> sample_time = sample_end - sample_begin;
    // std::cout << "Sample start/goal time: " << sample_time.count() << '\n';

    rrt_sample_time_ = std::chrono::duration<double>::zero();
    if (!startAndGoalConnected()) {
        auto sample_toc = std::chrono::high_resolution_clock::now();
        reSampling(options);
        rrt_sample_time_ = sample_toc - start_time_;

        start_time_ = std::chrono::high_resolution_clock::now();
        if (!startAndGoalConnected()) {
            log("Start and goal are not connected after resampling", LogLevel::ERROR);
            return false;
        }
    }

    auto heu_begin = std::chrono::high_resolution_clock::now();
    computeHeuristic(options);
    auto heu_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> heu_time = heu_end - heu_begin;
    // std::cout << "Heuristic computation time: " << heu_time.count() << '\n';
    return true;
    // visualizeRoadmap();
    // Pause for 300 seconds to allow for visualization
    // std::cout << "Sleeping for 300 seconds to allow for visualization..." << std::endl;
    // std::this_thread::sleep_for(std::chrono::seconds(300));
    // return false;
}

bool PRM::sampleConditionally(std::shared_ptr<Vertex> &new_sample) {
    RobotPose newpose = instance_->initRobotPose(robot_id_);
    bool found_sample = false;
    int tries = 0;
    int max_tries = 10;

    while (!found_sample && tries < max_tries) {
        tries++;
        //log("sample tries: " + std::to_string(tries), LogLevel::DEBUG);
        found_sample = instance_->sample(newpose);
        if (!found_sample) {
            continue;
        }
        new_sample = std::make_shared<Vertex>(newpose);
    }

    return found_sample;
}

void PRM::reSampling(const PlannerOptions &options) {
    auto resampler = std::make_shared<Resampler>(instance_, robot_id_, roadmap_);
    bool success = resampler->resample(options);
    if(!success) {
        log("RRT sampling failed for robot " + std::to_string(robot_id_), LogLevel::WARN);
        return;
    }
    // computeHeuristic(options);
    log("Added RRT path to roadmap for robot " + std::to_string(robot_id_), LogLevel::INFO);
}

void PRM::sampleStart(std::shared_ptr<Vertex> &new_start) {
    std::priority_queue<std::pair<double, std::shared_ptr<Vertex>>, std::vector<std::pair<double, std::shared_ptr<Vertex>>>, CompareEdge> nearest;
    for (auto vertex : roadmap_->vertices) {
        if (validateMotion(vertex, new_start)) {
            nearest.push({instance_->computeDistance(vertex->pose, new_start->pose), vertex});
        }
    }

    int k = roadmap_->num_neighbors;
    while (!nearest.empty() && k > 0) {
        auto neighbor = nearest.top().second;
        nearest.pop();
        roadmap_->addEdge(new_start, neighbor);
        k--;
    }
}


void PRM::sampleGoal(std::shared_ptr<Vertex> &new_goal) {
    std::priority_queue<std::pair<double, std::shared_ptr<Vertex>>, std::vector<std::pair<double, std::shared_ptr<Vertex>>>, CompareEdge> nearest;
    for (auto vertex : roadmap_->vertices) {
        if (validateMotion(vertex, new_goal)) {
            nearest.push({instance_->computeDistance(vertex->pose, new_goal->pose), vertex});
        }
    }

    int k = roadmap_->num_neighbors;
    while (!nearest.empty() && k > 0) {
        auto neighbor = nearest.top().second;
        nearest.pop();
        roadmap_->addEdge(new_goal, neighbor);
        k--;
    }
}


void PRM::computeHeuristic(const PlannerOptions &options) {
    //backward dijkstra to compute heuristic
    heuristic_.clear();
    std::priority_queue<std::pair<double, std::shared_ptr<Vertex>>, std::vector<std::pair<double, std::shared_ptr<Vertex>>>, CompareEdge> pq;
    std::unordered_map<int, double> dist;
    std::vector<bool> visited(roadmap_->size, false);
    for (auto vertex : roadmap_->vertices) {
        dist[vertex->id] = std::numeric_limits<double>::max();
    }

    auto goal = roadmap_->vertices[goal_id_];
    dist[goal->id] = 0;
    pq.push({0, goal});
    while (!pq.empty()) {
        auto [d, vertex] = pq.top();
        pq.pop();
        if (visited[vertex->id]) {
            continue;
        }
        visited[vertex->id] = true;

        auto nbrs_set = roadmap_->getNeighbors(vertex);
        for (auto neighbor : nbrs_set) {
            // PoseEdgeUndirected edge(vertex->pose, neighbor->pose);
            // const auto &edge_opening_map = roadmap_obj_->queryEdgeOpeningMap();
            // if(!dense_roadmap_ && edge_opening_map.find(edge) != edge_opening_map.end() && !edge_opening_map.at(edge)) {
            //     continue;
            // }
            // double new_dist = d + max_dist_;
            double new_dist = d + max_dist_;

            if (new_dist < dist[neighbor->id]) {
                dist[neighbor->id] = new_dist;
                pq.push({new_dist, neighbor});
            }
        }
    }
    heuristic_ = dist;
}

bool PRM::startAndGoalConnected() {
    // return heuristic_[roadmap_->vertices[start_id_]->id] != std::numeric_limits<double>::max();
    return roadmap_->inSameComponent(roadmap_->vertices[start_id_], roadmap_->vertices[goal_id_]);
}

/**
 * @brief Build the roadmap
 */
void PRM::buildRoadmap(const PlannerOptions &options) {
    max_dist_ = options.max_dist;
    num_samples_ = options.num_samples;
    roadmap_ = std::make_shared<Graph>();
    std::vector<std::shared_ptr<Vertex>> neighbors;

    //add vertices and edges
    RobotPose newpose = instance_->initRobotPose(robot_id_);
    while (roadmap_->size < num_samples_ - 2) {
        //RobotPose newpose = instance_->initRobotPose(robot_id_);
        if (instance_->sample(newpose)) {
            
            auto sample = roadmap_->addVertex(newpose);
            neighbors.clear();
            neighbors = roadmap_->vertices;
            
            for (auto neighbor: neighbors) {
                if (validateMotion(sample, neighbor)) {
                    roadmap_->addEdge(sample, neighbor);
                }
            }
            //if(roadmap_->size % 10 == 0)
            //    std::cout << "Roadmap size: " << roadmap_->size << std::endl;
        }

    }
}

/**
 * @brief Validate the motion between two vertices
 * @param v: Vertex u
 * @param neighbor: Vertex v
 */
bool PRM::validateMotion(const std::shared_ptr<Vertex> &u, 
                             const std::shared_ptr<Vertex> &v) {
    return (!(u->pose == v->pose) &&
            instance_->computeDistance(v->pose, u->pose) < max_dist_ && 
            instance_->connect(v->pose, u->pose));
}

/**
 * @brief Query the roadmap
 */
std::shared_ptr<Graph> PRM::queryRoadmap() {
    return roadmap_;
}

bool PRM::searchPath(const PlannerOptions &options) {
    low_level_expansions_ = 0;
    focal_build_time_ = std::chrono::duration<double>::zero();
    collision_check_time_ = std::chrono::duration<double>::zero();
    low_level_time_ = std::chrono::duration<double>::zero();

    struct StatsReporter {
        explicit StatsReporter(PRM *planner) : planner_(planner) {}
        ~StatsReporter() {
            if (planner_) {
                planner_->logSearchStats();
            }
        }
        PRM *planner_;
    } stats_reporter(this);

    struct NodeKey {
        int vertex_id;
        int time_index;
        bool operator==(const NodeKey &other) const {
            return vertex_id == other.vertex_id && time_index == other.time_index;
        }
    };

    struct NodeKeyHash {
        std::size_t operator()(const NodeKey &key) const {
            std::size_t seed = std::hash<int>{}(key.vertex_id);
            seed ^= std::hash<int>{}(key.time_index) + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
            return seed;
        }
    };

    using NodePtr = std::shared_ptr<AStarNode>;

    struct CompareOpenPtr {
        bool operator()(const NodePtr &a, const NodePtr &b) const {
            if (a->f < b->f) {
                return true;
            }
            if (a->f > b->f) {
                return false;
            }
            if (a->h < b->h) {
                return true;
            }
            if (a->h > b->h) {
                return false;
            }
            if (a->salt < b->salt) {
                return true;
            }
            if (a->salt > b->salt) {
                return false;
            }
            if (a->vertex->id < b->vertex->id) {
                return true;
            }
            if (a->vertex->id > b->vertex->id) {
                return false;
            }
            return false;
        }
    };

    struct CompareFocalPtr {
        bool operator()(const NodePtr &a, const NodePtr &b) const {
            if (a->num_conflicts < b->num_conflicts) {
                return true;
            }
            if (a->num_conflicts > b->num_conflicts) {
                return false;
            }
            if (a->f < b->f) {
                return true;
            }
            if (a->f > b->f) {
                return false;
            }
            if (a->h < b->h) {
                return true;
            }
            if (a->h > b->h) {
                return false;
            }
            if (a->salt < b->salt) {
                return true;
            }
            if (a->salt > b->salt) {
                return false;
            }
            if (a->vertex->id < b->vertex->id) {
                return true;
            }
            if (a->vertex->id > b->vertex->id) {
                return false;
            }
            return false;
        }
    };

    std::set<NodePtr, CompareOpenPtr> open_set;
    std::set<NodePtr, CompareFocalPtr> focal_set;
    std::unordered_map<NodeKey, NodePtr, NodeKeyHash> node_lookup;
    std::vector<NodePtr> path;

    const double comparison_epsilon = 1e-9;
    const double time_step = max_dist_ > 1e-9 ? max_dist_ : 1.0;
    auto compute_time_index = [&](double g_val) {
        return static_cast<int>(std::llround(g_val / time_step));
    };

    auto make_key = [&](const std::shared_ptr<Vertex> &vertex, int timestep) {
        return NodeKey{vertex->id, timestep};
    };

    auto remove_from_open = [&](const NodePtr &node) {
        if (node && node->in_open) {
            auto it = open_set.find(node);
            if (it != open_set.end()) {
                open_set.erase(it);
            }
            node->in_open = false;
        }
    };

    auto remove_from_focal = [&](const NodePtr &node) {
        if (node && node->in_focal) {
            auto it = focal_set.find(node);
            if (it != focal_set.end()) {
                focal_set.erase(it);
            }
            node->in_focal = false;
        }
    };

    auto clear_focal = [&]() {
        for (const auto &node : focal_set) {
            node->in_focal = false;
        }
        focal_set.clear();
    };

    auto push_to_open = [&](const NodePtr &node) {
        // remove_from_open(node);
        node->in_open = true;
        open_set.insert(node);
    };

    auto try_insert_focal = [&](const NodePtr &node, double focal_threshold) {
        if (node->in_focal || node->f > focal_threshold + comparison_epsilon) {
            return;
        }
        node->in_focal = true;
        focal_set.insert(node);
    };

    auto peek_open = [&]() -> NodePtr {
        if (open_set.empty()) {
            return nullptr;
        }
        return *open_set.begin();
    };

    auto pop_from_focal = [&]() -> NodePtr {
        if (focal_set.empty()) {
            return nullptr;
        }
        auto it = focal_set.begin();
        NodePtr node = *it;
        focal_set.erase(it);
        node->in_focal = false;
        remove_from_open(node);
        
        return node;
    };

    auto expand_focal_window = [&](double focal_threshold) {
        for (const auto &node : open_set) {
            if (node->f > focal_threshold + comparison_epsilon) {
                break;
            }
            if (!node->in_focal) {
                node->in_focal = true;
                focal_set.insert(node);
            }
        }
    };

    auto invalid_start_constraint = std::any_of(options.constraints.begin(), options.constraints.end(), [&](const Constraint &constraint) {
        return constraint.type == ConstraintType::VERTEX && constraint.robot_id == robot_id_ && constraint.pose == start_pose_ && constraint.time == 0;
    });
    if (invalid_start_constraint) {
        std::cout << "Start pose is in collision with constraint\n";
        return false;
    }

    double start_h = heuristic_[start_id_];
    double start_f = start_h;
    NodePtr start_node = std::make_shared<AStarNode>(roadmap_->vertices[start_id_], start_f, 0.0, start_h);
    start_node->num_conflicts = 0;
    start_node->parent = nullptr;
    start_node->in_open = false;
    start_node->in_focal = false;
    start_node->salt = nextSalt();
    start_node->timestep = 0;
    push_to_open(start_node);
    node_lookup.emplace(make_key(start_node->vertex, start_node->timestep), start_node);

    double min_f_val = 0;
    double focal_threshold = 0;

    auto update_threshold = [&]() {
        NodePtr top = peek_open();
        if (!top) return;
        double new_min_f = top->f;
        if (new_min_f > min_f_val) {
            min_f_val = new_min_f;
            focal_threshold = (1.0 + epsilon_) * min_f_val;
            expand_focal_window(focal_threshold);
        }
        
    };

    update_threshold();

    double min_cost; //earliest time the agent can hold its goal location 
    double max_cost; // the agent must reach before
    double static_timestep; // everything is static after this step
    findCostRange(options, min_cost, max_cost, static_timestep);
    num_col_checks_ = 0;
    bool isMakeSpan = checkIsMakeSpan(options);

    while (!open_set.empty()) {
        low_level_expansions_++;
        NodePtr current;
        
        auto focal_timer = std::chrono::high_resolution_clock::now();
        update_threshold();
        current = pop_from_focal();
        focal_build_time_ += std::chrono::high_resolution_clock::now() - focal_timer;

        if (!current) {
            break;
        }

        if (current->g > max_cost) {
            continue;
        }

        if (current->vertex->pose == goal_pose_ && current->g > min_cost) {
            path.clear();
            for (NodePtr trace = current; trace; trace = trace->parent) {
                path.push_back(trace);
            }
            std::reverse(path.begin(), path.end());

            solution_.robot_id = robot_id_;
            solution_.times.clear();
            solution_.trajectory.clear();
            for (const auto &node : path) {
                solution_.times.push_back(node->g);
                solution_.trajectory.push_back(node->vertex->pose);
            }
            solution_.cost = solution_.times.back();
            solution_.num_col_checks = num_col_checks_;

            lower_bound_ = min_f_val;
            // NodePtr next_best = peek_open();
            // if (!next_best) {
            //     lower_bound_ = current->g;
            // } else {
            //     lower_bound_ = next_best->g + heuristic_[next_best->vertex->id];
            // }

            auto current_time = std::chrono::high_resolution_clock::now();
            low_level_time_ = current_time - start_time_;
            return true;
        }

        if (terminated_) {
            auto current_time = std::chrono::high_resolution_clock::now();
            low_level_time_ = current_time - start_time_;
            log("Low-level search: Terminated PRM search for robot " + std::to_string(robot_id_), LogLevel::DEBUG);
            return false;
        }

        auto current_time = std::chrono::high_resolution_clock::now();
        low_level_time_ = current_time - start_time_;
        if (low_level_time_.count() > max_planning_time_) {
            log("Low-level search: Time limit exceeded for robot " + std::to_string(robot_id_), LogLevel::INFO);
            break;
        }

        auto nbrs_set = roadmap_->getNeighbors(current->vertex);
        std::vector<std::shared_ptr<Vertex>> neighbors(nbrs_set.begin(), nbrs_set.end());
        neighbors.push_back(current->vertex);
        std::sort(neighbors.begin(), neighbors.end(),
            [&](const std::shared_ptr<Vertex>& a, const std::shared_ptr<Vertex>& b){
            double ha = heuristic_[a->id], hb = heuristic_[b->id];
            if (ha != hb) return ha < hb;
            return a->id < b->id;
        });
        
        for (auto neighbor_vertex : neighbors) {
            double tentative_g = current->g + max_dist_;
            double neighbor_h = heuristic_[neighbor_vertex->id];
            double neighbor_f = tentative_g + neighbor_h;
            if (neighbor_f > max_cost) {
                continue;
            }
            int next_timestep = current->timestep + 1;
            if (next_timestep * max_dist_ > static_timestep) {
                // becomes static
                if (current->vertex == neighbor_vertex) {
                    continue; // cannot wait after becomes static
                }
                next_timestep --;
            }


            NodePtr candidate = std::make_shared<AStarNode>(neighbor_vertex, neighbor_f, tentative_g, neighbor_h);
            candidate->parent = current;
            candidate->timestep = next_timestep;
            candidate->num_conflicts = 0;
            candidate->salt = nextSalt();
            if (!checkConstraints(*candidate, options, isMakeSpan)) {
                continue;
            }

            //candidate->num_conflicts = checkNumConflicts(*current, *neighbor_node, tentative_g, options);
            
            focal_timer = std::chrono::high_resolution_clock::now();

            NodeKey key = make_key(neighbor_vertex, next_timestep);
            auto lookup_it = node_lookup.find(key);

            if (lookup_it == node_lookup.end()) {
                push_to_open(candidate);
                node_lookup.emplace(key, candidate);
                try_insert_focal(candidate, focal_threshold);
                
                continue;
            }

            NodePtr neighbor_node = lookup_it->second;

            bool add_to_focal = false;
            bool update_in_focal = false;
            // update the open list if the f improved
            if (candidate->f < neighbor_node->f) {
                remove_from_open(neighbor_node);
                neighbor_node->parent = current;
                neighbor_node->num_conflicts = candidate->num_conflicts;
                neighbor_node->f = candidate->f;
                neighbor_node->g = candidate->g;
                neighbor_node->h = candidate->h;
                push_to_open(neighbor_node);
            }

            // update the focal queue if the number of conflicts is smaller 
            if (candidate->num_conflicts < neighbor_node->num_conflicts) {
                auto focal_update_timer = std::chrono::high_resolution_clock::now();
                if (candidate->f + comparison_epsilon < focal_threshold) {
                    if (!neighbor_node->in_focal) {
                        add_to_focal = true;
                    } else {
                        update_in_focal = true;
                    }
                }

                if (update_in_focal) {
                    remove_from_focal(neighbor_node);
                }

                neighbor_node->parent = current;
                neighbor_node->num_conflicts = candidate->num_conflicts;


                if (add_to_focal) {
                    try_insert_focal(neighbor_node, focal_threshold);
                } else if (update_in_focal) {
                    try_insert_focal(neighbor_node, focal_threshold);
                }

            }
            focal_build_time_ += std::chrono::high_resolution_clock::now() - focal_timer;

        }
    }

    log("Low-level search: Did not find a feasible solution", LogLevel::DEBUG);
    return false;
}




double PRM::getPlanCost() const {
    return solution_.cost;
}

bool PRM::terminate(const PlannerOptions &options) {
    terminated_ = true;
    return true;
}

void PRM::applyDenseMap() {
    dense_roadmap_ = true;
}

void PRM::swapStartGoal() {
    swapped_ = true;
}

bool PRM::checkIsMakeSpan(const PlannerOptions &options) {
    bool isMakeSpan = true;
    double this_cost = 0;
    for(auto other_solution : other_solutions_) {
        if(other_solution.robot_id == robot_id_) {
            this_cost = other_solution.times.back();
            break;
        }
    }
    for(auto other_solution : other_solutions_) {
        if(other_solution.robot_id == robot_id_) {
            continue;
        }
        if(other_solution.times.back() >= this_cost) {
            isMakeSpan = false;
            break;
        }
    }
    return isMakeSpan;
}

void PRM::findCostRange(const PlannerOptions &options, double &minCost, double &maxCost, double &staticCostThresh)
{
    double min_cost = -1;
    double max_cost = std::numeric_limits<double>::max();
    staticCostThresh = 0;
    for(auto constraint : options.constraints) {
        if(constraint.robot_id != robot_id_) {
            continue;
        }

        min_cost = std::max(min_cost, constraint.time);

        if (constraint.type == ConstraintType::LEQLENGTH) {
            max_cost = std::min(max_cost, constraint.time);
        }
        staticCostThresh = std::max(staticCostThresh, constraint.time);
    }
    
    minCost = min_cost;
    maxCost = max_cost;

    for (auto &other_sol : other_solutions_) {
        staticCostThresh = std::max(staticCostThresh, other_sol.times.back());
    }
    staticCostThresh += max_dist_;
}

bool PRM::checkConstraints(const AStarNode &current, const PlannerOptions &options, bool isMakeSpan) {
    bool valid = true;
    for (auto constraint : options.constraints) {
        if (constraint.robot_id != robot_id_ && constraint.type != ConstraintType::LEQLENGTH) {
            continue;
        }
        if (constraint.robot_id == robot_id_ && constraint.type == ConstraintType::LEQLENGTH) {
            continue; //leq constraint should already be factored in maxCost, so no need to cheeck
        }

        if(constraint.isAvoidance) { // avoidance constraint
            if ((constraint.type == ConstraintType::VERTEX && constraint.time == current.g)
                || (constraint.type == ConstraintType::LEQLENGTH && constraint.time == current.g)){
                // auto vertex_key = std::make_pair(current.vertex->pose, constraint.pose);
                // int res = roadmap_obj_->queryVertexCollisionMap(vertex_key);
                // if(res == 1) {
                //     valid = false;
                //     break;
                // } else if(res == 0) {
                //     continue;
                // }
                num_col_checks_++;
                auto cc_start = std::chrono::high_resolution_clock::now();
                bool in_collision = instance_->checkCollision({current.vertex->pose, constraint.pose}, true);
                collision_check_time_ += std::chrono::high_resolution_clock::now() - cc_start;
                if(in_collision) {
                    valid = false;
                    // roadmap_obj_->updateVertexCollisionMap(vertex_key, true);
                    break;
                }
            } else if((constraint.type == ConstraintType::EDGE && current.parent && constraint.time == current.parent->g) // edge avoidance
                || (constraint.type == ConstraintType::LEQLENGTH && constraint.time < current.g)) { // or avoid goal
                int num_interpolations = options.num_interpolations;
                // PoseEdge edge_i = PoseEdge(current.parent->vertex->pose, current.vertex->pose);
                // PoseEdge edge_j = PoseEdge(constraint.pose, constraint.to_pose);
                // auto edge_key = std::make_pair(edge_i, edge_j);
                // int res = roadmap_obj_->queryEdgeCollisionMap(edge_key);
                // if(res == 1) {
                //     valid = false;
                //     break;
                // } else if(res == 0) {
                //     continue;
                // }
                num_col_checks_++;
                const std::vector<RobotPose> motion_start = {constraint.pose, current.parent->vertex->pose};
                const std::vector<RobotPose> motion_goal = {constraint.to_pose, current.vertex->pose};
                const double step_size = instance_->computeMotionStepSize(motion_start, motion_goal, num_interpolations);
                auto cc_start = std::chrono::high_resolution_clock::now();
                bool in_collision = instance_->checkMultiRobotMotion(motion_start, motion_goal, step_size, true);
                collision_check_time_ += std::chrono::high_resolution_clock::now() - cc_start;
                if (in_collision) {
                    valid = false;
                    // roadmap_obj_->updateEdgeCollisionMap(edge_key, true);
                } 
                // else {
                //     roadmap_obj_->updateEdgeCollisionMap(edge_key, false);
                // }
            }
        } else { // vertex or edge constraint
            if(constraint.type == ConstraintType::VERTEX && constraint.time == current.g && constraint.pose == current.vertex->pose) {
                valid = false;
                break;
            } else if(constraint.type == ConstraintType::EDGE && current.parent && constraint.time == current.parent->g 
                && constraint.pose == current.parent->vertex->pose && constraint.to_pose == current.vertex->pose) {
                valid = false;
                break;
            }
        }

        if(!valid) {
            break;
        }
    }
    return valid;
}

void PRM::addConstraints(PlannerOptions &options, const Conflict &conflict) {
    Constraint newConstraint;
    if(conflict.isTarget) {
        newConstraint.type = ConstraintType::VERTEX;
    } else {
        newConstraint.type = conflict.type;
    }
    newConstraint.robot_id = robot_id_;
    if(conflict.isTarget) {
        newConstraint.time = other_solutions_[conflict.robots[1]].times[conflict.time_idx[0]];
        newConstraint.pose = solution_.trajectory.back();
    } else {
        newConstraint.time = solution_.times[conflict.time_idx[0]];
        newConstraint.pose = solution_.trajectory[conflict.time_idx[0]];
    }
    if(newConstraint.type == ConstraintType::EDGE) {
        newConstraint.to_pose = solution_.trajectory[conflict.time_idx[0] + 1];
    }
    options.constraints.push_back(newConstraint);
}

int PRM::checkNumConflicts(
    const AStarNode &current,
    const AStarNode &neighbor,
    double tentative_g,
    const PlannerOptions &options) 
{
    int num_conflicts = 0;

    int other_size = static_cast<int>(other_solutions_.size());
    double max_dist = max_dist_; // local copy

    for (int i = 0; i < other_size; ++i) {
        const auto &other_solution = other_solutions_[i];
        if (other_solution.robot_id == robot_id_) 
            continue;

        int time_idx = tentative_g / max_dist;

        if (time_idx < static_cast<int>(other_solution.times.size())) {
            // Vertex check
            // auto vertex_key = std::make_pair(neighbor.vertex->pose,
            //                                  other_solution.trajectory[time_idx]);
            // int res_v = roadmap_obj_->queryVertexCollisionMap(vertex_key);

            // // Edge check
            // PoseEdge edge_i(current.vertex->pose, neighbor.vertex->pose);
            // PoseEdge edge_j(other_solution.trajectory[time_idx - 1],
            //                 other_solution.trajectory[time_idx]);
            // auto edge_key = std::make_pair(edge_i, edge_j);
            // int res_e = roadmap_obj_->queryEdgeCollisionMap(edge_key);

            // bool results_found = false;
            // if (res_v == 1) { num_conflicts++; results_found = true; }
            // if (res_e == 1) { num_conflicts++; results_found = true; }
            // if (res_v == 0 && res_e == 0) {
            //     results_found = true;
            // }
            // if (results_found) continue;

            const std::vector<RobotPose> motion_start = {
                current.vertex->pose,
                other_solution.trajectory[time_idx - 1]
            };
            const std::vector<RobotPose> motion_goal = {
                neighbor.vertex->pose,
                other_solution.trajectory[time_idx]
            };
            const double step_size = instance_->computeMotionStepSize(motion_start, motion_goal, options.num_interpolations);
            auto cc_start = std::chrono::high_resolution_clock::now();
            const bool collision_found = instance_->checkMultiRobotMotion(motion_start, motion_goal, step_size, true);
            collision_check_time_ += std::chrono::high_resolution_clock::now() - cc_start;
            num_col_checks_++;

            if (collision_found) {
                num_conflicts++;
                auto cc_goal_start = std::chrono::high_resolution_clock::now();
                const bool goal_collision = instance_->checkCollision(motion_goal, true);
                collision_check_time_ += std::chrono::high_resolution_clock::now() - cc_goal_start;
                // if (goal_collision) {
                //     roadmap_obj_->updateVertexCollisionMap(vertex_key, true);
                // } else {
                //     roadmap_obj_->updateEdgeCollisionMap(edge_key, true);
                // }
            } 
            // else {
            //     roadmap_obj_->updateVertexCollisionMap(vertex_key, false);
            //     roadmap_obj_->updateEdgeCollisionMap(edge_key, false);
            // }

        } else {
            // Past trajectory length, check target map
            // auto vertex_key = std::make_pair(neighbor.vertex->pose,
            //                                  other_solution.trajectory.back());
            // int res_v = roadmap_obj_->queryVertexCollisionMap(vertex_key);

            // PoseEdge edge_i(current.vertex->pose, neighbor.vertex->pose);
            // auto target_key = std::make_pair(other_solution.trajectory.back(), edge_i);
            // int res_t = roadmap_obj_->queryTargetCollisionMap(target_key);
            // bool results_found = false;
            // if (res_v == 1) { num_conflicts++; results_found = true; }
            // if (res_t == 1) { num_conflicts++; results_found = true; }
            // if (res_v == 0 && res_t == 0) {
            //     results_found = true;
            // }
            // if (results_found) continue;

            const std::vector<RobotPose> motion_start = {
                current.vertex->pose,
                other_solution.trajectory.back()
            };
            const std::vector<RobotPose> motion_goal = {
                neighbor.vertex->pose,
                other_solution.trajectory.back()
            };
            const double step_size = instance_->computeMotionStepSize(motion_start, motion_goal, options.num_interpolations);
            auto cc_start = std::chrono::high_resolution_clock::now();
            const bool collision_found = instance_->checkMultiRobotMotion(motion_start, motion_goal, step_size, true);
            collision_check_time_ += std::chrono::high_resolution_clock::now() - cc_start;
            num_col_checks_++;

            if (collision_found) {
                num_conflicts++;
                auto cc_goal_start = std::chrono::high_resolution_clock::now();
                const bool goal_collision = instance_->checkCollision(motion_goal, true);
                collision_check_time_ += std::chrono::high_resolution_clock::now() - cc_goal_start;
                // if (goal_collision) {
                //     roadmap_obj_->updateVertexCollisionMap(vertex_key, true);
                // } else {
                //     roadmap_obj_->updateTargetCollisionMap(target_key, true);
                // }
            } 
            // else {
            //     roadmap_obj_->updateVertexCollisionMap(vertex_key, false);
            //     roadmap_obj_->updateTargetCollisionMap(target_key, false);
            // }
        }

    }

    return num_conflicts;
}

double PRM::computeEdgeWorkspaceProximityHeuristic(const AStarNode &current,
                                               const AStarNode &neighbor,
                                               const PlannerOptions &options) {
    double total_score = 0.0;
    int num_interpolations = options.num_interpolations;

    double time = current.g;
    for (int i = 1; i <= num_interpolations; ++i) {
        double alpha = static_cast<double>(i) / (double)num_interpolations;
        RobotPose interp_pose = instance_->interpolate(current.vertex->pose, neighbor.vertex->pose, alpha);
        Eigen::Vector3d interp_ee = instance_->getEndEffectorPositionFromPose(interp_pose);

        double pose_score = 0.0;

        for (const auto &other : other_solutions_) {
            if (other.robot_id == robot_id_) continue;

            int t_idx = static_cast<int>(std::floor(neighbor.g / max_dist_));
            if (t_idx < 0 || t_idx >= static_cast<int>(other.trajectory.size())) continue;

            RobotPose other_pose = instance_->interpolate(other.trajectory[t_idx - 1], other.trajectory[t_idx], alpha);
            Eigen::Vector3d other_ee = instance_->getEndEffectorPositionFromPose(other_pose);

            double dist = (interp_ee - other_ee).norm();
            // std::cout << "dist: " << dist << std::endl;

            double score = std::exp(-dist / 2);  // adjust kernel width as needed
            pose_score += score;
        }

        total_score = std::max(total_score, pose_score);
    }

    return total_score;
}

double PRM::computeEdgeJointSpaceHeuristic(const AStarNode &current,
                                       const AStarNode &neighbor,
                                       const PlannerOptions &options) {
    double total_score = 0.0;
    int num_interpolations = options.num_interpolations;

    double time = current.g;
    for (int i = 1; i <= num_interpolations; ++i) {
        double alpha = static_cast<double>(i) / (num_interpolations + 1);
        RobotPose interp_pose = instance_->interpolate(current.vertex->pose, neighbor.vertex->pose, alpha);

        double pose_score = 0.0;

        for (const auto &other : other_solutions_) {
            if (other.robot_id == robot_id_) continue;

            int t_idx = static_cast<int>(std::floor(neighbor.g / max_dist_));
            if (t_idx < 0 || t_idx >= static_cast<int>(other.trajectory.size())) continue;

            const RobotPose &other_pose = instance_->interpolate(other.trajectory[t_idx - 1], other.trajectory[t_idx], alpha);

            double sq_dist = 0.0;
            for (size_t j = 0; j < interp_pose.joint_values.size(); ++j) {
                double diff = interp_pose.joint_values[j] - other_pose.joint_values[j];
                sq_dist += diff * diff;
            }

            double score = std::exp(-sq_dist / 10.0);  // adjust kernel width as needed
            pose_score += score;
        }

        total_score = std::max(total_score, pose_score);
    }

    return total_score;
}


void PRM::logSearchStats() const {
    std::string stats_msg = "PRM low-level search stats - expansions: " + std::to_string(low_level_expansions_)
        + ", low-level time: " + std::to_string(low_level_time_.count() * 1e3) + "ms"
        + ", focal build time: " + std::to_string(focal_build_time_.count() * 1e6) + "us"
        + ", collision check time: " + std::to_string(collision_check_time_.count() * 1e6) + "us"
        + ", RRT sample time: " + std::to_string(rrt_sample_time_.count() * 1e3) + "ms";
    log(stats_msg, LogLevel::DEBUG);
}
