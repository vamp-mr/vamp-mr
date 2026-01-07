#include "mr_planner/planning/roadmap.h"
#include "mr_planner/core/logger.h"

RoadMap::RoadMap(std::shared_ptr<PlanInstance> instance, int robot_id)
    : instance_(instance), robot_id_(robot_id) {
}

void RoadMap::setInstance(std::shared_ptr<PlanInstance> instance) {
    instance_ = instance;
}

void RoadMap::setBuildEnabled(bool enabled) {
    build_enabled_ = enabled;
}

/* SPARS Roadmap 
void RoadMap::buildRoadmap() {
    roadmap_ = std::make_shared<Graph>();
    prm_star_ = std::make_shared<Graph>();

    int failures = 0;

    while(failures < max_failures_) {
        std::shared_ptr<Vertex> newSample;
        if(sampleConditionally(newSample)) {
            // PRMStar
            addSampleToPRMStar(newSample);
            computeRepresentitive(newSample->pose);

            // SPARS
            std::vector<std::shared_ptr<Vertex>> visible_guards = findVisibleGuards(newSample);
            // Guard
            bool added = addGuard(newSample, visible_guards);
            // Connectivity
            if(!added) {
                added = addConnector(newSample, visible_guards);
            }
            // Interface
            if(!added) {
                added = addInterface(newSample, visible_guards);
            }
            // Shortcut
            if(!added) {
                added = addShortcut(newSample, visible_guards);
            }

            if(!added) {
                failures++;
            } else {
                updateRepresentitiveForAll(roadmap_->vertices.back());
            }
        }
        // std::cout << "Sampled and processed one configuration. Roadmap size: " << roadmap_->size << '\n';
        // Report progress for every 100 samples
        if(roadmap_->size % 100 == 0 && roadmap_->size > 0) {
            std::cout << "SPARS Roadmap size: " << roadmap_->size << ", Failures: " << failures << std::endl;
        }
    }
    // connectAllEdges();
    analyzePoseDistribution();
    std::cout << "SPARS roadmap size: " << roadmap_->size << std::endl;
} */

void RoadMap::buildRoadmap() {
    roadmap_ = std::make_shared<Graph>();

    if (!build_enabled_) {
        return;
    }

    while (roadmap_->size < num_samples_) {
        std::shared_ptr<Vertex> newSample;
        if (sampleConditionally(newSample)) {
            auto sample = roadmap_->addVertex(newSample->pose);
            auto neighbors = roadmap_->vertices;
            std::priority_queue<std::pair<double, std::shared_ptr<Vertex>>, std::vector<std::pair<double, std::shared_ptr<Vertex>>>, CompareEdge> nearest;
            for (auto neighbor : neighbors) {
                double dist = instance_->computeDistance(sample->pose, neighbor->pose);
                nearest.push(std::make_pair(dist, neighbor));
            }

            int k = roadmap_->num_neighbors;
            while (!nearest.empty() && k > 0) {
                auto neighbor = nearest.top().second;
                nearest.pop();
                if (validateMotion(sample, neighbor) && roadmap_->getNeighbors(neighbor).size() < roadmap_->num_neighbors) {
                    roadmap_->addEdge(sample, neighbor);
                    k--;
                    //std::cout << "Adding edge between " << sample->id << " and " << neighbor->id << std::endl;
                }
            }
            //std::cout << "# of neighbors: " << roadmap_->getNeighbors(sample).size() << std::endl;
            //instance_->plotEE(newSample->pose, roadmap_->size);

            //if (roadmap_->size % 10 == 0)
            //    std::cout << "Roadmap size: " << roadmap_->size << std::endl;
        }
    }
    log("Roadmap size: " + std::to_string(roadmap_->size), LogLevel::INFO);

    analyzePoseDistribution();
}

// SPARS
bool RoadMap::connect(const RobotPose &u, const RobotPose &v) {
    auto it = environment_collision_map_.find(std::make_pair(u, v));
    if(it != environment_collision_map_.end()) {
        return !it->second;
    }
    bool collision = !instance_->connect(u, v);
    updateEnvironmentCollisionMap(std::make_pair(u, v), collision);
    return !collision;
}

std::vector<std::shared_ptr<Vertex>> RoadMap::findVisibleGuards(const std::shared_ptr<Vertex> &new_sample) {
    std::vector<std::shared_ptr<Vertex>> visible_guards;
    for(const auto &vertex : roadmap_->vertices) {
        if(instance_->computeDistance(new_sample->pose, vertex->pose) < max_dist_ &&
           connect(new_sample->pose, vertex->pose)) {
            visible_guards.push_back(vertex);
        }
    }
    return visible_guards;
}

bool RoadMap::addGuard(const std::shared_ptr<Vertex> &new_sample, const std::vector<std::shared_ptr<Vertex>> &visible_guards) {
    if(visible_guards.empty()) {
        auto sample = roadmap_->addVertex(new_sample->pose);
        return true;
    }
    return false;
}

bool RoadMap::addConnector(const std::shared_ptr<Vertex> &new_sample, const std::vector<std::shared_ptr<Vertex>> &visible_guards) {
    for(int i = 0; i < visible_guards.size(); i++) {
        for(int j = i + 1; j < visible_guards.size(); j++) {
            auto u = visible_guards[i];
            auto v = visible_guards[j];
            if(!connected(u, v)) {
                auto sample = roadmap_->addVertex(new_sample->pose);
                roadmap_->addEdge(sample, u);
                roadmap_->addEdge(sample, v);
                PoseEdgeUndirected edge_1(sample->pose, u->pose);
                PoseEdgeUndirected edge_2(sample->pose, v->pose);
                updateEdgeOpeningMap(edge_1, true);
                updateEdgeOpeningMap(edge_2, true);
                return true;
            }
        }
    }
    return false;
}

bool RoadMap::addInterface(const std::shared_ptr<Vertex> &new_sample, const std::vector<std::shared_ptr<Vertex>> &visible_guards) {
    // Find nearest guards within max_dist_ ignoring obstacles
    std::priority_queue<std::pair<double, std::shared_ptr<Vertex>>, std::vector<std::pair<double, std::shared_ptr<Vertex>>>, CompareEdge> nearest_guards;
    for(const auto &vertex : roadmap_->vertices) {
        double dist = instance_->computeDistance(new_sample->pose, vertex->pose);
        if(dist < max_dist_) {
            nearest_guards.push(std::make_pair(dist, vertex));
        }
    }
    if(nearest_guards.size() > 1) {
        auto guard_1 = nearest_guards.top().second;
        nearest_guards.pop();
        auto guard_2 = nearest_guards.top().second;
        if(connect(new_sample->pose, guard_1->pose) &&
            connect(new_sample->pose, guard_2->pose) &&
            roadmap_->getNeighbors(guard_1).find(guard_2) == roadmap_->getNeighbors(guard_1).end()) {
            if(connect(guard_1->pose, guard_2->pose)) {
                roadmap_->addEdge(guard_1, guard_2);
                PoseEdgeUndirected edge(guard_1->pose, guard_2->pose);
                updateEdgeOpeningMap(edge, true);
            } else {
                auto sample = roadmap_->addVertex(new_sample->pose);
                roadmap_->addEdge(sample, guard_1);
                roadmap_->addEdge(sample, guard_2);
                PoseEdgeUndirected edge_1(sample->pose, guard_1->pose);
                PoseEdgeUndirected edge_2(sample->pose, guard_2->pose);
                updateEdgeOpeningMap(edge_1, true);
                updateEdgeOpeningMap(edge_2, true);
                return true;
            }
        }
    }
    return false;
}

bool RoadMap::addShortcut(const std::shared_ptr<Vertex> &new_sample, const std::vector<std::shared_ptr<Vertex>> &visible_guards) {
    auto v = computeRepresentitive(new_sample->pose);
    if(v == nullptr) {
        std::cout << "No valid representitve found, this should be a bug.\n";
        return false;
    }
    std::shared_ptr<Vertex> q;
    for(const auto &vertex : prm_star_->vertices) {
        if(vertex->pose == new_sample->pose) {
            q = vertex;
            break;
        }
    }
    auto neighbors = prm_star_->getNeighbors(q);
    // Representitives of neighbors of the dense roadmap
    std::vector<std::shared_ptr<Vertex>> representitives; 
    for(const auto &neighbor : neighbors) {
        auto r = computeRepresentitive(neighbor->pose);
        if(r != nullptr && r != v && std::find(representitives.begin(), representitives.end(), r) == representitives.end()) {
            representitives.push_back(r);
        }
    }
    std::cout << "Found " << representitives.size() << " representitives to check for shortcutting.\n";

    bool added = false;
    for(const auto &v1 : representitives) {
        for(const auto &v2 : roadmap_->vertices) {
            if(v1->pose == v2->pose) continue;
            if(roadmap_->getNeighbors(v).find(v2) == roadmap_->getNeighbors(v).end() || 
                roadmap_->getNeighbors(v1).find(v2) != roadmap_->getNeighbors(v1).end() || 
                !share_interface(v, v2)) {
                continue;
            }

            std::cout << "Trying to shortcut between representitives.\n";
            auto pi_s = max_spanner_path(v, v1, v2);
            std::cout << "Max spanner path length: " << pi_s.size() << "\n";
            auto support_poses = interface_support(v, v2);
            std::cout << "Found " << support_poses.size() << " support poses for the interface.\n";
            auto pi_d = computeShortestPath(new_sample, support_poses);
            std::cout << "Shortest path length: " << pi_d.size() << "\n";
            double len_s = 0;
            for(int i = 1; i < pi_s.size(); i++) {
                len_s += instance_->computeDistance(pi_s[i-1]->pose, pi_s[i]->pose);
            }
            double len_d = 0;
            for(int i = 1; i < pi_d.size(); i++) {
                len_d += instance_->computeDistance(pi_d[i-1]->pose, pi_d[i]->pose);
            }

            if(t * len_d < len_s) {
                if(connect(v1->pose, v2->pose)) {
                    std::cout << "Successfully connected " << v1->id << " and " << v2->id << " in the roadmap.\n";
                    roadmap_->addEdge(v1, v2);
                    PoseEdgeUndirected edge(v1->pose, v2->pose);
                    updateEdgeOpeningMap(edge, true);
                } else {
                    std::cout << "Could not connect " << v1->id << " and " << v2->id << " directly, adding the shortcut path instead.\n";
                    // Add the path v1 -> pi_d -> v2 to the roadmap
                    addPath(v1, pi_d, v2);
                }
                added = true;
            } else {
                std::cout << "No shortcut found between " << v1->id << " and " << v2->id << ".\n";
            }
        }
    }
    return added;
}

std::shared_ptr<Vertex> RoadMap::computeRepresentitive(const RobotPose &pose) {
    if(representitive_map_.find(pose) != representitive_map_.end()) {
        return representitive_map_[pose];
    }
    double min_dist = std::numeric_limits<double>::max();
    std::shared_ptr<Vertex> representitive = nullptr;
    for(const auto &vertex : roadmap_->vertices) {
        if(pose == vertex->pose)
            return vertex;
        double dist = instance_->computeDistance(pose, vertex->pose);
        if(dist < min_dist && dist < max_dist_ && connect(pose, vertex->pose)) {
            min_dist = dist;
            representitive = vertex;
        }
        // std::cout << "Iteration " << &vertex - &roadmap_->vertices[0] << " out of " << roadmap_->vertices.size() << " in computeRepresentitive.\n";
    }
    if(representitive != nullptr) {
        updateRepresentitiveMap(pose, representitive);
    }
    return representitive;
}

void RoadMap::updateRepresentitiveForAll(std::shared_ptr<Vertex> &sample) {
    for(const auto &v : prm_star_->vertices) {
        if(representitive_map_.find(v->pose) == representitive_map_.end()) {
            continue;
        }
        double previous_best = instance_->computeDistance(v->pose, representitive_map_[v->pose]->pose);
        double dist = instance_->computeDistance(sample->pose, v->pose);
        if(dist < previous_best) {
            updateRepresentitiveMap(v->pose, sample);
        }
    }
}

bool RoadMap::share_interface(const std::shared_ptr<Vertex> &u, const std::shared_ptr<Vertex> &v) {
    auto densemap_vertices = prm_star_->vertices;
    for(int i = 0; i < densemap_vertices.size(); i++) {
        for(int j = i + 1; j < densemap_vertices.size(); j++) {
            if(std::find(prm_star_->getNeighbors(densemap_vertices[i]).begin(), prm_star_->getNeighbors(densemap_vertices[i]).end(), densemap_vertices[j]) == prm_star_->getNeighbors(densemap_vertices[i]).end()) {
                continue;
            }
            auto r1 = computeRepresentitive(densemap_vertices[i]->pose);
            auto r2 = computeRepresentitive(densemap_vertices[j]->pose);
            if((r1 == u && r2 == v) || (r1 == v && r2 == u)) {
                std::cout << "Yes!\n";
                return true;
            }
        }
    }
    std::cout << "Unfortunately no.\n";
    return false;
}

std::vector<std::shared_ptr<Vertex>> RoadMap::max_spanner_path(const std::shared_ptr<Vertex> &v, const std::shared_ptr<Vertex> &v1, const std::shared_ptr<Vertex> &v2) {
    std::vector<std::vector<std::shared_ptr<Vertex>>> paths;
    paths.push_back({v1, v, v2});
    for(const auto &x : roadmap_->getNeighbors(v)) {
        auto x_neighbors = roadmap_->getNeighbors(x);
        if(std::find(x_neighbors.begin(), x_neighbors.end(), v2) != x_neighbors.end() && 
           std::find(x_neighbors.begin(), x_neighbors.end(), v1) == x_neighbors.end()) {
            paths.push_back({v1, v, x});
        }
    }

    // Return the longest path
    double max_length = 0;
    std::vector<std::shared_ptr<Vertex>> longest_path;
    for(const auto &path : paths) {
        double length = 0;
        for(int i = 1; i < path.size(); i++) {
            length += instance_->computeDistance(path[i-1]->pose, path[i]->pose);
        }
        if(length > max_length) {
            max_length = length;
            longest_path = path;
        }
    }
    return longest_path;
}

std::vector<std::shared_ptr<Vertex>> RoadMap::interface_support(const std::shared_ptr<Vertex> &v, const std::shared_ptr<Vertex> &v1) {
    std::vector<std::shared_ptr<Vertex>> support_poses;
    for(const auto &vertex : prm_star_->vertices) {
        auto r = computeRepresentitive(vertex->pose);
        if(r != v) continue;

        for(const auto &other : prm_star_->vertices) {
            if(other == vertex) continue;
            auto r2 = computeRepresentitive(other->pose);
            if(r2 == v1 && connect(vertex->pose, other->pose)) {
                support_poses.push_back(vertex);
            }
        }
    }
    return support_poses;
}

std::vector<std::shared_ptr<Vertex>> RoadMap::computeShortestPath(const std::shared_ptr<Vertex> &pose, const std::vector<std::shared_ptr<Vertex>> &support_poses) {
    // From shortest path between pose to all the support_poses using AStar search, take the shortest one
    std::vector<std::shared_ptr<Vertex>> shortest_path;
    double min_cost = std::numeric_limits<double>::max();
    for(const auto &support : support_poses) {
        // Astar
        std::priority_queue<AStar, std::vector<AStar>, CompareAStar> open;
        std::unordered_set<int> closed;

        // AStar start(pose, 0, instance_->computeDistance(pose->pose, support->pose), nullptr);
        AStar start;
        start.vertex = pose;
        start.g = 0;
        start.h = instance_->computeDistance(pose->pose, support->pose);
        start.parent = nullptr;
        open.push(start);

        while(!open.empty()) {
            auto current = open.top();
            open.pop();
            if(current.vertex == support) {
                // Reconstruct path
                std::vector<std::shared_ptr<Vertex>> path;
                auto temp = current;
                while(temp.parent != nullptr) {
                    path.push_back(temp.vertex);
                    temp = *temp.parent;
                }
                path.push_back(temp.vertex);
                std::reverse(path.begin(), path.end());
                double cost = 0;
                for(int i = 1; i < path.size(); i++) {
                    cost += instance_->computeDistance(path[i-1]->pose, path[i]->pose);
                }
                if(cost < min_cost) {
                    min_cost = cost;
                    shortest_path = path;
                }
                break;
            }
            closed.insert(current.vertex->id);
            for(const auto &neighbor : prm_star_->getNeighbors(current.vertex)) {
                if(closed.find(neighbor->id) != closed.end()) continue;
                double g = current.g + instance_->computeDistance(current.vertex->pose, neighbor->pose);
                double h = instance_->computeDistance(neighbor->pose, support->pose);
                // AStar neighbor_node(neighbor, g, h, std::make_shared<AStar>(current));
                AStar neighbor_node;
                neighbor_node.vertex = neighbor;
                neighbor_node.g = g;
                neighbor_node.h = h;
                neighbor_node.parent = std::make_shared<AStar>(current);
                open.push(neighbor_node);
            }
        }
    }
    return shortest_path;
}

void RoadMap::addPath(const std::shared_ptr<Vertex> &v1, const std::vector<std::shared_ptr<Vertex>> pi_d, const std::shared_ptr<Vertex> &v2) { // Needs optimization -> try to smooth the whole path to make the added vertices as few as possible
    std::shared_ptr<Vertex> prev = v1;
    for(int i = 0; i < pi_d.size(); i++) {
        auto sample = roadmap_->addVertex(pi_d[i]->pose);
        roadmap_->addEdge(prev, sample);
        PoseEdgeUndirected edge(prev->pose, sample->pose);
        updateEdgeOpeningMap(edge, true);
        prev = sample;
    }
    roadmap_->addEdge(prev, v2);
    PoseEdgeUndirected edge(prev->pose, v2->pose);
    updateEdgeOpeningMap(edge, true);
}

// PRM*
void RoadMap::addSampleToPRMStar(const std::shared_ptr<Vertex> &new_sample) {
    auto sample = prm_star_->addVertex(new_sample->pose);
    int n = prm_star_->vertices.size();
    double radius = eta_ * std::pow((log(n) / n), 1.0 / new_sample->pose.joint_values.size());
    // Find neighbors within epsilon radius
    for(const auto &vertex : prm_star_->vertices) {
        if(vertex == sample) {
            continue;
        }
        double dist = instance_->computeDistance(sample->pose, vertex->pose);
        if(!(sample->pose == vertex->pose) && dist < radius && connect(sample->pose, vertex->pose)) {
            prm_star_->addEdge(sample, vertex);
        }
    }
}

void RoadMap::connectAllEdges() {
    int k = roadmap_->num_neighbors;
    int n = roadmap_->vertices.size();

    for (int i = 0; i < n; i++) {
        if(i % 10 == 0)
            std::cout << "Connecting edges for vertex " << i << std::endl;
        auto sample = roadmap_->vertices[i];

        // Find k nearest
        std::priority_queue<std::pair<double, std::shared_ptr<Vertex>>,
            std::vector<std::pair<double, std::shared_ptr<Vertex>>>,
            CompareEdge> nearest;

        for (int j = 0; j < i; j++) {
            double dist = instance_->computeDistance(sample->pose, roadmap_->vertices[j]->pose);
            nearest.emplace(dist, roadmap_->vertices[j]);
        }

        int remaining = k;
        while (!nearest.empty() && remaining > 0) {
            auto neighbor = nearest.top().second;
            nearest.pop();

            if (validateMotion(sample, neighbor) && roadmap_->getNeighbors(neighbor).size() < k) {
                roadmap_->addEdge(sample, neighbor);
                remaining--;
                PoseEdgeUndirected edge(sample->pose, neighbor->pose);
                auto edge_opening = queryEdgeOpeningMap().find(edge);
                if (edge_opening == queryEdgeOpeningMap().end()) {
                    updateEdgeOpeningMap(edge, false);
                }
            }
        }
    }
}

bool RoadMap::connected(const std::shared_ptr<Vertex> &u, const std::shared_ptr<Vertex> &v) {
    // See whether there is a path in the roadmap
    std::unordered_set<std::shared_ptr<Vertex>> visited;
    std::queue<std::shared_ptr<Vertex>> to_visit;
    to_visit.push(u);
    while (!to_visit.empty()) {
        auto current = to_visit.front();
        to_visit.pop();
        if (current == v) {
            return true;
        }
        visited.insert(current);
        for (const auto &neighbor : roadmap_->getNeighbors(current)) {
            if (visited.find(neighbor) == visited.end()) {
                to_visit.push(neighbor);
            }
        }
    }
    return false;
}

/* Uniform Roadmap
void RoadMap::buildRoadmap() {
    roadmap_ = std::make_shared<Graph>();

    while (roadmap_->size < num_samples_) {
        std::shared_ptr<Vertex> newSample;
        if (sampleConditionally(newSample)) {
            auto neighbors = roadmap_->vertices;
            std::priority_queue<std::pair<double, std::shared_ptr<Vertex>>, std::vector<std::pair<double, std::shared_ptr<Vertex>>>, CompareEdge> nearest;
            bool valid = true;
            int neighbor_count = 0;
            for (auto neighbor : neighbors) {
                double dist = instance_->computeDistance(newSample->pose, neighbor->pose);
                nearest.push(std::make_pair(dist, neighbor));
                if (dist < max_dist_) {
                    neighbor_count++;
                }
            }
            valid = neighbor_count <= roadmap_->num_neighbors;
            if(!valid) {
                continue;
            }
            auto sample = roadmap_->addVertex(newSample->pose);

            int k = roadmap_->num_neighbors;
            while (!nearest.empty() && k > 0) {
                auto neighbor = nearest.top().second;
                nearest.pop();
                if (validateMotion(sample, neighbor) && roadmap_->getNeighbors(neighbor).size() < k) {
                    roadmap_->addEdge(sample, neighbor);
                    k--;
                    //std::cout << "Adding edge between " << sample->id << " and " << neighbor->id << std::endl;
                }
            }
            //std::cout << "# of neighbors: " << roadmap_->getNeighbors(sample).size() << std::endl;
            //instance_->plotEE(newSample->pose, roadmap_->size);

            if (roadmap_->size % 10 == 0)
                std::cout << "Roadmap size: " << roadmap_->size << std::endl;
        }
    }

    analyzePoseDistribution();
}
*/

bool RoadMap::sampleConditionally(std::shared_ptr<Vertex> &new_sample) {
    RobotPose newpose = instance_->initRobotPose(robot_id_);
    bool found_sample = false;
    int tries = 0;
    int max_tries = 10;

    while (!found_sample && tries < max_tries) {
        tries++;
        found_sample = instance_->sample(newpose);
        if (!found_sample) {
            continue;
        }
        new_sample = std::make_shared<Vertex>(newpose);
    }

    return found_sample;
}

bool RoadMap::validateMotion(const std::shared_ptr<Vertex> &u, const std::shared_ptr<Vertex> &v) {
    return (!(u->pose == v->pose) &&
            instance_->computeDistance(v->pose, u->pose) < 2 * max_dist_ && 
            instance_->connect(v->pose, u->pose));
}

std::shared_ptr<Graph> RoadMap::getRoadmap() {
    return roadmap_;
}

void RoadMap::updateVertexCollisionMap(const std::pair<RobotPose, RobotPose> &vertex_pair, bool collision) {
    boost::unique_lock<boost::shared_mutex> lock(vertex_map_mutex_);
    vertex_collision_map_[vertex_pair] = collision;
}

void RoadMap::updateEdgeCollisionMap(const std::pair<PoseEdge, PoseEdge> &edge_pair, bool collision) {
    boost::unique_lock<boost::shared_mutex> lock(edge_map_mutex_);
    edge_collision_map_[edge_pair] = collision;
}

void RoadMap::updateTargetCollisionMap(const std::pair<RobotPose, PoseEdge> &vertex_edge_pair, bool collision) {
    boost::unique_lock<boost::shared_mutex> lock(target_map_mutex_);
    vertex_edge_map_[vertex_edge_pair] = collision;
}

void RoadMap::updateEdgeOpeningMap(const PoseEdgeUndirected &edge, bool opening) {
    edge_opening_map_[edge] = opening;
}

void RoadMap::updateRepresentitiveMap(const RobotPose &pose, const std::shared_ptr<Vertex> &representitive) {
    representitive_map_[pose] = representitive;
}

void RoadMap::updateEnvironmentCollisionMap(const std::pair<RobotPose, RobotPose> &vertex_pair, bool collision) {
    environment_collision_map_[vertex_pair] = collision;
}

int RoadMap::queryVertexCollisionMap(const std::pair<RobotPose, RobotPose> &vertex_pair) const {
    boost::shared_lock<boost::shared_mutex> lock(vertex_map_mutex_);
    auto it = vertex_collision_map_.find(vertex_pair);
    if (it != vertex_collision_map_.end()) {
        return it->second ? 1 : 0;
    }
    return -1; // unknown
}

int RoadMap::queryEdgeCollisionMap(const std::pair<PoseEdge, PoseEdge> &edge_pair) const {
    boost::shared_lock<boost::shared_mutex> lock(edge_map_mutex_);
    auto it = edge_collision_map_.find(edge_pair);
    if (it != edge_collision_map_.end()) {
        return it->second ? 1 : 0;
    }
    return -1; // unknown
}

int RoadMap::queryTargetCollisionMap(const std::pair<RobotPose, PoseEdge> &vertex_edge_pair) const {
    boost::shared_lock<boost::shared_mutex> lock(target_map_mutex_);
    auto it = vertex_edge_map_.find(vertex_edge_pair);
    if (it != vertex_edge_map_.end()) {
        return it->second ? 1 : 0;
    }
    return -1; // unknown
}

const std::unordered_map<PoseEdgeUndirected, bool, PoseEdgeUndirectedHash>& RoadMap::queryEdgeOpeningMap() const {
    return edge_opening_map_;
}

std::unordered_map<RobotPose, int, RobotPoseHash> RoadMap::getStartGoalMap() {
    return start_goal_map_;
}

void RoadMap::updateStartGoalMap(const RobotPose &pose, int id) {
    start_goal_map_[pose] = id;
}

void RoadMap::analyzePoseDistribution() const {
    if (!roadmap_ || roadmap_->vertices.empty()) {
        std::cout << "No vertices in roadmap to analyze." << std::endl;
        return;
    }

    size_t dof = roadmap_->vertices.front()->pose.joint_values.size();
    size_t n = roadmap_->vertices.size();
    std::vector<std::vector<double>> joint_data(dof);

    // Collect joint values
    for (const auto &vertex : roadmap_->vertices) {
        const auto &joints = vertex->pose.joint_values;
        for (size_t i = 0; i < dof; ++i) {
            joint_data[i].push_back(joints[i]);
        }
    }

    std::cout << "\n=== Joint Value Distribution (Robot ID " << robot_id_ << ") ===\n";

    for (size_t i = 0; i < dof; ++i) {
        const auto &vals = joint_data[i];
        double min_val = *std::min_element(vals.begin(), vals.end());
        double max_val = *std::max_element(vals.begin(), vals.end());
        double mean = std::accumulate(vals.begin(), vals.end(), 0.0) / vals.size();
        double var = 0.0;
        for (double v : vals) var += (v - mean) * (v - mean);
        var /= vals.size();

        std::cout << "Joint " << i << ": "
                  << "min=" << std::setprecision(4) << min_val << ", "
                  << "max=" << max_val << ", "
                  << "mean=" << mean << ", "
                  << "std=" << std::sqrt(var) << "\n";

        // Generate histogram
        const int num_bins = 10;
        std::vector<int> bin_counts(num_bins, 0);
        double bin_width = (max_val - min_val) / num_bins;
        bin_width = std::max(bin_width, 1e-3);  // avoid zero width

        for (double v : vals) {
            int bin_idx = std::min(int((v - min_val) / bin_width), num_bins - 1);
            bin_counts[bin_idx]++;
        }

        std::cout << "Histogram:\n";
        for (int b = 0; b < num_bins; ++b) {
            double bin_start = min_val + b * bin_width;
            double bin_end   = bin_start + bin_width;
            std::cout << "[" << std::fixed << std::setprecision(2)
                      << bin_start << ", " << bin_end << "): ";

            int stars = bin_counts[b];
            for (int s = 0; s < stars * 40 / n; ++s) std::cout << "*";  // normalize to max 40 stars
            std::cout << " (" << bin_counts[b] << ")\n";
        }

        std::cout << "----------------------------------------\n";
    }
}


void RoadMap::printToFile(const std::string& filename) const {
    if (!roadmap_) {
        std::cerr << "No roadmap to print!\n";
        return;
    }

    std::ofstream out(filename);
    if (!out.is_open()) {
        std::cerr << "Failed to open file: " << filename << "\n";
        return;
    }

    out << "# NODES\n";
    for (const auto& v : roadmap_->vertices) {
        out << v->id;
        // print 7 joint values
        out << std::fixed << std::setprecision(6);  // consistent precision
        Eigen::Vector3d ee_pos = instance_->getEndEffectorPositionFromPose(v->pose);
        out << " " << ee_pos.x() << " " << ee_pos.y() << " " << ee_pos.z();
        out << "\n";
    }

    out << "# EDGES\n";
    for (size_t i = 0; i < roadmap_->adjList.size(); ++i) {
        for (const auto& neighbor : roadmap_->adjList[i]) {
            if (i < static_cast<size_t>(neighbor->id)) {
                out << i << " " << neighbor->id << "\n";
            }
        }
    }

    out.close();
    std::cout << "Roadmap written to " << filename << "\n";
}

//RoadMap Class
