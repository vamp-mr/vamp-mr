#include "mr_planner/execution/adg.h"
#include "mr_planner/core/logger.h"
#include "mr_planner/execution/policy.h"

#include <algorithm>
#include <cmath>
#include <utility>
#include <unordered_set>

namespace tpg {

namespace
{
    constexpr std::size_t kType2SweepWindow = 8;

    using SweepSegment = std::vector<NodePtr>;
    using EdgeRequest = std::pair<NodePtr, NodePtr>;

    auto collectSegment(NodePtr start, NodePtr stop) -> std::vector<NodePtr>
    {
        std::vector<NodePtr> segment;
        for (auto node = start; node && segment.size() < kType2SweepWindow; node = node->Type1Next)
        {
            segment.push_back(node);
            if (node == stop)
            {
                break;
            }
        }
        return segment;
    }

    auto buildSegments(NodePtr start, NodePtr stop) -> std::vector<SweepSegment>
    {
        std::vector<SweepSegment> segments;
        NodePtr node = start;
        while (node && node->timeStep <= stop->timeStep) 
        {
            auto segment = collectSegment(node, stop);

            if (!segment.empty())
            {
                node = segment.back()->Type1Next;
                segments.push_back(std::move(segment));
            }
        }
        return segments;
    }
}

ShortcutSamplerADG::ShortcutSamplerADG(const TPGConfig &config, std::shared_ptr<ActivityGraph> act_graph,
                    const std::vector<std::vector<NodePtr>> &intermediate_nodes)
    : ShortcutSampler(config)
{
    act_graph_ = act_graph;
    intermediate_nodes_ = intermediate_nodes;
    skip_home_ = !config.sync_task;
}

void ShortcutSamplerADG::init(const std::vector<NodePtr> &start_nodes,
    const std::vector<int> &numNodes, const std::vector<std::vector<int>> &earliest_t,
    const std::vector<std::vector<NodePtr>> &timed_nodes)
{
    num_robots_ = start_nodes.size();
    timed_nodes_ = timed_nodes;
    nodes_.clear();
    act_ids_.clear();
    act_lengths_.clear();

    nodes_.resize(num_robots_);
    act_ids_.resize(num_robots_);
    act_lengths_.resize(num_robots_);

    // save all the nodes, and activity ids, and activity lengths

    for (int i = 0; i < num_robots_; i++) {
        auto node = start_nodes[i];
        int j = 0;

        std::vector<NodePtr> nodes_i;
        std::vector<int> act_ids_i;
        int act_id = 0;

        while (node != nullptr) {
            // add node
            nodes_i.push_back(node);

            // add activity id
            auto act_i = act_graph_->get(i, act_id);
            // we skip home activity
            while (skip_home_&& act_id < act_graph_->num_activities(i) - 1  && act_i->is_skippable()) {
                act_id++;
                act_i = act_graph_->get(i, act_id);
            }
            act_ids_i.push_back(act_id);

            // check if we need to switch to next activity
            if (intermediate_nodes_[i][act_id * 2 + 1]->timeStep == node->timeStep) {
                act_id++;
            }

            // iterate to next node
            j++;
            node = node->Type1Next;
        }
        nodes_[i] = nodes_i;
        act_ids_[i] = act_ids_i;

        act_lengths_[i].resize(act_graph_->num_activities(i), 0);
        for (int act_id : act_ids_[i]) {
            act_lengths_[i][act_id]++;
        }

    }
    numNodes_ = numNodes;

    resetFailedShortcuts();
}


bool ShortcutSamplerADG::sampleUniform(Shortcut &shortcut) {
    int i = std::rand() % num_robots_;

    int startNode = std::rand() % numNodes_[i];
    int act_id = act_ids_[i][startNode];
    int act_length = 0;
    shortcut.activity = act_graph_->get(i, act_id);

    // for (int j = 0; j <= act_id; j++) {
    //     act_length += act_lengths_[i][j];
    // }

    // if (startNode >= act_length - 2) {
    //     return false;
    // }
    // int length = std::rand() % (act_length - startNode - 2) + 2;
    // int endNode = p_ + length;
    for (int j = 0; j < act_id; j++) {
        act_length += act_lengths_[i][j];
    }
    int endNode = act_length + std::rand() % act_lengths_[i][act_id];
    if (endNode < startNode) {
        std::swap(startNode, endNode);
    }
    if (startNode + 1 >= endNode) {
        return false;
    }

    shortcut.ni = nodes_[i][startNode];
    shortcut.nj = nodes_[i][endNode];
    assert(shortcut.activity != nullptr);

    log("Sampled shortcut from robot " + std::to_string(i) + " activity " + shortcut.activity->type_string() + 
        " timestep " + std::to_string(shortcut.ni.lock()->timeStep) +
        " to timestep " + std::to_string(shortcut.nj.lock()->timeStep), LogLevel::DEBUG);
    
    return true;
}

bool ShortcutSamplerADG::sampleComposite(Shortcut &shortcut) {
    return sampleUniform(shortcut);
}

ShortcutIteratorADG::ShortcutIteratorADG(const TPGConfig &config, std::shared_ptr<ActivityGraph> act_graph,
                    const std::vector<std::vector<NodePtr>> &intermediate_nodes)
    : ShortcutIterator(config)
{
    act_graph_ = act_graph;
    skip_home_ = !config.sync_task;

    for (int i = 0; i < act_graph->num_robots(); i++) {
        std::vector<NodePtr> seg_node;
        int act_id = 0;
        while (act_id < act_graph->num_activities(i)) {
            seg_node.push_back(intermediate_nodes[i][act_id * 2]);
            while (skip_home_ && act_id < act_graph->num_activities(i) - 1 && act_graph->get(i, act_id)->is_skippable()) {
                act_id++;
            }
            seg_node.push_back(intermediate_nodes[i][act_id * 2 + 1]);
            act_id++;
        }
        seg_nodes_.push_back(seg_node);
    }
}

void ShortcutIteratorADG::init(const std::vector<NodePtr> &start_nodes,
                        const std::vector<int> &numNodes,
                        const std::vector<NodePtr> &end_nodes)
{
    // init the queue
    q_i = std::queue<NodePtr>();
    q_j = std::queue<NodePtr>();
    num_robots_ = start_nodes.size();
    start_nodes_ = start_nodes;
    end_nodes_ = end_nodes;
    numNodes_ = numNodes;

    // find all the node's start segment node and end segment node
    for (int i = 0; i < num_robots_; i++) {
        for (int seg_id = 0; seg_id < seg_nodes_[i].size()/2; seg_id ++) {
            auto node_i = seg_nodes_[i][seg_id*2];
            if (skip_home_ && act_graph_->get(i, node_i->actId)->is_skippable()) {
                if (backward_doubleloop) {
                    q_i.push(node_i);
                    q_j.push(seg_nodes_[i][seg_id*2+1]);
                }
                else {
                    q_i.push(node_i);
                    q_j.push(node_i->Type1Next);
                }
                rob_seg_idx.push(std::make_pair(i, seg_id));
                std::cout << node_i->timeStep << " " << seg_nodes_[i][seg_id*2+1]->timeStep << "\n";
            }   
        }
    }
    std::cout << "finished initializing shortcut iterator\n";
}

bool ShortcutIteratorADG::step_begin(Shortcut &shortcut) {
    NodePtr node_i = q_i.front();
    NodePtr node_j = q_j.front();
    int robot_id = rob_seg_idx.front().first;
    int seg_id = rob_seg_idx.front().second;
    q_i.pop();
    q_j.pop();
    rob_seg_idx.pop();
    
    auto seg_start = seg_nodes_[robot_id][seg_id*2];
    auto seg_end = seg_nodes_[robot_id][seg_id*2+ 1];

    if (backward_doubleloop) {
        if (node_i->timeStep == node_j->timeStep) {
            q_i.push(seg_start);
            q_j.push(seg_end);
            log("robot " + std::to_string(robot_id) + 
                " seg " + std::to_string(seg_id) + " restart outer loop", LogLevel::DEBUG);
            rob_seg_idx.push(std::make_pair(robot_id, seg_id));
            return false;
        }
        else if (node_i->timeStep + 1 == node_j->timeStep) {
            q_i.push(node_i->Type1Next);
            q_j.push(seg_end);
            log("robot " + std::to_string(robot_id) + 
                " seg " + std::to_string(seg_id) + " restart inner loop", LogLevel::DEBUG);
            rob_seg_idx.push(std::make_pair(robot_id, seg_id));
            return false;
        }
       
    }
    else {
        //std::cout << "robot " << robot_id << " seg " << seg_id << " " 
        //    << node_i->timeStep << " " << node_j->timeStep << " " << seg_end->timeStep << "\n";
        if (node_j->timeStep >= seg_end->timeStep) {
            if (node_i->timeStep < seg_end->timeStep - 1) {
                q_i.push(node_i->Type1Next);
                q_j.push(node_i->Type1Next->Type1Next);
                rob_seg_idx.push(std::make_pair(robot_id, seg_id));
                log("robot " + std::to_string(robot_id) + 
                " seg " + std::to_string(seg_id) + " restart inner loop", LogLevel::DEBUG);
            }
            else {
                q_i.push(seg_start);
                q_j.push(seg_start->Type1Next);
                rob_seg_idx.push(std::make_pair(robot_id, seg_id));
                log("robot " + std::to_string(robot_id) +
                " seg " + std::to_string(seg_id) + " restart outer loop", LogLevel::DEBUG);
            }
            return false;
        }
    }

    shortcut.ni = node_i;
    shortcut.nj = node_j;
    int act_id = node_i->actId;
    while (skip_home_ && act_id < act_graph_->num_activities(robot_id) - 1 && act_graph_->get(robot_id, act_id)->is_skippable()) {
        act_id++;
    }
    shortcut.activity = act_graph_->get(robot_id, act_id);
    rob_seg_idx.push(std::make_pair(robot_id, seg_id));
    return true;
}

void ShortcutIteratorADG::step_end(const Shortcut &shortcut) {
    ShortcutIterator::step_end(shortcut);
}


void ADG::reset()
{
    TPG::reset();
    intermediate_nodes_.clear();
}

ADG::ADG(std::shared_ptr<ActivityGraph> activity_graph) : act_graph_(activity_graph) {
    num_robots_ = act_graph_->num_robots();
    start_nodes_.resize(num_robots_);
    end_nodes_.resize(num_robots_);
    intermediate_nodes_.resize(num_robots_);
    numNodes_.resize(num_robots_, 0);
    solution_.resize(num_robots_);
    exec_start_act_.resize(num_robots_, 0);
}

bool ADG::init_from_asynctrajs(std::shared_ptr<PlanInstance> instance, const TPGConfig &config, const MRTrajectory &trajectories) {
    if (!instance) {
        log("ADG::init_from_asynctrajs requires a valid PlanInstance", LogLevel::ERROR);
        return false;
    }
    instance_ = instance;
    dt_ = config.dt;
    config_ = config;
    
    instance->resetScene(true);
    
    std::vector<std::vector<int>> reached_t;
    reached_t.resize(num_robots_);

    auto t_start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < num_robots_; i++) {
        std::vector<NodePtr> nodes_i;
        std::vector<NodePtr> inter_nodes;
       
        for (int idx = 0; idx < trajectories[i].times.size(); idx++) {
            auto node = std::make_shared<Node>(i, idx);
            node->pose = trajectories[i].trajectory[idx];
            node->actId = trajectories[i].act_ids[idx];
            nodes_i.push_back(node);
            if (idx == 0) {
                int act_id_begin = trajectories[i].act_ids.front();
                for (int j = 0; j < act_id_begin; j++) {
                    inter_nodes.push_back(node);
                    inter_nodes.push_back(node);
                }
                inter_nodes.push_back(node);
            }
            else {
                if (trajectories[i].act_ids[idx - 1] != trajectories[i].act_ids[idx]) {
                    inter_nodes.push_back(nodes_i[idx-1]);
                    int skipped_act_count = trajectories[i].act_ids[idx] - trajectories[i].act_ids[idx - 1] - 1;
                    for (int j = 0; j < skipped_act_count; j++) {
                        inter_nodes.push_back(node);
                        inter_nodes.push_back(node);
                    }
                    inter_nodes.push_back(node);
                }
            }
            // times[] may have tiny floating point error relative to dt_ (especially when reconstructed from JSON).
            // Use rounding instead of truncation to avoid off-by-one reached_t values that can introduce cycles.
            const double scaled = trajectories[i].times[idx] / dt_;
            reached_t[i].push_back(static_cast<int>(std::lround(scaled)));
        }
        inter_nodes.push_back(nodes_i.back());
        
        if (!config_.sync_task && !config_.preserve_skippable_acts) {
            // remove repeatedly waiting nodes in home and home_handover
            int num_acts = act_graph_->num_activities(i);
            for (int j = num_acts - 1; j >= 1; j--) {
                auto act = act_graph_->get(i, j);
                if (!act || !act->is_skippable()) {
                    continue;
                }
                if (!act->type2_next.empty() || !act->type2_prev.empty()) {
                    continue;
                }
                const int idx_a = j * 2;
                const int idx_b = j * 2 + 1;
                if (idx_b >= static_cast<int>(inter_nodes.size()) || idx_a < 0) {
                    continue;
                }

                const int stepa = inter_nodes[static_cast<std::size_t>(idx_a)]->timeStep;
                const int stepb = inter_nodes[static_cast<std::size_t>(idx_b)]->timeStep;
                if (stepa < 0 || stepb < stepa || stepb >= static_cast<int>(nodes_i.size())) {
                    continue;
                }
                if ((stepb - stepa) <= 1) {
                    act_graph_->remove_act(i, j);
                    nodes_i.erase(nodes_i.begin() + stepa, nodes_i.begin() + stepb + 1);
                    inter_nodes.erase(inter_nodes.begin() + idx_a, inter_nodes.begin() + idx_a + 2);
                    reached_t[i].erase(reached_t[i].begin() + stepa, reached_t[i].begin() + stepb + 1);
                }
            }
        }

        int act_id = 0;
        const int num_acts = act_graph_->num_activities(i);
        for (int j = 0; j < static_cast<int>(nodes_i.size()); ++j)
        {
            nodes_i[j]->Type1Prev = (j > 0) ? nodes_i[static_cast<std::size_t>(j - 1)] : nullptr;
            nodes_i[j]->Type1Next =
                (j + 1 < static_cast<int>(nodes_i.size())) ? nodes_i[static_cast<std::size_t>(j + 1)] : nullptr;
            nodes_i[j]->timeStep = j;

            if (act_id >= num_acts)
            {
                act_id = std::max(0, num_acts - 1);
            }
            nodes_i[j]->actId = act_id;

            while (act_id < num_acts && (act_id * 2 + 1) < static_cast<int>(inter_nodes.size()) &&
                   nodes_i[j] == inter_nodes[static_cast<std::size_t>(act_id * 2 + 1)])
            {
                ++act_id;
            }
        }

        numNodes_[i] = nodes_i.size();
        start_nodes_[i] = nodes_i.front();
        end_nodes_[i] = nodes_i.back();
        intermediate_nodes_[i] = inter_nodes;
    
    }

    if (addTaskDeps() == false) {
        if (hasCycle()) {
            log("Naive TPG already has cycle after adding task deps", LogLevel::ERROR);
            return false;
        }
        return false;
    }

    // add collision dependencies type 2 edges
    if (findCollisionDeps(instance, reached_t, false) == false) {
        return false;
    }
    
    t_init_ = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - t_start).count() * 1e-6;
    t_start = std::chrono::high_resolution_clock::now();

    transitiveReduction();
    t_simplify_ = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - t_start).count() * 1e-6; 

    if (hasCycle()) {
        log("Naive TPG already has cycle", LogLevel::ERROR);
        return false;
    }

    int numtype2edges = getTotalType2Edges(); 
    log("ADG initialized with " + std::to_string(getTotalNodes()) + " nodes and " + std::to_string(numtype2edges) + " type 2 edges in "
        + std::to_string(t_init_ + t_simplify_) + "s", LogLevel::HLINFO);

    findFlowtimeMakespan(pre_shortcut_flowtime_, pre_shortcut_makespan_);
    post_shortcut_flowtime_ = pre_shortcut_flowtime_;
    post_shortcut_makespan_ = pre_shortcut_makespan_;

    computePathLength(instance);
    double nowait_time = 0;
    for (int i = 0; i < num_robots_; i++) {
        nowait_time += end_nodes_[i]->timeStep * config_.dt;
    }
    wait_time_ = pre_shortcut_flowtime_ - nowait_time;
    log("Flowtime: " + std::to_string(pre_shortcut_flowtime_) + " Makespan: " + std::to_string(pre_shortcut_makespan_)
        + " Wait Time " + std::to_string(wait_time_), LogLevel::INFO);

    return true;
    
}

bool ADG::init_from_tpgs(std::shared_ptr<PlanInstance> instance, const TPGConfig &config, const std::vector<std::shared_ptr<TPG>> &tpgs) {
    if (!instance) {
        log("ADG::init_from_tpgs requires a valid PlanInstance", LogLevel::ERROR);
        return false;
    }
    instance_ = instance;
    dt_ = config.dt;
    config_ = config;
    
    instance->resetScene(true);

    auto t_start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < num_robots_; i++) {
        std::vector<NodePtr> inter_nodes;
        std::vector<double> hand_values(instance->getHandDOF(i), 0.0);
        int tpg_id = 0;
        for (int act_id = 0; act_id < act_graph_->num_activities(i); act_id++) {
            std::shared_ptr<const Activity> act = act_graph_->get(i, act_id);
            NodePtr inter_start_node;
            NodePtr inter_end_node;

            if (act->type == Activity::Type::open_gripper || act->type == Activity::Type::close_gripper) {
                // these tasks do not have a trajectory, so we just set the start node and end node as before
                if (inter_nodes.empty()) {
                    log("gripper open/close task should not be the first task", LogLevel::ERROR);
                    return false;
                }
                auto last_node = inter_nodes.back();
                inter_start_node = std::make_shared<Node>(i, last_node->timeStep + 1);
                inter_start_node->actId = act_id;
                inter_start_node->pose = last_node->pose;
                hand_values = act->end_pose.hand_values;
                inter_start_node->pose.hand_values = hand_values;
                inter_end_node = inter_start_node;
                numNodes_[i]++;
            }
            else {
                inter_start_node = tpgs[tpg_id]->getStartNode(i);
                inter_end_node = tpgs[tpg_id]->getEndNode(i);

                NodePtr iter_node = inter_start_node;
                while (iter_node != inter_end_node) {
                    iter_node->timeStep += numNodes_[i];
                    iter_node->actId = act_id;
                    iter_node->pose.hand_values = hand_values;
                    iter_node = iter_node->Type1Next;
                }
                inter_end_node->actId = act_id;
                inter_end_node->timeStep += numNodes_[i];
                inter_end_node->pose.hand_values = hand_values;

                numNodes_[i] += tpgs[tpg_id]->getNumNodes(i);
                tpg_id++;
            }
            if (act_id > 0) {
                inter_nodes.back()->Type1Next = inter_start_node;
                inter_start_node->Type1Prev = inter_nodes.back();
            }

            inter_nodes.push_back(inter_start_node);
            inter_nodes.push_back(inter_end_node);

        }
        intermediate_nodes_[i] = inter_nodes;
        start_nodes_[i] = inter_nodes.front();
        end_nodes_[i] = inter_nodes.back();
    }
    
    if (addTaskDeps() == false) {
        return false;
    }

    // recompute the time step for each node
    std::vector<std::vector<int>> reached_t;
    std::vector<int> reached_end;
    int num_act = act_graph_->num_activities(0);
    for (int i = 0; i < num_robots_; i++) {
        if (num_act != act_graph_->num_activities(i)) {
            log("number of activities for each robot should be the same", LogLevel::ERROR);
            return false;
        }
    }
    findEarliestReachTimeSyncAct(num_act, reached_t, reached_end);
    
    // add collision dependencies type 2 edges
    if (findCollisionDeps(instance, reached_t, true) == false) {
        return false;
    }
    
    t_init_ = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - t_start).count() * 1e-6;
    t_start = std::chrono::high_resolution_clock::now();

    transitiveReduction();
    t_simplify_ = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - t_start).count() * 1e-6; 

    if (hasCycle()) {
        log("Naive TPG already has cycle", LogLevel::ERROR);
        return false;
    }

    int numtype2edges = getTotalType2Edges(); 
    log("ADG initialized with " + std::to_string(getTotalNodes()) + " nodes and " + std::to_string(numtype2edges) + " type 2 edges in "
        + std::to_string(t_init_ + t_simplify_) + "s", LogLevel::HLINFO);

    findFlowtimeMakespan(pre_shortcut_flowtime_, pre_shortcut_makespan_);
    computePathLength(instance);
    double nowait_time = 0;
    for (int i = 0; i < num_robots_; i++) {
        nowait_time += end_nodes_[i]->timeStep * config_.dt;
    }
    wait_time_ = pre_shortcut_flowtime_ - nowait_time;

    log("Flowtime: " + std::to_string(pre_shortcut_flowtime_) + " Makespan: " + std::to_string(pre_shortcut_makespan_), LogLevel::INFO);

    return true;
}

bool ADG::addTaskDeps() {
    // add task dependencies type 2 edges
    idType2Edges_ = 10000;
    for (int i = 0; i < num_robots_; i++) {
        std::cout << "robot " << i << " num activities " << act_graph_->num_activities(i) << "\n";
        for (int act_id = 0; act_id < act_graph_->num_activities(i); act_id++) {
            std::shared_ptr<const Activity> act = act_graph_->get(i, act_id);
            if (act == nullptr) {
                log("activity is null, robot " + std::to_string(i) + " act " + std::to_string(act_id), LogLevel::ERROR);
                return false;
            }
            for (auto dep : act->type2_prev) {
                std::shared_ptr<type2Edge> edge = std::make_shared<type2Edge>();
                edge->edgeId = idType2Edges_++;
                auto dep_end_node = intermediate_nodes_[dep->robot_id][dep->act_id * 2 + 1];
                auto cur_start_node = intermediate_nodes_[i][act_id * 2];
                assert(dep_end_node != nullptr);
                if (dep_end_node->Type1Next == nullptr) {
                    log("activity depend on another terminal activity, this is deadlock!", LogLevel::ERROR);
                    return false;
                }
                edge->nodeFrom = dep_end_node->Type1Next;
                edge->nodeTo = cur_start_node;
                dep_end_node->Type1Next->Type2Next.push_back(edge);
                cur_start_node->Type2Prev.push_back(edge);
            }

            //if (act->type != 2 || act->type != 3 || act->type != 5 || act->type != 8 || act->type != 9 || act->type != 11) {
                if (config_.sync_task && act_id > 0) {
                    // enforce that tasks are executed synchronously by adding type2 dependencies
                    auto cur_start_node = intermediate_nodes_[i][act_id * 2];
                    for (int j = 0; j < num_robots_; j++) {
                        if (i == j) {
                            continue;
                        }
                        if (act_id > act_graph_->num_activities(j)) {
                            continue;
                        }
                        
                        std::shared_ptr<type2Edge> edge = std::make_shared<type2Edge>();
                        edge->edgeId = idType2Edges_++;
                        auto dep_end_node = intermediate_nodes_[j][act_id * 2 - 1];
                        if (dep_end_node == nullptr) {
                            log("activity end node somehow does not exist, check code again..", LogLevel::ERROR);
                            return false;
                        }
                        edge->nodeFrom = dep_end_node;
                        edge->nodeTo = cur_start_node;
                        dep_end_node->Type2Next.push_back(edge);
                        cur_start_node->Type2Prev.push_back(edge);
                        
                    }
                }
            //}
        } 
    }
    return true;
}

bool ADG::findCollisionDeps(std::shared_ptr<PlanInstance> instance, const std::vector<std::vector<int>> &reached_t,
                            bool skip_sametask) {
    // for each pair of activities between each pair of robot
    // if the activity does not depend on each other, then
    // update the planning scene (add objects, attach objects, or detach objects)
    // run collision check for each pair of nodes between the two activities
    // add type 2 edge if there is collision
    idType2Edges_ = 20000;
 
    for (int i = 0; i < num_robots_; i++) {
        for (int j = 0; j < num_robots_; j++) {
            if (i == j) {
                continue;
            }
            for (int act_id_i = 0; act_id_i < act_graph_->num_activities(i); act_id_i++) {
                auto act_i = act_graph_->get(i, act_id_i);
                
                // updated attached / detached object
                updateScene(instance, act_i);

                auto act_i_start_node = intermediate_nodes_[i][act_id_i * 2];
                auto act_i_end_node = intermediate_nodes_[i][act_id_i * 2 + 1];

                // // run bfs on the task graph
                // std::vector<std::vector<bool>> visited;
                // for (int k = 0; k < num_robots_; k++) {
                //     visited.push_back(std::vector<bool>(act_graph_->num_activities(i), false));
                // }
                // act_graph_->bfs(act_i, visited, true);
                // act_graph_->bfs(act_i, visited, false);

                auto tic = std::chrono::high_resolution_clock::now();
                // run bfs on the node graph
                std::vector<std::vector<bool>> visited;
                for (int k = 0; k < num_robots_; k++) {
                    std::vector<bool> v(numNodes_[k], false);
                    visited.push_back(v);
                }
                bfs(act_i_start_node, visited, false);

                // keep track of attached objects for robot j
                ActPtr attached_act_j = nullptr, detacht_act_j = nullptr;
                // remove any existing attached object for robot j, if any
                for (int act_id_j = 0; act_id_j < act_graph_->num_activities(j); act_id_j++) {
                    // updated attached / detached object
                    auto act_j = act_graph_->get(j, act_id_j);
                    // if (visited[j][act_id_j]) {
                    //     // skip if the two activities are dependent
                    //     continue;
                    // }
                    auto act_j_start_node = intermediate_nodes_[j][act_id_j * 2];
                    auto act_j_end_node = intermediate_nodes_[j][act_id_j * 2 + 1];

                    if(reached_t[j][act_j_start_node->timeStep] > reached_t[i][act_i_end_node->timeStep]) {
                        log("Finished building edges to robot " + std::to_string(i) + " activity " + act_i->type_string() + " timestep " 
                            + std::to_string(act_i_end_node->timeStep), LogLevel::INFO);
                        break;
                    }
                    
                    if (visited[j][act_j_end_node->timeStep] || (skip_sametask && act_id_j == act_id_i)) {
                        // skip if the two activities are dependent
                        // skip if they are in the same tpg because type-2 dependencies would have already been build
                        if (act_j->obj_attached.size() > 0) {
                            attached_act_j = act_j;
                        }
                        if (act_j->obj_detached.size() > 0 && attached_act_j != nullptr) {
                            attached_act_j = nullptr;
                        }
                        continue;
                    }

                    if (attached_act_j != nullptr) {
                        updateScene(instance, attached_act_j, i);
                        attached_act_j = nullptr;
                    }
                    updateScene(instance, act_j, j);

                    if(config_.parallel) {
                        findCollisionDepsTaskParallel(i, j, act_i, act_j, act_i_start_node, act_j_start_node, act_i_end_node, act_j_end_node, instance, reached_t, visited);
                    }
                    else {
                        findCollisionDepsTask(i, j, act_i, act_j, act_i_start_node, act_j_start_node, act_i_end_node, act_j_end_node, instance, reached_t, visited);
                    }

                }
                auto attached_obj_j = instance->getAttachedObjects(j);
                if (attached_obj_j.size() > 0) {
                    Object obj = attached_obj_j[0];
                    instance->detachObjectFromRobot(obj.name, act_graph_->get(j, act_graph_->num_activities(j) - 1)->end_pose);
                    if (config_.print_contact) {
                        instance->updateScene();
                    }
                    if (act_graph_->get_last_obj(obj.name)->vanish) {
                        instance->removeObject(obj.name);
                        if (config_.print_contact) {
                            instance->updateScene();
                        }
                        log("remove object " + obj.name + " from the scene", LogLevel::DEBUG);
                    }
                }
            }
            auto attached_obj_i = instance->getAttachedObjects(i);
            if (attached_obj_i.size() > 0) {
                Object obj = attached_obj_i[0];
                instance->detachObjectFromRobot(obj.name, act_graph_->get(i, act_graph_->num_activities(i) - 1)->end_pose);
                if (config_.print_contact) {
                    instance->updateScene();
                }
                if (act_graph_->get_last_obj(obj.name)->vanish) {
                    instance->removeObject(obj.name);
                    if (config_.print_contact) {
                        instance->updateScene();
                    }
                    log("remove object " + obj.name + " from the scene", LogLevel::DEBUG);
                }
            }
        }
    }
    return true;
}

void ADG::addType2EdgesFromRequests(int i, int j, ActPtr act_i, ActPtr act_j,
    const std::vector<std::pair<NodePtr, NodePtr>> &requests,
    const std::vector<std::vector<int>> &reached_t)
{
    struct NodePairHash
    {
        std::size_t operator()(const std::pair<const Node *, const Node *> &pair) const noexcept
        {
            std::size_t h1 = std::hash<const Node *>{}(pair.first);
            std::size_t h2 = std::hash<const Node *>{}(pair.second);
            return h1 ^ (h2 << 1);
        }
    };

    std::unordered_set<std::pair<const Node *, const Node *>, NodePairHash> seen;

    for (const auto &request : requests)
    {
        if (!request.first || !request.second)
        {
            continue;
        }

        const auto key = std::make_pair(request.first.get(), request.second.get());
        if (!seen.insert(key).second)
        {
            continue;
        }

        auto edge = std::make_shared<type2Edge>();
        edge->edgeId = idType2Edges_++;
        edge->nodeFrom = request.first;
        edge->nodeTo = request.second;
        request.first->Type2Next.push_back(edge);
        request.second->Type2Prev.push_back(edge);

        log("add sweep-based type 2 edge from robot " + std::to_string(j) + " activity " +
                act_j->type_string() + " timestep " + std::to_string(request.first->timeStep) + " " +
                std::to_string(reached_t[j][request.first->timeStep]) + " to robot " +
                std::to_string(i) + " activity " + act_i->type_string() + " timestep " +
                std::to_string(request.second->timeStep) + " " +
                std::to_string(reached_t[i][request.second->timeStep]),
            LogLevel::DEBUG);
    }
}

void ADG::findCollisionDepsTask(int i, int j, ActPtr act_i, ActPtr act_j,
                                NodePtr act_i_start_node, NodePtr act_j_start_node, NodePtr act_i_end_node,
                                NodePtr act_j_end_node, std::shared_ptr<PlanInstance> instance,
                                const std::vector<std::vector<int>> &reached_t,
                                const std::vector<std::vector<bool>> &visited)
{
    if (config_.use_sweep_type2)
    {
        const auto segments_i = buildSegments(act_i_start_node, act_i_end_node);
        const auto segments_j = buildSegments(act_j_start_node, act_j_end_node);
        std::vector<EdgeRequest> edge_requests;

        for (const auto &segment_i : segments_i)
        {
            if (segment_i.empty())
            {
                continue;
            }

            NodePtr node_i_start = segment_i.front();

            for (const auto &segment_j : segments_j)
            {
                if (segment_j.empty())
                {
                    continue;
                }

                NodePtr node_j_start = segment_j.front();
                if (reached_t[j][node_j_start->timeStep] >= reached_t[i][node_i_start->timeStep])
                {
                    continue;
                }

                if (visited[j][node_j_start->timeStep])
                {
                    continue;
                }

                MRTrajectory sweep(2);
                sweep[0].robot_id = i;
                sweep[0].trajectory.reserve(segment_i.size());
                for (const auto &node : segment_i)
                {
                    sweep[0].trajectory.push_back(node->pose);
                }

                sweep[1].robot_id = j;
                sweep[1].trajectory.reserve(segment_j.size());
                for (const auto &node : segment_j)
                {
                    sweep[1].trajectory.push_back(node->pose);
                }

                if (instance->checkMultiRobotSweep(sweep, true))
                {
                    edge_requests.emplace_back(segment_j.back(), node_i_start);
                }
            }
        }

        addType2EdgesFromRequests(i, j, act_i, act_j, edge_requests, reached_t);
        return;
    }

    // check collision
    NodePtr iter_node_i = act_i_start_node;
    NodePtr iter_node_j_start = act_j_start_node;
    while (iter_node_i != nullptr && iter_node_i->timeStep <= act_i_end_node->timeStep) {
        NodePtr iter_node_j = iter_node_j_start;
        bool inCollision = false;
        while (iter_node_j != nullptr &&
            iter_node_j->timeStep <= act_j_end_node->timeStep &&
            reached_t[j][iter_node_j->timeStep] < reached_t[i][iter_node_i->timeStep]) 
        {
            if (visited[j][iter_node_j->timeStep]) {
                iter_node_j = iter_node_j->Type1Next;
                continue;
            }
            bool has_collision = instance->checkCollision({iter_node_i->pose, iter_node_j->pose}, true);
            if (has_collision) {
                if (config_.print_contact) {
                    //instance->printKnownObjects();
                    instance->checkCollision({iter_node_i->pose, iter_node_j->pose}, true, true);
                }
                inCollision = true;
            }
            else if (inCollision) {
                inCollision = false;
                std::shared_ptr<type2Edge> edge = std::make_shared<type2Edge>();
                edge->edgeId = idType2Edges_++;
                edge->nodeFrom = iter_node_j;
                edge->nodeTo = iter_node_i;
                iter_node_j->Type2Next.push_back(edge);
                iter_node_i->Type2Prev.push_back(edge);
                iter_node_j_start = iter_node_j;
                log("add type 2 edge from robot " + std::to_string(j) + " activity " 
                    + act_j->type_string() + " timestep " + std::to_string(iter_node_j->timeStep)
                    + " " + std::to_string(reached_t[j][iter_node_j->timeStep])
                    + " to robot " + std::to_string(i) + " activity " 
                    + act_i->type_string() +  " timestep " + std::to_string(iter_node_i->timeStep)
                    + " " + std::to_string(reached_t[i][iter_node_i->timeStep]) , LogLevel::INFO);

            }
            iter_node_j = iter_node_j->Type1Next;
        }
        if (inCollision) {
            assert(iter_node_j != nullptr);
            std::shared_ptr<type2Edge> edge = std::make_shared<type2Edge>();
            edge->edgeId = idType2Edges_++;
            edge->nodeFrom = iter_node_j;
            edge->nodeTo = iter_node_i;
            iter_node_j->Type2Next.push_back(edge);
            iter_node_i->Type2Prev.push_back(edge);
            iter_node_j_start = iter_node_j;
            log("add type 2 edge (end act) from robot " + std::to_string(j) + " activity " 
                    + act_j->type_string() + " timestep " + std::to_string(iter_node_j->timeStep)
                    + " " + std::to_string(reached_t[j][iter_node_j->timeStep])
                    + " to robot " + std::to_string(i) + " activity " 
                    + act_i->type_string() +  " timestep " + std::to_string(iter_node_i->timeStep)
                    + " " + std::to_string(reached_t[i][iter_node_i->timeStep]) , LogLevel::INFO);

        }
        iter_node_i = iter_node_i->Type1Next;
    }
}

void ADG::findCollisionDepsTaskParallel(int i, int j, ActPtr act_i, ActPtr act_j,
    NodePtr act_i_start_node, NodePtr act_j_start_node, NodePtr act_i_end_node,
    NodePtr act_j_end_node, std::shared_ptr<PlanInstance> instance,
    const std::vector<std::vector<int>> &reached_t,
    const std::vector<std::vector<bool>> &visited)
{
    if (config_.use_sweep_type2)
    {
        const auto segments_i = buildSegments(act_i_start_node, act_i_end_node);
        const auto segments_j = buildSegments(act_j_start_node, act_j_end_node);

        if (segments_i.empty() || segments_j.empty())
        {
            return;
        }

        log("Number of 8-wide segments to check for robot " + std::to_string(i) + " activity " +
                act_i->type_string() + ": " + std::to_string(segments_i.size()) +
                ", robot " + std::to_string(j) + " activity " +
                act_j->type_string() + ": " + std::to_string(segments_j.size()),
            LogLevel::DEBUG);

        std::vector<EdgeRequest> edge_requests;

        #pragma omp parallel
        {
            std::vector<EdgeRequest> thread_requests;

            #pragma omp for schedule(dynamic)
            for (std::size_t idx_i = 0; idx_i < segments_i.size(); ++idx_i)
            {
                const auto &segment_i = segments_i[idx_i];
                if (segment_i.empty())
                {
                    continue;
                }

                NodePtr node_i_start = segment_i.front();

                for (std::size_t idx_j = 0; idx_j < segments_j.size(); ++idx_j)
                {
                    const auto &segment_j = segments_j[idx_j];
                    if (segment_j.empty())
                    {
                        continue;
                    }

                    NodePtr node_j_start = segment_j.front();
                    if (reached_t[j][node_j_start->timeStep] >= reached_t[i][node_i_start->timeStep])
                    {
                        continue;
                    }

                    if (visited[j][node_j_start->timeStep])
                    {
                        continue;
                    }

                    MRTrajectory sweep(2);
                    sweep[0].robot_id = i;
                    sweep[0].trajectory.reserve(segment_i.size());
                    for (const auto &node : segment_i)
                    {
                        sweep[0].trajectory.push_back(node->pose);
                    }

                    sweep[1].robot_id = j;
                    sweep[1].trajectory.reserve(segment_j.size());
                    for (const auto &node : segment_j)
                    {
                        sweep[1].trajectory.push_back(node->pose);
                    }

                    if (instance->checkMultiRobotSweep(sweep, true))
                    {
                        thread_requests.emplace_back(segment_j.back(), node_i_start);
                    }
                }
            }

            #pragma omp critical
            {
                edge_requests.insert(edge_requests.end(), thread_requests.begin(), thread_requests.end());
            }
        }

        addType2EdgesFromRequests(i, j, act_i, act_j, edge_requests, reached_t);
        return;
    }

    // First, create vectors of nodes for easier indexing
    std::vector<NodePtr> nodes_i, nodes_j;
    
    // Populate node vectors
    for (NodePtr node = act_i_start_node; 
         node != nullptr && node->timeStep <= act_i_end_node->timeStep; 
         node = node->Type1Next) {
        nodes_i.push_back(node);
    }
    
    // Populate nodes_j in reverse order
    for (NodePtr node = act_j_end_node; 
         node != nullptr && node->timeStep >= act_j_start_node->timeStep; 
         node = node->Type1Prev) {
        nodes_j.push_back(node);
    }

    log("Number of nodes to check for robot " + std::to_string(i) + " activity " 
        + act_i->type_string() + ": " + std::to_string(nodes_i.size())
        + ", robot " + std::to_string(j) + " activity " 
        + act_j->type_string() + ": " + std::to_string(nodes_j.size()), LogLevel::DEBUG);

    std::vector<std::tuple<NodePtr, NodePtr, bool>> collision_results;
    std::mutex results_mutex;

    #pragma omp parallel
    {
        std::vector<std::tuple<NodePtr, NodePtr, bool>> thread_results;
        
        #pragma omp for
        for (int idx_i = 0; idx_i < nodes_i.size(); idx_i++) {
            NodePtr iter_node_i = nodes_i[idx_i];
            
            for (int idx_j = 0; idx_j < nodes_j.size(); idx_j++) {
                NodePtr iter_node_j = nodes_j[idx_j];
                
                if (reached_t[j][iter_node_j->timeStep] >= reached_t[i][iter_node_i->timeStep]) {
                    continue;
                }
                
                if (visited[j][iter_node_j->timeStep]) {
                    continue;
                }
                
                bool has_collision = instance->checkCollision(
                    {iter_node_i->pose, iter_node_j->pose}, true);
                
                if (has_collision) {
                    if (config_.print_contact) {
                        #pragma omp critical
                        {
                            instance->checkCollision(
                                {iter_node_i->pose, iter_node_j->pose}, true, true);
                        }
                    }
                    
                    // Since we're iterating backwards, this is our collision exit point
                    #pragma omp critical
                    {
                        thread_results.emplace_back(iter_node_j, iter_node_i, true);
                    }
                    break;  // Break inner loop as we found our collision exit point
                }
            }
        }
        
        #pragma omp critical
        {
            collision_results.insert(collision_results.end(), 
                thread_results.begin(), thread_results.end());
        }
    }
    
    // Process results and create edges as before
    for (const auto& [node_j, node_i, collision] : collision_results) {
        std::shared_ptr<type2Edge> edge = std::make_shared<type2Edge>();
        edge->edgeId = idType2Edges_++;
        edge->nodeFrom = node_j;
        edge->nodeTo = node_i;
        node_j->Type2Next.push_back(edge);
        node_i->Type2Prev.push_back(edge);
        
        log("add type 2 edge from robot " + std::to_string(j) + " activity "
            + act_j->type_string() + " timestep " + std::to_string(node_j->timeStep)
            + " " + std::to_string(reached_t[j][node_j->timeStep])
            + " to robot " + std::to_string(i) + " activity "
            + act_i->type_string() + " timestep " + std::to_string(node_i->timeStep)
            + " " + std::to_string(reached_t[i][node_i->timeStep]) , LogLevel::DEBUG);
    }
}

void ADG::findEarliestReachTimeSyncAct(int num_act, std::vector<std::vector<int>> &reached_t, std::vector<int> &reached_end)
{
    reached_t.clear();
    reached_end.clear();
    std::vector<NodePtr> nodes;
    nodes.resize(num_robots_);
    reached_end.resize(num_robots_, -1);
    
    for (int i = 0; i < num_robots_; i++) {
        std::vector<int> v(numNodes_[i], -1);
        reached_t.push_back(v);
    }

    int flowtime_i = 0;
    int j = 0;
    for (int act_id = 0; act_id < num_act; act_id++) {
        for (int i = 0; i < num_robots_; i++) {
            nodes[i] = intermediate_nodes_[i][act_id * 2];
            reached_end[i] = -1;
        }

        // we synchronize the activities for all robots
        // i.e. a robot has to wait for all other robots to finish the previous activity
        bool allReached = false;
        while(!allReached) {
            for (int i = 0; i < num_robots_; i++) {
                if (reached_t[i][nodes[i]->timeStep] == -1) {
                    reached_t[i][nodes[i]->timeStep] = j;
                }
            }
            for (int i = 0; i < num_robots_; i++) {
                if (nodes[i]->timeStep < intermediate_nodes_[i][act_id * 2 + 1]->timeStep) {
                    bool safe = true;
                    for (auto edge : nodes[i]->Type1Next->Type2Prev) {
                        if (reached_t[edge->nodeFrom->robotId][edge->nodeFrom->timeStep] == -1) {
                            safe = false;
                            break;
                        }
                    }
                    if (safe) {
                        nodes[i] = nodes[i]->Type1Next;
                    }
                }
                else if (reached_end[i] == -1) {
                    reached_end[i] = j;
                    flowtime_i += j;
                }
            }

            allReached = true;
            for (int i = 0; i < num_robots_; i++) {
                allReached &= (reached_end[i] != -1);
            }
            j++;
        }
    }
 

}


bool ADG::saveToDotFile(const std::string& filename) const {

    std::ofstream out(filename);
    out << "digraph G {" << std::endl;

    // define node attributes here
    out << "node [shape=ellipse];" << std::endl;
    out << "rankdir=LR;" << std::endl;

    // define all the nodes
    for (int i = 0; i < num_robots_; i++) {
        out << "subgraph cluster_" << i << " {" << std::endl;
        out << "label = \"Robot " << i << "\";" << std::endl;
        out << "rank=same;" << std::endl;
        NodePtr node_i = start_nodes_[i];
        std::vector<NodePtr> salient_nodes;
        while (node_i != nullptr) {
            if (node_i->Type1Prev == nullptr) {
                out << "n" << i << "_" << node_i->timeStep << " [label=\"" << act_graph_->get(i, node_i->actId)->type_string() << node_i->timeStep << "\"];" << std::endl;
                salient_nodes.push_back(node_i);
            }
            else if (node_i->Type1Next == nullptr) {
                out << "n" << i << "_" << node_i->timeStep << " [label=\"" << act_graph_->get(i, node_i->actId)->type_string() << node_i->timeStep << "\"];" << std::endl;
                salient_nodes.push_back(node_i);
            }
            else if (node_i->Type1Next->actId > node_i->actId &&
                act_graph_->get(i, node_i->Type1Next->actId)->type != act_graph_->get(i, node_i->actId)->type) {
                out << "n" << i << "_" << node_i->timeStep << " [label=\"" << act_graph_->get(i, node_i->actId)->type_string() << node_i->timeStep << "\"];" << std::endl;
                salient_nodes.push_back(node_i);
            }
            else if (node_i->Type2Prev.size() > 0) {
                out << "n" << i << "_" << node_i->timeStep << " [label=\"" << act_graph_->get(i, node_i->actId)->type_string() << node_i->timeStep << "\"];" << std::endl;
                salient_nodes.push_back(node_i);
            }
            else if (node_i->Type2Next.size() > 0) {
                out << "n" << i << "_" << node_i->timeStep << " [label=\"" << act_graph_->get(i, node_i->actId)->type_string() << node_i->timeStep << "\"];" << std::endl;
                salient_nodes.push_back(node_i);
            }
            
            node_i = node_i->Type1Next;
        }
        assert(salient_nodes.size() > 0);

        node_i = salient_nodes[0];
        out << "n" << i << "_" << node_i->timeStep;
        for (int j = 1; j < salient_nodes.size(); j++) {
            out << " -> " << "n" << i << "_" << salient_nodes[j]->timeStep;
        }
        out << ";" << std::endl;
        out << "}" << std::endl;
    }

    // define all the edges
    for (int i = 0; i < num_robots_; i++) {
        NodePtr node_i = start_nodes_[i];
        while (node_i != nullptr) {
            for (auto edge : node_i->Type2Prev) {
                out << "n" << edge->nodeFrom->robotId << "_" << edge->nodeFrom->timeStep << " -> " << "n" << i << "_" << node_i->timeStep << ";" << std::endl;
            }
            node_i = node_i->Type1Next;
        }
    }

    out << "}" << std::endl;
    out.close();

    std::string command = "dot -Tpng " + filename + " -o " + filename + ".png";
    int result = system(command.c_str());

    return result == 0;
}

void ADG::update_joint_states(const std::vector<double> &joint_states, int robot_id)
{
    TPG::update_joint_states(joint_states, robot_id);
    if (policy_ != nullptr) {
        policy_->update_joint_states(joint_states, robot_id);
    }
    
    if (num_robots_ > executed_acts_.size()) {
        return;
    }
   
    int act_id = executed_acts_[robot_id]->load();
    if (act_id >= act_graph_->num_activities(robot_id)) {
        return;
    }
    auto act = act_graph_->get(robot_id, act_id);

    NodePtr node = intermediate_nodes_[robot_id][act_id * 2];
    if (isPolicyNode(node)) {
        // executed node is only updated after policy execution
        return;
    }

    NodePtr end_node = intermediate_nodes_[robot_id][act_id * 2 + 1];
    int executed_step = executed_steps_[robot_id]->load();

    if (executed_step >= end_node->timeStep) {
        executed_acts_[robot_id]->fetch_add(1);
        log("Robot " + std::to_string(robot_id) + " finished activity " + act->type_string(), LogLevel::INFO);
        int act_id = executed_acts_[robot_id]->load();
        while (act_id < act_graph_->num_activities(robot_id) - 1 && act_graph_->get(robot_id, act_id)->is_skippable()){
            act_id ++;
            executed_acts_[robot_id]->fetch_add(1);
        }

        // update any attached object
        if (act_id < act_graph_->num_activities(robot_id)) {
            auto act = act_graph_->get(robot_id, act_id);
            update_attached_object(robot_id, act);
        }
        
    }
    
}

void ADG::update_attached_object(int robot_id, ActPtr act) {
    if (act == nullptr) {
        return;
    }
    for (auto obj : act->obj_attached) {
        if (obj->handover && act->type == Activity::Type::home_receive) {
            // the handover object is attached when it was detached
            continue;
        }
        if (instance_ != nullptr) {
            if (obj->vanish) {
                instance_->addMoveableObject(obj->obj);
                instance_->updateScene();
            }
            instance_->attachObjectToRobot(obj->obj.name, robot_id, obj->next_attach_link, act->start_pose);
            instance_->updateScene();
            log("attach object " + obj->obj.name + " to robot " + std::to_string(robot_id), LogLevel::INFO);
        }
        if (robot_id >= 0 && robot_id < static_cast<int>(in_hand_obj_names_.size())) {
            in_hand_obj_names_[robot_id] = obj->obj.name;
        }
    }
    for (auto obj : act->obj_detached) {
        if (robot_id >= 0 && robot_id < static_cast<int>(in_hand_obj_names_.size())) {
            in_hand_obj_names_[robot_id] = "None";
        }
        if (instance_ != nullptr) {
            instance_->detachObjectFromRobot(obj->obj.name, act->start_pose);
            instance_->updateScene();
            if (obj->vanish) {
                instance_->removeObject(obj->obj.name);
                instance_->updateScene();
            }
            log("detach object " + obj->obj.name + " from robot " + std::to_string(robot_id), LogLevel::INFO);
        }
        if (obj->handover && act->type == Activity::Type::handover_twist) {
            // also attach the obj
            auto home_receive_act = obj->next_attach;
            if (instance_ != nullptr) {
                instance_->attachObjectToRobot(obj->obj.name, home_receive_act->robot_id, obj->next_attach_link, home_receive_act->start_pose);
                instance_->updateScene();
                log("attach object " + obj->obj.name + " to robot " + std::to_string(home_receive_act->robot_id), LogLevel::INFO);
            }
            int receiver = home_receive_act->robot_id;
            if (receiver >= 0 && receiver < static_cast<int>(in_hand_obj_names_.size())) {
                in_hand_obj_names_[receiver] = obj->obj.name;
            }
        }
    }
}

void ADG::init_executed_steps() {
    TPG::init_executed_steps();
    for (int i = 0; i < num_robots_; i++) {
        int act_id = exec_start_act_[i];
        executed_acts_.push_back(std::make_unique<std::atomic<int>>(act_id));
        while (act_id < act_graph_->num_activities(i) - 1 && act_graph_->get(i, act_id)->is_skippable()){
            act_id ++;
            executed_acts_[i]->fetch_add(1);
        }
    }
}

#if MR_PLANNER_WITH_ROS
bool ADG::moveit_execute(std::shared_ptr<PlanInstance> instance, 
            std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group) 
{
    instance_ = instance;
    return TPG::moveit_execute(instance, move_group);
}

bool ADG::moveit_mt_execute(std::shared_ptr<PlanInstance> instance, const std::vector<std::vector<std::string>> &joint_names, std::vector<ros::ServiceClient> &clients) 
{
    instance_ = instance;
    return TPG::moveit_mt_execute(instance, joint_names, clients);
}
#endif

void ADG::initSampler(const std::vector<std::vector<int>> &earliest_t, const std::vector<std::vector<NodePtr>> &timed_nodes) {
    shortcut_sampler_ = std::make_unique<ShortcutSamplerADG>(config_, act_graph_, intermediate_nodes_);
    shortcut_sampler_->init(start_nodes_, numNodes_, earliest_t, timed_nodes);
}

void ADG::initIterator() {
    shortcut_iterator_ = std::make_unique<ShortcutIteratorADG>(config_, act_graph_, intermediate_nodes_);
    shortcut_iterator_->init(start_nodes_, numNodes_, end_nodes_);
}

void ADG::updateScene(std::shared_ptr<PlanInstance> instance, ActPtr act, int existing_robot) const {
    std::vector<Object> attached_obj;
    if (existing_robot >= 0) {
        attached_obj = instance->getAttachedObjects(existing_robot);
    }

    int robot_id = act->robot_id;
    for (auto obj : act->obj_attached) {
        if (attached_obj.size() > 0 && obj->handover && obj->obj.name == attached_obj[0].name) {
            continue;
        }
        instance->moveRobot(robot_id, act->start_pose);
        if (config_.print_contact) {
            instance->updateScene();
        }
        if (obj->vanish && !instance->hasObject(obj->name())) {
            instance->addMoveableObject(obj->obj);
            if (config_.print_contact) {
                instance->updateScene();
            }
            log("add object " + obj->obj.name + " to the scene", LogLevel::DEBUG);
        }
        else {
            instance->moveObject(obj->obj);
            if (config_.print_contact) {
                instance->updateScene();
            }
        }
        instance->attachObjectToRobot(obj->obj.name, robot_id, obj->next_attach_link, act->start_pose);
        if (config_.print_contact) {
            instance->updateScene();
        }
    }
    for (auto obj : act->obj_detached) {
        if (attached_obj.size() > 0 && obj->handover && obj->obj.name == attached_obj[0].name) {
            continue;
        }
        log("detach object " + obj->obj.name + " from robot " + std::to_string(robot_id), LogLevel::DEBUG);
        instance->detachObjectFromRobot(obj->obj.name, act->start_pose);
        if (config_.print_contact) {
            instance->updateScene();
        }
         if (obj->vanish) {
            log("remove object " + obj->obj.name + " from the scene", LogLevel::DEBUG);
            instance->removeObject(obj->obj.name);
            if (config_.print_contact) {
                instance->updateScene();
            }
        }
    }
    
    for (auto col_node : act->collision_nodes) {
        instance->setCollision(col_node.obj_name, col_node.link_name, col_node.allow);
    }
}

void ADG::checkShortcuts(std::shared_ptr<PlanInstance> instance, Shortcut &shortcut,
        const std::vector<std::vector<NodePtr>> &timedNodes) const {
    auto ni = shortcut.ni.lock();
    auto nj = shortcut.nj.lock();
    // check if there is a shortcut between ni and nj
    assert(ni->robotId == nj->robotId && ni->timeStep < nj->timeStep - 1);

    int robot_id = ni->robotId;

    // build collision environment
    ActPtr cur_act = shortcut.activity;
    assert(cur_act != nullptr);
    
    if (config_.print_contact) {
        instance->resetScene(true);
    }
    else {
        instance->resetScene(false);
    }
    // add all static objects that needs to be collision checked
    std::vector<ObjPtr> indep_objs = act_graph_->find_indep_obj(cur_act);
    for (auto obj : indep_objs) {
        if (instance->hasObject(obj->obj.name)) {
            Object obj_copy = obj->obj;
            obj_copy.name = obj->obj.name + "_copy";
            instance->addMoveableObject(obj_copy);
        }
        else {
            instance->addMoveableObject(obj->obj);
        }
        instance->setCollision(obj->obj.name, obj->obj.name, true);
        if (config_.print_contact) {
            instance->updateScene();
        }
    }

    for (int act_id = 0; act_id <= cur_act->act_id; act_id++) {
        // updated attached / detached object
        ActPtr act_j = act_graph_->get(robot_id, act_id);
        updateScene(instance, act_j);
    }

    int shortcutSteps = shortcut.path.size() + 2;
    // for (int i = 1; i < shortcutSteps - 1; i++) {
    //     RobotPose pose_i = shortcut.path[i - 1];

    //     // check environment collision
    //     if (instance->checkCollision({pose_i}, false) == true) {
    //         if (config_.print_contact) {
    //             instance->checkCollision({pose_i}, false, true);
    //         }
    //         shortcut.col_type = CollisionType::STATIC; // collide with the evnrionment
    //         return;
    //     }
    // }
    
    MRTrajectory trajectory(1);
    trajectory[0].robot_id = robot_id;
    trajectory[0].trajectory = shortcut.path;
    if (instance->checkMultiRobotTrajectory(trajectory, false)) {
        shortcut.col_type = CollisionType::STATIC; // collide with the evnrionment
        return;
    }

    auto tic = std::chrono::high_resolution_clock::now();

    // find dependent parent and child nodes
    std::vector<std::vector<bool>> visited_act;
    for (int i = 0; i < num_robots_; i++) {
        visited_act.push_back(std::vector<bool>(act_graph_->num_activities(i), false));
    }
    act_graph_->bfs(cur_act, visited_act, true);
    act_graph_->bfs(cur_act, visited_act, false);

    std::vector<std::vector<bool>> visited;
    for (int i = 0; i < num_robots_; i++) {
        std::vector<bool> v(numNodes_[i], false);
        visited.push_back(v);
    }
    visited[nj->robotId][nj->timeStep] = true;
    bfs(nj, visited, true);

    visited[ni->robotId][ni->timeStep] = true;
    bfs(ni, visited, false);

    const auto buildPoseSegments = [](const std::vector<RobotPose> &poses) {
        std::vector<std::vector<RobotPose>> segments;
        std::size_t offset = 0;
        const std::size_t total = poses.size();

        while (offset < total) {
            std::size_t end = offset + kType2SweepWindow;
            if (end > total) {
                end = total;
            }
            segments.emplace_back(poses.begin() + offset, poses.begin() + end);
            offset = end;
        }

        return segments;
    };

    if (shortcutSteps <= 2) {
        // special case, check if the static node collides with any other independent nodes
        std::vector<RobotPose> shortcut_poses{ni->pose, nj->pose};
        const auto shortcut_segments = buildPoseSegments(shortcut_poses);
        
        for (int j = 0; j < num_robots_; j++) {
            if (j == ni->robotId) {
                continue;
            }
            for (int act_id_j = 0; act_id_j < act_graph_->num_activities(j); act_id_j++) {

                // updated attached / detached object
                auto act_j = act_graph_->get(j, act_id_j);
                updateScene(instance, act_j, robot_id);
                if (visited_act[j][act_id_j] == true) {
                    continue;
                }
                auto act_j_start_node = intermediate_nodes_[j][act_id_j * 2];
                auto act_j_end_node = intermediate_nodes_[j][act_id_j * 2 + 1];
                while (!config_.sync_task && act_j->is_skippable() && act_id_j < act_graph_->num_activities(j) - 1) {
                    act_id_j ++;
                    act_j_end_node = intermediate_nodes_[j][act_id_j * 2 + 1];
                    act_j = act_graph_->get(j, act_id_j);
                }
                const auto node_segments = buildSegments(act_j_start_node, act_j_end_node);
                for (const auto &node_segment : node_segments) {
                    RobotTrajectory robot_segment;
                    robot_segment.robot_id = j;
                    NodePtr collision_node = nullptr;
                    for (const auto &node_j : node_segment) {
                        if (visited[j][node_j->timeStep] == false) {
                            if (collision_node == nullptr) {
                                collision_node = node_j;
                            }
                            robot_segment.trajectory.push_back(node_j->pose);
                        }
                    }

                    if (robot_segment.trajectory.empty()) {
                        continue;
                    }

                    for (const auto &shortcut_segment : shortcut_segments) {
                        MRTrajectory sweep(2);
                        sweep[0].robot_id = robot_id;
                        sweep[0].trajectory = shortcut_segment;
                        sweep[1].robot_id = j;
                        sweep[1].trajectory = robot_segment.trajectory;

                        if (instance->checkMultiRobotSweep(sweep, true)) {
                            shortcut.col_type = CollisionType::ROBOT; // collide with other robots
                            shortcut.n_robot_col = collision_node;
                            return;
                        }
                    }
                }
            }
        }
        shortcut.col_type = CollisionType::NONE;
        return;
    }

    // for (int j = 0; j < num_robots_; j++) {
    //     Eigen::MatrixXi col_matrix_j(shortcutSteps, numNodes_[j]);
    //     col_matrix_j.setZero();
    //     col_matrix.push_back(col_matrix_j);
    // }

    auto t_bfs = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - tic).count();

    const auto shortcut_segments = buildPoseSegments(shortcut.path);

    // check robot-robot collision
   
    for (int j = 0; j < num_robots_; j++) {
        if (j == ni->robotId) {
            continue;
        }
        for (int act_id_j = 0; act_id_j < act_graph_->num_activities(j); act_id_j++) {
            // updated attached / detached object
            auto act_j = act_graph_->get(j, act_id_j);
            updateScene(instance, act_j, robot_id);
            if (visited_act[j][act_id_j] == true) {
                continue;
            }
            auto act_j_start_node = intermediate_nodes_[j][act_id_j * 2];
            auto act_j_end_node = intermediate_nodes_[j][act_id_j * 2 + 1];

            while (!config_.sync_task && act_j->is_skippable() && act_id_j < act_graph_->num_activities(j) - 1) {
                act_id_j ++;
                act_j_end_node = intermediate_nodes_[j][act_id_j * 2 + 1];
                act_j = act_graph_->get(j, act_id_j);
            }
            if (config_.parallel) {
                bool collision_found = false;
                std::vector<NodePtr> node_js;
                NodePtr node_j = act_j_start_node;
                while (node_j != nullptr && node_j->timeStep <= act_j_end_node->timeStep) {
                    node_js.push_back(node_j);
                    node_j = node_j->Type1Next;
                }

                for (int i = 1; i < shortcutSteps - 1; i++) {
                    RobotPose pose_i = shortcut.path[i - 1];
                    // Parallel region for the inner loop:
                    #pragma omp parallel default(none) shared(collision_found, shortcut, instance, config_, visited, j, pose_i, node_js) firstprivate(i)
                    { 
                        # pragma omp for
                        for (int k = 0; k < node_js.size(); k++) {
                            NodePtr node_j = node_js[k];
                            if (collision_found || visited[j][node_j->timeStep] == true) {
                                continue;
                            }

                            RobotPose pose_j = node_j->pose;
                            bool has_collision = instance->checkCollision({pose_i, pose_j}, true);
                            if (has_collision) {
                                #pragma omp critical
                                {
                                    if (!collision_found) {
                                        collision_found = true;
                                        if (config_.print_contact) {
                                            instance->checkCollision({pose_i, pose_j}, true, true);
                                        }
                                        shortcut.n_robot_col = node_j;
                                        shortcut.col_type = CollisionType::ROBOT; // collide with other robots
                                    }
                                } // end omp critical
                            }
                        } // end pragma omp for

                    } // end pragma omp parallel
                    if (collision_found) {
                        return;
                    }
                }
            }
            else {
                const auto node_segments = buildSegments(act_j_start_node, act_j_end_node);
                for (const auto &node_segment : node_segments) {
                    RobotTrajectory robot_segment;
                    robot_segment.robot_id = j;
                    NodePtr collision_node = nullptr;
                    for (const auto &node_j : node_segment) {
                        if (visited[j][node_j->timeStep] == false) {
                            if (collision_node == nullptr) {
                                collision_node = node_j;
                            }
                            robot_segment.trajectory.push_back(node_j->pose);
                        }
                    }

                    if (robot_segment.trajectory.empty()) {
                        continue;
                    }

                    for (const auto &shortcut_segment : shortcut_segments) {
                        MRTrajectory sweep(2);
                        sweep[0].robot_id = robot_id;
                        sweep[0].trajectory = shortcut_segment;
                        sweep[1].robot_id = j;
                        sweep[1].trajectory = robot_segment.trajectory;

                        if (instance->checkMultiRobotSweep(sweep, true)) {
                            shortcut.n_robot_col = collision_node;
                            shortcut.col_type = CollisionType::ROBOT; // collide with other robots
                            return;
                        }
                    }
                }
            }
        }
    }

    shortcut.col_type = CollisionType::NONE;
    return;
}

bool ADG::isPolicyNode(NodePtr node) const {
    if (config_.run_policy == false && config_.test_recovery == false) {
        return false;
    }
    int robot_id = node->robotId;
    Activity::Type type = act_graph_->get(robot_id, node->actId)->type;

    if (type == Activity::Type::drop_down || type == Activity::Type::pick_down ||
        type == Activity::Type::support || type == Activity::Type::handover_down ||
        type == Activity::Type::place_up || type == Activity::Type::press_down) {
        return true; 
    }
    
    return false;
}

bool ADG::executePolicy(const NodePtr &startNode, NodePtr &endNode) {
    int robot_id = startNode->robotId;
    int act_id = startNode->actId;
    Activity::Type type = act_graph_->get(robot_id, act_id)->type;

    bool success = false;
    log("robot " + std::to_string(robot_id) + " executing policy " + act_graph_->get(robot_id, act_id)->type_string(), LogLevel::INFO);
    executed_steps_[robot_id]->store(startNode->timeStep);
    executed_acts_[robot_id]->store(act_id);
    ActPtr act_attachdetach = nullptr;
    if (type == Activity::Type::drop_down || type == Activity::Type::pick_down 
        || type == Activity::Type::handover_down || type == Activity::Type::place_up) {
        endNode = intermediate_nodes_[robot_id][act_id * 2 + 5];
        success = policy_->execute(startNode, endNode, type);
        if (!success) {
            return false; // policy execution failed
        }
        act_attachdetach = act_graph_->get(robot_id, act_id+1); // obj is attached/detached to twist, which is always the next of down/up
    }
    else if (type == Activity::Type::support || type == Activity::Type::press_down) {
        int last_sup_act_id = act_id;
        Activity::Type next_type = act_graph_->get(robot_id, last_sup_act_id + 1)->type;
        while (next_type == type) {
            last_sup_act_id++;
            next_type = act_graph_->get(robot_id, last_sup_act_id + 1)->type;
        }
        endNode = intermediate_nodes_[robot_id][last_sup_act_id * 2 + 1];
        success = policy_->execute(startNode, endNode, type);
    }

    executed_steps_[robot_id]->store(endNode->timeStep + 1);
    executed_acts_[robot_id]->store(endNode->actId + 1);
    int end_act_id = endNode->actId;
    update_attached_object(robot_id, act_attachdetach);
    log("robot " + std::to_string(robot_id) + " finished executing policy " 
        + act_graph_->get(robot_id, end_act_id)->type_string(), LogLevel::INFO);

    return success;
}

bool ADG::replanRecovery(const NodePtr &startNode, NodePtr &endNode) {
    // replan a set of recovery tasks and trajectories
    if (policy_ == nullptr) {
        log("policy is not initialized, cannot replan recovery", LogLevel::ERROR);
        return false;
    }
    log("====== Recovery planning started =======", LogLevel::INFO);

    // 1. first identify the where the recovery goal should be
    NodePtr recovery_goal = nullptr;
    Policy::RecoverySkillType recovery_type;
    int robot_id = startNode->robotId;
    std::vector<ActPtr> skipped_acts;
    bool success = policy_->find_recover_goal(robot_id, startNode, recovery_goal, skipped_acts, recovery_type);
    if (!success) {
        log("Cannot find recovery goal for robot " + std::to_string(robot_id), LogLevel::ERROR);
        return false;
    }

    // 1.5 then identify the cut point in TPG/ADG for other nodes
    std::vector<NodePtr> cut_nodes;
    for (int i = 0; i < num_robots_; i++) {
        if (i == robot_id) {
            continue;
        }
        int cur_step_i = executed_steps_[i]->load();
        NodePtr cur_node = start_nodes_[i];
        
        for (int i = 0; i < cur_step_i; i++) {
            if (cur_node->Type1Next != nullptr) {
                cur_node = cur_node->Type1Next;
            }
        }

        // find the next node with incoming dependecies
        NodePtr cut_node_i = cur_node;
        while (cut_node_i->Type1Next != nullptr && cut_node_i->Type1Next->Type2Prev.size() == 0) {
            cut_node_i = cut_node_i->Type1Next;
        }
        if (cut_node_i == nullptr) {
            log("Cannot find cut node for robot " + std::to_string(i), LogLevel::ERROR);
            return false;
        }
        cut_nodes.push_back(cut_node_i);
        // we found the cut node for robot i
        log("Found cut node for robot " + std::to_string(i) + ": " + std::to_string(cut_node_i->timeStep), LogLevel::INFO);
    }
    

    // 2. replan trajectories for the recovery activities
    std::vector<ActPtr> recovery_acts;
    std::vector<RobotTrajectory> recovery_trajs;
    success = policy_->plan_recover(robot_id, startNode, recovery_goal, recovery_type, recovery_acts, recovery_trajs, cut_nodes);
    if (!success) {
        log("recovery planning failed", LogLevel::ERROR);
        return false;
    }

    log("recovery motion planning succeeded, found " + std::to_string(recovery_acts.size()) + " recovery activities", LogLevel::INFO);
    
    // 3 update the TPG/ADG with the recovery path 
    log("Starting recovery insertion for robot " + std::to_string(robot_id) + 
        " startNode timestep " + std::to_string(startNode->timeStep) + 
        " recovery_goal timestep " + std::to_string(recovery_goal->timeStep), LogLevel::INFO);

    if (recovery_acts.size() != recovery_trajs.size()) {
        log("number of recovery activities and trajectories do not match", LogLevel::ERROR);
        return false;
    }

    // delete the removed trajectory's activity
    for (auto act : skipped_acts) {
        act_graph_->remove_act(robot_id, act->act_id);
        log("Removed skipped activity at act_id " + std::to_string(act->act_id) + " for robot " + std::to_string(robot_id), LogLevel::INFO);
    }
    if (skipped_acts.size() > 0) {
        // remove intermediate nodes
        intermediate_nodes_[robot_id].erase(
            intermediate_nodes_[robot_id].begin() + skipped_acts[0]->act_id * 2,
            intermediate_nodes_[robot_id].begin() + (skipped_acts.back()->act_id + 1) * 2
        );
    }

    int cur_act_id = startNode->actId;
    std::vector<std::vector<NodePtr>> recovery_nodes;
    for (int i = 0; i < recovery_acts.size(); i++) {
        cur_act_id++;
        act_graph_->insert_act(robot_id, recovery_acts[i], cur_act_id);
        log("Inserted recovery activity at act_id " + std::to_string(cur_act_id) + " for robot " + std::to_string(robot_id), LogLevel::INFO);

        // convert the recovery trajectory to tpg nodes
        std::vector<NodePtr> recovery_nodes_i;
        for (int t = 0; t < recovery_trajs[i].trajectory.size(); t++) {
            NodePtr node = std::make_shared<Node>(robot_id, t);
            node->actId = cur_act_id; // the current recovery activity
            node->pose = recovery_trajs[i].trajectory[t];
            // connect with previous node
            if (recovery_nodes_i.size() > 0) {
                node->Type1Prev = recovery_nodes_i.back();
                node->Type1Prev->Type1Next = node;
            }
            recovery_nodes_i.push_back(node);
        }
        log("Created " + std::to_string(recovery_nodes_i.size()) + " nodes for recovery activity " + std::to_string(cur_act_id), LogLevel::INFO);
        recovery_nodes.push_back(recovery_nodes_i);
        if (i > 0) {
            recovery_nodes[i - 1].back()->Type1Next = recovery_nodes[i][0];
            recovery_nodes[i][0]->Type1Prev = recovery_nodes[i - 1].back();
            log("Connected recovery segment " + std::to_string(i-1) + " to segment " + std::to_string(i), LogLevel::INFO);
        }

        // insert a start node and end node to intermediate_nodes
        intermediate_nodes_[robot_id].insert(
            intermediate_nodes_[robot_id].begin() + cur_act_id * 2,
            recovery_nodes_i.front()
        );
        intermediate_nodes_[robot_id].insert(
            intermediate_nodes_[robot_id].begin() + cur_act_id * 2 + 1,
            recovery_nodes_i.back()
        );
    }

    // delete existing nodes between startNode and recovery_goal, and remove their outgoing deps
    log("Removing existing nodes between startNode and recovery_goal for robot " + std::to_string(robot_id), LogLevel::INFO);
    NodePtr iter_node = startNode;
    while (iter_node->Type1Next != nullptr && iter_node->Type1Next != recovery_goal) {       
        for (auto edge : iter_node->Type1Next->Type2Next) {
            edge->nodeFrom = nullptr;
            edge->nodeTo = nullptr;
            log("Cleared outgoing type2 edge for removed node at timestep " + std::to_string(iter_node->timeStep), LogLevel::DEBUG);
        }
        for (auto edge : iter_node->Type1Next->Type2Prev) {
            edge->nodeFrom = nullptr;
            edge->nodeTo = nullptr;
            log("Cleared incoming type2 edge for removed node at timestep " + std::to_string(iter_node->timeStep), LogLevel::DEBUG);
        }
        iter_node->Type1Next->Type2Next.clear();
        iter_node->Type1Next->Type2Prev.clear();
        iter_node = iter_node->Type1Next;
        log("Removing node at timestep " + std::to_string(iter_node->timeStep) + " actId " + std::to_string(iter_node->actId), LogLevel::DEBUG);
        // remove type1 edges
        iter_node->Type1Prev->Type1Next = nullptr;
        iter_node->Type1Prev = nullptr;
    }
    iter_node->Type1Next = nullptr;


    // add the recovery trajectory in the adg
    startNode->Type1Next = recovery_nodes[0][0];
    recovery_nodes[0][0]->Type1Prev = startNode;
    recovery_goal->Type1Prev = recovery_nodes.back().back();
    recovery_nodes.back().back()->Type1Next = recovery_goal;
    log("Linked recovery trajectory: startNode -> recovery_segments -> recovery_goal", LogLevel::INFO);

    // update the timestep of all the nodes in the adg after recovery trajectory
    log("Updating timesteps after inserting recovery trajectory for robot " + std::to_string(robot_id), LogLevel::INFO);
    // and also update the actId of nodes after recovery trajectory
    iter_node = startNode;
    while (iter_node ->Type1Next != nullptr && iter_node->Type1Next != recovery_goal) {
        iter_node = iter_node->Type1Next;
        iter_node->timeStep = iter_node->Type1Prev->timeStep + 1;
    }
    int next_act = cur_act_id + 1;
    while (iter_node->Type1Next != nullptr) {
        iter_node = iter_node->Type1Next;
        iter_node->timeStep = iter_node->Type1Prev->timeStep + 1;
        iter_node->actId = next_act;
        if (intermediate_nodes_[robot_id][next_act * 2 + 1] == iter_node) next_act++;
    }

    numNodes_[robot_id] = iter_node->timeStep + 1;
    log("Updated numNodes for robot " + std::to_string(robot_id) + " to " + std::to_string(numNodes_[robot_id]), LogLevel::INFO);


    // 4 insert type 2 dependencies for the recovery path
    // first check fort all other nodes, from their current node to cut, if there are any collisions
    // if there is, add a type 2 edge from recovery nodes to them

    log("Checking collisions from recovery path to earlier nodes (before cuts)", LogLevel::INFO);
    for (auto &cut : cut_nodes) {
        int rid = cut->robotId;
        int r_cur_node_id = executed_steps_[rid]->load();
        // NodePtr r_cur_node = start_nodes_[rid];
        // while (r_cur_node->timeStep != r_cur_node_id && r_cur_node != nullptr) {
        //     r_cur_node = r_cur_node->Type1Next;
        // }
        // if (r_cur_node == nullptr) {
        //     log("Error reading current node for robot " + std::to_string(rid) + " at timestep " + std::to_string(r_cur_node_id), LogLevel::ERROR);
        // }

        NodePtr iter_node = cut;
        NodePtr iter_end_node_i = recovery_nodes.back().back();
        // iterate from cut node to the current node
        while (iter_node != nullptr && iter_node->timeStep > r_cur_node_id) {
            for (int i = 0; i < recovery_nodes.size(); i++) {
                for (int t = 0; t < recovery_nodes[i].size(); t++) {
                    NodePtr node_it = recovery_nodes[i][t];
                    if (node_it->timeStep >= iter_end_node_i->timeStep) {
                        break;
                    }
                    // check for collisions with the recovery trajectory
                    bool in_collision = instance_->checkCollision({node_it->pose, iter_node->pose}, true);
                    if (in_collision) {
                        // add a type 2 edge from iter_node next to collision node
                        auto edge = std::make_shared<type2Edge>();
                        edge->edgeId = idType2Edges_++;
                        edge->nodeFrom = iter_node->Type1Next;
                        edge->nodeTo = node_it;
                        iter_node->Type1Next->Type2Next.push_back(edge);
                        node_it->Type2Prev.push_back(edge);
                        iter_end_node_i = node_it;
                        log("Added type2 edge (earlier) from robot " + std::to_string(iter_node->robotId) + 
                            " timestep " + std::to_string(iter_node->Type1Next->timeStep) +
                            " -> robot " + std::to_string(node_it->robotId) + " timestep " + std::to_string(node_it->timeStep),
                            LogLevel::INFO);
                        break;
                    }
                }
                if (recovery_nodes[i].back()->timeStep >= iter_end_node_i->timeStep) {
                    break;
                }
            }
            iter_node = iter_node->Type1Prev;
        }

    }

    // 5 add type 2 edge from recovery path to future pathes
    log("Checking collisions from recovery path to future nodes (after cuts)", LogLevel::INFO);
    for (auto &cut : cut_nodes) {
        int rid = cut->robotId;

        // iterate backwards from the recovery trajectory
        NodePtr iter_node_end = end_nodes_[rid];
        // run bfs to find all visited
        std::vector<std::vector<bool>> visited;
        for (int k = 0; k < num_robots_; k++) {
            std::vector<bool> v(numNodes_[k], false);
            visited.push_back(v);
        }
        bfs(recovery_goal, visited, false);

        for (int i = recovery_nodes.size() - 1; i >= 0; i--) {
            for (int t = recovery_nodes[i].size() - 1; t >= 0; t--) {
                NodePtr node_it = recovery_nodes[i][t];
                NodePtr iter_node = cut->Type1Next;
                while (iter_node != nullptr && iter_node->timeStep < iter_node_end->timeStep && !visited[iter_node->robotId][iter_node->timeStep]) {
                    // check if this is visited, break if already visited

                    // then check for collision
                    bool in_collision = instance_->checkCollision({node_it->pose, iter_node->pose}, true);
                    if (in_collision) {
                        // add a type 2 edge from collision node next to iter_node
                        auto edge = std::make_shared<type2Edge>();
                        edge->edgeId = idType2Edges_++;
                        edge->nodeFrom = node_it->Type1Next;
                        edge->nodeTo = iter_node;
                        node_it->Type1Next->Type2Next.push_back(edge);
                        iter_node->Type2Prev.push_back(edge);
                        iter_node_end = iter_node;
                        log("Added type2 edge (future) from robot " + std::to_string(node_it->robotId) + 
                            " timestep " + std::to_string(node_it->Type1Next->timeStep) +
                            " -> robot " + std::to_string(iter_node->robotId) + " timestep " + std::to_string(iter_node->timeStep),
                            LogLevel::INFO);
                        break;
                    }
                    iter_node = iter_node->Type1Next;
                }
            }
        }

    }

    // 6 save the updated activity graph
    act_graph_->saveGraphToFile(config_.output_dir + "/activity_graph_recover.txt");

    log("=========== Finished inserting recovery trajectory and type2 edges for robot " + std::to_string(robot_id) + " ========", LogLevel::INFO);
    return true;
}

bool ADG::shiftPolicyNodeType2Edges() {
    for (int i = 0; i < num_robots_; i++) {
        NodePtr policy_start_node = nullptr;
        NodePtr policy_end_node = nullptr;
        for (int act_id_i = 0; act_id_i < act_graph_->num_activities(i); act_id_i++) {
            auto act = act_graph_->get(i, act_id_i);
            Activity::Type type = act->type;
            // for policy nodes, we need to shift incoming type 2 edges to start node
            // for policy nodes, we need to shift outgoing type 2 edges to end node
            
            if (type == Activity::Type::drop_down || type == Activity::Type::pick_down 
                || type == Activity::Type::handover_down || type == Activity::Type::place_up) {
                policy_start_node = intermediate_nodes_[i][act_id_i * 2];
                policy_end_node = intermediate_nodes_[i][act_id_i * 2 + 5];
            }
            else if (type == Activity::Type::support || type == Activity::Type::press_down) {
                int last_sup_act_id = act_id_i;
                Activity::Type next_type = act_graph_->get(i, last_sup_act_id + 1)->type;
                while (next_type == type) {
                    last_sup_act_id++;
                    next_type = act_graph_->get(i, last_sup_act_id + 1)->type;
                }
                policy_start_node = intermediate_nodes_[i][act_id_i * 2];
                policy_end_node = intermediate_nodes_[i][last_sup_act_id * 2 + 1];
            }
            
            if (policy_start_node != nullptr) {
                if (type == Activity::Type::drop_down || type == Activity::Type::pick_down ||
                    type == Activity::Type::support || type == Activity::Type::handover_down ||
                    type == Activity::Type::place_up || type == Activity::Type::press_down ||
                    type == Activity::Type::drop_twist || type == Activity::Type::pick_twist ||
                    type == Activity::Type::handover_twist || type == Activity::place_twist ||
                    type == Activity::Type::drop_twist_up || type == Activity::Type::pick_twist_up ||
                    type == Activity::Type::handover_twist_up || type == Activity::Type::place_twist_down ||
                    type == Activity::Type::support_pre || type == Activity::Type::press_up) {
                
                    NodePtr node_i = intermediate_nodes_[i][act_id_i * 2]->Type1Next;
                    NodePtr end_node = intermediate_nodes_[i][act_id_i * 2 + 1];
                    while (node_i->timeStep <= end_node->timeStep && node_i->timeStep < policy_end_node->timeStep) {
                        for (auto edge : node_i->Type2Prev) {
                            edge->nodeTo = policy_start_node;
                            policy_start_node->Type2Prev.push_back(edge);
                            log("Moving type 2 edge -> " + act->type_string() + " " +  std::to_string(node_i->timeStep) + 
                                " to earlier node at timestep " + std::to_string(policy_start_node->timeStep), LogLevel::INFO);
                        }
                        node_i->Type2Prev.clear();
                        for (auto edge : node_i->Type2Next) {
                            edge->nodeFrom = policy_end_node;
                            policy_end_node->Type2Next.push_back(edge);
                            log("Moving type 2 edge <- " + act->type_string() + " " +  std::to_string(node_i->timeStep) + 
                                " to later node at timestep " + std::to_string(policy_end_node->timeStep), LogLevel::INFO);
                        }
                        node_i->Type2Next.clear();
                        node_i = node_i->Type1Next;
                    }
                } else {
                    policy_start_node = nullptr;
                }
            }

        }
    }

    if (hasCycle()) {
        log("cycle detected after shifting policy nodes", LogLevel::ERROR);
        return false;
    }
    return true;
}

void ADG::setExecStartAct(int robot_id, int act_id) {
    exec_start_act_.resize(num_robots_);
    if (robot_id >= exec_start_act_.size()) {
        log("robot id " + std::to_string(robot_id) + " is out of range", LogLevel::ERROR);
    }
    exec_start_act_[robot_id] = act_id;
}

NodePtr ADG::getExecStartNode(int robot_id) const {
    int act_id = exec_start_act_[robot_id];
    if (robot_id >= intermediate_nodes_.size()) {
        log("robot id is out of range for intermediate_nodes_", LogLevel::ERROR);
        return nullptr;
    }
    if ((act_id * 2 + 1) >= intermediate_nodes_[robot_id].size()) {
        log("act id is out of range", LogLevel::ERROR);
        return nullptr;
    }
    return intermediate_nodes_[robot_id][act_id * 2];
}

int ADG::getExecutedAct(int robot_id) const {
    if (executed_acts_.size() == 0) {
        return 0;
    }
    if (robot_id >= executed_acts_.size()) {
        log("robot id " + std::to_string(robot_id) + " is out of range for executed acts", LogLevel::WARN);
        return -1;
    }
    return executed_acts_[robot_id]->load();
}



}
