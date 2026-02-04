#ifndef GRAPH_H
#define GRAPH_H

#include "mr_planner/planning/SingleAgentPlanner.h"
#include <vector>
#include <unordered_set>
#include <memory>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/unordered_set.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/shared_ptr.hpp>

struct SafeInterval {
    double t_start;
    double t_end;
    RobotPose pose;
};

class Vertex {
public:
    Vertex() {} // default ctor for serialization

    Vertex(const RobotPose &pose) : pose(pose) {};

    Vertex(const RobotPose &pose, int id) : pose(pose), id(id) {};

    void setPose(const RobotPose &pose) {
        this->pose = pose;
    }

    void setTime(double time) {
        this->time = time;
        time_set = true;
    }

    void setOtherTime(double time) {
        this->other_time = time;
    }   

    void setSafeInterval(const SafeInterval &si) {
        this->si = si;
    }

    void addParent(std::shared_ptr<Vertex> parent) {
        this->parent = parent;
        // find the root of the tree
        this->root = parent;
        while (this->root->parent != nullptr) {
            this->root = this->root->parent;
        }
    }

    void addOtherParent(std::shared_ptr<Vertex> other_parent) {
        this->otherParent = other_parent;

        this->otherRoot = other_parent;
        while (this->otherRoot->parent != nullptr) {
            this->otherRoot = this->otherRoot->parent;
        }
    }

    RobotPose pose;
    SafeInterval si;
    int id;
    double time;
    double other_time;
    bool time_set = false;
    std::shared_ptr<Vertex> parent;
    std::shared_ptr<Vertex> otherParent; // if this node is a connection point, it has another parent
    std::shared_ptr<Vertex> root;
    std::shared_ptr<Vertex> otherRoot;

private:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version) {
        ar & pose;
        ar & id;
        ar & time;
        ar & time_set;
        // Currently no serialization for pointer members.
    }
};
using VertexPtr = std::shared_ptr<Vertex>;

// Union find with all optimizations
class UnionFind {
public:
    UnionFind() = default;

    explicit UnionFind(int n) { reset(n); }

    void reset(int n) {
        parent.resize(n);
        rank.assign(n, 0);
        for (int i = 0; i < n; ++i) {
            parent[i] = i;
        }
    }

    void ensureSize(int n) {
        if (n <= static_cast<int>(parent.size())) {
            return;
        }
        const int old_size = static_cast<int>(parent.size());
        parent.resize(n);
        rank.resize(n, 0);
        for (int i = old_size; i < n; ++i) {
            parent[i] = i;
        }
    }

    int find(int u) {
        if (parent[u] != u) {
            parent[u] = find(parent[u]); // Path compression
        }
        return parent[u];
    }
    void unionSets(int u, int v) {
        int rootU = find(u);
        int rootV = find(v);
        if (rootU != rootV) {
            // Union by rank
            if (rank[rootU] > rank[rootV]) {
                parent[rootV] = rootU;
            } else if (rank[rootU] < rank[rootV]) {
                parent[rootU] = rootV;
            } else {
                parent[rootV] = rootU;
                rank[rootU]++;
            }
        }
    }
private:
    std::vector<int> parent;
    std::vector<int> rank;

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version) {
        ar & parent;
        ar & rank;
    }
};

class Tree {
public:
    Tree() {};

    void addVertex(VertexPtr vertex) {
        vertices.push_back(vertex);
        vertex_map[vertex->pose].push_back(vertex);
    }

    void addRoot(VertexPtr root) {
        roots.push_back(root);
        vertices.push_back(root);
        vertex_map[root->pose].push_back(root);
    }

    void removeVertex(VertexPtr vertex) {
        for (auto it = vertices.begin(); it != vertices.end(); ++it) {
            if (*it == vertex) {
                vertices.erase(it);
                break;
            }
        }
    }

    std::vector<VertexPtr> roots;
    std::vector<VertexPtr> vertices;
    std::unordered_map<RobotPose, std::vector<VertexPtr>> vertex_map;
};

enum GrowState {
    ADVANCED,
    TRAPPED,
    REACHED
};


class Graph {
    public:
        Graph() {}

        std::shared_ptr<Vertex> addVertex(const RobotPose &pose) {
            auto vertex = std::make_shared<Vertex>(pose, size);
            vertices.push_back(vertex);
            adjList.emplace_back();
            size++;
            vertex_map_[pose] = vertex;
            uf_.ensureSize(size);
            return vertex;
        }

        void addEdge(std::shared_ptr<Vertex> u, std::shared_ptr<Vertex> v) {
            adjList[u->id].insert(v);
            adjList[v->id].insert(u);
            uf_.unionSets(u->id, v->id);
        }
        
        const std::unordered_set<std::shared_ptr<Vertex>> &getNeighbors(const std::shared_ptr<Vertex> &v) const {
            return adjList[v->id];
        }

        std::shared_ptr<Vertex> getVertex(const RobotPose &pose) {
            auto it = vertex_map_.find(pose);
            if (it != vertex_map_.end()) {
                return it->second;
            }
            return nullptr;
        }

        bool inSameComponent(const std::shared_ptr<Vertex> &u, const std::shared_ptr<Vertex> &v) {
            return uf_.find(u->id) == uf_.find(v->id);
        }

        int size = 0;
        int num_neighbors = 25;
        std::vector<std::shared_ptr<Vertex>> vertices;
        std::vector<std::unordered_set<std::shared_ptr<Vertex>>> adjList;
        std::unordered_map<RobotPose, std::shared_ptr<Vertex>, RobotPoseHash> vertex_map_;
        UnionFind uf_;

    private:
        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive & ar, const unsigned int version) {
            ar & size;
            ar & num_neighbors;
            ar & vertices;
            ar & adjList;
            ar & vertex_map_;
            ar & uf_;
        }
};

#endif // GRAPH_H
