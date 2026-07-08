#ifndef ASTAR_H
#define ASTAR_H

#include <QHash>
#include <QList>
#include <QMap>
#include <QObject>
#include <QPoint>
#include <QSet>
#include <QTimer>

struct Node
{
    QPoint coords;  // coordinates of node
    // int g_score;    // currently known cost of the cheapest path from start to node
    // int f_score;    // current best guess as to how cheap a path could be from start to finish if it goes through node

    Node() { }
    Node(const QPoint &coords) { this->coords = coords; /*g_score = f_score = 0;*/ }
};

inline bool operator==(const Node &n1, const Node &n2)
{
    return n1.coords == n2.coords;
}

inline bool operator!=(const Node &n1, const Node &n2)
{
    return n1.coords != n2.coords;
}

inline size_t qHash(const Node &key, size_t seed)
{
    Q_UNUSED(seed);
    return (key.coords.x() & 0xffff) | ((key.coords.y() & 0xffff) << 16);
}

typedef QList<Node> NodeList;
typedef QSet<Node> NodeSet;
typedef QMap<int, NodeSet> NodePriorityMap;
typedef QHash<Node, int> NodeCost;
typedef QHash<Node, Node> CameFrom;

class OpenSet
{
public:
    explicit OpenSet();

    enum Implementation { ImplementationSet, ImplementationPriorityMap, };

    const NodeSet &set() const;
    const NodePriorityMap priority_map() const;

    int count() const;
    bool isEmpty() const;
    bool contains(const Node &node, int f_score) const;
    void clear();
    void add(const Node &node, int f_score);
    bool remove(const Node &node, int f_score);

    Implementation implementation() const;
    void setImplementation(Implementation newImplementation);

private:
    Implementation _implementation;
    NodeSet _set;
    NodePriorityMap _priority_map;
};

class AStar : public QObject
{
    Q_OBJECT
public:
    explicit AStar(QObject *parent = nullptr);

    enum NodeSelectorMethod { NodeSelectorFirst, NodeSelectorLowestSequential, NodeSelectorLowestPriorityMap, };
    Q_ENUM(NodeSelectorMethod);
    enum NodeSelectorHeuristicMethod { NodeSelectorHeuristicDijkstra, NodeSelectorHeuristicManhattan, NodeSelectorHeuristicEuclidean, NodeSelectorHeuristicEuclideanWeighted, };
    Q_ENUM(NodeSelectorHeuristicMethod);
    enum NodeState { StateRemoved, StateCurrent, StateOpen, StateClosed, StatePath };
    Q_ENUM(NodeState);
    enum FindPathStep { StepPickCurrent, StepAddNeighbors };

    bool show_progress() const;
    void setShow_progress(bool newShow_progress);
    void set_coord_sizes(int x, int y);
    void set_edge_length(int new_edge_length);
    void set_node_selector_method(NodeSelectorMethod nodeSelectorMethod);
    void set_node_selector_heuristic_method(NodeSelectorHeuristicMethod nodeSelectorHeuristicMethod);
    NodeList find_path(const QPoint &startCoords, const QPoint &goalCoords);
    void find_path_async(const QPoint &startCoords, const QPoint &goalCoords);
    void cancel_find_path_async();

signals:
    void findPathAsyncStarted();
    void findPathAsyncStopped();
    void nodeStatusChanged(Node node, NodeState state);

private slots:
    void async_timer_timeout();

private:
    bool _show_progress;
    int edge_length;
    int x_coord_size, y_coord_size;
    NodeSelectorMethod nodeSelectorMethod;
    typedef Node (AStar::*NodeSelector)() const;
    NodeSelector nodeSelector;
    NodeSelectorHeuristicMethod nodeSelectorHeuristicMethod;
    typedef int (AStar::*NodeSelectorHeuristic)(const Node &reached, const QPoint &goalCoords) const;
    NodeSelectorHeuristic nodeSelectorHeuristic;
    FindPathStep findPathNextStep;

    mutable struct Stats {
        int iterations;
        int longest_open_set_count;
        int total_open_set_count;
        int node_lowest_f_score_count;

        int node_lowest_f_score_average()
        {
            return iterations > 0 ? int((node_lowest_f_score_count + iterations / 2) / iterations) : 0;
        }
    } stats;

    CameFrom came_from;
    NodeCost g_score, f_score;
    OpenSet open_set;
    QPoint start_coords, goal_coords;
    Node current_node;

    QTimer async_timer;

    void async_timer_start();
    void async_timer_stop();
    void emit_node_status_changed(const Node &node, NodeState state);

    bool find_path_start();
    void find_path_step();
    bool find_path_is_finished();
    void find_path_finish();
    NodeList find_path_result();

    NodeList reconstruct_path(const Node &reached) const;
    void report_stats() const;
    int heuristic_dijkstra(const Node &reached, const QPoint &goalCoords) const;
    int heuristic_manhattan(const Node &reached, const QPoint &goalCoords) const;
    int heuristic_euclidean(const Node &reached, const QPoint &goalCoords) const;
    int heuristic_euclidean_weighted(const Node &reached, const QPoint &goalCoords) const;
    Node node_first_f_score() const;
    Node node_lowest_sequential_f_score() const;
    Node node_lowest_priority_map_f_score() const;
    NodeList get_neighbors(const Node &node) const;
    bool neighbor_traversable(const Node &from, const Node &to) const;
};

#endif // ASTAR_H
