#ifndef ASTAR_H
#define ASTAR_H

#include <QHash>
#include <QList>
#include <QMap>
#include <QObject>
#include <QPoint>
#include <QSet>

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

typedef QSet<Node> NodeSet;
typedef QMap<int, NodeSet> NodePriorityMap;
typedef QHash<Node, int> NodeCost;
typedef QHash<Node, Node> CameFrom;

class OpenSet;
class AStar : public QObject
{
    Q_OBJECT
public:
    explicit AStar(QObject *parent = nullptr);

    enum NodeSelectorMethod { NodeSelectorFirst, NodeSelectorLowestSequential, NodeSelectorLowestPriorityMap, };
    Q_ENUM(NodeSelectorMethod);

    void set_coord_sizes(int x, int y);
    void set_node_selector_method(NodeSelectorMethod nodeSelectorMethod);
    QList<Node> find_path(const QPoint &startCoords, const QPoint &goalCoords) const;

signals:

private:
    int x_coord_size, y_coord_size;
    NodeSelectorMethod nodeSelectorMethod;
    typedef Node (AStar::*NodeSelector)(const OpenSet &, const NodeCost &) const;
    NodeSelector nodeSelector;
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

    QList<Node> reconstruct_path(const CameFrom &came_from, const Node &reached) const;
    void report_stats() const;
    int heuristic(const Node &reached, const QPoint &goalCoords) const;
    Node node_first_f_score(const OpenSet &open_set, const NodeCost &f_score) const;
    Node node_lowest_sequential_f_score(const OpenSet &open_set, const NodeCost &f_score) const;
    Node node_lowest_priority_map_f_score(const OpenSet &open_set, const NodeCost &f_score) const;
    QList<Node> get_neighbors(const Node &node) const;
    bool neighbor_traversable(const Node &from, const Node &to) const;
};

class OpenSet
{
public:
    explicit OpenSet(AStar::NodeSelectorMethod node_selector_method);

    const NodeSet &set() const;
    const NodePriorityMap priority_map() const;

    int count() const;
    bool isEmpty() const;
    bool contains(const Node &node, int f_score) const;
    void add(const Node &node, int f_score);
    bool remove(const Node &node, int f_score);

private:
    AStar::NodeSelectorMethod node_selector_method;
    NodeSet _set;
    NodePriorityMap _priority_map;
};

#endif // ASTAR_H
