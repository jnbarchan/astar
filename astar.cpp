#include <climits>

#include <QDebug>
#include <QRect>

#include "astar.h"

AStar::AStar(QObject *parent)
    : QObject{parent}
{
    x_coord_size = y_coord_size = 0;
    nodeSelector = &AStar::node_first_f_score;
}

void AStar::set_coord_sizes(int x, int y)
{
    x_coord_size = x;
    y_coord_size = y;
}

void AStar::set_node_selector_method(NodeSelectorMethod nodeSelectorMethod)
{
    switch (nodeSelectorMethod)
    {
    case NodeSelectorFirst: nodeSelector = &AStar::node_first_f_score; break;
    case NodeSelectorLowest: nodeSelector = &AStar::node_lowest_f_score; break;
    default: Q_ASSERT(false);
    }
}

// A* finds a path from start to goal.
// h is the heuristic function. h(n) estimates the cost to reach goal from node n.
QList<Node> AStar::find_path(const QPoint &startCoords, const QPoint &goalCoords) const
{
    QRect rect(0, 0, x_coord_size, y_coord_size);
    if (!rect.contains(startCoords) || !rect.contains(goalCoords))
        return QList<Node>();

    // // The set of discovered nodes that may need to be (re-)expanded.
    // // Initially, only the start node is known.
    // // This is usually implemented as a min-heap or priority queue rather than a hash-set.
    // open_set := {start}
    NodeSet open_set;
    Node start(startCoords);
    open_set += start;

    // // For node n, came_from[n] is the node immediately preceding it on the cheapest path from the start
    // // to n currently known.
    // came_from := an empty map
    CameFrom came_from;

    // // For node n, g_score[n] is the currently known cost of the cheapest path from start to n.
    // g_score := map with default value of Infinity
    // g_score[start] := 0
    NodeCost g_score;
    g_score[start] = 0;

    // // For node n, f_score[n] := g_score[n] + h(n). f_score[n] represents our current best guess as to
    // // how cheap a path could be from start to finish if it goes through n.
    // f_score := map with default value of Infinity
    // f_score[start] := h(start)
    NodeCost f_score;
    f_score[start] = heuristic(start, goalCoords);

    // while open_set is not empty
    //     // This operation can occur in O(Log(N)) time if open_set is a min-heap or a priority queue
    //     current := the node in open_set having the lowest f_score[] value
    //     if current = goal
    //         return reconstruct_path(came_from, current)

    //     open_set.remove(current)
    //     for each neighbor of current
    //         // d(current,neighbor) is the weight of the edge from current to neighbor
    //         // tentative_g_score is the distance from start to the neighbor through current
    //         tentative_g_score := g_score[current] + d(current, neighbor)
    //         if tentative_g_score < g_score[neighbor]
    //             // This path to neighbor is better than any previous one. Record it!
    //             came_from[neighbor] := current
    //             g_score[neighbor] := tentative_g_score
    //             f_score[neighbor] := tentative_g_score + h(neighbor)
    //             if neighbor not in open_set
    //                 open_set.add(neighbor)

    stats.iterations = 0;
    stats.longest_open_set_count = 0;
    stats.node_lowest_f_score_count = 0;
    stats.total_open_set_count = 0;

    // Open set is empty but goal was never reached
    // return failure;
    while (!open_set.isEmpty())
    {
        stats.iterations++;
        Node current;
        current = (this->*nodeSelector)(open_set, f_score);
        if (current.coords == goalCoords)
        {
            report_stats();
            return reconstruct_path(came_from, current);
        }

        open_set.remove(current);

        Q_ASSERT(g_score.contains(current));
        int g_score_current = g_score.value(current);
        QList<Node> neighbors = get_neighbors(current);
        for (const Node &neighbor : neighbors)
        {
            // //VERYTEMPORARY
            // if (g_score.contains(neighbor))
            //     continue;

            int tentative_g_score = g_score_current + 1;
            int g_score_neighbor = g_score.value(neighbor, INT_MAX);
            if (tentative_g_score < g_score_neighbor)
            {
                came_from[neighbor] = current;
                g_score[neighbor] = tentative_g_score;
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goalCoords);
                if (!open_set.contains(neighbor))
                {
                    open_set += neighbor;
                    stats.total_open_set_count++;
                    if (open_set.count() > stats.longest_open_set_count)
                        stats.longest_open_set_count = open_set.count();
                }
            }
        }
    }

    // Open set is empty but goal was never reached
    report_stats();
    return QList<Node>();
}

QList<Node> AStar::reconstruct_path(const CameFrom &came_from, const Node &reached) const
{
    // total_path := {current}
    // while current in came_from.keys:
    //     current := came_from[current]
    //     total_path.prepend(current)
    // return total_path
    QList<Node> total_path = {reached};
    Node current(reached);
    while (came_from.contains(current))
    {
        current = came_from.value(current);
        total_path.prepend(current);
    }
    return total_path;
}

void AStar::report_stats() const
{
    qDebug() << "iterations:" << stats.iterations;
    qDebug() << "longest_open_set_count:" << stats.longest_open_set_count << ","
             << "total_open_set_count:" << stats.total_open_set_count << ","
             << "node_lowest_f_score_count:" << stats.node_lowest_f_score_count << ","
             << "node_lowest_f_score_average:" << stats.node_lowest_f_score_average();
}

int AStar::heuristic(const Node &reached, const QPoint &goalCoords) const
{
    QPoint delta(goalCoords - reached.coords);
    return delta.manhattanLength();
}

Node AStar::node_first_f_score(const NodeSet &open_set, const NodeCost &f_score) const
{
    Q_UNUSED(f_score);
    auto it = open_set.cbegin();
    Node lowest = *it;
    Q_ASSERT(f_score.contains(lowest));
    return lowest;
}

Node AStar::node_lowest_f_score(const NodeSet &open_set, const NodeCost &f_score) const
{
    auto it = open_set.cbegin(), end = open_set.cend();
    Node lowest = *it;
    Q_ASSERT(f_score.contains(lowest));
    while (++it != end)
    {
        stats.node_lowest_f_score_count++;
        Node node(*it);
        Q_ASSERT(node != lowest);
        Q_ASSERT(f_score.contains(node));
        if (f_score.value(node) < f_score.value(lowest))
            lowest = node;
    }
    return lowest;
}

QList<Node> AStar::get_neighbors(const Node &node) const
{
    QList<Node> neighbors;
    int x, y;
    if ((x = node.coords.x() - 1) >= 0)
        neighbors.append(Node(QPoint(x, node.coords.y())));
    if ((x = node.coords.x() + 1) < x_coord_size)
        neighbors.append(Node(QPoint(x, node.coords.y())));
    if ((y = node.coords.y() - 1) >= 0)
        neighbors.append(Node(QPoint(node.coords.x(), y)));
    if ((y = node.coords.y() + 1) < y_coord_size)
        neighbors.append(Node(QPoint(node.coords.x(), y)));

    for (int i = neighbors.length() - 1; i >= 0; i--)
        if (!neighbor_traversable(node, neighbors.at(i)))
            neighbors.removeAt(i);

    return neighbors;
}

bool AStar::neighbor_traversable(const Node &from, const Node &to) const
{
    return true;
}
