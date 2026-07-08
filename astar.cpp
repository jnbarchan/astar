#include <climits>

#include <QDebug>
#include <QRect>

#include "astar.h"


OpenSet::OpenSet()
{
}

OpenSet::Implementation OpenSet::implementation() const
{
    return _implementation;
}

void OpenSet::setImplementation(Implementation newImplementation)
{
    _implementation = newImplementation;
}

const NodeSet &OpenSet::set() const
{
    return _set;
}

const NodePriorityMap OpenSet::priority_map() const
{
    return _priority_map;
}

int OpenSet::count() const
{
    switch (_implementation)
    {
    case ImplementationSet:
        return _set.count();
    case ImplementationPriorityMap: {
        int total = 0;
        for (auto it = _priority_map.cbegin(), end = _priority_map.end(); it != end; it++)
            total += it->count();
        return total;
    }
    }
    __builtin_unreachable();
}

bool OpenSet::isEmpty() const
{
    switch (_implementation)
    {
    case ImplementationSet:
        return _set.isEmpty();
    case ImplementationPriorityMap: {
        for (auto it = _priority_map.cbegin(), end = _priority_map.end(); it != end; it++)
            if (it->count() != 0)
                return false;
        return true;
    }
    }
    __builtin_unreachable();
}

bool OpenSet::contains(const Node &node, int f_score) const
{
    switch (_implementation)
    {
    case ImplementationSet:
        Q_UNUSED(f_score);
        return _set.contains(node);
    case ImplementationPriorityMap:
        return _priority_map.value(f_score).contains(node);
    }
    __builtin_unreachable();
}

void OpenSet::clear()
{
    _set.clear();
    _priority_map.clear();
}

void OpenSet::add(const Node &node, int f_score)
{
    switch (_implementation)
    {
    case ImplementationSet:
        Q_UNUSED(f_score);
        _set.insert(node);
        return;
    case ImplementationPriorityMap:
        _priority_map[f_score].insert(node);
        return;
    }
    __builtin_unreachable();
}

bool OpenSet::remove(const Node &node, int f_score)
{
    switch (_implementation)
    {
    case ImplementationSet:
        Q_UNUSED(f_score);
        return _set.remove(node);
    case ImplementationPriorityMap:
        if (!_priority_map.contains(f_score))
            return false;
        if (!_priority_map[f_score].remove(node))
            return false;
        if (_priority_map[f_score].isEmpty())
            _priority_map.remove(f_score);
        return true;
    }
    __builtin_unreachable();
}


AStar::AStar(QObject *parent)
    : QObject{parent}
{
    edge_length = 1000;
    x_coord_size = y_coord_size = 0;
    nodeSelectorMethod = AStar::NodeSelectorFirst;
    nodeSelector = &AStar::node_first_f_score;
    nodeSelectorHeuristic = nullptr;

    async_timer.setInterval(50);
    connect(&async_timer, &QTimer::timeout, this, &AStar::async_timer_timeout);
}

bool AStar::show_progress() const
{
    return _show_progress;
}

void AStar::setShow_progress(bool newShow_progress)
{
    _show_progress = newShow_progress;
}

void AStar::set_coord_sizes(int x, int y)
{
    x_coord_size = x;
    y_coord_size = y;
}

void AStar::set_edge_length(int new_edge_length)
{
    edge_length = new_edge_length;
}

void AStar::set_node_selector_method(NodeSelectorMethod nodeSelectorMethod)
{
    switch (nodeSelectorMethod)
    {
    case NodeSelectorFirst: nodeSelector = &AStar::node_first_f_score; break;
    case NodeSelectorLowestSequential: nodeSelector = &AStar::node_lowest_sequential_f_score; break;
    case NodeSelectorLowestPriorityMap: nodeSelector = &AStar::node_lowest_priority_map_f_score; break;
    default: Q_ASSERT(false);
    }
    this->nodeSelectorMethod = nodeSelectorMethod;
}

void AStar::set_node_selector_heuristic_method(NodeSelectorHeuristicMethod nodeSelectorHeuristicMethod)
{
    switch (nodeSelectorHeuristicMethod)
    {
    case NodeSelectorHeuristicDijkstra: nodeSelectorHeuristic = &AStar::heuristic_dijkstra; break;
    case NodeSelectorHeuristicManhattan: nodeSelectorHeuristic = &AStar::heuristic_manhattan; break;
    case NodeSelectorHeuristicEuclidean: nodeSelectorHeuristic = &AStar::heuristic_euclidean; break;
    case NodeSelectorHeuristicEuclideanWeighted: nodeSelectorHeuristic = &AStar::heuristic_euclidean_weighted; break;
    default: Q_ASSERT(false);
    }
    this->nodeSelectorHeuristicMethod = nodeSelectorHeuristicMethod;
}

// A* finds a path from start to goal.
// h is the heuristic function. h(n) estimates the cost to reach goal from node n.
NodeList AStar::find_path(const QPoint &startCoords, const QPoint &goalCoords)
{
    start_coords = startCoords;
    goal_coords = goalCoords;

    async_timer_stop();

    // initialise
    if (!find_path_start())
        return NodeList();

    // while open_set is not empty and current_node != goal
    while (!find_path_is_finished())
        find_path_step();

    // finished finding path
    find_path_finish();

    // return find path result (reconstructed path)
    return find_path_result();
}

void AStar::find_path_async(const QPoint &startCoords, const QPoint &goalCoords)
{
    start_coords = startCoords;
    goal_coords = goalCoords;

    async_timer_stop();

    // initialise
    if (!find_path_start())
        return;

    // start async timer, calling async_timer_timeout()
    async_timer_start();
}

void AStar::cancel_find_path_async()
{
    async_timer_stop();
}

void AStar::async_timer_start()
{
    // start async timer, calling async_timer_timeout()
    async_timer.start();
    emit findPathAsyncStarted();
}

void AStar::async_timer_stop()
{
    async_timer.stop();
    emit findPathAsyncStopped();
}

void AStar::async_timer_timeout()
{
    // while open_set is not empty and current_node != goal
    if (!find_path_is_finished())
    {
        // do next step in finding path
        find_path_step();
        return;
    }

    // stop async timer
    async_timer_stop();

    // finished finding path
    find_path_finish();
    // show find path result (reconstructed path)
    NodeList path = find_path_result();
    Q_UNUSED(path);
}

void AStar::emit_node_status_changed(const Node &node, NodeState state)
{
    if (show_progress())
        emit nodeStatusChanged(node, state);
}

bool AStar::find_path_start()
{
    QRect rect(0, 0, x_coord_size, y_coord_size);
    if (!rect.contains(start_coords) || !rect.contains(goal_coords))
        return false;

    stats.iterations = 0;
    stats.longest_open_set_count = 0;
    stats.node_lowest_f_score_count = 0;
    stats.total_open_set_count = 0;

    current_node = Node({-1, -1});
    Node start(start_coords);

    // // For node n, came_from[n] is the node immediately preceding it on the cheapest path from the start
    // // to n currently known.
    // came_from := an empty map
    came_from.clear();

    // // For node n, g_score[n] is the currently known cost of the cheapest path from start to n.
    // g_score := map with default value of Infinity
    // g_score[start] := 0
    g_score.clear();
    g_score[start] = 0;

    // // For node n, f_score[n] := g_score[n] + h(n). f_score[n] represents our current best guess as to
    // // how cheap a path could be from start to finish if it goes through n.
    // f_score := map with default value of Infinity
    // f_score[start] := h(start)
    f_score.clear();
    f_score[start] = (this->*nodeSelectorHeuristic)(start, goal_coords);

    // // The set of discovered nodes that may need to be (re-)expanded.
    // // Initially, only the start node is known.
    // // This is usually implemented as a min-heap or priority queue rather than a hash-set.
    // open_set := {start}
    open_set.clear();
    switch (nodeSelectorMethod)
    {
    case NodeSelectorFirst:
    case NodeSelectorLowestSequential:
        open_set.setImplementation(OpenSet::ImplementationSet); break;
    case NodeSelectorLowestPriorityMap:
        open_set.setImplementation(OpenSet::ImplementationPriorityMap); break;
    }
    open_set.add(start, f_score[start]);
    emit_node_status_changed(start, StateOpen);

    stats.longest_open_set_count = 1;
    stats.total_open_set_count = 1;

    findPathNextStep = StepPickCurrent;

    return true;
}

void AStar::find_path_step()
{
    switch (findPathNextStep)
    {
    case StepPickCurrent:
    {
        // while open_set is not empty
        //     // This operation can occur in O(Log(N)) time if open_set is a min-heap or a priority queue
        //     current := the node in open_set having the lowest f_score[] value
        //     if current = goal
        //         return reconstruct_path(came_from, current)

        if (open_set.isEmpty())
            return;

        stats.iterations++;
        current_node = (this->*nodeSelector)();
        emit_node_status_changed(current_node, StateCurrent);
        if (current_node.coords == goal_coords)
            return;

        findPathNextStep = StepAddNeighbors;
        return;
    }
    case StepAddNeighbors:
    {
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

        Q_ASSERT(g_score.contains(current_node));
        Q_ASSERT(f_score.contains(current_node));
        int g_score_current = g_score.value(current_node);
        int f_score_current = f_score.value(current_node);

        //TEMPORARY
        if (show_progress())
            qDebug("Closing (%d,%d) g=%d h=%d f=%d",
                   current_node.coords.x(), current_node.coords.y(),
                   g_score_current, f_score_current - g_score_current, f_score_current);

        bool removed = open_set.remove(current_node, f_score_current);
        Q_ASSERT(removed);
        if (removed)
            emit_node_status_changed(current_node, StateClosed);

        NodeList neighbors = get_neighbors(current_node);
        for (const Node &neighbor : neighbors)
        {
            int tentative_g_score = g_score_current + edge_length;
            int g_score_neighbor = g_score.value(neighbor, INT_MAX);
            if (tentative_g_score < g_score_neighbor)
            {
                int tentative_f_score = tentative_g_score + (this->*nodeSelectorHeuristic)(neighbor, goal_coords);
                int f_score_neighbor = f_score.value(neighbor, INT_MAX);
                came_from[neighbor] = current_node;
                g_score[neighbor] = tentative_g_score;
                f_score[neighbor] = tentative_f_score;
                if (f_score_neighbor != INT_MAX)
                    open_set.remove(neighbor, f_score_neighbor);
                open_set.add(neighbor, tentative_f_score);
                emit_node_status_changed(neighbor, StateOpen);
                stats.total_open_set_count++;
                int open_set_count = open_set.count();
                if (open_set_count > stats.longest_open_set_count)
                    stats.longest_open_set_count = open_set_count;
            }
        }

        findPathNextStep = StepPickCurrent;
        return;
    }
    }
    Q_ASSERT(false);
}

bool AStar::find_path_is_finished()
{
    return open_set.isEmpty() || current_node.coords == goal_coords;
}

void AStar::find_path_finish()
{
    report_stats();
}

NodeList AStar::find_path_result()
{
    if (open_set.isEmpty() || current_node.coords != goal_coords)
        return NodeList();
    NodeList path = reconstruct_path(current_node);
    for (const Node &node : path)
        emit_node_status_changed(node, StatePath);
    return path;
}


NodeList AStar::reconstruct_path(const Node &reached) const
{
    // total_path := {current}
    // while current in came_from.keys:
    //     current := came_from[current]
    //     total_path.prepend(current)
    // return total_path
    NodeList total_path = {reached};
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

int AStar::heuristic_dijkstra(const Node &reached, const QPoint &goalCoords) const
{
    Q_UNUSED(reached);
    Q_UNUSED(goalCoords);
    return 0;
}

int AStar::heuristic_manhattan(const Node &reached, const QPoint &goalCoords) const
{
    QPoint delta(goalCoords - reached.coords);
    return delta.manhattanLength() * edge_length;
}

int AStar::heuristic_euclidean(const Node &reached, const QPoint &goalCoords) const
{
    QPoint delta(goalCoords - reached.coords);
    return std::lround(std::hypot(delta.x(), delta.y()) * edge_length);
}

int AStar::heuristic_euclidean_weighted(const Node &reached, const QPoint &goalCoords) const
{
    QPoint delta(goalCoords - reached.coords);
    return 2 * std::lround(std::hypot(delta.x(), delta.y()) * edge_length);
}

Node AStar::node_first_f_score() const
{
    const NodeSet &set(open_set.set());
    auto it = set.cbegin();
    Node lowest = *it;
    Q_ASSERT(f_score.contains(lowest));
    stats.node_lowest_f_score_count++;
    return lowest;
}

Node AStar::node_lowest_sequential_f_score() const
{
    const NodeSet &set(open_set.set());
    auto it = set.cbegin(), end = set.cend();
    Node lowest = *it;
    Q_ASSERT(f_score.contains(lowest));
    stats.node_lowest_f_score_count++;
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

Node AStar::node_lowest_priority_map_f_score() const
{
    const NodePriorityMap &priority_map(open_set.priority_map());
    for (auto it = priority_map.cbegin(); it != priority_map.cend(); it++)
    {
        stats.node_lowest_f_score_count++;
        const NodeSet &set(it.value());
        if (set.isEmpty())
            continue;
        auto it2 = set.cbegin();
        auto best = *it2;
        // algorithm picks Node with longest g_score
        while (++it2 != set.cend())
            if (g_score[*it2] > g_score[best])
                best = *it2;
        return best;
    }
    Q_ASSERT(false);
    return Node();
}


NodeList AStar::get_neighbors(const Node &node) const
{
    NodeList neighbors;
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
