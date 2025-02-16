//
// Created by roc on 25-2-16.
//
#include "dijkstra.h"

vector<int> dijkstra(
    const unordered_map<int, vector<pair<int, int>>>& graph,
    int src,
    int des
) {
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<>> pq;
    unordered_map<int, int> distances;
    unordered_map<int, int> predecessors;

    for (const auto& city : graph) {
        distances[city.first] = numeric_limits<int>::max();
    }
    distances[src] = 0;

    pq.emplace(0, src);

    while (!pq.empty()) {
        int current_dist = pq.top().first;
        int current_node = pq.top().second;
        pq.pop();

        if (current_node == des) break;

        if (current_dist > distances[current_node]) continue;

        for (const auto& neighbor : graph.at(current_node)) {
            int neighbor_node = neighbor.first;
            int edge_weight = neighbor.second;
            int new_dist = current_dist + edge_weight;

            if (new_dist < distances[neighbor_node]) {
                distances[neighbor_node] = new_dist;
                predecessors[neighbor_node] = current_node;
                pq.emplace(new_dist, neighbor_node);
            }
        }
    }

    vector<int> path;
    int current_node = des;
    while (current_node != src) {
        path.push_back(current_node);
        current_node = predecessors.at(current_node);
    }
    path.push_back(src);
    reverse(path.begin(), path.end());

    return path;
}