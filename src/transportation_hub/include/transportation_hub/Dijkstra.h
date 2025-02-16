//
// Created by roc on 25-2-16.
//

#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include <unordered_map>
#include <vector>
#include <queue>

using namespace std;

vector<int> dijkstra(
    const unordered_map<int, vector<pair<int, int>>>& graph,
    int src,
    int des
);

#endif // DIJKSTRA_H