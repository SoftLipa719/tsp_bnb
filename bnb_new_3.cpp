#include <iostream>
#include <vector>
#include <fstream>
#include <chrono>
#include <limits>
#include <algorithm>
#include <queue>
#include <unordered_map>
#include <list>

using namespace std;

const double INF = numeric_limits<double>::infinity();
atomic<bool> timeout_occurred(false);
double hitNum = 0;
double allNum = 0;

// Struct to hold node index and combined value
struct Node {
    int index;
    double combined_value;
};

// Function to read the distance matrix from file
vector<vector<double>> read_distance_matrix(const string& filename) {
    ifstream inputFile(filename);
    int N;
    inputFile >> N;
    vector<vector<double>> adj(N, vector<double>(N));
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            inputFile >> adj[i][j];
        }
    }
    return adj;
}

// Function to compute Minimum Spanning Tree (MST) cost using Prim's algorithm
double compute_mst_cost(const vector<vector<double>>& adj, const vector<bool>& visited) {
    int N = adj.size();
    vector<double> key(N, INF);
    vector<bool> inMST(N, false);
    double mst_cost = 0;

    // Priority queue to select the minimum edge
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pq;

    pq.push({0, 0}); // Start from the first node

    while (!pq.empty()) {
        int u = pq.top().second;
        double w = pq.top().first;
        pq.pop();

        if (inMST[u]) continue; // Skip if already in MST

        mst_cost += w;
        inMST[u] = true;

        for (int v = 0; v < N; ++v) {
            if (!inMST[v] && !visited[v] && adj[u][v] < key[v]) {
                key[v] = adj[u][v];
                pq.push({key[v], v});
            }
        }
    }

    return mst_cost;
}

// Comparison function for sorting nodes based on their combined value
bool compareNodes(const Node& a, const Node& b) {
    return a.combined_value < b.combined_value;
}

// Branch and Bound TSP algorithm with dynamic MST heuristic and cache
void branch_and_bound_tsp(const vector<vector<double>>& adj, vector<bool>& visited, int node, int depth, double cost, double& min_cost, vector<int>& best_tour, vector<int>& current_tour, unordered_map<vector<bool>, double>& mst_cache, list<vector<bool>>& cache_order, int cache_size, chrono::steady_clock::time_point endTime, int unchanged_count, int prune_threshold) {
    if(chrono::steady_clock::now() > endTime) {
        //timeout_occurred.store(true);
        return;
    }

    int N = adj.size();
    if (depth == N) { // If all nodes are visited
        if (cost + adj[node][0] < min_cost) { // Check if the tour is better than the current best
            min_cost = cost + adj[node][0];
            best_tour = current_tour;
            best_tour.push_back(0); // Complete the tour
        }
        return;
    }

    // Check if MST heuristic is in cache
    if (mst_cache.find(visited) != mst_cache.end()) {
        double mst_cost = mst_cache[visited];
        if (cost + mst_cost >= min_cost) return; // Prune if the current path is already worse than the best found solution
    } else {
        // Compute MST heuristic if not in cache
        double mst_cost = compute_mst_cost(adj, visited);
        mst_cache[visited] = mst_cost;
        cache_order.push_back(visited);
        if (mst_cache.size() > cache_size) {
            // Evict the least recently used entry from the cache if it exceeds the size limit
            mst_cache.erase(cache_order.front());
            cache_order.pop_front();
        }
        if (cost + mst_cost >= min_cost) return; // Prune if the current path is already worse than the best found solution
    }

    vector<Node> unvisited_nodes;
    // Calculate combined value for each unvisited node and sort them
    for (int i = 0; i < N; ++i) {
        if (!visited[i]) {
            Node treeNode;
            treeNode.index = i;
            treeNode.combined_value = cost + adj[node][i] + compute_mst_cost(adj, visited);
            unvisited_nodes.push_back(treeNode);
        }
    }
    sort(unvisited_nodes.begin(), unvisited_nodes.end(), compareNodes);

    // Explore unvisited nodes in sorted order
    for (const auto& unvisited_node : unvisited_nodes) {
        int i = unvisited_node.index;
        visited[i] = true;
        current_tour.push_back(i);
        branch_and_bound_tsp(adj, visited, i, depth + 1, cost + adj[node][i], min_cost, best_tour, current_tour, mst_cache, cache_order, cache_size, endTime, unchanged_count + 1, prune_threshold);
        current_tour.pop_back();
        visited[i] = false;

        // Prune if the current path hasn't changed for too many iterations
        if (unchanged_count >= prune_threshold) {
            return;
        }
    }
}

int main(int argc, char* argv[]) {
    // Input filename from user
    string fname = argv[1];

    // Read distance matrix from file
    vector<vector<double>> adj = read_distance_matrix(fname);

    // Start measuring time
    auto start = chrono::high_resolution_clock::now();
    auto endTime = start + chrono::seconds(600);

    // Initialize variables for best tour and its cost
    double min_cost = INF;
    vector<int> best_tour;

    // Start with the first node as the initial tour
    vector<bool> visited(adj.size(), false);
    visited[0] = true; // Mark the starting node as visited
    vector<int> current_tour = {0}; // Start the tour with the first node

    // Initialize cache and cache order
    unordered_map<vector<bool>, double> mst_cache;
    list<vector<bool>> cache_order;
    const int cache_size = 100000000; // Maximum cache size

    // Set pruning parameters
    int unchanged_count = 0; // Counter for unchanged iterations
    int prune_threshold = 2; // Threshold for pruning unchanged iterations

    // Perform branch and bound TSP with dynamic MST heuristic and cache
    branch_and_bound_tsp(adj, visited, 0, 1, 0, min_cost, best_tour, current_tour, mst_cache, cache_order, cache_size, endTime, unchanged_count, prune_threshold);

    // Stop measuring time
    auto end = chrono::high_resolution_clock::now();

    if (timeout_occurred.load()) {
        cout << "Timeout occurred. Best solution found within the given time:";
        cout << "Minimum cost: " << min_cost << endl;
        cout << "Path: ";
        for (int i : best_tour) {
            cout << i + 1 << " ";
        }
        cout<<endl;
    } else {
        cout << "Minimum cost: " << min_cost << endl;
        cout << "Path: ";
        for (int i : best_tour) {
            cout << i + 1 << " ";
        }
        cout << endl;
    }

    double hitRate = hitNum/allNum;

    // Output the running time
    auto duration = chrono::duration_cast<chrono::milliseconds>(end - start).count();
    cout << "Running Time: " << duration << " ms" << endl;
    //cout<<"Hit rate: "<<hitRate<<endl;
    return 0;
}
