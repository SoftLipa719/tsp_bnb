#include <iostream>
#include <vector>
#include <fstream>
#include <chrono>
#include <limits>
#include <mutex>
#include <thread>
#include <algorithm>
#include <atomic>

using namespace std;

const double INF = numeric_limits<double>::infinity();
mutex mtx;
atomic<bool> timeout_occurred(false); // Atomic flag to indicate timeout

struct Solution {
    double cost = INF;
    vector<int> path;

    Solution() : cost(INF) {}
    Solution(double c, vector<int> p) : cost(c), path(move(p)) {}
};

double calculate2OptHeuristic(const vector<vector<double>>& adj, const vector<int>& path) {
    double totalCost = 0;
    int N = path.size();
    for (int i = 0; i < N - 1; ++i) {
        totalCost += adj[path[i]][path[i + 1]];
    }
    totalCost += adj[path[N - 1]][path[0]];
    return totalCost;
}

void TSP2OptRec(const vector<vector<double>>& adj, vector<int>& currentPath, vector<bool>& visited, double& minCost, vector<int>& bestPath, double currentCost, chrono::steady_clock::time_point endTime) {
    if (chrono::steady_clock::now() > endTime || timeout_occurred.load()) {
        timeout_occurred.store(true); // Indicate that a timeout has occurred
        return;
    }

    int N = adj.size();
    if (currentPath.size() == N) {
        currentCost += adj[currentPath[N - 1]][currentPath[0]];
        if (currentCost < minCost) {
            minCost = currentCost;
            bestPath = currentPath;
        }
        return;
    }

    for (int i = 0; i < N; ++i) {
        if (!visited[i]) {
            visited[i] = true;
            currentPath.push_back(i);
            double newCost = currentCost + adj[currentPath[currentPath.size() - 2]][i];

            if (newCost < minCost) {
                TSP2OptRec(adj, currentPath, visited, minCost, bestPath, newCost, endTime);
            }

            visited[i] = false;
            currentPath.pop_back();
        }
    }
}

void ParallelTSP2Opt(vector<vector<double>>& adj, int startNode, double& globalMinCost, vector<int>& globalBestPath, int N, chrono::steady_clock::time_point endTime) {
    vector<Solution> solutions(N);

    for (int nextNode = 0; nextNode < N; ++nextNode) {
        if (startNode == nextNode || timeout_occurred.load()) break;

        vector<int> currentPath = {startNode, nextNode};
        vector<bool> visited(N, false);
        visited[startNode] = true;
        visited[nextNode] = true;
        double minCost = INF;
        vector<int> bestPath;

        TSP2OptRec(adj, currentPath, visited, minCost, bestPath, adj[startNode][nextNode], endTime);

        if (minCost < solutions[nextNode].cost) {
            solutions[nextNode].cost = minCost;
            solutions[nextNode].path = bestPath;
        }
    }

    for (const auto& sol : solutions) {
        if (sol.cost < globalMinCost) {
            lock_guard<mutex> guard(mtx);
            globalMinCost = sol.cost;
            globalBestPath = sol.path;
        }
    }
}

int main() {
    string fname;
    cout << "Enter the complete input filename: ";
    cin >> fname;

    ifstream inputFile(fname);
    if (!inputFile.is_open()) {
        cerr << "Error opening: " << fname << endl;
        return 1;
    }

    int N;
    inputFile >> N;
    vector<vector<double>> adj(N, vector<double>(N));
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            inputFile >> adj[i][j];

    auto start = chrono::high_resolution_clock::now();
    auto endTime = start + chrono::seconds(10); // Set a 10-second timeout

    double globalMinCost = INF;
    vector<int> globalBestPath;

    vector<thread> threads;
    for (int startNode = 0; startNode < N; ++startNode) {
        threads.push_back(thread(ParallelTSP2Opt, ref(adj), startNode, ref(globalMinCost), ref(globalBestPath), N, endTime));
    }

    for (auto& t : threads) {
        t.join();
    }

    auto end = chrono::high_resolution_clock::now();

    if (timeout_occurred.load()) {
        cout << "Timeout occurred. Best solution found within the given time:";
        cout << "Minimum cost: " << globalMinCost << endl;
        cout << "Path: ";
        for (int i : globalBestPath) {
            cout << i + 1 << " ";
        }
        cout<<endl;
    } else {
        cout << "Minimum cost: " << globalMinCost << endl;
        cout << "Path: ";
        for (int i : globalBestPath) {
            cout << i + 1 << " ";
        }
        cout << endl;
    }

    auto duration = chrono::duration_cast<chrono::milliseconds>(end - start).count();
    cout << "Running Time: " << duration << " ms" << endl;
    return 0;
}
