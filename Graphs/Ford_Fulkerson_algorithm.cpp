Here's the code for the Ford-Fulkerson algorithm in C++ along with an explanation:

#include <iostream>
#include <vector>
#include <queue>
#include <limits.h>

using namespace std;

// Structure to represent an edge in the graph
struct Edge {
    int v;            // destination vertex
    int capacity;     // capacity of the edge
    int flow;         // current flow through the edge

    Edge(int v, int capacity) : v(v), capacity(capacity), flow(0) {}
};

// Function to add an edge to the graph
void addEdge(vector<vector<Edge>>& graph, int u, int v, int capacity) {
    graph[u].push_back(Edge(v, capacity));
    graph[v].push_back(Edge(u, 0));  // Residual edge
}

// Function to perform Breadth-First Search on the graph
bool bfs(vector<vector<Edge>>& graph, int source, int sink, vector<int>& parent) {
    int n = graph.size();
    vector<bool> visited(n, false);

    queue<int> q;
    q.push(source);
    visited[source] = true;
    parent[source] = -1;

    while (!q.empty()) {
        int u = q.front();
        q.pop();

        for (const Edge& edge : graph[u]) {
            int v = edge.v;
            int capacity = edge.capacity;
            int flow = edge.flow;

            if (!visited[v] && capacity > flow) {
                q.push(v);
                visited[v] = true;
                parent[v] = u;
            }
        }
    }

    // If we reached the sink, there is a path from source to sink
    return visited[sink];
}

// Function to find the maximum flow in a graph using the Ford-Fulkerson algorithm
int fordFulkerson(vector<vector<Edge>>& graph, int source, int sink) {
    int n = graph.size();
    vector<int> parent(n);
    int maxFlow = 0;

    while (bfs(graph, source, sink, parent)) {
        int pathFlow = INT_MAX;

        // Find the minimum residual capacity along the path
        // from source to sink found by BFS
        for (int v = sink; v != source; v = parent[v]) {
            int u = parent[v];

            for (Edge& edge : graph[u]) {
                if (edge.v == v) {
                    int remainingCapacity = edge.capacity - edge.flow;
                    pathFlow = min(pathFlow, remainingCapacity);
                    break;
                }
            }
        }

        // Update the flow and residual capacities along the path
        for (int v = sink; v != source; v = parent[v]) {
            int u = parent[v];

            for (Edge& edge : graph[u]) {
                if (edge.v == v) {
                    edge.flow += pathFlow;
                    break;
                }
            }

            // Update the reverse edge (residual capacity)
            for (Edge& edge : graph[v]) {
                if (edge.v == u) {
                    edge.flow -= pathFlow;
                    break;
                }
            }
        }

        // Add the path flow to the maximum flow
        maxFlow += pathFlow;
    }

    return maxFlow;
}

int main() {
    int n = 6;  // Number of vertices in the graph

    // Create an empty graph with n vertices
    vector<vector<Edge>> graph(n);

    // Add edges to the graph
    addEdge(graph, 0, 1, 16);
    addEdge(graph, 0, 2, 13);
    addEdge(graph, 1, 2, 10);
    addEdge(graph, 1, 3, 12);
    addEdge(graph, 2, 1, 4);
    addEdge(graph, 2, 4, 14);
    addEdge(graph, 3, 2, 9);
    addEdge(graph, 3, 5, 20);
    addEdge(graph, 4, 3, 7);
    addEdge(graph, 4, 5, 4);

    int source = 0;  // Source vertex
    int sink = 5;    // Sink vertex

    int maxFlow = fordFulkerson(graph, source, sink);

    cout << "Maximum Flow: " << maxFlow << endl;

    return 0;
}
Explanation:

The Edge structure represents an edge in the graph. It contains the destination vertex (v), the capacity of the edge (capacity), and the current flow through the edge (flow).

The addEdge function is used to add an edge to the graph. It takes the graph, source vertex (u), destination vertex (v), and capacity of the edge as parameters. It adds the edge from u to v with the given capacity and also adds the corresponding residual edge from v to u with 0 capacity.

The bfs function performs a Breadth-First Search on the graph to find a path from the source to the sink. It takes the graph, source vertex, sink vertex, and a vector to store the parent information as parameters. It returns true if a path is found, and false otherwise.

The fordFulkerson function implements the Ford-Fulkerson algorithm to find the maximum flow in the graph. It takes the graph, source vertex, and sink vertex as parameters. It returns the maximum flow value.

Inside the fordFulkerson function, a parent vector is used to store the parent information for the path from the source to the sink. The maximum flow is initially set to 0.

The algorithm repeatedly calls the bfs function to find a path from the source to the sink. If a path exists, it finds the minimum residual capacity (pathFlow) along the path.

It then updates the flow and residual capacities along the path by adding pathFlow to the forward edges and subtracting it from the backward (residual) edges.

The addEdge function and the main function demonstrate an example usage of the Ford-Fulkerson algorithm. The graph is initialized, edges are added, and the maximum flow is calculated by calling the fordFulkerson function.

Finally, the maximum flow value is printed to the console.

You can run the code and see the output, which will show the maximum flow in the given graph.
