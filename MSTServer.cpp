#include <iostream>
#include <vector>
#include <list>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <memory>
#include <cstring>
#include <algorithm>
#include <sstream>
#include <arpa/inet.h>
#include <unistd.h>
#include "MSTFactory.cpp"

#define PORT 8080
#define NUM_THREADS 4

using namespace std;
using namespace std::chrono;

bool stopServer = false;

// **Stage 1: Parse Command**
class ParseCommandStage {
public:
    static pair<string, vector<string>> parse(const string& command) {
        vector<string> tokens;
        string token;
        stringstream ss(command);
        while (getline(ss, token, ' ')) {
            tokens.push_back(token);
        }
        string commandType = tokens.empty() ? "" : tokens[0];
        tokens.erase(tokens.begin());
        return {commandType, tokens};
    }
};

// **Stage 2: Execute Command**
class CommandExecutionStage {
public:
    static string execute(const string& commandType, const vector<string>& args, unique_ptr<Graph>& graph) {
        if (commandType == "Newgraph") {
            return createGraph(args, graph);
        } else if (commandType == "Newedge") {
            return addEdge(args, graph);
        } else if (commandType == "Removeedge") {
            return removeEdge(args, graph);
        } else if (commandType == "Boruvka" || commandType == "Prim") {
            return calculateMST(commandType, graph);
        }
        return "Error: Invalid command\n";
    }

private:
    static string createGraph(const vector<string>& args, unique_ptr<Graph>& graph) {
        if (args.size() < 2) return "Error: Invalid Newgraph command format\n";
        int n = stoi(args[0]);
        graph = make_unique<Graph>(n);
        return "Graph created\n";
    }

    static string addEdge(const vector<string>& args, unique_ptr<Graph>& graph) {
        if (!graph) return "Error: Graph not initialized\n";
        if (args.size() < 3) return "Error: Invalid Newedge command format\n";
        int u = stoi(args[0]);
        int v = stoi(args[1]);
        double weight = stod(args[2]);
        graph->addEdge(u, v, weight);
        return "Edge added\n";
    }

    static string removeEdge(const vector<string>& args, unique_ptr<Graph>& graph) {
        if (!graph) return "Error: Graph not initialized\n";
        if (args.size() < 2) return "Error: Invalid Removeedge command format\n";
        int u = stoi(args[0]);
        int v = stoi(args[1]);
        graph->removeEdge(u, v);
        return "Edge removed\n";
    }

    static string calculateMST(const string& algorithmType, unique_ptr<Graph>& graph) {
        if (!graph) return "Error: Graph not initialized\n";
        unique_ptr<IMSTSolver> solver = MSTFactory::createSolver(algorithmType);
        vector<pair<int, pair<int, double>>> edges = graph->getEdges();

        auto start = high_resolution_clock::now();
        list<pair<int, int>> mstEdges = solver->solve(graph->getNumVertices(), edges);
        auto end = high_resolution_clock::now();
        auto duration = duration_cast<milliseconds>(end - start).count();

        string result = "MST:\n";
        for (const auto& edge : mstEdges) {
            result += to_string(edge.first) + " - " + to_string(edge.second) + "\n";
        }
        result += "Time taken: " + to_string(duration) + " ms\n";
        return result;
    }
};

// **Stage 3: Format Result and Send to Client**
class ResultFormattingStage {
public:
    static void sendResult(int clientSocket, const string& result) {
        send(clientSocket, result.c_str(), result.size(), 0);
        close(clientSocket);
    }
};

// Task class for thread pool
class Task {
public:
    Task(int clientSocket, string command, unique_ptr<Graph>& graph)
            : clientSocket(clientSocket), command(command), graph(graph) {}

    void execute() {
        auto [commandType, args] = ParseCommandStage::parse(command);
        string result = CommandExecutionStage::execute(commandType, args, graph);
        ResultFormattingStage::sendResult(clientSocket, result);
    }

private:
    int clientSocket;
    string command;
    unique_ptr<Graph>& graph;
};

// Leader-Follower Thread Pool
class ThreadPool {
public:
    ThreadPool(size_t numThreads) : stop(false) {
        for (size_t i = 0; i < numThreads; ++i) {
            workers.emplace_back([this] {
                while (true) {
                    Task* task;
                    {
                        unique_lock<mutex> lock(queueMutex);
                        condition.wait(lock, [this] { return stop || !tasks.empty(); });
                        if (stop && tasks.empty()) return;
                        task = tasks.front();
                        tasks.pop();
                    }
                    task->execute();
                    delete task;
                }
            });
        }
    }

    void enqueue(Task* task) {
        {
            unique_lock<mutex> lock(queueMutex);
            tasks.push(task);
        }
        condition.notify_one();
    }

    ~ThreadPool() {
        {
            unique_lock<mutex> lock(queueMutex);
            stop = true;
        }
        condition.notify_all();
        for (thread& worker : workers) worker.join();
    }

private:
    vector<thread> workers;
    queue<Task*> tasks;
    mutex queueMutex;
    condition_variable condition;
    bool stop;
};

// Server function
void serverThread(ThreadPool& pool, unique_ptr<Graph>& graph) {
    int server_fd, newSocket;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);

    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("Socket failed");
        exit(EXIT_FAILURE);
    }

    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
        perror("setsockopt failed");
        exit(EXIT_FAILURE);
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
        perror("Bind failed");
        exit(EXIT_FAILURE);
    }

    if (listen(server_fd, 3) < 0) {
        perror("Listen failed");
        exit(EXIT_FAILURE);
    }

    while (!stopServer) {
        cout << "Waiting for a connection..." << endl;
        if ((newSocket = accept(server_fd, (struct sockaddr*)&address, (socklen_t*)&addrlen)) < 0) {
            perror("Accept failed");
            continue;
        }
        cout << "Connection accepted" << endl;

        char buffer[1024] = {0};
        read(newSocket, buffer, 1024);
        string command(buffer);

        Task* task = new Task(newSocket, command, graph);
        pool.enqueue(task);
    }

    close(server_fd);
}

// Main function
//int main() {
//    ThreadPool pool(NUM_THREADS);
//    unique_ptr<Graph> graph = nullptr;
//
//    thread server(serverThread, ref(pool), ref(graph));
//    cout << "Enter 'stop' to shutdown the server: ";
//    string input;
//    while (cin >> input && input != "stop") {}
//
//    stopServer = true;
//    server.join();
//
//    return 0;
//}
