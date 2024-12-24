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
#include "Graph.cpp"
#include <fcntl.h>

#define PORT 8080
#define NUM_THREADS 4

using namespace std;

bool stopServer = false;

// **Utility: Sanitize Input**
void sanitizeInput(string& command) {
    command.erase(remove(command.begin(), command.end(), '\n'), command.end());
    command.erase(remove(command.begin(), command.end(), '\r'), command.end());
}

// **Stage 1: Parse Command**
class ParseCommandStage {
public:
    static pair<string, vector<string>> parse(const string& command) {
        vector<string> tokens;
        string token;
        stringstream ss(command);
        while (getline(ss, token, ' ')) {
            size_t pos = 0;
            while ((pos = token.find(',')) != string::npos) {
                tokens.push_back(token.substr(0, pos));
                token.erase(0, pos + 1);
            }
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
        try {
            int n = stoi(args[0]);
            graph = make_unique<Graph>(n);
            return "Graph created\n";
        } catch (const exception& e) {
            return "Error: Invalid arguments for Newgraph\n";
        }
    }

    static string addEdge(const vector<string>& args, unique_ptr<Graph>& graph) {
        if (!graph) return "Error: Graph not initialized\n";
        if (args.size() < 3) return "Error: Invalid Newedge command format\n";
        try {
            int u = stoi(args[0]);
            int v = stoi(args[1]);
            double weight = stod(args[2]);
            graph->addEdge(u, v, weight);
            return "Edge added\n";
        } catch (const exception& e) {
            return "Error: Invalid arguments for Newedge\n";
        }
    }

    static string removeEdge(const vector<string>& args, unique_ptr<Graph>& graph) {
        if (!graph) return "Error: Graph not initialized\n";
        if (args.size() < 2) return "Error: Invalid Removeedge command format\n";
        try {
            int u = stoi(args[0]);
            int v = stoi(args[1]);
            graph->removeEdge(u, v);
            return "Edge removed\n";
        } catch (const exception& e) {
            return "Error: Invalid arguments for Removeedge\n";
        }
    }

    static string calculateMST(const string& algorithmType, unique_ptr<Graph>& graph) {
        if (!graph) return "Error: Graph not initialized\n";
        unique_ptr<IMSTSolver> solver = MSTFactory::createSolver(algorithmType);
        vector<pair<int, pair<int, double>>> edges = graph->getEdges();

        string result = "MST:\n";
        auto start = chrono::high_resolution_clock::now();
        list<pair<int, int>> mstEdges = solver->solve(graph->getNumVertices(), edges);
        auto end = chrono::high_resolution_clock::now();

        for (const auto& edge : mstEdges) {
            result += to_string(edge.first) + " - " + to_string(edge.second) + "\n";
        }
        result += "Time taken: " + to_string(chrono::duration_cast<chrono::milliseconds>(end - start).count()) + " ms\n";
        return result;
    }
};

// **Task Class**
class Task {
public:
    Task(int clientSocket, unique_ptr<Graph>& graph) : clientSocket(clientSocket), graph(graph) {}

    void execute() {
        char buffer[1024] = {0};
        while (true) {
            int bytesRead = read(clientSocket, buffer, sizeof(buffer));
            if (bytesRead <= 0) {
                cout << "Client disconnected." << endl;
                break;
            }

            string command(buffer, bytesRead);
            sanitizeInput(command);
            cout << "Received command: '" << command << "'" << endl;

            if (command == "quit") {
                cout << "Client requested to close the connection." << endl;
                break;
            }

            auto [commandType, args] = ParseCommandStage::parse(command);
            string result = CommandExecutionStage::execute(commandType, args, graph);
            send(clientSocket, result.c_str(), result.size(), 0);
            memset(buffer, 0, sizeof(buffer));
        }
        close(clientSocket);
    }

private:
    int clientSocket;
    unique_ptr<Graph>& graph;
};

// **Thread Pool**
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

// **Server Thread**
void serverThread(ThreadPool& pool, unique_ptr<Graph>& graph) {
    int server_fd;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);

    // Create and bind socket
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("Socket creation failed");
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

    // Set the socket to non-blocking mode
    fcntl(server_fd, F_SETFL, O_NONBLOCK);

    cout << "Server running on port " << PORT << ". Waiting for connections..." << endl;

    while (!stopServer) {
        int clientSocket = accept(server_fd, (struct sockaddr*)&address, (socklen_t*)&addrlen);
        if (clientSocket >= 0) {
            cout << "Connection accepted from " << inet_ntoa(address.sin_addr) << ":" << ntohs(address.sin_port) << endl;
            Task* task = new Task(clientSocket, graph);
            pool.enqueue(task);
        } else if (errno == EWOULDBLOCK || errno == EAGAIN) {
            // No connection, check the stopServer flag
            this_thread::sleep_for(chrono::milliseconds(100));
        } else {
            perror("Accept failed");
        }
    }

    close(server_fd);
    cout << "Server shut down." << endl;
}

int main() {
    ThreadPool pool(NUM_THREADS);
    unique_ptr<Graph> graph = nullptr;

    // Start the server thread
    thread server(serverThread, ref(pool), ref(graph));
    cout << "Enter 'stop' to shutdown the server: ";

    string input;
    while (cin >> input) {
        if (input == "stop") {
            stopServer = true;  // Signal the server thread to stop
            break;
        }
    }

    server.join();  // Wait for the server thread to finish
    cout << "Server has been shut down." << endl;

    return 0;
}
