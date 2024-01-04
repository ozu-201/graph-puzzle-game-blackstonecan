//
// Created by Olcay Taner YILDIZ on 8.05.2023.
//

#include "Graph.h"
#include "../DisjointSet.h"
#include "../Queue.h"
#include "../Heap/Heap.h"
#include "../Heap/MinHeap.h"
#include <iostream>
#include <string>

namespace array{

    using namespace std;

    Graph::Graph(int vertexCount) : AbstractGraph(vertexCount){
        edges = new int*[vertexCount];
        words = new string[vertexCount];
        for (int i = 0; i < vertexCount; i++){
            edges[i] = new int[vertexCount];
        }
        for (int i = 0; i < vertexCount; i++){
            for (int j = 0; j < vertexCount; j++){
                edges[i][j] = 0;
            }
        }
    }

    Graph::~Graph() {
        for (int i = 0; i < vertexCount; i++){
            delete[] edges[i];
        }
        delete[] edges;
    }

    void Graph::addEdge(int from, int to) {
        edges[from][to] = 1;
    }

    void Graph::addEdge(int from, int to, int weight) {
        edges[from][to] = weight;
    }

    void Graph::addEdge(string from, string to) {
        int fromIndex = findIndex(from);
        int toIndex = findIndex(to);

        if(fromIndex == -1 || toIndex == -1){
            if(fromIndex == -1) cout << from << " - word not exist in words." << endl;
            if(toIndex == -1) cout << to << " - word not exist in words." << endl;
            return;
        }

        if(from == to){
            cerr << "Source and target words are same" << endl;
            return;
        }

        addEdge(fromIndex, toIndex);
        addEdge(toIndex, fromIndex);
    }

    void Graph::addEdge(string from, string to, int weight) {
        int fromIndex = findIndex(from);
        int toIndex = findIndex(to);

        if(fromIndex == -1 || toIndex == -1){
            if(fromIndex == -1) cout << from << " - word not exist in words." << endl;
            if(toIndex == -1) cout << to << " - word not exist in words." << endl;
            return;
        }

        if(from == to){
            cerr << "Source and target words are same" << endl;
            return;
        }

        addEdge(fromIndex, toIndex, weight);
        addEdge(toIndex, fromIndex, weight);
    }

    // It includes word
    void Graph::addWord(string word){
        words[lastVertex] = word;
        lastVertex++;
    }

    // It includes word and connect other words automatically
    void Graph::addWordAuto(string word){
        words[lastVertex] = word;

        for (int i = 0; i < lastVertex + 1; ++i) {
            string temp = words[i];

            if(temp.length() == word.length()){
                int count = 0;
                for (int j = 0; j < word.length(); ++j) {
                    if(temp[j] != word[j]) count++;
                }

                if(count == 1){
                    addEdge(lastVertex, i);
                    addEdge(i, lastVertex);
                }
            }
        }

        lastVertex++;
    }

    // It gives index of word
    int Graph::findIndex(string word){
        for (int i = 0; i < lastVertex; ++i) {
            if(words[i] == word) return i;
        }

        return -1;
    }

    // It gives neighbor count of word's index
    int Graph::getNeighborCount(int index){
        int count = 0;

        for (int i = 0; i < lastVertex; ++i) {
            if(edges[index][i] > 0){
                count++;
            }
        }

        return count;
    }

    // It gives neighbors of word's index
    int* Graph::getNeighbors(int index){
        int* neighbors = new int[getNeighborCount(index)];

        int nIndex = 0;
        for (int i = 0; i < lastVertex; ++i) {
            if(edges[index][i] > 0){
                neighbors[nIndex] = i;
                nIndex++;
            }
        }

        return neighbors;
    }

    // It uses BFS algorithm to find the shortest path from source to target.
    void Graph::BFS(string source, string target){
        int sourceIndex = findIndex(source);
        int targetIndex = findIndex(target);

        if(sourceIndex == -1 || targetIndex == -1){
            if(sourceIndex == -1) cout << source << " - word not exist in words." << endl;
            if(targetIndex == -1) cout << target << " - word not exist in words." << endl;
            return;
        }

        if(sourceIndex == targetIndex){
            cerr << "Source and target words are same" << endl;
            return;
        }

        bool* visited = new bool[lastVertex];
        for (int i = 0; i < lastVertex; ++i) {
            visited[i] = false;
        }

        int* map = new int[lastVertex];
        for (int i = 0; i < lastVertex; ++i) {
            map[i] = -1;
        }

        Queue queue = Queue(lastVertex);
        queue.enqueue(Element(sourceIndex));

        while(!queue.isEmpty()){
            int current = queue.dequeue().getData();
            visited[current] = true;

            int neighborCount = getNeighborCount(current);
            int* neighbors = getNeighbors(current);

            for (int i = 0; i < neighborCount; ++i) {
                int neighbor = neighbors[i];

                if(!visited[neighbor]){
                    visited[neighbor] = true;
                    map[neighbor] = current;

                    if(neighbor == targetIndex){
                        string path;

                        for (int at = targetIndex; at != -1; at = map[at]) {
                            path = words[at] + " " + path;
                        }

                        cout << "With using BFS, the path which is from '" << source << "' to '" << target << "' path is found." << endl;
                        cout << path << endl;

                        return;
                    }

                    queue.enqueue(Element(neighbor));
                }
            }
        }

        cout << "With using BFS, the path which is from '" << source << "' to '" << target << "' path is not found." << endl;
    }

    // It uses Dijkstra algorithm to find the shortest path from source to target.
    void Graph::Dijkstra(string source, string target){
        int sourceIndex = findIndex(source);
        int targetIndex = findIndex(target);

        if(sourceIndex == -1 || targetIndex == -1){
            if(sourceIndex == -1) cout << source << " - word not exist in words." << endl;
            if(targetIndex == -1) cout << target << " - word not exist in words." << endl;
            return;
        }

        if(sourceIndex == targetIndex){
            cerr << "Source and target words are same" << endl;
            return;
        }

        bool* visited = new bool[lastVertex];
        for (int i = 0; i < lastVertex; ++i) {
            visited[i] = false;
        }

        int* map = new int[lastVertex];
        for (int i = 0; i < lastVertex; ++i) {
            map[i] = -1;
        }

        Queue queue = Queue(lastVertex);
        queue.enqueue(Element(targetIndex));

        int depth = 1;

        while(!queue.isEmpty()){
            int current = queue.dequeue().getData();
            visited[current] = true;

            int neighborCount = getNeighborCount(current);
            int* neighbors = getNeighbors(current);

            for (int i = 0; i < neighborCount; ++i) {
                int neighbor = neighbors[i];

                if(!visited[neighbor]){
                    visited[neighbor] = true;
                    map[neighbor] = depth;

                    if(neighbor == sourceIndex) break;

                    queue.enqueue(Element(neighbor));
                }
            }

            depth++;
        }

        if(map[sourceIndex] != -1){
            string path = source;

            int index = sourceIndex;

            while (index != targetIndex){
                int min = INT_MAX;
                int minIndex = -1;

                int neighborCount = getNeighborCount(index);
                int* neighbors = getNeighbors(index);

                for (int i = 0; i < neighborCount; ++i) {
                    int neighbor = neighbors[i];

                    if(map[neighbor] < min){
                        min = map[neighbor];
                        minIndex = neighbor;
                    }
                }

                path += " " + words[minIndex];
                index = minIndex;
            }

            cout << "With using Dijkstra, the path which is from '" << source << "' to '" << target << "' path is found." << endl;
            cout << path << endl;
        }else{
            cout << "With using Dijkstra, the path which is from '" << source << "' to '" << target << "' path is not found." << endl;
        }

    }

    void Graph::connectedComponentDisjointSet() {
        DisjointSet sets = DisjointSet(vertexCount);
        for (int fromNode = 0; fromNode < vertexCount; fromNode++){
            for (int toNode = 0; toNode < vertexCount; toNode++){
                if (edges[fromNode][toNode] > 0){
                    if (sets.findSetRecursive(fromNode) != sets.findSetRecursive(toNode)){
                        sets.unionOfSets(fromNode, toNode);
                    }
                }
            }
        }
    }

    void Graph::depthFirstSearch(bool *visited, int fromNode) {
        for (int toNode = 0; toNode < vertexCount; toNode++){
            if (edges[fromNode][toNode] > 0){
                if (!visited[toNode]){
                    visited[toNode] = true;
                    depthFirstSearch(visited, toNode);
                }
            }
        }
    }

    void Graph::breadthFirstSearch(bool *visited, int startNode) {
        int fromNode;
        Queue queue = Queue(100);
        queue.enqueue( Element(startNode));
        while (!queue.isEmpty()){
            fromNode = queue.dequeue().getData();
            for (int toNode = 0; toNode < vertexCount; toNode++) {
                if (edges[fromNode][toNode] > 0) {
                    if (!visited[toNode]){
                        visited[toNode] = true;
                        queue.enqueue( Element(toNode));
                    }
                }
            }
        }
    }

    Path *Graph::bellmanFord(int source) {
        Path* shortestPaths = initializePaths(source);
        for (int i = 0; i < vertexCount - 1; i++){
            for (int fromNode = 0; fromNode < vertexCount; fromNode++){
                for (int toNode = 0; toNode < vertexCount; toNode++){
                    int newDistance = shortestPaths[fromNode].getDistance() + edges[fromNode][toNode];
                    if (newDistance < shortestPaths[toNode].getDistance()){
                        shortestPaths[toNode].setDistance(newDistance);
                        shortestPaths[toNode].setPrevious(fromNode);
                    }
                }
            }
        }
        return shortestPaths;
    }

    Path *Graph::dijkstra(int source) {
        Path* shortestPaths = initializePaths(source);
        MinHeap heap = MinHeap(vertexCount);
        for (int i = 0; i < vertexCount; i++){
            heap.insert( HeapNode(shortestPaths[i].getDistance(), i));
        }
        while (!heap.isEmpty()){
            HeapNode node = heap.deleteTop();
            int fromNode = node.getName();
            for (int toNode = 0; toNode < vertexCount; toNode++){
                int newDistance = shortestPaths[fromNode].getDistance() + edges[fromNode][toNode];
                if (newDistance < shortestPaths[toNode].getDistance()){
                    int position = heap.search(toNode);
                    heap.update(position, newDistance);
                    shortestPaths[toNode].setDistance(newDistance);
                    shortestPaths[toNode].setPrevious(fromNode);
                }
            }
        }
        return shortestPaths;
    }

    int **Graph::floydWarshall() {
        int** distances;
        distances = new int*[vertexCount];
        for (int i = 0; i < vertexCount; i++){
            distances[i] = new int[vertexCount];
            for (int j = 0; j < vertexCount; j++){
                distances[i][j] = edges[i][j];
            }
        }
        for (int k = 0; k < vertexCount; k++){
            for (int i = 0; i < vertexCount; i++){
                for (int j = 0; j < vertexCount; j++){
                    int newDistance = distances[i][k] + distances[k][j];
                    if (newDistance < distances[i][j]){
                        distances[i][j] = newDistance;
                    }
                }
            }
        }
        return distances;
    }

    Edge *Graph::edgeList(int &edgeCount) {
        Edge* list;
        edgeCount = 0;
        for (int i = 0; i < vertexCount; i++){
            for (int j = 0; j < vertexCount; j++){
                if (edges[i][j] > 0){
                    edgeCount++;
                }
            }
        }
        list = new Edge[edgeCount];
        int index = 0;
        for (int i = 0; i < vertexCount; i++){
            for (int j = 0; j < vertexCount; j++){
                if (edges[i][j] > 0){
                    list[index] = Edge(i, j, edges[i][j]);
                    index++;
                }
            }
        }
        return list;
    }

    void Graph::prim() {
        Path* paths = initializePaths(0);
        MinHeap heap = MinHeap(vertexCount);
        for (int i = 0; i < vertexCount; i++){
            heap.insert(HeapNode(paths[i].getDistance(), i));
        }
        while (!heap.isEmpty()){
            HeapNode node = heap.deleteTop();
            int fromNode = node.getName();
            for (int toNode = 0; toNode < vertexCount; toNode++){
                if (paths[toNode].getDistance() > edges[fromNode][toNode]){
                    int position = heap.search(toNode);
                    heap.update(position, edges[fromNode][toNode]);
                    paths[toNode].setDistance(edges[fromNode][toNode]);
                    paths[toNode].setPrevious(fromNode);
                }
            }
        }
    }

}
