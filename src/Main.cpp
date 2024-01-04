//
// Created by ck028958 on 1/4/2024.
//

#include <iostream>
#include <fstream>
#include <string>
#include "./Array/Graph/Graph.h"

using namespace std;
using namespace array;

// Control which words will be included to graph
bool controlWord(string word){
    if(word.length() >= 3 && word.length() <= 5) return true;
    return false;
}

int readCount(){
    ifstream inputFile;

    inputFile.open("../english-dictionary.txt");

    if(!inputFile.is_open()){
        cerr << "File is not open" << endl;
        return 0;
    }

    string line;
    int count = 0;
    while (getline(inputFile, line)){
        if(controlWord(line)) count++;
    }

    inputFile.close();

    return count;
}

string* read(int count){
    string* words = new string[count];

    ifstream inputFile;

    inputFile.open("../english-dictionary.txt");

    if(!inputFile.is_open()){
        cerr << "File is not open" << endl;
        return words;
    }

    string line;
    int index = 0;
    while (getline(inputFile, line)){
        if(controlWord(line)){
            words[index] = line;
            index++;
        }
    }

    inputFile.close();

    return words;
}

int main(){
    int wordsLength = readCount();
    string *words = read(wordsLength);

    Graph *graph = new Graph(wordsLength);

    for (int i = 0; i < wordsLength; ++i) {
        graph->addWordAuto(words[i]);
    }

    // Algorithms
    graph->BFS("aaa","abb");
    graph->Dijkstra("aaa","abb");

    // Add word
    graph->addWord("aab");

    // Add word and auto connect to other words
    graph->addWordAuto("eck");

    // Add edge between given words
    graph->addEdge("aaa","aab");


    return 0;
};