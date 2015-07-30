//Author: Daniel Aven Bross
//Date: 2/26/2014
//
//Weighted tree implementation and A* search function for the tree

#ifndef WTREE_H
#define WTREE_H

#include<random>
#include<vector>
using std::vector;
#include<iostream>
using std::cout;
using std::ostream;

/*
* Okay, my tree implementation is a little weird, but it works
* very well for the search I wrote and even generates random trees!
* Hopefully this is fine!
*/

//Node of weighted tree
//Contains a vector of the indexes of adjacent nodes and a second
//vector of the corresponding weights of the edges to those nodes
class WtreeNode{
public:
    //Connects this node to the node of the given index with and edge of the given weight
    void connect(int index, int weight){
        _adjNodes.push_back(index);
        _weights.push_back(weight);
    }
    
    //Returns the number of adjacent nodes
    int numAdjNodes() const{
        return _adjNodes.size();
    }
    
    //Returns the edge weight to the given adjacent node
    int getEdgeWeight(int node) const{
        for(int i=0; i<_weights.size(); i++)
            if(_adjNodes[i]==node)
                return _weights[i];
        return -1;
    }
    
    //Returns the index of the given adjacent node
    int getAdjNode(int node) const{
        return _adjNodes[node];
    }
private:
    vector<int> _adjNodes;    //Contains the indexes of neighbors
    vector<int> _weights;    //Contains weights of the edges to neighbors
};

//Implementation of a weighted tree
//Has a vector of nodes, and functions to generate a random tree
class Wtree{
public:
    Wtree(): _rd(), _el(_rd()), _weightDist(0, 6), _probDist(0,1) {
        _nodes.push_back(WtreeNode());
        generate(0, 0.);
    }

    //Gets the number of nodes
    int size() const{
        return _nodes.size();
    }

    //Gets the node of the given index
    const WtreeNode & getNode(int i) const{
        return _nodes[i];
    }
private:
    //Recursively generates random tree, called by the constructor
    void generate(int i, double depth){
        if(_probDist(_el) < .2*depth){
            return;
        }
        WtreeNode a;
        int weight = _weightDist(_el);
        _nodes[i].connect(_nodes.size(), weight);
        a.connect(i, weight);
        _nodes.push_back(a);
        generate(_nodes.size()-1, depth+1);
        generate(i, depth+1);
    }

    vector<WtreeNode> _nodes;    //Vector of nodes in the tree

    std::random_device _rd;    //Random device
    std::mt19937 _el;    //Use 32 bit mersenne twister

    //Uniform distribution for random edge weights
    std::uniform_int_distribution<int> _weightDist;

    //Uniform distribution for deciding when to stop generating the tree
    std::uniform_real_distribution<double> _probDist;    
};

//Print operator for Wtree
ostream & operator<<(ostream & os, const Wtree & other){
    for(int i=0; i<other.size(); i++){
        WtreeNode a = other.getNode(i);
        os<< "Node "<< i << ":\n";
        os<<"Adjacent Nodes:\n";
        for(int j=0; j<a.numAdjNodes(); j++){
            os << a.getAdjNode(j) << " ";
        }
        os<<"\n";
        os<<"Weights:\n";
        for(int j=0; j<a.numAdjNodes(); j++){
            os << a.getEdgeWeight(a.getAdjNode(j)) << " ";
        }
        os<<"\n\n";
    }
    return os;
}


//Simple heuristic function for aStar
//Returns the minimum weight of the edge to an adjacent node
int h(int start, int goal, const Wtree & tree){
    int min = 10000;
    for(int i=0; i<tree.getNode(start).numAdjNodes(); i++){
        int weight = tree.getNode(start).getEdgeWeight(tree.getNode(start).getAdjNode(i));
        if(weight<min) min = weight;
    }    
    return min;
}

//Reconstructs a path from current_node back to start
//Helper function for aStar
vector<int> reconstructPath(vector<int> cameFrom, int current_node, int start){
    vector<int> rest;
    if(cameFrom[current_node]!=start){
        rest = reconstructPath(cameFrom, cameFrom[current_node], start);
        rest.push_back(current_node);
        return rest;
    }
    else{
        rest.push_back(start);
        rest.push_back(current_node);
        return rest;
    }
}

//Example A* Call
//aStar(0, 5, myTree);
//
//Output vector:
//{0, 2, 5}
//
//This represents the path 1 -> 2 -> 5

//A* search, returns a vector representing the path from start to goal in tree
vector<int> aStar(int start, int goal, const Wtree & tree){
    vector<int> closedSet;    //The set of nodes already evaluated.
    vector<int> openSet;    //The set of nodes to be evaluated
    openSet.push_back(start);    //Add the start node to the open set
    vector<int> cameFrom;    //Map of where nodes came from in the path
 
    vector<int> gScore;    //Vector of g scores for all nodes
    vector<int> fScore;    //Vector of f scores for all nodes

    //Start off g and f scores at infinity or 100000, since our weights are very small this works
    for(int i=0; i<tree.size(); i++){
        gScore.push_back(100000);
        cameFrom.push_back(-1);
        fScore.push_back(100000);
    }

    //Cost from start along best known path.
    gScore[start] = 0;

    //Estimated total cost from start to goal
    fScore[start] = gScore[start] + h(start, goal, tree);
 
    //While there are still nodes left to check
    while(!openSet.empty()){

        //Find the node with the lowest f score in open set
        int currInd = 0;
        for(int i=1; i<openSet.size(); i++){
            if(fScore[openSet[i]]<fScore[currInd])
            currInd=i;    
        }
            int current = openSet[currInd];

        //If we have reached our goal, find the path
            if(current == goal)
                return reconstructPath(cameFrom, goal, start);
     
        //Remove current node from the open set
        openSet.erase(openSet.begin()+currInd);

        //Add current node to the closed set
            closedSet.push_back(current);

        //Examine current nodes neighbors
        for(int i=0; i<tree.getNode(current).numAdjNodes(); i++){
            int neighbor = tree.getNode(current).getAdjNode(i);

            //Check if this neighbor is in the closed set
            bool inclosed = false;
            for(int j=0; j<closedSet.size(); j++)
                if(closedSet[j]==neighbor)
                    inclosed = true;
            
            if(inclosed) continue;

            //Calculate tentative g score
            int tGscore = gScore[current] + tree.getNode(current).getEdgeWeight(neighbor);

            //Check if this neighbor is already in the open set
            bool inOpen = false;
            for(int j=0; j<openSet.size(); j++)
                if(openSet[j]==neighbor)
                    inOpen = true;  

            //If not, or if we have found a route with a lower g score
            if(!inOpen || tGscore < gScore[neighbor]){ 
                //Mark where this node came from in the path
                cameFrom[neighbor] = current;
                //Set its g score to the tentative one
                gScore[neighbor] = tGscore;
                //Find the f score
                fScore[neighbor] = gScore[neighbor] + h(neighbor, goal, tree);
                //Add it to the open set if it wasn't already
                if(!inOpen){
                   openSet.push_back(neighbor);
                }
            }
        }
    }

    //If something goes wrong, return an empty vector
    vector<int> fail; 
    return fail;
}


#endif
