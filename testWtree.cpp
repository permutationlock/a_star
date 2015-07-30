//Author: Aven Bross
//Date: 3-24-2014
//Test program for weighted tree A* searching

#include "lib/wtree.h"

int main(){
	cout<<"This is the best way I could think of to describe\nthe tree in text.  In this example the A* finds\nthe path between the first and last nodes. Sometimes\nthis is a single step sometimes more, the tree is\nrandomly generated. Running it again will give you\na different tree and path!\n\n";

	//Create a random tree and find a path from the first to last node
	Wtree tree;
	vector<int> path = aStar(0, tree.size()-1, tree);

	//Print a description of the random tree
	cout<<"Description of random tree:\n"<<tree;

	//Print out the path
	cout<<"A* path from 0 to " << tree.size()-1 << ":\n";
	for(int i=0; i<path.size()-1; i++)
		cout<< path[i] << " -> ";
	cout<< path[path.size()-1] << "\n";
}

