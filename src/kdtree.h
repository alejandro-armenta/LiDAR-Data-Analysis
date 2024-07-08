/* \author Aaron Brown */
// Quiz on implementing kd tree

//#include "../../render/render.h"

#include "render/render.h"

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void createNode(Node *& node, std::vector<float> point, int id, int depth)
	{
		if(node == NULL)
		{
			node = new Node(point, id);
		}
		else
		{
			insertNode(node, point, id, depth);
		}
	}

	//NOTE: this is a pointer reference!
	void insertNode(Node *& node, std::vector<float> point, int id, int depth)
	{
		int index = (depth%3);
		
		if(point[index] < node->point[index])
		{
			createNode(node->left, point, id, depth + 1);
		}
		else
		{
			createNode(node->right, point, id, depth + 1);
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 

		createNode(root, point, id, 0);
	}

	bool isWithinSphere(std::vector<float> target, float distanceTol, std::vector<float> point)
	{
		float xDist = target[0] - point[0];
		float yDist = target[1] - point[1];
		float zDist = target[2] - point[2];

		float dist = sqrtf(xDist*xDist + yDist*yDist + zDist*zDist);
		return (dist < distanceTol);
	}

	void searchNode(Node * node, std::vector<int>& ids, std::vector<float> target, float distanceTol, int depth)
	{
		if(node != NULL)
		{
			//std::cout << node->id << std::endl;
			if (isWithinSphere(target, distanceTol, node->point))
			{
				ids.push_back(node->id);
			}

			int index = (depth % 3);
			
			if(target[index] + distanceTol > node->point[index])
			{
				searchNode(node->right, ids, target, distanceTol, depth + 1);
			}

			if(target[index] - distanceTol < node->point[index])
			{
				searchNode(node->left, ids, target, distanceTol, depth + 1);
			}
		}
		
		
		
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		searchNode(root, ids, target, distanceTol, 0);

		return ids;
	}
	

};




