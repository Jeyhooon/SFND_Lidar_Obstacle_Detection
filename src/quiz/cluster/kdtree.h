/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node *left;
	Node *right;

	Node(std::vector<float> arr, int setId)
		: point(arr), id(setId), left(NULL), right(NULL)
	{
	}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node *root;

	KdTree()
		: root(NULL)
	{
	}

	~KdTree()
	{
		delete root;
	}

	void insert_helper(Node *&head, int depth, std::vector<float> point, int id)
	{
		// empty tree
		if (head == NULL)
		{
			// head contains the address of the pointer that points to the current head node.
			// when it's NULL and we are assigning a new node to it; the address of that new node will be save in the head.
			head = new Node(point, id);
		}
		else
		{
			// unsigned int --> since we won't have a negative depth
			uint residual_depth = depth % 2;

			// residual_depth: either 0 or 1 --> when 0 we compare x element from point which is the 0 index; when 1 we compare y element which is index 1
			// hence we can write the if statement this way:
			// instead of: (residual_depth == 0 && head->point[0] > point[0]) or (residual_depth == 1 && head->point[1] > point[1])

			if (head->point[residual_depth] > point[residual_depth])
			{
				// when compared x or y is smaller than head --> traverse to the left node
				insert_helper(head->left, depth + 1, point, id);
			}
			else
			{
				// traverse to the right node
				insert_helper(head->right, depth + 1, point, id);
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		insert_helper(root, 0, point, id);
	}

	void search_helper(std::vector<float> target, Node *head, int depth, float distanceTol, std::vector<int> &ids)
	{
		if (head != NULL)
		{
			if ((target[0] - distanceTol <= head->point[0]) && (head->point[0] <= target[0] + distanceTol) && (target[1] - distanceTol <= head->point[1]) && (head->point[1] <= target[1] + distanceTol))
			{
				// inside box:
				float distance = sqrt((head->point[0] - target[0]) * (head->point[0] - target[0]) + (head->point[1] - target[1]) * (head->point[1] - target[1]));
				if (distance < distanceTol)
					ids.push_back(head->id);
			}

			// check accross boundary
			if ((target[depth % 2] - distanceTol) < head->point[depth % 2])
				search_helper(target, head->left, depth + 1, distanceTol, ids);

			if ((target[depth % 2] + distanceTol) > head->point[depth % 2])
				search_helper(target, head->right, depth + 1, distanceTol, ids);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		search_helper(target, root, 0, distanceTol, ids);
		return ids;
	}
};
