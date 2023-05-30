/* \author Aaron Brown */
// Quiz on implementing kd tree
#ifndef KD_TREE_H
#define KD_TREE_H

#include "processPointClouds.h"


// Structure to represent node of kd tree
struct Node
{
	pcl::PointXYZ point;
	int id;
	Node *left;
	Node *right;

	Node(pcl::PointXYZ arr, int setId)
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

	void insert_helper(Node *&head, int depth, pcl::PointXYZ point, int id)
	{
		if (head == NULL)
		{
			head = new Node(point, id);
		}
		else
		{
			uint residual_depth = depth % 3;
			float point_value[3] = { point.x, point.y, point.z };
			float head_value[3] = { head->point.x, head->point.y, head->point.z };

			if (head_value[residual_depth] > point_value[residual_depth])
			{
				insert_helper(head->left, depth + 1, point, id);
			}
			else
			{
				insert_helper(head->right, depth + 1, point, id);
			}
		}
	}

	void insert(pcl::PointXYZ point, int id)
	{
		insert_helper(root, 0, point, id);
	}

	void search_helper(pcl::PointXYZ target, Node *head, int depth, float distanceTol, std::vector<int> &ids)
	{
		if (head != NULL)
		{
			float point_value[3] = { head->point.x, head->point.y, head->point.z };
			float target_value[3] = { target.x, target.y, target.z };
			
			if ((target_value[0] - distanceTol <= point_value[0]) && (point_value[0] <= target_value[0] + distanceTol) &&
				(target_value[1] - distanceTol <= point_value[1]) && (point_value[1] <= target_value[1] + distanceTol) &&
				(target_value[2] - distanceTol <= point_value[2]) && (point_value[2] <= target_value[2] + distanceTol))
			{
				float distance = sqrt((point_value[0] - target_value[0]) * (point_value[0] - target_value[0]) +
									  (point_value[1] - target_value[1]) * (point_value[1] - target_value[1]) +
									  (point_value[2] - target_value[2]) * (point_value[2] - target_value[2]));
				if (distance <= distanceTol)
					ids.push_back(head->id);
			}

			if ((target_value[depth % 3] - distanceTol) < point_value[depth % 3])
				search_helper(target, head->left, depth + 1, distanceTol, ids);

			if ((target_value[depth % 3] + distanceTol) > point_value[depth % 3])
				search_helper(target, head->right, depth + 1, distanceTol, ids);
		}
	}

	std::vector<int> search(pcl::PointXYZ target, float distanceTol)
	{
		std::vector<int> ids;
		search_helper(target, root, 0, distanceTol, ids);
		return ids;
	}
};

#endif // KD_TREE_H
