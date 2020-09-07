/* \author Aaron Brown */
// Quiz on implementing kd tree


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

	void insertHelper(Node **node, int depth, std::vector<float> point, int id)
	{
		uint cd = depth % 2;
		if(*node == NULL)
		{
			*node = new Node(point,id);
		}
		else if(point[cd] < ((*node)->point[cd]))
		{
			insertHelper(&((*node)->left), depth+1, point, id);
		}
		else
		{
			insertHelper(&((*node)->right), depth+1, point, id);
		}
		
		

	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root,0,point,id);

	}

	void searchHelper(std::vector<int> &ids, Node *node, int depth, std::vector<float> target, float distanceTol)
	{
		if(node!=NULL)
		{
			uint cd = depth%2;
			if((fabs(node->point[0]-target[0])<=distanceTol) && (fabs(node->point[1]-target[1])<=distanceTol))
			{
				float d=sqrt((node->point[0]-target[0])*(node->point[0]-target[0])+(node->point[1]-target[1])*(node->point[1]-target[1]));
				if(d<=distanceTol)
					ids.push_back(node->id);
			}

			if((target[cd]+distanceTol) > (node->point[cd]))
				searchHelper(ids, (node->right),depth+1,target,distanceTol);
			if((target[cd]-distanceTol) < (node->point[cd]))
				searchHelper(ids, (node->left),depth+1,target,distanceTol);

		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(ids, root,0,target,distanceTol);
		return ids;
	}
	

};




