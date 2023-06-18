#ifndef RTREE_HPP
# define RTREE_HPP

#include <Bnd_Box2d.hxx>
#include <vector>
#include <array>
#include <list>
#include <stack>
#include <algorithm>
#include "SurfacePoint.hpp"

#define MAXCOUNT 1000
#define MINCOUNT 400

using namespace std;

enum Axis {
	X_ASIS,
	Y_AXIS
};

class RTreeNode {
private:
	Bnd_Box2d m_bnd_box;
	vector<RTreeNode*> m_children;
	RTreeNode* m_parent;
	bool m_is_leaf;

public:
	// constructor (need max count fo optimization)
	RTreeNode();

	// check if this node is a leaf
	bool isLeaf() const;
	// check if this node's children are leafes
	bool childrenAreLeafes() const;

	// interface
	void setIsLeaf(bool new_is_leaf);

	const Bnd_Box2d* getBndBox() const;
	void setBndBox(const Bnd_Box2d& new_bnd_box);

	vector<RTreeNode*>* getChildren();
	void setChildren(const vector<RTreeNode*>& new_children);

	RTreeNode* getParent();
	void setParent(RTreeNode* node);
};

class RTree {
private:
	int m_max_count;
	int m_min_count;
	RTreeNode* m_root;
	list<RTreeNode> m_nodes;
	int array_size;

public:
	// constructor of the tree
	RTree();

	// interface to insert node to the tree
	void insertNode(double x, double y, double gap);
	void insertNode(double x, double y, double x_gap, double y_gap);

	// interface to define collision of some node with the elemets of the tree
	bool testNodeIsNear(double x, double y) const;

private:
	// internal function to insert a node
	void insertNodeInternal(RTreeNode* node_to_insert);

	// calculate overlap of two bounding boxes
	double calcBndBoxesOverlap(const Bnd_Box2d& box_a, const Bnd_Box2d& box_b) const;
	// calculate bounding box square
	double calcBndBoxSquare(const Bnd_Box2d& box) const;
	// calculate bounding box square
	double calcBndBoxPerimeter(const Bnd_Box2d& box) const;
	// calculate summ of two bounding boxes
	Bnd_Box2d summOfBndBoxes(const RTreeNode& first_node, const RTreeNode& second_node) const;

	// find optimal leaf to add a new pont amoung children of the parent node
	int findOptimalLeafToPlace(RTreeNode& parent_node, const RTreeNode& new_node) const;
	// find optimal child of the node to add a new point
	int findOptimalNodeToPlace(RTreeNode& parent_node, const RTreeNode& new_node) const;

	// criteria to compare left sides bounding boxes of the nodes to sort them
	static bool compareLeftSidesOfNodes(const RTreeNode& node_a, const RTreeNode& node_b);
	// criteria to compare right sides bounding boxes of the nodes to sort them
	static bool compareRightSidesOfNodes(const RTreeNode& node_a, const RTreeNode& node_b);
	// criteria to compare down sides bounding boxes of the nodes to sort them
	static bool compareDownSidesOfNodes(const RTreeNode& node_a, const RTreeNode& node_b);
	// criteria to compare upper sides bounding boxes of the nodes to sort them
	static bool compareUpSidesOfNodes(const RTreeNode& node_a, const RTreeNode& node_b);

	// choose axis for the optimal node splitting
	Axis chooseSplitAxis(RTreeNode& node_to_split, RTreeNode& node_to_insert) const;

	// choose distribution which is optimal for node splitting along X axis
	void chooseSplitIndexXAxis(RTreeNode& node_to_split, RTreeNode& node_to_insert, int& optimal_border, bool& left_sort_is_optimal) const;
	// choose distribution which is optimal for node splitting along Y axis
	void chooseSplitIndexYAxis(RTreeNode& node_to_split, RTreeNode& node_to_insert, int& optimal_border, bool& down_sort_is_optimal) const;

	// check all possible node distributions and accumulate summs of perimeters
	double sumOfDistrinutionPerimeters(const vector<RTreeNode*>& nodes) const;

	// calc overlaps and squares of different distributions
	void calcOverlapsAndSquares(const vector <RTreeNode*>& nodes, vector <double>& overlaps, vector<double>& squares) const;

	// calculate bounding box of the node
	void calcNodeBndBox(RTreeNode& node);

	// chose optimal subtree to place a new node
	RTreeNode* chooseSubtree(RTreeNode& parent_node, RTreeNode& new_node) const;

	// split node in optimal way to insert a new child
	void splitNode(RTreeNode& node_to_split, RTreeNode& new_node);
};

#endif

