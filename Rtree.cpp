#include "h.h"

// done
RTree::RTree() : m_min_count(MINCOUNT), m_max_count(MAXCOUNT) {
	// initialize root
	m_nodes.push_back(RTreeNode());
	m_root = &(*m_nodes.begin());
}

// done
RTreeNode* RTree::chooseSubtree(RTreeNode& parent_node, RTreeNode& new_node) const {
	RTreeNode* current_node = &parent_node;
	vector<RTreeNode*>* current_node_children = current_node->getChildren();
	int optimal_node_index;

	while (true) {
		// if this is a leaf, we return it
		if (current_node->isLeaf())
			return current_node;

		// if children of this node are leafes, we chose an optimal leaf
		if (current_node->childrenAreLeafes())
			optimal_node_index = findOptimalLeafToPlace(*current_node, new_node);

		// if children of this node are not leafes, we find optimal child node
		else
			optimal_node_index = findOptimalNodeToPlace(*current_node, new_node);

		current_node = current_node->getChildren()->at(optimal_node_index);
	}
}

// done
void RTree::splitNode(RTreeNode& node_to_split, RTreeNode& node_to_insert) {
	vector<RTreeNode*> nodes = *node_to_split.getChildren();
	nodes.push_back(&node_to_insert);
	vector<RTreeNode*> first_node_children, second_node_children;
	int first_node_children_count, second_node_children_count;
	RTreeNode first_new_node;
	RTreeNode second_new_node;
	int optimal_border;
	Bnd_Box2d tmp_bnd_box;
	RTreeNode* first_new_node_ptr, * second_new_node_ptr;

	// find optimal axis for splitting
	Axis optimal_axis = chooseSplitAxis(node_to_split, node_to_insert);

	// find optimal distribution along selected axis
	if (optimal_axis == Axis::X_ASIS) {
		bool left_sort_is_optimal;
		chooseSplitIndexXAxis(node_to_split, node_to_insert, optimal_border, left_sort_is_optimal);
		if (left_sort_is_optimal)
			sort(nodes.begin(), nodes.end(), [](RTreeNode* node_a, RTreeNode* node_b) {return compareLeftSidesOfNodes(*node_a, *node_b); });
		else
			sort(nodes.begin(), nodes.end(), [](RTreeNode* node_a, RTreeNode* node_b) {return compareRightSidesOfNodes(*node_a, *node_b); });
	}
	else {
		bool down_sort_is_optimal;
		chooseSplitIndexYAxis(node_to_split, node_to_insert, optimal_border, down_sort_is_optimal);
		if (down_sort_is_optimal)
			sort(nodes.begin(), nodes.end(), [](RTreeNode* node_a, RTreeNode* node_b) {return compareDownSidesOfNodes(*node_a, *node_b); });
		else
			sort(nodes.begin(), nodes.end(), [](RTreeNode* node_a, RTreeNode* node_b) {return compareUpSidesOfNodes(*node_a, *node_b); });
	}

	// split children array of the node to two arrays
	first_node_children_count = optimal_border - (int)1;
	second_node_children_count = nodes.size() - first_node_children_count;
	first_node_children.reserve(first_node_children_count);
	second_node_children.reserve(second_node_children_count);
	for (int i = 0; i < first_node_children_count; ++i)
		first_node_children.push_back(nodes[i]);
	for (int i = first_node_children_count; i < first_node_children_count + second_node_children_count; ++i)
		second_node_children.push_back(nodes[i]);

	// create two new nodes
	first_new_node.setChildren(first_node_children);
	second_new_node.setChildren(second_node_children);
	calcNodeBndBox(first_new_node);
	calcNodeBndBox(second_new_node);
	first_new_node.setIsLeaf(true);
	second_new_node.setIsLeaf(true);
	first_new_node.setParent(&node_to_split);
	second_new_node.setParent(&node_to_split);

	// push new nodes to the global vector
	m_nodes.push_back(first_new_node);
	first_new_node_ptr = &(*(--m_nodes.end()));
	m_nodes.push_back(second_new_node);
	second_new_node_ptr = &(*(--m_nodes.end()));

	RTreeNode* tmp_node = node_to_split.getChildren()->at(2);

	// edit parent node
	node_to_split.getChildren()->at(0) = first_new_node_ptr;
	node_to_split.getChildren()->at(1) = second_new_node_ptr;

	for (int i = 2; i < node_to_split.getChildren()->size(); ++i)
		node_to_split.getChildren()->at(i) = nullptr;
	node_to_split.getChildren()->resize(2);

	node_to_split.setIsLeaf(false);
}

// done
void RTree::insertNode(double x, double y, double gap) {
	if (gap < 0)
		throw exception("Wrong gap specifiend when inserting node to the R-tree!\n");

	Bnd_Box2d tmp_bnd_box;
	tmp_bnd_box.Add(gp_Pnt2d(x, y));
	tmp_bnd_box.SetGap(gap);
	RTreeNode node_to_insert;
	node_to_insert.setBndBox(tmp_bnd_box);

	m_nodes.push_back(node_to_insert);
	RTreeNode* node_to_insert_ptr = &(*(--m_nodes.end()));

	insertNodeInternal(node_to_insert_ptr);
}

void RTree::insertNode(double x, double y, double x_gap, double y_gap) {
	if (x_gap < 0 || y_gap < 0)
		throw exception("Wrong gap specifiend when inserting node to the R-tree!\n");

	Bnd_Box2d tmp_bnd_box;
	tmp_bnd_box.SetGap(0);
	tmp_bnd_box.Add(gp_Pnt2d(x - x_gap, y - y_gap));
	tmp_bnd_box.Add(gp_Pnt2d(x + x_gap, y + y_gap));
	RTreeNode node_to_insert;
	node_to_insert.setBndBox(tmp_bnd_box);

	m_nodes.push_back(node_to_insert);
	RTreeNode* node_to_insert_ptr = &(*(--m_nodes.end()));

	insertNodeInternal(node_to_insert_ptr);
}

// done
bool RTree::testNodeIsNear(double x, double y) const {
	gp_Pnt2d point_to_test(x, y);
	RTreeNode* current_node;
	RTreeNode* candidates_stack[50];
	int current_stack_length = 0;
	candidates_stack[current_stack_length] = m_root;
	current_stack_length++;
	vector <RTreeNode*>* current_node_children;

	// iterate through the tree and put all candidates to the stack
	while (current_stack_length) {
		current_node = candidates_stack[current_stack_length - 1];
		current_node_children = current_node->getChildren();
		current_stack_length--;

		if (current_node->isLeaf()) {
			for (int i = 0; i < current_node_children->size(); ++i)
				if (!current_node_children->at(i)->getBndBox()->IsOut(point_to_test))
					return true;
			return false;
		}

		else {
			for (int i = 0; i < current_node_children->size(); ++i)
				if (!current_node_children->at(i)->getBndBox()->IsOut(point_to_test)) {
					candidates_stack[current_stack_length] = current_node_children->at(i);
					current_stack_length++;
				}
		}
	}

	return false;
}

// done
void RTree::calcNodeBndBox(RTreeNode& node) {
	vector <RTreeNode*>* node_children = node.getChildren();
	if (node_children->empty())
		return;

	Bnd_Box2d tmp_bnd_box;
	for (int i = 0; i < node_children->size(); ++i)
		tmp_bnd_box.Add(*node_children->at(i)->getBndBox());
	node.setBndBox(tmp_bnd_box);
}

// done
Axis RTree::chooseSplitAxis(RTreeNode& node_to_split, RTreeNode& node_to_insert) const {
	vector<RTreeNode*> nodes = *node_to_split.getChildren();
	nodes.push_back(&node_to_insert);
	double x_sum_of_prerimeters = 0;
	double y_sum_of_prerimeters = 0;

	// sort nodes by left side of the bounding box and calculate sum of perimeters for different distributions
	sort(nodes.begin(), nodes.end(), [](RTreeNode* node_a, RTreeNode* node_b) {return compareLeftSidesOfNodes(*node_a, *node_b); });
	x_sum_of_prerimeters += sumOfDistrinutionPerimeters(nodes);

	// sort nodes by the right side of the bounding box and calculate sum of perimeters for different distributions
	sort(nodes.begin(), nodes.end(), [](RTreeNode* node_a, RTreeNode* node_b) {return compareRightSidesOfNodes(*node_a, *node_b); });
	x_sum_of_prerimeters += sumOfDistrinutionPerimeters(nodes);

	// sort nodes by the down side of the bounding box and calculate sum of perimeters for different distributions
	sort(nodes.begin(), nodes.end(), [](RTreeNode* node_a, RTreeNode* node_b) {return compareDownSidesOfNodes(*node_a, *node_b); });
	y_sum_of_prerimeters += sumOfDistrinutionPerimeters(nodes);

	// sort nodes by the upper side of the bounding box and calculate sum of perimeters for different distributions
	sort(nodes.begin(), nodes.end(), [](RTreeNode* node_a, RTreeNode* node_b) {return compareUpSidesOfNodes(*node_a, *node_b); });
	y_sum_of_prerimeters += sumOfDistrinutionPerimeters(nodes);

	// choose axis with the minimum summ of perimeters
	if (x_sum_of_prerimeters > y_sum_of_prerimeters)
		return Axis::Y_AXIS;
	return Axis::X_ASIS;
}

// done
void RTree::chooseSplitIndexXAxis(RTreeNode& node_to_split, RTreeNode& node_to_insert, int& optimal_border, bool& left_sort_is_optimal) const {
	vector<RTreeNode*> nodes = *node_to_split.getChildren();
	nodes.push_back(&node_to_insert);
	vector<double> left_sorted_overlaps, left_sorted_squares;
	vector<double> right_sorted_overlaps, right_sorted_squares;
	double minimim_overlap, left_min_overlap, right_min_overlap;
	double minimum_square;
	int k;

	// calc overlaps and squares for different distributions of left sorted nodes
	sort(nodes.begin(), nodes.end(), [](RTreeNode* node_a, RTreeNode* node_b) {return compareLeftSidesOfNodes(*node_a, *node_b); });
	calcOverlapsAndSquares(nodes, left_sorted_overlaps, left_sorted_squares);

	// calc overlaps and squares for different distributions of right sorted nodes
	sort(nodes.begin(), nodes.end(), [](RTreeNode* node_a, RTreeNode* node_b) {return compareRightSidesOfNodes(*node_a, *node_b); });
	calcOverlapsAndSquares(nodes, right_sorted_overlaps, right_sorted_squares);

	// find minimum overlap amoung all distributions
	left_min_overlap = *min_element(left_sorted_overlaps.begin(), left_sorted_overlaps.end());
	right_min_overlap = *min_element(right_sorted_overlaps.begin(), right_sorted_overlaps.end());
	if (left_min_overlap > right_min_overlap)
		minimim_overlap = right_min_overlap;
	else
		minimim_overlap = left_min_overlap;

	// find distribution with the minimum overlap and the least square
	minimum_square = INT_MAX;
	for (int i = 0; i < left_sorted_overlaps.size(); ++i) {
		if (left_sorted_overlaps[i] == minimim_overlap && left_sorted_squares[i] < minimum_square) {
			minimum_square = left_sorted_squares[i];
			k = i + 1;
			optimal_border = (m_min_count - 1) + k + 1;
			left_sort_is_optimal = true;
		}

		if (right_sorted_overlaps[i] == minimim_overlap && right_sorted_squares[i] < minimum_square) {
			minimum_square = right_sorted_squares[i];
			k = i + 1;
			optimal_border = (m_min_count - 1) + k + 1;
			left_sort_is_optimal = false;
		}
	}
}

// done
void RTree::chooseSplitIndexYAxis(RTreeNode& node_to_split, RTreeNode& node_to_insert, int& optimal_border, bool& down_sort_is_optimal)  const {
	vector<RTreeNode*> nodes = *node_to_split.getChildren();
	nodes.push_back(&node_to_insert);
	vector<double> down_sorted_overlaps, down_sorted_squares;
	vector<double> up_sorted_overlaps, up_sorted_squares;
	double minimim_overlap, down_min_overlap, up_min_overlap;
	double minimum_square;

	// calc overlaps and squares for different distributions of left sorted nodes
	sort(nodes.begin(), nodes.end(), [](RTreeNode* node_a, RTreeNode* node_b) {return compareDownSidesOfNodes(*node_a, *node_b); });
	calcOverlapsAndSquares(nodes, down_sorted_overlaps, down_sorted_squares);

	// calc overlaps and squares for different distributions of right sorted nodes
	sort(nodes.begin(), nodes.end(), [](RTreeNode* node_a, RTreeNode* node_b) {return compareUpSidesOfNodes(*node_a, *node_b); });
	calcOverlapsAndSquares(nodes, up_sorted_overlaps, up_sorted_squares);

	// find minimum overlap amoung all distributions
	down_min_overlap = *min_element(down_sorted_overlaps.begin(), down_sorted_overlaps.end());
	up_min_overlap = *min_element(up_sorted_overlaps.begin(), up_sorted_overlaps.end());
	if (down_min_overlap > up_min_overlap)
		minimim_overlap = up_min_overlap;
	else
		minimim_overlap = down_min_overlap;

	// find distribution with the minimum overlap and the least square
	minimum_square = INT_MAX;
	for (int i = 0; i < down_sorted_overlaps.size(); ++i) {
		if (down_sorted_overlaps[i] == minimim_overlap && down_sorted_squares[i] < minimum_square) {
			minimum_square = down_sorted_squares[i];
			optimal_border = (m_min_count - 1) + (i + 1) + 1;
			down_sort_is_optimal = true;
		}

		if (up_sorted_overlaps[i] == minimim_overlap && up_sorted_squares[i] < minimum_square) {
			minimum_square = up_sorted_squares[i];
			optimal_border = (m_min_count - 1) + (i + 1) + 1;
			down_sort_is_optimal = false;
		}
	}
}

void RTree::insertNodeInternal(RTreeNode* node_to_insert) {
	RTreeNode* node_to_push_into;

	// chose node to push new node to
	node_to_push_into = chooseSubtree(*m_root, *node_to_insert);

	// simply put new node to the tree
	if (node_to_push_into->getChildren()->size() < m_max_count) {
		node_to_insert->setParent(node_to_push_into);
		node_to_insert->setIsLeaf(false);
		node_to_push_into->getChildren()->push_back(node_to_insert);
		for (RTreeNode* node = node_to_push_into; node != nullptr; node = node->getParent())
			calcNodeBndBox(*node);
	}

	// we have to split some node to push a new one
	else {
		splitNode(*node_to_push_into, *node_to_insert);
		for (RTreeNode* node = node_to_push_into; node != nullptr; node = node->getParent())
			calcNodeBndBox(*node);
	}
}



// done
double RTree::calcBndBoxesOverlap(const Bnd_Box2d& box_a, const Bnd_Box2d& box_b) const {
	double a_min_x, a_max_x, a_min_y, a_max_y;
	double b_min_x, b_max_x, b_min_y, b_max_y;
	double overlap_width, overlap_height;

	box_a.Get(a_min_x, a_min_y, a_max_x, a_max_y);
	box_b.Get(b_min_x, b_min_y, b_max_x, b_max_y);

	double left = max(a_min_x, b_min_x);
	double top = min(a_max_y, b_max_y);
	double right = min(a_max_x, b_max_x);
	double bottom = max(a_min_y, b_min_y);

	overlap_width = right - left;
	overlap_height = top - bottom;

	if (overlap_width < 0 || overlap_height < 0)
		return 0;

	return overlap_width * overlap_height;
}

// done
double RTree::calcBndBoxSquare(const Bnd_Box2d& box) const {
	double x_min, x_max, y_min, y_max;
	box.Get(x_min, y_min, x_max, y_max);

	return (x_max - x_min) * (y_max - y_min);
}

// done
double RTree::calcBndBoxPerimeter(const Bnd_Box2d& box) const {
	double x_min, x_max, y_min, y_max;
	box.Get(x_min, y_min, x_max, y_max);
	return 2 * (x_max - x_min) + 2 * (y_max - y_min);
}

// done
Bnd_Box2d RTree::summOfBndBoxes(const RTreeNode& first_node, const RTreeNode& second_node) const {
	Bnd_Box2d result = *first_node.getBndBox();
	result.Add(*second_node.getBndBox());
	return result;
}

// done
int RTree::findOptimalLeafToPlace(RTreeNode& parent_node, const RTreeNode& new_node)  const {
	// checks
	if (parent_node.isLeaf())
		return INT_MAX;
	if (!parent_node.childrenAreLeafes())
		return INT_MAX;

	Bnd_Box2d tmp_bnd_box, tmp_bnd_box_2;
	vector<double> overlap_before_addition, overlap_after_addition, summs_of_overlap_deltas;
	vector<double> squares_after_addition, square_deltas;
	vector <int> candidates;
	vector<int> new_candidates;
	int number_of_candidates;
	vector<RTreeNode*>* leafes = parent_node.getChildren();
	double summ_of_overlap_deltas;
	double square_before_addition, square_after_addition;

	overlap_before_addition.reserve(leafes->size());
	overlap_after_addition.reserve(leafes->size());
	summs_of_overlap_deltas.reserve(leafes->size());
	squares_after_addition.reserve(leafes->size());
	square_deltas.reserve(leafes->size());
	candidates.reserve(leafes->size());

	// mark all leafes as candidates
	for (int i = 0; i < leafes->size(); ++i)
		candidates.push_back(i);

	// try to insert point into every leaf
	// i is a leaf we try to add a point
	for (int i = 0; i < leafes->size(); ++i) {
		// firstly calculates squares before and afer addition of the new point and delta
		tmp_bnd_box = *leafes->at(i)->getBndBox();
		square_before_addition = calcBndBoxSquare(tmp_bnd_box);
		// j is a leaf we calc overlap with
		for (int j = 0; j < leafes->size(); ++j) {
			tmp_bnd_box_2 = *leafes->at(j)->getBndBox();
			overlap_before_addition.push_back(calcBndBoxesOverlap(tmp_bnd_box, tmp_bnd_box_2));
		}

		// j is a leaf we calc overlap with
		tmp_bnd_box = summOfBndBoxes(*(leafes->at(i)), new_node);
		square_after_addition = calcBndBoxSquare(tmp_bnd_box);
		squares_after_addition.push_back(square_after_addition);
		square_deltas.push_back(square_after_addition - square_before_addition);
		for (int j = 0; j < leafes->size(); ++j) {
			tmp_bnd_box_2 = *leafes->at(j)->getBndBox();
			overlap_after_addition.push_back(calcBndBoxesOverlap(tmp_bnd_box, tmp_bnd_box_2));
		}

		// now calclulate summ of overlap deltas for child i
		summ_of_overlap_deltas = 0;
		for (int j = 0; j < leafes->size(); ++j)
			summ_of_overlap_deltas += overlap_after_addition[j] - overlap_before_addition[j];

		// put summ to vector
		summs_of_overlap_deltas.push_back(summ_of_overlap_deltas);

		overlap_before_addition.clear();
		overlap_after_addition.clear();
	}

	// now we can analyse vectors and chose the best leafe to insert point in
	// firstly we look for leafes with the least overlap delta
	auto min_overlap_delta = min_element(summs_of_overlap_deltas.begin(), summs_of_overlap_deltas.end());
	number_of_candidates = count(summs_of_overlap_deltas.begin(), summs_of_overlap_deltas.end(), *min_overlap_delta);

	// if there is only one candidate, retrun it's index
	if (number_of_candidates == 1)
		return min_overlap_delta - summs_of_overlap_deltas.begin();

	// else we put their indices into vector
	new_candidates.clear();
	new_candidates.reserve(number_of_candidates);
	for (int i = 0; i < candidates.size(); ++i) {
		if (summs_of_overlap_deltas[i] == *min_overlap_delta)
			new_candidates.push_back(i);
		else {
			square_deltas[candidates[i]] = INT_MAX;
			squares_after_addition[candidates[i]] = INT_MAX;
		}
	}
	candidates = new_candidates;

	// and find candidates with the least square delta amoung them
	auto min_square_delta = min_element(square_deltas.begin(), square_deltas.end());
	number_of_candidates = count(square_deltas.begin(), square_deltas.end(), *min_square_delta);

	// if there is only one candidate, retrun it's index
	if (number_of_candidates == 1)
		return min_square_delta - square_deltas.begin();

	// else we put their indices into vector
	new_candidates.clear();
	new_candidates.reserve(number_of_candidates);
	for (int i = 0; i < candidates.size(); ++i) {
		if (square_deltas[i] == *min_square_delta)
			new_candidates.push_back(i);
		else
			squares_after_addition[candidates[i]] = INT_MAX;
	}
	candidates = new_candidates;

	// and find candidate with the least square
	auto min_square = min_element(squares_after_addition.begin(), squares_after_addition.end());

	// we need one of this candates
	return min_square - squares_after_addition.begin();
}

// done
int RTree::findOptimalNodeToPlace(RTreeNode& parent_node, const RTreeNode& new_node) const {
	// checks
	if (parent_node.isLeaf())
		return INT_MAX;
	if (parent_node.childrenAreLeafes())
		return INT_MAX;

	Bnd_Box2d tmp_bnd_box;
	vector<double> squares_after_addition, square_deltas;
	vector <int> candidates;
	int number_of_candidates;
	vector<RTreeNode*>* children = parent_node.getChildren();
	double square_before_addition, square_after_addition;

	squares_after_addition.reserve(children->size());
	square_deltas.reserve(children->size());
	candidates.reserve(children->size());

	// mark all children as candidates
	for (int i = 0; i < children->size(); ++i)
		candidates.push_back(i);

	// try to insert point into every leaf
	// i is a leaf we try to add a point
	for (int i = 0; i < children->size(); ++i) {
		// calculate squares before and afer addition of the new point and delta
		tmp_bnd_box = *children->at(i)->getBndBox();
		square_before_addition = calcBndBoxSquare(tmp_bnd_box);
		tmp_bnd_box = summOfBndBoxes(*(children->at(i)), new_node);
		square_after_addition = calcBndBoxSquare(tmp_bnd_box);
		squares_after_addition.push_back(square_after_addition);
		square_deltas.push_back(square_after_addition - square_before_addition);
	}

	// now we can analyse vectors and chose the best leafe to insert point in
	// firstly we look for nodes with the least square delta
	auto min_square_delta = min_element(square_deltas.begin(), square_deltas.end());
	number_of_candidates = count(square_deltas.begin(), square_deltas.end(), *min_square_delta);

	// if there is only one candidate, retrun it's index
	if (number_of_candidates == 1)
		return min_square_delta - square_deltas.begin();

	for (int i = 0; i < candidates.size(); ++i)
		if (square_deltas[i] != *min_square_delta)
			squares_after_addition[i] = INT_MAX;

	// and return candidate with the least square
	auto min_square = min_element(squares_after_addition.begin(), squares_after_addition.end());

	return min_square - squares_after_addition.begin();
}

// done
bool RTree::compareLeftSidesOfNodes(const RTreeNode& node_a, const RTreeNode& node_b) {
	double a_left, b_left;
	double right, down, up;

	node_a.getBndBox()->Get(a_left, down, right, up);
	node_b.getBndBox()->Get(b_left, down, right, up);

	return a_left < b_left;
}

// done
bool RTree::compareRightSidesOfNodes(const RTreeNode& node_a, const RTreeNode& node_b) {
	double a_right, b_right;
	double left, down, up;

	node_a.getBndBox()->Get(left, down, a_right, up);
	node_b.getBndBox()->Get(left, down, b_right, up);

	return a_right > b_right;
}

// done
bool RTree::compareDownSidesOfNodes(const RTreeNode& node_a, const RTreeNode& node_b) {
	double a_down, b_down;
	double left, right, up;

	node_a.getBndBox()->Get(left, a_down, right, up);
	node_b.getBndBox()->Get(left, b_down, right, up);

	return a_down < b_down;
}

// done
bool RTree::compareUpSidesOfNodes(const RTreeNode& node_a, const RTreeNode& node_b) {
	double a_up, b_up;
	double left, right, down;

	node_a.getBndBox()->Get(left, down, right, a_up);
	node_b.getBndBox()->Get(left, down, right, b_up);

	return a_up > b_up;
}

// done
double RTree::sumOfDistrinutionPerimeters(const vector<RTreeNode*>& nodes) const {
	int k_max = m_max_count - 2 * m_min_count + 2;
	int border_between_two_groups;
	double summ_of_perimeters = 0;
	Bnd_Box2d bnd_box_1_group, bnd_box_2_group;

	// check all possible node distributions and accumulate summs of perimeters
	for (int k = 1; k <= k_max; ++k) {
		border_between_two_groups = (m_min_count - 1) + k;
		bnd_box_1_group = *nodes[0]->getBndBox();
		for (int i = 0; i < border_between_two_groups; ++i)
			bnd_box_1_group.Add(*nodes[i]->getBndBox());

		bnd_box_2_group = *nodes[border_between_two_groups]->getBndBox();
		for (int i = border_between_two_groups; i < nodes.size(); ++i)
			bnd_box_2_group.Add(*nodes[i]->getBndBox());

		summ_of_perimeters += calcBndBoxPerimeter(bnd_box_1_group) + calcBndBoxPerimeter(bnd_box_2_group);
	}

	return summ_of_perimeters;
}

// done
void RTree::calcOverlapsAndSquares(const vector <RTreeNode*>& nodes, vector <double>& overlaps, vector<double>& squares) const {
	int k_max = m_max_count - 2 * m_min_count + 2;
	int border_between_two_groups;
	vector<int> candidates, new_candidates;
	Bnd_Box2d bnd_box_1_group, bnd_box_2_group;

	overlaps.clear();
	squares.clear();
	overlaps.reserve(k_max);
	squares.reserve(k_max);

	// calc overlaps and squares of different distributions
	for (int k = 1; k <= k_max; ++k) {
		border_between_two_groups = (m_min_count - 1) + k;
		bnd_box_1_group = *nodes[0]->getBndBox();
		for (int i = 0; i < border_between_two_groups; ++i)
			bnd_box_1_group.Add(*nodes[i]->getBndBox());

		bnd_box_2_group = *nodes[border_between_two_groups]->getBndBox();
		for (int i = border_between_two_groups; i < nodes.size(); ++i)
			bnd_box_2_group.Add(*nodes[i]->getBndBox());

		overlaps.push_back(calcBndBoxesOverlap(bnd_box_1_group, bnd_box_2_group));
		squares.push_back(calcBndBoxSquare(bnd_box_1_group) + calcBndBoxSquare(bnd_box_2_group));
	}
}

RTreeNode::RTreeNode() : m_parent(nullptr), m_is_leaf(true) {
	m_bnd_box.SetVoid();
	m_children.clear();
	m_children.reserve(MAXCOUNT);
}

bool RTreeNode::isLeaf() const {
	return m_is_leaf;
}

bool RTreeNode::childrenAreLeafes()  const {
	if (m_children.empty())
		return false;
	return m_children.at(0)->isLeaf();
}

void RTreeNode::setIsLeaf(bool new_is_leaf) {
	m_is_leaf = new_is_leaf;
}

const Bnd_Box2d* RTreeNode::getBndBox() const {
	return &m_bnd_box;
}

void RTreeNode::setBndBox(const Bnd_Box2d& new_bnd_box) {
	m_bnd_box = new_bnd_box;
}

vector<RTreeNode*>* RTreeNode::getChildren() {
	return &m_children;
}

void RTreeNode::setChildren(const vector<RTreeNode*>& new_children) {
	m_children = new_children;
}

RTreeNode* RTreeNode::getParent() {
	return m_parent;
}

void RTreeNode::setParent(RTreeNode* new_parent) {
	m_parent = new_parent;
}
