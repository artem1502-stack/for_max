#ifndef SURFACEWIRE_HPP
# define SURFACEWIRE_HPP

class SurfaceWire {
public:
	// constructor
	SurfaceWire(TopoDS_Wire& wire, int index);

	//get Topo_DS_Wire pointer
	const TopoDS_Wire* getTopoDSWire() const;

	// add edge to the wire
	void pushEdgeIndex(int new_index);
	// add point to the wire
	void pushPointIndex(int new_index);

	// set pointer to the global edges and point vectors
	void setPointers(vector<SurfaceEdge>* edges_pointer, vector<SurfacePoint>* points_pointer);

	// get number of edges in this wire
	int getEdgeCount() const;
	//get number of points in this wire
	int getPointCount() const;

	// get point with the given local index from the wire
	SurfacePoint* getPoint(int index);
	// get edge with the given local index from the wire
	SurfaceEdge* getEdge(int index);

	// get index of the wire
	int getIndex() const;
	// get local index of the edge
	int getEdgeLocalIndex(const SurfaceEdge& edge) const;
	// get local index of the point
	int getPointLocalIndex(const SurfacePoint& point) const;

private:
	TopoDS_Wire m_wire;
	int m_index;
	vector<int> m_edges_indices;
	vector<int> m_points_indices;
	vector<SurfaceEdge>* m_edges_pointer;
	vector<SurfacePoint>* m_points_pointer;
};

#endif
