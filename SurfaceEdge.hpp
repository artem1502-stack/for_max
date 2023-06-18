#ifndef SURFACEEDGE_HPP
# define SURFACEEDGE_HPP

class SurfaceEdge {
public:
	// constructor
	SurfaceEdge(const TopoDS_Edge edge, int index);

	// get index of the edge
	int getIndex() const;

	// get TopoDSEdge of the edge
	const TopoDS_Edge* getTopoDSEdge() const;
	// get Geom_Curve of the edge
	const Geom_Curve* getGeomCurve() const;

	// add point to the edge
	void pushPointIndex(int new_index);

	// set pointer to the global points vector
	void setPointsPointer(vector<SurfacePoint>* points);

	// calc V parameters of all points of the edge
	void calcPointsParameters();

	// get number of points of the edge
	int getPointCount() const;
	// get point of the edge
	SurfacePoint* getPoint(int index) const;
	// get local index of the point
	int getPointLocalIndex(const SurfacePoint& point) const;
	// get V parameter of the point
	double getPointParameter(int index) const;

private:
	TopoDS_Edge m_edge;
	Handle(Geom_Curve) m_curve;
	vector<SurfacePoint>* m_points_pointer;
	vector<int> m_points_indices;
	array<double, 2> m_points_parameters;
	int m_index;
};

#endif
