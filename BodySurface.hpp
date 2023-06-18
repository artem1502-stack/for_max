#ifndef BODYSURFACE_HPP
# define BODYSURFACE_HPP

# include <vector>
# include <map>
# include <algorithm>
# include <BRepTools.hxx>
# include <BRep_Tool.hxx>
# include <TopoDS_Face.hxx>
# include <TopoDS_Wire.hxx>
# include <TopoDS_Edge.hxx>
# include <BRepBuilderAPI_MakeEdge.hxx>
# include "SurfaceWire.h"
# include "SurfaceEdge.h"
# include "SurfacePoint.h"

class BodySurface {
public:
	// constructor
	BodySurface(const TopoDS_Face& face, int index);

	// get TopoDS_Face
	const TopoDS_Face* getTopoDSFace() const;
	// get Geom_Surface
	const Geom_Surface* getGeomSurface() const;

	// add wire to the surface
	void pushWireIndex(int new_index);
	// add edge to the surface
	void pushEdgeIndex(int new_index);
	// add point to the surface
	void pushPointIndex(int new_index);

	// get number of the wires of the surface
	int getWireCount() const;
	// get edges of the wires of the surface
	int getEdgeCount() const;
	// get points of the wires of the surface
	int getPointCount() const;

	// get wire from the surface
	SurfaceWire* getWire(int index);
	// get edge from the surface
	SurfaceEdge* getEdge(int index);
	// get point from the surface
	SurfacePoint* getPoint(int index);

	// get index of the surface
	int getIndex() const;

	// set pointers to the global wires, edges and points vectors
	void setPointers(vector<SurfaceWire>* wires, vector<SurfaceEdge>* edges,
		vector<SurfacePoint>* points);

	// calculate minimum and maximum U, V values of the surface
	void calcUVBoundaries();
	// calculate U,V coords of all points of the surface
	void calcUVCoordsOfPoints();
	// calculate images of the edges on the U, V plane and
	// calculate minimum and maximum parameters of this 2d curves
	void calcEdgesInUVSpace();

	// get coordinates of surface point on the parametric plane
	void getUVCoordsOfPoint(int index, double& u, double& v) const;
	// get minimum and maximum U, V values of the surface
	void getUVBoundaries(double& u_min, double& u_max,
		double& v_min, double& v_max) const;

	// get edge image on the U,V plane
	Geom2d_Curve* getEdgeInUVSpace(int index);
	// get minimum and maximum parameters the edge image on the U, V plane
	void getEdgeInUVSpaceParameters(int index, double& u, double& v) const;

	// get local index of the wire
	int getWiretLocalIndex(const SurfaceWire& wire) const;
	// get local index of the edge
	int getEdgeLocalIndex(const SurfaceEdge& edge) const;
	// get local index of the point
	int getPointLocalIndex(const SurfacePoint& point) const;

private:
	TopoDS_Face m_face;
	Handle(Geom_Surface) m_surface;
	int m_index;
	double m_u_min, m_u_max, m_v_min, m_v_max;
	vector<SurfaceWire>* m_wires_pointer;
	vector<SurfaceEdge>* m_edges_pointer;
	vector<SurfacePoint>* m_points_pointer;
	vector<int> m_wires_indices;
	vector<int> m_edges_indices;
	vector<int> m_points_indices;
	vector<pair<double, double>> m_points_uv_coords;
	vector<Handle(Geom2d_Curve)> m_edges_in_uv_space;
	vector<pair<double, double>> m_edges_in_uv_space_parameters;
};

#endif
