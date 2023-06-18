#ifndef SURFACEMESHGENERATOR_HPP
# define SURFACEMESHGENERATOR_HPP

# include <Geom_Surface.hxx>
# include <Geom_Curve.hxx>
# include <gp_Ax1.hxx>
# include <Standard_Handle.hxx>
# include <BRepTools.hxx>
# include <BRep_Tool.hxx>
# include <GeomAPI_ProjectPointOnSurf.hxx>
# include <cmath>
# include <queue>
# include <string>
# include "Loader.hpp"	
# include "BodySurface.hpp"
# include "SurfaceEdge.hpp"
# include "SurfacePoint.hpp"
# include "SurfacePoint.hpp"
# include "Solver2DPoint.hpp"
# include "Solver2D.hpp"
# include "Rtree.hpp"

# include <BRepBuilderAPI_MakeVertex.hxx>
# include <BRepExtrema_DistShapeShape.hxx>
# include <BRepClass_FaceClassifier.hxx>

class SurfaceMeshGenerator {
public:
	// constructor that gets loader pointer
	explicit SurfaceMeshGenerator(Loader* loader);

	// generate meshes for all surfaces with given step and use given
	// precision while solving boundary problem
	void generateSurfaceMesh(double mesh_step, double precision);

	// get all points of the mesh
	const vector<gp_Pnt>* getSurfaceMeshPoints() const;

	// get indices of the points that lay on the surface
	const vector<int>* getMeshPointsIndicesOfSurface(int surface_index) const;

	// get number of surfaces
	int getSurfaceCount() const;

	// get log of the module
	string getLog() const;

	void performDelaunayTriangulation();

private:
	// add points of the surface edge to the mesh and to the queue
	void sheduleEdgePoints(BodySurface& surface, queue<pointQueueElement>& points_queue, RTree& rtree, const Solver2D& solver);

	// generate mesh startring from he points from the queue
	void createMesh(BodySurface& surface, queue<pointQueueElement>& points_queue, RTree& rtree, Solver2D& solver);

	// calculate delta vectors between current node and four new nodes that we will try to place
	void calclulateDeltasForNewNodes(const BodySurface& surface, const gp_Pnt2d& point_parameters,
		const Solver2D& solver, gp_Vec2d& delta_1, gp_Vec2d& delta_2) const;

	// get u,v coordinates of the point on the surface
	void calcPointParametersOnSurface(const BodySurface& surface, const gp_Pnt& point, gp_Pnt2d& point_parameters) const;

	// check if the point with u,v parameters is on the surface
	// bool pointIsOnSurface(BodySurface& surface, gp_Pnt2d &point) const;

private:
	Loader* m_loader;
	string m_log;
	double m_mesh_step;
	double m_precision;
	vector<vector<int>> m_surfaces_mesh_point_indices;
	vector<vector<int>> m_edges_mesh_point_indices;
	vector<gp_Pnt> m_mesh_points;
};

#endif
