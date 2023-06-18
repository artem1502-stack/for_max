#ifndef SOLVER2D_HPP
# define SOLVER2D_HPP

class Solver2D {
public:
	// constructor
	Solver2D(Loader* loader, BodySurface* surface, double mesh_step,
		double precision);

	// solve doundary problem
	bool solve();

	// get basis rotation angle at specific point
	double getAngle(double u_coord, double v_coord) const;

	// check if specific point is on surface  
	bool isOnSurface(double u_coord, double v_coord) const;

	//get logs of the module
	string getLog() const;

	// debug functions
	void coutAngles();
	void coutA();
	void coutB();

private:
	// calculate angle between s1 vector of local basis and vector that is 
	// tangent to the boundary at some point
	double calcAngle(SurfaceEdge* edge, double point_u_coord,
		double point_v_coord, double point_edge_parameter) const;

	// calculate everage value of angle between s1 vector of local basis 
	// and vector that is tangent to the boundary for the surface. We use
	// everage angle is an initial angle for the solution
	double calcAverageAngle() const;

	// generate mesh for the parametric plane
	bool generateMesh();

	// set boundary angles of the problem
	bool setBoundaryConditions();

	// check if the point is boundary and calculate angle in this point
	void setConditionIfBoundaryPoint(SurfaceEdge* edge, gp_Pnt2d* new_point,
		double new_point_edge_parameter);

	// handle with cyclic surfaces
	void handleCyclicSurfaces();

	// calculates A = cos(4*angle) and B = sin(4*angle)
	void calcInitialValues();

	// solve boundary problem for A and B
	void solveBoundaryProblem();

	// calculate angle based on A and B at all points
	void calcResult();

	// check if the point is near border of the surface
	bool pointIsNearBorder(int u_index, int v_index) const;

	// check is the point is correct
	bool pointIsCorrect(int u_index, int v_index) const;

	// append map of points of the mesh to the log
	void appendPointsMapToLog();

private:
	Loader* m_loader;
	BodySurface* m_surface;
	double m_u_step, m_v_step, m_min_step, m_u_step_square, m_v_step_square;
	int m_number_of_nodes_along_axis;
	double m_precision;
	map<pair<int, int>, Solver2DPoint> m_mesh;
	int m_min_u_index, m_max_u_index, m_min_v_index, m_max_v_index;
	string m_log;
};

#endif
