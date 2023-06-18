#include "h.h"

void SurfaceMeshGenerator::generateSurfaceMesh(double mesh_step, double precision) {
	m_log += "\nStart surface mesh generation\n";

	m_mesh_step = mesh_step;
	m_precision = precision;
	BodySurface* current_surface;

	// generations mesh for the surface
	for (int i = 0; i < m_loader->getSurfacesCount(); ++i) {
		m_log += "Solving boundary problem for face " + to_string(i) + "\n";
		current_surface = m_loader->getSurface(i);

		Solver2D solver(m_loader, current_surface, m_mesh_step, m_precision);
		solver.solve();

		m_log += solver.getLog();

		m_log += "Generating surface mesh for face " + to_string(i) + "\n";
		RTree rtree;
		queue<pointQueueElement> points_queue;
		sheduleEdgePoints(*current_surface, points_queue, rtree, solver);
		createMesh(*current_surface, points_queue, rtree, solver);

		m_log += "Surface mesh for face " + to_string(i) + " is done\n";
		m_log += "It contains " + to_string(m_surfaces_mesh_point_indices.at(i).size()) + " nodes\n\n";
	}

	m_log += "Surfacce mesh is done! It contains " + to_string(m_mesh_points.size()) + " nodes\n";
}

const vector<gp_Pnt>* SurfaceMeshGenerator::getSurfaceMeshPoints() const {
	return &m_mesh_points;
}

const vector<int>* SurfaceMeshGenerator::getMeshPointsIndicesOfSurface(int surface_index) const {
	return &m_surfaces_mesh_point_indices.at(surface_index);
}

int SurfaceMeshGenerator::getSurfaceCount() const {
	return m_surfaces_mesh_point_indices.size();
}

string SurfaceMeshGenerator::getLog() const {
	return m_log;
}

void SurfaceMeshGenerator::calclulateDeltasForNewNodes(const BodySurface& surface, const gp_Pnt2d& point_parameters,
	const Solver2D& solver, gp_Vec2d& delta_1, gp_Vec2d& delta_2) const {
	const Geom_Surface* current_surface = surface.getGeomSurface();
	gp_Vec local_basis_u, local_basis_v, normal;
	gp_Vec direction_field_u, direction_field_v;
	gp_Pnt2d curent_point_parameters;
	gp_Pnt current_point;
	double basis_rotation_angle;
	double metric_tensor[2][2];
	double metric_tensor_determinant;
	double inverse_metric_tensor[2][2];
	double right_side_of_the_system[2];

	//get local basis in the point
	current_surface->D1(point_parameters.X(), point_parameters.Y(), current_point, local_basis_u, local_basis_v);

	// calc normal in the point
	normal = local_basis_u;
	normal.Cross(local_basis_v);
	normal.Normalize();

	// get angle to rotate local basis and get directional basis
	basis_rotation_angle = solver.getAngle(curent_point_parameters.X(), curent_point_parameters.Y());
	gp_Dir axis_direction(normal);
	gp_Ax1 rotation_axis(current_point, axis_direction);

	// calculate diresctional basis
	direction_field_u = local_basis_u;
	direction_field_u.Rotate(rotation_axis, basis_rotation_angle);

	direction_field_v = direction_field_u;
	direction_field_v.Cross(normal);

	direction_field_u.Normalize();
	direction_field_v.Normalize();
	direction_field_u.Scale(m_mesh_step);
	direction_field_v.Scale(m_mesh_step);

	// calculate metric tensor
	metric_tensor[0][0] = local_basis_u.Dot(local_basis_u);
	metric_tensor[0][1] = local_basis_u.Dot(local_basis_v);
	metric_tensor[1][0] = local_basis_v.Dot(local_basis_u);
	metric_tensor[1][1] = local_basis_v.Dot(local_basis_v);
	metric_tensor_determinant = metric_tensor[0][0] * metric_tensor[1][1] -
		metric_tensor[0][1] * metric_tensor[1][0];

	// calculate inverse metric tensor
	inverse_metric_tensor[0][0] = metric_tensor[1][1] / metric_tensor_determinant;
	inverse_metric_tensor[0][1] = -metric_tensor[1][0] / metric_tensor_determinant;
	inverse_metric_tensor[1][0] = -metric_tensor[0][1] / metric_tensor_determinant;
	inverse_metric_tensor[1][1] = metric_tensor[0][0] / metric_tensor_determinant;

	// calculate right side of the first system
	right_side_of_the_system[0] = direction_field_u.Dot(local_basis_u);
	right_side_of_the_system[1] = direction_field_u.Dot(local_basis_v);

	// find delta vectors for two new nodes
	delta_1.SetX(inverse_metric_tensor[0][0] * right_side_of_the_system[0] +
		inverse_metric_tensor[0][1] * right_side_of_the_system[1]);
	delta_1.SetY(inverse_metric_tensor[1][0] * right_side_of_the_system[0] +
		inverse_metric_tensor[1][1] * right_side_of_the_system[1]);

	// calculate right side of the second system
	right_side_of_the_system[0] = direction_field_v.Dot(local_basis_u);
	right_side_of_the_system[1] = direction_field_v.Dot(local_basis_v);

	// find delta vectors for other two new nodes
	delta_2.SetX(inverse_metric_tensor[0][0] * right_side_of_the_system[0] +
		inverse_metric_tensor[0][1] * right_side_of_the_system[1]);
	delta_2.SetY(inverse_metric_tensor[1][0] * right_side_of_the_system[0] +
		inverse_metric_tensor[1][1] * right_side_of_the_system[1]);
}

void SurfaceMeshGenerator::calcPointParametersOnSurface(const BodySurface& surface,
	const gp_Pnt& point, gp_Pnt2d& point_parameters) const {
	double surface_u_min, surface_u_max, surface_v_min, surface_v_max;
	double point_u, point_v;
	Handle(Geom_Surface) surface_handle;
	GeomAPI_ProjectPointOnSurf point_projector;

	// we project point to the surface to get u,v coordinates
	surface_handle = surface.getGeomSurface();
	surface.getUVBoundaries(surface_u_min, surface_u_max, surface_v_min, surface_v_max);
	point_projector.Init(point, surface_handle, surface_u_min, surface_u_max,
		surface_v_min, surface_v_max, m_mesh_step / 10.);
	point_projector.Parameters(1, point_u, point_v);

	point_parameters.SetCoord(point_u, point_v);
}

void SurfaceMeshGenerator::sheduleEdgePoints(BodySurface& surface, queue<pointQueueElement>& points_queue,
	RTree& rtree, const Solver2D& solver) {
	SurfaceEdge* current_edge;
	SurfacePoint* current_point;
	int point_mesh_index, point_local_index, point_global_index, edge_local_index,
		edge_global_index, surface_global_index;
	gp_Pnt2d new_point_parameters;
	gp_Vec2d new_nodes_delta_1, new_nodes_delta_2;
	double u_gap, v_gap;
	vector<int> indices_of_proceeded_points;
	bool point_was_already_proceeded = false;
	Handle(Geom2d_Curve) edge_uv;
	double start_parameter, end_parameter;
	double edge_uv_length;
	double new_point_edge_parameter;
	gp_Pnt new_point;

	surface_global_index = surface.getIndex();

	// iterate throught edges of the surface and proceed edges that
	// were discretized already
	for (int i = 0; i < surface.getEdgeCount(); ++i) {
		current_edge = surface.getEdge(i);
		edge_local_index = surface.getEdgeLocalIndex(*current_edge);
		edge_global_index = current_edge->getIndex();

		// if edge was already discretized
		if (!m_edges_mesh_point_indices.at(edge_global_index).empty()) {

			// iterate throught points that are on the edge
			for (int j = 0; j < m_edges_mesh_point_indices.at(edge_global_index).size(); ++j) {
				point_mesh_index = m_edges_mesh_point_indices.at(edge_global_index).at(j);
				new_point = m_mesh_points.at(point_mesh_index);

				// if this point is the start point of the edge or the end of the edge
				if (j < current_edge->getPointCount()) {
					// get node global index
					point_global_index = current_edge->getPoint(j)->getIndex();

					// determine if we have already proceeded this point
					auto iter = find(indices_of_proceeded_points.begin(), indices_of_proceeded_points.end(), point_global_index);
					point_was_already_proceeded = (iter != indices_of_proceeded_points.end());
				}

				// if it is the start point of the edge or the end of the edge and it was not proceded before
				// or it is not start point of the edge or the end of the edge
				if ((!point_was_already_proceeded && j < current_edge->getPointCount()) ||
					j >= current_edge->getPointCount()) {
					// get point parameters
					calcPointParametersOnSurface(surface, new_point, new_point_parameters);

					// get  gaps for bounding box
					calclulateDeltasForNewNodes(surface, new_point_parameters, solver, new_nodes_delta_1, new_nodes_delta_2);
					u_gap = fmax(abs(new_nodes_delta_1.X()), abs(new_nodes_delta_2.X()));
					v_gap = fmax(abs(new_nodes_delta_1.Y()), abs(new_nodes_delta_2.Y()));

					// add this point to the queue of points, rtree, and surface
					points_queue.push(pointQueueElement(new_point_parameters, false));
					rtree.insertNode(new_point_parameters.X(), new_point_parameters.Y(), u_gap * 0.7, v_gap * 0.7);
					m_surfaces_mesh_point_indices.at(surface_global_index).push_back(point_mesh_index);

					// if it is the start point of the edge or the end of the edge and it was not proceded before
					// save this point's index to the vector of proceeded points
					if (j < current_edge->getPointCount())
						indices_of_proceeded_points.push_back(point_global_index);
					// or it is not start point of the edge or the end of the edge, we dont't need it's index
					else
						indices_of_proceeded_points.push_back(INT_MAX);
				}
			}
		}
	}

	// iterate throught edges of the surface and proceed edges that
	// were not  discretized already
	for (int i = 0; i < surface.getEdgeCount(); ++i) {
		current_edge = surface.getEdge(i);
		edge_local_index = surface.getEdgeLocalIndex(*current_edge);
		edge_global_index = current_edge->getIndex();

		if (m_edges_mesh_point_indices.at(edge_global_index).empty()) {
			// iterate throught points of the edge (the first and the lasrt point)
			for (int j = 0; j < current_edge->getPointCount(); ++j) {
				// get point of the surface
				current_point = current_edge->getPoint(j);
				new_point = *current_point->getGpPoint();
				point_global_index = current_point->getIndex();
				point_local_index = surface.getPointLocalIndex(*current_point);

				// determine if this point was proceeded already
				auto iter = find(indices_of_proceeded_points.begin(), indices_of_proceeded_points.end(), point_global_index);
				point_was_already_proceeded = (iter != indices_of_proceeded_points.end());

				// if the point wasn't proceed already
				if (!point_was_already_proceeded) {
					// give an index for this point
					point_mesh_index = m_mesh_points.size();

					// get point parameters
					calcPointParametersOnSurface(surface, new_point, new_point_parameters);

					// get  gaps for bounding box
					calclulateDeltasForNewNodes(surface, new_point_parameters, solver, new_nodes_delta_1, new_nodes_delta_2);
					u_gap = fmax(abs(new_nodes_delta_1.X()), abs(new_nodes_delta_2.X()));
					v_gap = fmax(abs(new_nodes_delta_1.Y()), abs(new_nodes_delta_2.Y()));

					// add this point to the queue of points, rtree, surface, edge and mesh
					points_queue.push(pointQueueElement(new_point_parameters, false));
					rtree.insertNode(new_point_parameters.X(), new_point_parameters.Y(), u_gap * 0.7, v_gap * 0.7);
					m_surfaces_mesh_point_indices.at(surface_global_index).push_back(point_mesh_index);
					m_edges_mesh_point_indices.at(edge_global_index).push_back(point_mesh_index);
					m_mesh_points.push_back(*current_point->getGpPoint());

					// put this node's index to the array of proceeded points
					indices_of_proceeded_points.push_back(point_global_index);
				}

				// if the point was proceed already
				else {
					// get mesh index of this point
					point_mesh_index =
						m_surfaces_mesh_point_indices.at(surface_global_index).at(distance(indices_of_proceeded_points.begin(), iter));

					// add this point to the edge
					m_edges_mesh_point_indices.at(edge_global_index).push_back(point_mesh_index);
				}
			}

			// now we should discretize this edge

			// get this edge in u,v space
			edge_uv = surface.getEdgeInUVSpace(edge_local_index);

			// get it's boundary parameters and length
			surface.getEdgeInUVSpaceParameters(edge_local_index, start_parameter, end_parameter);
			edge_uv_length = GCPnts_AbscissaPoint::Length(Geom2dAdaptor_Curve(edge_uv, start_parameter, end_parameter));

			// make steps along the edge in u,v space and try to create a new point on each step
			for (double distance_from_the_start = m_mesh_step; distance_from_the_start + m_mesh_step <= edge_uv_length;
				distance_from_the_start += m_mesh_step) {
				// get new point surface parameters
				GCPnts_AbscissaPoint point_calculator(Geom2dAdaptor_Curve(edge_uv, start_parameter, end_parameter),
					distance_from_the_start, start_parameter, m_mesh_step / 5.);
				new_point_edge_parameter = point_calculator.Parameter();
				edge_uv->D0(new_point_edge_parameter, new_point_parameters);

				// get  gaps for bounding box
				calclulateDeltasForNewNodes(surface, new_point_parameters, solver, new_nodes_delta_1, new_nodes_delta_2);
				u_gap = fmax(abs(new_nodes_delta_1.X()), abs(new_nodes_delta_2.X()));
				v_gap = fmax(abs(new_nodes_delta_1.Y()), abs(new_nodes_delta_2.Y()));

				// if this point is near to the existing one, go to the next step
				if (rtree.testNodeIsNear(new_point_parameters.X(), new_point_parameters.Y()))
					continue;

				// else find it's coordinates
				surface.getGeomSurface()->D0(new_point_parameters.X(), new_point_parameters.Y(), new_point);

				// give an index for this point
				point_mesh_index = m_mesh_points.size();

				// add this point to the queue of points, rtree, surface, edge and mesh
				points_queue.push(pointQueueElement(new_point_parameters, false));
				rtree.insertNode(new_point_parameters.X(), new_point_parameters.Y(), u_gap * 0.7, v_gap * 0.7);
				m_surfaces_mesh_point_indices.at(surface_global_index).push_back(point_mesh_index);
				m_edges_mesh_point_indices.at(edge_global_index).push_back(point_mesh_index);
				m_mesh_points.push_back(new_point);

				// put this node to the array of proceeded points, but we don't need it's index
				indices_of_proceeded_points.push_back(INT_MAX);
			}
		}
	}
}


void SurfaceMeshGenerator::createMesh(BodySurface& surface, queue<pointQueueElement>& points_queue,
	RTree& rtree, Solver2D& solver) {
	const Geom_Surface* current_surface = surface.getGeomSurface();
	bool need_to_try_to_insert, point_is_on_surface, there_is_enough_space;
	gp_Pnt2d current_point_parameters;
	gp_Pnt current_point;
	gp_Pnt2d new_point_parameters;
	gp_Vec2d new_nodes_delta_1, new_nodes_delta_2;
	double u_gap, v_gap;
	int surface_global_index = surface.getIndex();
	int new_mesh_point_index;
	pointQueueElement current_element;

	// work until queue is not empty
	while (!points_queue.empty()) {
		current_element = points_queue.front();
		current_point_parameters = current_element.m_point_parameters;
		need_to_try_to_insert = current_element.m_need_to_try_to_insert;
		points_queue.pop();

		// calculate coordinates of the point
		current_surface->D0(current_point_parameters.X(), current_point_parameters.Y(), current_point);

		// check if the point is on the surface and there is enough space for it
		point_is_on_surface = solver.isOnSurface(current_point_parameters.X(), current_point_parameters.Y());

		if (point_is_on_surface)
			there_is_enough_space = !rtree.testNodeIsNear(current_point_parameters.X(), current_point_parameters.Y());

		// conditions are true, proceed the node
		if (point_is_on_surface) {
			// get gaps for bounding box
			calclulateDeltasForNewNodes(surface, new_point_parameters, solver, new_nodes_delta_1, new_nodes_delta_2);
			u_gap = fmax(abs(new_nodes_delta_1.X()), abs(new_nodes_delta_2.X()));
			v_gap = fmax(abs(new_nodes_delta_1.Y()), abs(new_nodes_delta_2.Y()));

			// insert node to the mesh
			if (need_to_try_to_insert && there_is_enough_space) {
				// give index to the new node
				new_mesh_point_index = m_mesh_points.size();

				//  add this point to the rtree, surface and mesh 
				rtree.insertNode(current_point_parameters.X(), current_point_parameters.Y(), u_gap * 0.7, v_gap * 0.7);
				m_surfaces_mesh_point_indices.at(surface_global_index).push_back(new_mesh_point_index);
				m_mesh_points.push_back(current_point);
			}

			// create four new nodes
			if (there_is_enough_space || !need_to_try_to_insert) {
				new_point_parameters = current_point_parameters;
				new_point_parameters.Translate(new_nodes_delta_1);
				if (solver.isOnSurface(new_point_parameters.X(), new_point_parameters.Y()) &&
					!rtree.testNodeIsNear(new_point_parameters.X(), new_point_parameters.Y()))
					points_queue.push(pointQueueElement(new_point_parameters, true));

				new_point_parameters = current_point_parameters;
				new_point_parameters.Translate(-1 * new_nodes_delta_1);
				if (solver.isOnSurface(new_point_parameters.X(), new_point_parameters.Y()) &&
					!rtree.testNodeIsNear(new_point_parameters.X(), new_point_parameters.Y()))
					points_queue.push(pointQueueElement(new_point_parameters, true));

				new_point_parameters = current_point_parameters;
				new_point_parameters.Translate(new_nodes_delta_2);
				if (solver.isOnSurface(new_point_parameters.X(), new_point_parameters.Y()) &&
					!rtree.testNodeIsNear(new_point_parameters.X(), new_point_parameters.Y()))
					points_queue.push(pointQueueElement(new_point_parameters, true));

				new_point_parameters = current_point_parameters;
				new_point_parameters.Translate(-1 * new_nodes_delta_2);
				if (solver.isOnSurface(new_point_parameters.X(), new_point_parameters.Y()) &&
					!rtree.testNodeIsNear(new_point_parameters.X(), new_point_parameters.Y()))
					points_queue.push(pointQueueElement(new_point_parameters, true));
			}
		}
	}
}


void SurfaceMeshGenerator::performDelaunayTriangulation() {
	DelaunayTriangulation D_triang = DelaunayTriangulation(m_loader);
	D_triang.getResult();
}
