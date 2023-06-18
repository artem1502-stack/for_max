#include "h.h"

Solver2D::Solver2D(Loader* loader, BodySurface* surface, double mesh_step, double precision) : m_loader(loader),
m_surface(surface), m_u_step(INT_MAX), m_u_step_square(INT_MAX), m_v_step(INT_MAX), m_v_step_square(INT_MAX), m_precision(precision) {
	double max_number_of_nodes_along_axis = 0;
	double current_number_of_nodes_along_axis;
	double start_parameter, end_parameter;
	double edge_length;
	double u_min, u_max, v_min, v_max;
	double u_lenght, v_lenght;

	for (int i = 0; i < m_surface->getEdgeCount(); ++i) {
		SurfaceEdge* edge = m_surface->getEdge(i);
		start_parameter = edge->getPointParameter(0);
		end_parameter = edge->getPointParameter(1);

		edge_length = GCPnts_AbscissaPoint::Length(GeomAdaptor_Curve(edge->getGeomCurve(), start_parameter, end_parameter));

		current_number_of_nodes_along_axis = edge_length / mesh_step;
		if (current_number_of_nodes_along_axis > max_number_of_nodes_along_axis)
			max_number_of_nodes_along_axis = current_number_of_nodes_along_axis;
	}

	m_number_of_nodes_along_axis = max_number_of_nodes_along_axis * 2;

	m_surface->getUVBoundaries(u_min, u_max, v_min, v_max);
	if (u_min > u_max)
		swap<double>(u_min, u_max);

	if (v_min > v_max)
		swap<double>(u_min, u_max);

	u_lenght = u_max - u_min;
	v_lenght = v_max - v_min;

	m_u_step = u_lenght / m_number_of_nodes_along_axis;
	m_v_step = v_lenght / m_number_of_nodes_along_axis;
	m_u_step_square = m_u_step * m_u_step;
	m_v_step_square = m_v_step * m_v_step;
	m_min_step = fmin(m_u_step, m_v_step);

	u_min -= m_u_step;
	u_max += m_u_step;
	v_min -= m_v_step;
	v_max += m_v_step;

	m_min_u_index = (int)(u_min / m_u_step) - (m_min_u_index < 0);
	m_max_u_index = (int)(u_max / m_u_step) + 1;
	m_min_v_index = (int)(v_min / m_v_step) - (m_min_v_index < 0);
	m_max_v_index = (int)(v_max / m_v_step) + 1;
}

bool Solver2D::solve() {
	if (!generateMesh()) {
		string message = "Error, while generating initial mesh!\n";
		message += "\n\nLogs:\n";
		message += m_log;
		throw exception(message.c_str());
	}

	if (!setBoundaryConditions()) {
		string message = "Error, while setting initial mesh values!\n";
		message += "\n\nLogs:\n";
		message += m_log;
		throw exception(message.c_str());
	}

	handleCyclicSurfaces();

	calcInitialValues();

	solveBoundaryProblem();

	calcResult();

	// appendPointsMapToLog();

	return true;
}

double Solver2D::getAngle(double u_coord, double v_coord) const {
	// check is this point is on the surface
	if (!isOnSurface(u_coord, v_coord))
		return INT_MAX;

	int current_point_u_index = (int)(u_coord / m_u_step) - (u_coord < 0);
	int current_point_v_index = (int)(v_coord / m_v_step) - (v_coord < 0);

	return m_mesh.at(pair<int, int>(current_point_u_index, current_point_v_index)).getAngle();
}

bool Solver2D::isOnSurface(double u_coord, double v_coord) const {
	int current_point_u_index = (int)(u_coord / m_u_step) - (u_coord < 0);
	int current_point_v_index = (int)(v_coord / m_v_step) - (v_coord < 0);

	// check if there is such point in solver mesh
	if (!pointIsCorrect(current_point_u_index, current_point_v_index))
		return false;

	// check point type
	PointType point_type = m_mesh.at(pair<int, int>(current_point_u_index, current_point_v_index)).getPointType();
	if (point_type == PointType::EXTERNAL_POINT)
		return false;

	return true;
}

double Solver2D::calcAverageAngle() const {
	double angle_accumulator = 0.;
	SurfacePoint* point;
	double angle;
	int point_local_index;
	double point_u_coord, point_v_coord, point_edge_parameter;

	int points_counter = 0;
	for (int i = 0; i < m_surface->getEdgeCount(); ++i) {
		SurfaceEdge* edge = m_surface->getEdge(i);

		for (int j = 0; j < edge->getPointCount(); ++j) {
			point = edge->getPoint(j);
			point_local_index = m_surface->getPointLocalIndex(*point);
			m_surface->getUVCoordsOfPoint(point_local_index, point_u_coord, point_v_coord);
			point_edge_parameter = edge->getPointParameter(j);

			angle = calcAngle(edge, point_u_coord, point_v_coord, point_edge_parameter);
			angle_accumulator += angle;
			++points_counter;
		}
	}

	return angle_accumulator / points_counter;
}

bool Solver2D::generateMesh() {
	IntTools_FClass2d point_tester(*(m_surface->getTopoDSFace()), m_min_step / 2.);
	TopAbs_State state;

	double average_angle = calcAverageAngle();

	int inner_points_counter = 0;
	double u_coord, v_coord;
	bool is_near_border;
	for (int u_index = m_min_u_index; u_index <= m_max_u_index; ++u_index) {

		for (int v_index = m_min_v_index; v_index <= m_max_v_index; ++v_index) {
			u_coord = u_index * m_u_step;
			v_coord = v_index * m_v_step;
			state = point_tester.Perform(gp_Pnt2d(u_coord, v_coord));
			is_near_border = (u_index == m_min_u_index) || (u_index == m_max_u_index) || (v_index == m_min_v_index) || (v_index == m_max_v_index);

			if (state == TopAbs_IN && !is_near_border) {
				m_mesh.emplace(pair<double, double>(u_index, v_index), Solver2DPoint(u_coord, v_coord, average_angle, PointType::INNER_POINT));
				++inner_points_counter;
			}

			else
				m_mesh.emplace(pair<double, double>(u_index, v_index), Solver2DPoint(u_coord, v_coord, INT_MIN, PointType::EXTERNAL_POINT));
		}
	}

	if (inner_points_counter < 5) {
		m_log += "Error, not enough points in mesh!\n";
		return false;
	}

	return true;
}

bool Solver2D::setBoundaryConditions() {
	SurfaceEdge* edge;
	int edge_local_index;
	Handle(Geom2d_Curve) edge_uv;
	double start_parameter, end_parameter;
	double edge_uv_length;
	double new_point_parameter;
	gp_Pnt2d new_point;

	for (int i = 0; i < m_surface->getEdgeCount(); ++i) {
		edge = m_surface->getEdge(i);
		edge_local_index = m_surface->getEdgeLocalIndex(*edge);
		edge_uv = m_surface->getEdgeInUVSpace(edge_local_index);
		m_surface->getEdgeInUVSpaceParameters(edge_local_index, start_parameter, end_parameter);
		edge_uv_length = GCPnts_AbscissaPoint::Length(Geom2dAdaptor_Curve(edge_uv, start_parameter, end_parameter));

		for (double distance_from_the_start = 0; distance_from_the_start <= edge_uv_length; distance_from_the_start += m_min_step) {
			GCPnts_AbscissaPoint point_calculator(Geom2dAdaptor_Curve(edge_uv, start_parameter, end_parameter), distance_from_the_start, start_parameter, m_min_step / 5.);

			if (!point_calculator.IsDone())
				return false;

			new_point_parameter = point_calculator.Parameter();
			edge_uv->D0(new_point_parameter, new_point);

			setConditionIfBoundaryPoint(edge, &new_point, new_point_parameter);
		}
	}

	return true;
}

double Solver2D::calcAngle(SurfaceEdge* edge, double point_u_coord,
	double point_v_coord, double point_edge_parameter) const {
	int edge_local_index_in_surface, point_local_index_in_edge, point_local_index_in_surface;
	TopAbs_Orientation edge_orientation;

	gp_Pnt tmp_point;
	gp_Vec tangent_vector;
	edge->getGeomCurve()->D1(point_edge_parameter, tmp_point, tangent_vector);

	edge_orientation = BRepTools::OriEdgeInFace(*(edge->getTopoDSEdge()), *(m_surface->getTopoDSFace()));
	if (edge_orientation == TopAbs_REVERSED)
		tangent_vector.Multiply(-1.);

	gp_Vec u_derr, v_derr;
	m_surface->getGeomSurface()->D1(point_u_coord, point_v_coord, tmp_point, u_derr, v_derr);
	gp_Vec normal = u_derr;
	normal.Cross(v_derr);

	return u_derr.AngleWithRef(tangent_vector, normal);
}

void Solver2D::setConditionIfBoundaryPoint(SurfaceEdge* edge, gp_Pnt2d* new_point, double new_point_edge_parameter) {
	int new_point_u_index = (int)(new_point->X() / m_u_step) - (new_point->X() < 0);
	int new_point_v_index = (int)(new_point->Y() / m_v_step) - (new_point->Y() < 0);
	double boundary_angle;
	bool is_near_border;
	Solver2DPoint* point_to_check;

	// u looks down
	// v looks right
	// points
	// 1  2 
	// 5  6 
	// we start from 1

	for (int i = 0; i < 2; ++i)
		for (int j = 0; j < 2; ++j) {
			if (!pointIsCorrect(new_point_u_index, new_point_v_index))
				continue;

			is_near_border = pointIsNearBorder(new_point_u_index + i, new_point_v_index + j);
			point_to_check = &m_mesh.at(pair<int, int>(new_point_u_index + i, new_point_v_index + j));

			if (point_to_check->getPointType() == PointType::EXTERNAL_POINT || is_near_border) {
				boundary_angle = calcAngle(edge, new_point->X(), new_point->Y(), new_point_edge_parameter);
				point_to_check->setAngle(boundary_angle);
				point_to_check->setType(PointType::BOUNDARY_POINT);
			}
		}
}

void Solver2D::appendPointsMapToLog() {
	m_log += "Map of points\n";
	Solver2DPoint* point;
	double angle;
	string string_angle;

	for (int i = m_min_v_index; i <= m_max_v_index; ++i) {
		for (int j = m_min_u_index; j <= m_max_u_index; ++j) {
			point = &m_mesh.at(pair<int, int>(j, i));

			switch (point->getPointType()) {
			case PointType::INNER_POINT:
				m_log += " o ";
				break;
			case PointType::BOUNDARY_POINT:
				m_log += " x ";
				break;
			case PointType::EXTERNAL_POINT:
				m_log += "   ";
				break;
			}
		}
		m_log += '\n';
	}
}

string Solver2D::getLog() const {
	return m_log;
}

bool Solver2D::pointIsNearBorder(int u_index, int v_index) const {
	return (u_index == m_min_u_index) || (u_index == m_max_u_index) ||
		(v_index == m_min_v_index) || (v_index == m_max_v_index);
}

bool Solver2D::pointIsCorrect(int u_index, int v_index) const {
	return (u_index >= m_min_u_index) && (u_index <= m_max_u_index) &&
		(v_index >= m_min_v_index) && (v_index <= m_max_v_index);
}

void Solver2D::coutA() {
	cout << "A" << endl;
	Solver2DPoint* point;
	double a;

	for (int i = m_min_v_index; i <= m_max_v_index; ++i) {
		for (int j = m_min_u_index; j <= m_max_u_index; ++j) {
			point = &m_mesh.at(pair<int, int>(j, i));
			cout << " ";
			if (point->getPointType() != PointType::EXTERNAL_POINT) {
				a = point->getA();
				cout << setw(5) << floor(a * 100.) / 100.;
			}
			else
				cout << setw(5) << 0.;
		}
		cout << endl;
	}
	cout << endl;
}

void Solver2D::coutB() {
	cout << "B" << endl;
	Solver2DPoint* point;
	double b;

	for (int i = m_min_v_index; i <= m_max_v_index; ++i) {
		for (int j = m_min_u_index; j <= m_max_u_index; ++j) {
			point = &m_mesh.at(pair<int, int>(j, i));
			cout << " ";
			if (point->getPointType() != PointType::EXTERNAL_POINT) {
				b = point->getB();
				cout << setw(5) << floor(b * 100.) / 100.;
			}
			else
				cout << setw(5) << 0.;
		}
		cout << endl;
	}
	cout << endl;
}

void Solver2D::coutAngles() {
	cout << "Angle" << endl;
	Solver2DPoint* point;
	double angle;

	for (int i = m_min_v_index; i <= m_max_v_index; ++i) {
		for (int j = m_min_u_index; j <= m_max_u_index; ++j) {
			point = &m_mesh.at(pair<int, int>(j, i));
			cout << " ";
			if (point->getPointType() != PointType::EXTERNAL_POINT) {
				angle = point->getAngle();
				cout << setw(5) << floor(angle * 100.) / 100.;
			}
			else
				cout << setw(5) << 0.;
		}
		cout << endl;
	}
	cout << endl;
}

void Solver2D::solveBoundaryProblem() {
	Solver2DPoint* point;
	double old_a, new_a, old_b, new_b, delta, max_delta;
	unsigned int iterations_counter = 0;

	max_delta = INT_MAX;
	while (max_delta > m_precision) {
		max_delta = INT_MIN;
		++iterations_counter;

		for (int i = m_min_u_index; i <= m_max_u_index; ++i)

			for (int j = m_min_v_index; j <= m_max_v_index; ++j) {
				point = &m_mesh.at(pair<int, int>(i, j));

				if (point->getPointType() == PointType::INNER_POINT) {
					old_a = point->getA();
					new_a = 0;
					new_a += ((&m_mesh.at(pair<int, int>(i, j - 1)))->getA() + (&m_mesh.at(pair<int, int>(i, j + 1)))->getA()) * m_u_step_square;
					new_a += ((&m_mesh.at(pair<int, int>(i - 1, j)))->getA() + (&m_mesh.at(pair<int, int>(i + 1, j)))->getA()) * m_v_step_square;
					new_a /= 2 * (m_u_step_square + m_v_step_square);
					point->setA(new_a);

					delta = abs(new_a - old_a);
					if (delta > max_delta)
						max_delta = delta;

					old_b = point->getB();
					new_b = 0;
					new_b += ((&m_mesh.at(pair<int, int>(i, j - 1)))->getB() + (&m_mesh.at(pair<int, int>(i, j + 1)))->getB()) * m_u_step_square;
					new_b += ((&m_mesh.at(pair<int, int>(i - 1, j)))->getB() + (&m_mesh.at(pair<int, int>(i + 1, j)))->getB()) * m_v_step_square;
					new_b /= 2 * (m_u_step_square + m_v_step_square);
					point->setB(new_b);

					delta = abs(new_b - old_b);
					if (delta > max_delta)
						max_delta = delta;
				}
			}
	}

	m_log += "Boundary problem was solved after " + to_string(iterations_counter) + " iterations\n";
}

void Solver2D::handleCyclicSurfaces() {
	if (m_surface->getGeomSurface()->IsUPeriodic()) {
		int from_u_index = m_min_u_index;
		int to_u_index = m_max_u_index;
		bool swapped = false;
		PointType type;
		double angle;

		if ((&m_mesh.at(pair<int, int>(from_u_index, 0)))->getPointType() != PointType::BOUNDARY_POINT) {
			swap<int>(from_u_index, to_u_index);
			swapped = true;
		}

		for (int v = m_min_v_index; v <= m_max_v_index; ++v) {
			type = (&m_mesh.at(pair<int, int>(from_u_index, v)))->getPointType();
			angle = (&m_mesh.at(pair<int, int>(from_u_index, v)))->getAngle();
			(&m_mesh.at(pair<int, int>(to_u_index, v)))->setType(type);
			(&m_mesh.at(pair<int, int>(to_u_index, v)))->setAngle(-angle);
		}

		if (swapped == false) {
			++from_u_index;
			--to_u_index;
		}
		else {
			--from_u_index;
			++to_u_index;
		}

		for (int v = m_min_v_index; v <= m_max_v_index; ++v) {
			type = (&m_mesh.at(pair<int, int>(from_u_index, v)))->getPointType();
			angle = (&m_mesh.at(pair<int, int>(from_u_index, v)))->getAngle();
			(&m_mesh.at(pair<int, int>(to_u_index, v)))->setType(type);
			(&m_mesh.at(pair<int, int>(to_u_index, v)))->setAngle(-angle);
		}
	}

	if (m_surface->getGeomSurface()->IsVPeriodic()) {
		int from_v_index = m_min_v_index;
		int to_v_index = m_max_v_index;
		bool swapped = false;
		PointType type;
		double angle;

		if ((&m_mesh.at(pair<int, int>(0, from_v_index)))->getPointType() != PointType::BOUNDARY_POINT) {
			swap<int>(from_v_index, to_v_index);
			swapped = true;
		}

		for (int u = m_min_u_index; u <= m_max_u_index; ++u) {
			type = (&m_mesh.at(pair<int, int>(u, from_v_index)))->getPointType();
			angle = (&m_mesh.at(pair<int, int>(u, from_v_index)))->getAngle();
			(&m_mesh.at(pair<int, int>(u, to_v_index)))->setType(type);
			(&m_mesh.at(pair<int, int>(u, to_v_index)))->setAngle(-angle);
		}

		if (swapped == false) {
			++from_v_index;
			--to_v_index;
		}
		else {
			--from_v_index;
			++to_v_index;
		}

		for (int u = m_min_u_index; u <= m_max_u_index; ++u) {
			type = (&m_mesh.at(pair<int, int>(u, from_v_index)))->getPointType();
			angle = (&m_mesh.at(pair<int, int>(u, from_v_index)))->getAngle();
			(&m_mesh.at(pair<int, int>(u, to_v_index)))->setType(type);
			(&m_mesh.at(pair<int, int>(u, to_v_index)))->setAngle(-angle);
		}
	}
}

void Solver2D::calcInitialValues() {
	for (auto iter = m_mesh.begin(); iter != m_mesh.end(); ++iter)
		iter->second.calcAB();
}

void Solver2D::calcResult() {
	for (auto iter = m_mesh.begin(); iter != m_mesh.end(); ++iter)
		iter->second.calcAngle();
}
