#ifndef DELAUNAYTRIANGULATON_HPP
# define DELAUNAYTRIANGULATON_HPP

# include "h.h"

class DelaunayTriangulation {
private:
	int id;
	std::vector<int> delete_id;
	std::vector<std::array<double,2>> coords;
	std::map<int, std::array<int, 3>> triangles;
	std::map<int, std::array<int, 3>> neighbor_triangles;
	std::map<int, std::array<double, 3>> circles;
	std::map<int, std::array<int, 3>> mesh;
	std::set<int> act_points;
	NURBSSurface surface;
	double eps;
	double metr;
public:
	DelaunayTriangulation(double u0, double u1, double v0, double v1, NURBSSurface surface, double eps, double metr);
	std::array<double, 3> circumcenter(std::array<int, 3> tri);
	bool inCircle(int tri_id, std::array<double, 2> p);
	void addPoint(std::array<double, 2> p, bool euclid_flag);
	std::map<int, std::array<int, 3>> exportTriangles();
	void boundaryRecovery(std::vector<std::array<double, 2>> front, bool frame_flag);
	bool checkVector(int i, std::vector<int> vec);
	void recalculateCircumcentre();
	std::array<double, 3> circumdiskCenter(std::array<int, 3> tri);
	bool inCircumdisk(int tri_id, std::array<double, 2> p);
	double CurveLength(std::array<double, 2> A, std::array<double, 2> B);
	double ScalarMulV3(double* v1, double* v2);
	void deleteBaseTriangle(std::array<double, 2> a, std::array<double, 2> b, std::array<double, 2> c);
	std::vector<std::array<double, 2>> getCoords();
	std::map<int, std::array<int, 3>> getMesh();
	std::array<double, 2> selectFromDT(std::array<double, 2> a, std::array<double, 2> b);
	void lastTriangle(std::array<double, 2> a, std::array<double, 2> b, std::array<double, 2> c);
	bool initTriangle(int tri_id, std::array<double, 2> p);
	bool actualPoints(std::array<double, 2> p);
};

#endif
