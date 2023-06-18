#include "h.h"

void DelaunayTriangulation::addPoint(std::array<double, 2> p, bool euclid_flag) {
	int idx = this->coords.size();
	this->coords.push_back(p);
	std::vector<int> bad_triangles;
	int T_id;
	if (idx == 234) {
		for (auto i : this->coords) { std::cout << "(" << i[0] << ", " << i[1] << ")," << std::endl; }
		std::cout << std::endl << std::endl << std::endl;
		for (auto i : this->mesh) {
			std::cout << "(" << i.second[0] << ", " << i.second[1] << ", " << i.second[2] << ")," << std::endl;
		}
		std::cout << std::endl << std::endl << std::endl;
		for (auto i : this->neighbor_triangles) {
			std::cout << "(" << this->triangles[i.first][0] << ", " << this->triangles[i.first][1] << ", " << this->triangles[i.first][2] << ")," << std::endl;
		}
	}
	//std::cout << std::endl << std::endl << std::endl;
	//for (auto i : this->coords) { std::cout << "(" << i[0] << ", " << i[1] << ")," << std::endl; }
	//std::cout << std::endl << std::endl << std::endl;
	for (auto tri : this->neighbor_triangles) {
		if (this->initTriangle(tri.first, p)) {
			T_id = tri.first;
			break;
		}
	}
	bad_triangles.push_back(T_id);
	std::vector<std::array<int, 3>> boundary;
	int edge = 0;
	int tri_op_id;
	int prev = 0;
	int next = 0;
	int pos = 0;
	//for (auto i : bad_triangles) {
	//	std::cout << "(" << this->triangles[i][0] << ", " << this->triangles[i][1] << ", " << this->triangles[i][2] << ")," << std::endl;
	//}
	//std::cout << std::endl << std::endl << std::endl;
	if (euclid_flag) {
		while (true) {
			tri_op_id = this->neighbor_triangles[T_id][edge];
			if (tri_op_id==-1 || !this->inCircle(tri_op_id, p)) {
				if (edge == 0) { prev = 2; next = 1; }
				if (edge == 1) { prev = 0; next = 2; }
				if (edge == 2) { prev = 1; next = 0; }
				std::array<int, 3> item = { this->triangles[T_id][next], this->triangles[T_id][prev], tri_op_id };
				boundary.push_back(item);
				edge = (edge + 1) % 3;
				if (boundary[0][0] == boundary[boundary.size() - 1][1]) { break; }
			}
			else {
				if (!this->checkVector(tri_op_id, bad_triangles)) { bad_triangles.push_back(tri_op_id); }
				for (int i = 0; i < 3; i++) {
					if (T_id == this->neighbor_triangles[tri_op_id][i]) {
						pos = i;
						break;
					}
				}
				edge = (pos + 1) % 3;
				T_id = tri_op_id;
			}
		}
	}
	else {
		while (true) {
			tri_op_id = this->neighbor_triangles[T_id][edge];
			if (tri_op_id == -1 || !this->inCircumdisk(tri_op_id, p)) {
				if (edge == 0) { prev = 2; next = 1; }
				if (edge == 1) { prev = 0; next = 2; }
				if (edge == 2) { prev = 1; next = 0; }
				std::array<int, 3> item = { this->triangles[T_id][next], this->triangles[T_id][prev], tri_op_id };
				boundary.push_back(item);
				edge = (edge + 1) % 3;
				if (boundary[0][0] == boundary[boundary.size() - 1][1]) { break; }
			}
			else {
				if (!this->checkVector(tri_op_id, bad_triangles)) { bad_triangles.push_back(tri_op_id); }
				for (int i = 0; i < 3; i++) {
					if (T_id == this->neighbor_triangles[tri_op_id][i]) {
						pos = i;
						break;
					}
				}
				edge = (pos + 1) % 3;
				T_id = tri_op_id;
			}
		}
	}
	int count = 0;
	int step = -1;
	std::vector<int> new_triangles;
	for (auto item : boundary) {
		std::array<int, 3> T = { idx, item[0], item[1] };
		this->id += 1;
		if (this->id == 1906) {
			std::cout << std::endl;
		}
		this->triangles.emplace(this->id, T);
		if(euclid_flag)
			this->circles.emplace(this->id, this->circumcenter(T));
		else
			this->circles.emplace(this->id, this->circumdiskCenter(T));
		std::array<int,3> n_T = { item[2], -1, -1 };
		this->neighbor_triangles.emplace(this->id, n_T);
		if (item[2] != -1) {
			for (auto i : this->neighbor_triangles[item[2]]) {
				step++;
				count = 0;
				if (i != -1) {
					for (int j = 0; j < 3; j++) {
						if ((this->triangles[i][j] == item[0]) || (this->triangles[i][j] == item[1]))
							count++;
					}
				}
				if (count == 2) {
					this->neighbor_triangles[item[2]][step] = this->id;
					step = -1;
					break;
				}
			}
		}
		new_triangles.push_back(this->id);
	}
	step = -1;
	for (auto T : bad_triangles) {
		this->delete_id.push_back(T);
		this->neighbor_triangles.erase(T);
		this->circles.erase(T);
	}
	for (auto T : new_triangles) {
		step++;
		prev = step - 1;
		next = (step + 1) % new_triangles.size();
		if (prev == -1) { prev = new_triangles.size() - 1; }
		this->neighbor_triangles[T][1] = new_triangles[next];
		this->neighbor_triangles[T][2] = new_triangles[prev];
	}
}

std::map<int, std::array<int, 3>> DelaunayTriangulation::exportTriangles() {
	return this->triangles;
}

void DelaunayTriangulation::boundaryRecovery(std::vector<std::array<double, 2>> front, bool frame_flag) {
	int id;
	std::vector<int> vec;
	for (int i : this->delete_id)
		this->triangles.erase(i);
	this->delete_id.clear();
	if (frame_flag) {
		for (auto i : this->triangles) {
			for (auto j : i.second) {
				if ((j == 0) || (j == 1) || (j == 2) || (j == 3)) {
					id = i.first;
					vec.push_back(id);
					break;
				}
			}
			for (auto k : this->neighbor_triangles) {
				for (int l = 0; l < 3; l++) {
					if (k.second[l] == id)
						this->neighbor_triangles[k.first][l] = -1;
				}
			}
		}
		for (int i : vec) {
			this->triangles.erase(i);
			this->neighbor_triangles.erase(i);
			this->circles.erase(i);
		}
	}
	else {
		std::array<double, 2> A;
		std::array<double, 2> B;
		int a_id;
		int b_id;
		int c_id;
		int count;
		for (int i=0; i < front.size() - 1; i++) {
			A = front[i];
			B = front[i + 1];
			count = 0;
			for (auto item : front) {
				if (item != A && item != B && CurveLength(A, item) < (this->metr + this->eps) && CurveLength(B, item) < (this->metr + this->eps)) {
					for (int i = 0; i < this->coords.size(); i++) {
						if (this->coords.at(i) == A)
							a_id = i;
						if (this->coords.at(i) == B)
							b_id = i;
						if (this->coords.at(i) == item)
							c_id = i;
					}
					std::array<int, 3> new_T = { a_id,b_id,c_id };
					this->mesh.emplace(this->mesh.size(), new_T);
					count++;
					if (count > 1) { break; }
				}
			}
		}
	}
}
