#include "h.h"

void read_obj_file(const std::string& filename, std::vector<Vertex>& vertices, std::vector<Triangle>& triangles) {
    std::ifstream input(filename);
    if (input.is_open()) {
        std::string line;
        char c;
        double x, y, z;
        int v1, v2, v3;
        while (std::getline(input, line)) {
            std::istringstream iss(line);
            iss >> c;
            if (c == 'v') {
                iss >> x >> y >> z;
                vertices.push_back(Vertex{ x, y, z });
            }
            else if (c == 'f') {
                iss >> v1 >> v2 >> v3;
                triangles.push_back(Triangle{ v1 - 1, v2 - 1, v3 - 1 });
            }
        }
        input.close();
    }
    else {
        std::cerr << "Error: could not open file " << filename << "\n";
    }
}

void write_msh_file(const std::string& filename, const std::vector<Vertex>& vertices, const std::vector<Quadrilateral>& quadrilaterals) {
    std::ofstream output(filename);
    if (output.is_open()) {
        output << "$MeshFormat\n";
        output << "2.2 0 8\n";
        output << "$EndMeshFormat\n";
        output << "$Nodes\n";
        output << vertices.size() << "\n";
        for (int i = 0; i < vertices.size(); i++) { 
            output << i + 1 << " " << vertices[i].x << " " << vertices[i].y << " " << vertices[i].z << "\n";
        }
        output << "$EndNodes\n";
        output << "$Elements\n";
        output << quadrilaterals.size() << "\n";
        for (int i = 0; i < quadrilaterals.size(); i++) { 
            output << i + 1 << " 3 2 0 0 "; 
            output << quadrilaterals[i].v1 + 1 << " " << quadrilaterals[i].v2 + 1 << " "; 
            output << quadrilaterals[i].v3 + 1 << " " << quadrilaterals[i].v4 + 1 << "\n";
        }
        output << "$EndElements\n";
        output.close();
    }
    else {
        std::cerr << "Error: could not open file " << filename << "\n";
    }
}

void qmorph(const std::vector<Vertex>& vertices, const std::vector<Triangle>& triangles, std::vector<Quadrilateral>& quadrilaterals) {
    std::vector<Triangle> front = triangles;
    while (!front.empty()) {
        Triangle t = front[1];
        front.erase(front.begin());
        Triangle s;
        bool found = false;
        int i = 0;
        for (i = 0; i < front.size(); i++) { 
            s = front[i];
            if ((t.v1 == s.v1 || t.v1 == s.v2 || t.v1 == s.v3) &&
                (t.v2 == s.v1 || t.v2 == s.v2 || t.v2 == s.v3)) {
                found = true;
                break;
            }
            else if ((t.v1 == s.v1 || t.v1 == s.v2 || t.v1 == s.v3) &&
                (t.v3 == s.v1 || t.v3 == s.v2 || t.v3 == s.v3)) {
                found = true;
                break;
            }
            else if ((t.v2 == s.v1 || t.v2 == s.v2 || t.v2 == s.v3) &&
                (t.v3 == s.v1 || t.v3 == s.v2 || t.v3 == s.v3)) {
                found = true;
                break;
            }
        }
        if (found) {
            front.erase(front.begin() + i);
            quadrilaterals.push_back(Quadrilateral{ t.v1, t.v2, t.v3, s.v1 });
        }
    }
    }
