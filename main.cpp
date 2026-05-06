#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>
#include <cmath>
#include <random>
#include <omp.h>
#include <map>
#include <string>
#include <fstream>


#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#ifndef M_PI
#define M_PI 3.14159265358979323856
#endif
 
static std::default_random_engine engine[32];
static std::uniform_real_distribution<double> uniform(0, 1);

double sqr(double x) { return x * x; };

class Vector {
public:
	explicit Vector(double x = 0, double y = 0, double z = 0) {
		data[0] = x;
		data[1] = y;
		data[2] = z;
	}
	double norm2() const {
		return data[0] * data[0] + data[1] * data[1] + data[2] * data[2];
	}
	double norm() const {
		return sqrt(norm2());
	}
	void normalize() {
		double n = norm();
		data[0] /= n;
		data[1] /= n;
		data[2] /= n;
	}
	double operator[](int i) const { return data[i]; };
	double& operator[](int i) { return data[i]; };
	double data[3];
};

Vector operator+(const Vector& a, const Vector& b) {
	return Vector(a[0] + b[0], a[1] + b[1], a[2] + b[2]);
}
Vector operator-(const Vector& a, const Vector& b) {
	return Vector(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}
Vector operator*(const double a, const Vector& b) {
	return Vector(a*b[0], a*b[1], a*b[2]);
}
Vector operator*(const Vector& a, const double b) {
	return Vector(a[0]*b, a[1]*b, a[2]*b);
}
Vector operator*(const Vector& a, const Vector& b){
	return Vector(a[0]*b[0], a[1]*b[1], a[2]*b[2]);
}
Vector operator/(const Vector& a, const double b) {
	return Vector(a[0] / b, a[1] / b, a[2] / b);
}
double dot(const Vector& a, const Vector& b) {
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
Vector cross(const Vector& a, const Vector& b) {
	return Vector(a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]);
}

class Ray {
public:
	Ray(const Vector& origin, const Vector& unit_direction) : O(origin), u(unit_direction) {};
	Vector O, u;
};

class Object {
public:
	Object(const Vector& albedo, bool mirror = false, bool transparent = false) : albedo(albedo), mirror(mirror), transparent(transparent) {};

	virtual bool intersect(const Ray& ray, Vector& P, double& t, Vector& N) const = 0;

	Vector albedo;
	bool mirror, transparent;
};

class Sphere : public Object {
public:
	Sphere(const Vector& center, double radius, const Vector& albedo, bool mirror = false, bool transparent = false) : ::Object(albedo, mirror, transparent), C(center), R(radius) {};

	// returns true iif there is an intersection between the ray and the sphere
	// if there is an intersection, also computes the point of intersection P, 
	// t>=0 the distance between the ray origin and P (i.e., the parameter along the ray)
	// and the unit normal N
	bool intersect(const Ray& ray, Vector& P, double &t, Vector& N) const {
		 // TODO (lab 1) : compute the intersection (just true/false at the begining of lab 1, then P, t and N as well)
		 double delta = dot(ray.u, ray.O - this->C)*dot(ray.u, ray.O - this->C) - ((ray.O - this->C).norm2() - this->R*this->R);
		 double t_left = dot(ray.u, this->C - ray.O);

		 if(delta < 0){
			//we have no solutions to the equation
			return false;
		 }

		 double delta_square_root = sqrt(delta); //square root the delta


		 if(t_left < 0){
			//the only way it can intersect is if t_left + sqrt(delta) is positive
			if(t_left + delta_square_root >= 0){
				t = t_left + delta_square_root; // update our t
				P = ray.O + t*ray.u; //compute P from the equation
				N = (P-this->C);
				N.normalize();
				return true;
			}
		 }else{
			//we first check if t_left - delta_square_root is positive as that would be the first point of intersection
			if(t_left - delta_square_root > 0){
				t = t_left - delta_square_root;
				P = ray.O + t*ray.u;
				N = (P-C);
				N.normalize();
				return true;
			}else if(t_left + delta_square_root > 0){
				t = t_left + delta_square_root;
				P = ray.O + t*ray.u;
				N = (P-C);
				N.normalize();
				return true;	
			}
		 }
		return false;
	}

	double R;
	Vector C;
};


// Class only used in labs 3 and 4 
class TriangleIndices {
public:
	TriangleIndices(int vtxi = -1, int vtxj = -1, int vtxk = -1, int ni = -1, int nj = -1, int nk = -1, int uvi = -1, int uvj = -1, int uvk = -1, int group = -1) {
		vtx[0] = vtxi; vtx[1] = vtxj; vtx[2] = vtxk;
		uv[0] = uvi; uv[1] = uvj; uv[2] = uvk;
		n[0] = ni; n[1] = nj; n[2] = nk;
		this->group = group;
	};
	int vtx[3]; // indices within the vertex coordinates array
	int uv[3];  // indices within the uv coordinates array
	int n[3];   // indices within the normals array
	int group;  // face group
};

struct BVHNode {
    Vector B_min, B_max;
    int left, right;  // indices into bvh_nodes (-1 if leaf)
    int start, end;   // range in indices[]
};

// Class only used in labs 3 and 4 
class TriangleMesh : public Object {
public:
	TriangleMesh(const Vector& albedo, bool mirror = false, bool transparent = false) : ::Object(albedo, mirror, transparent) {};
	
	void find_bounds() {
		B_min = Vector(std::numeric_limits<double>::max(),std::numeric_limits<double>::max(),std::numeric_limits<double>::max());
		B_max = Vector(-std::numeric_limits<double>::max(),-std::numeric_limits<double>::max(),-std::numeric_limits<double>::max());
		for (const auto& v : vertices) {
			for (int i = 0; i < 3; i++) {
				if (v[i] < B_min[i]) B_min[i] = v[i];
				if (v[i] > B_max[i]) B_max[i] = v[i];
			}
		}
	}

	BVHNode compute_node_bounds(int start, int end) {
		BVHNode node;
		node.start = start;
		node.end = end;
		node.left = node.right = -1;
		node.B_min = Vector( std::numeric_limits<double>::max(),
							std::numeric_limits<double>::max(),
							std::numeric_limits<double>::max());
		node.B_max = Vector(-std::numeric_limits<double>::max(),
							-std::numeric_limits<double>::max(),
							-std::numeric_limits<double>::max());
		for (int i = start; i < end; i++) {
			for (int k = 0; k < 3; k++) {
				const Vector& v = vertices[indices[i].vtx[k]];
				for (int ax = 0; ax < 3; ax++) {
					if (v[ax] < node.B_min[ax]) node.B_min[ax] = v[ax];
					if (v[ax] > node.B_max[ax]) node.B_max[ax] = v[ax];
				}
			}
		}
		return node;
	}

	void build_bvh(int node_idx, int start, int end) {
		bvh_nodes[node_idx] = compute_node_bounds(start, end);

		// // we stop the recursion when not a lot of triangle are there
		if(end - start <= 3){
			return;
		}

		// split along the longest axis at the midpoint
		Vector extent = bvh_nodes[node_idx].B_max - bvh_nodes[node_idx].B_min;
		int axis = 0;

		if (extent[1] > extent[axis]){
			axis = 1;
		}
		if(extent[2] > extent[axis]){
			axis = 2;
		}
		double mid = (bvh_nodes[node_idx].B_min[axis] + bvh_nodes[node_idx].B_max[axis])/2;

		// partition triangles around the midpoint on chosen axis
		int split = start;
		for (int i = start; i < end; i++) {
			// use centroid of triangle to decide which side it goes on
			double centroid = (vertices[indices[i].vtx[0]][axis]
							+ vertices[indices[i].vtx[1]][axis]
							+ vertices[indices[i].vtx[2]][axis]) / 3.0;
			if (centroid < mid) {
				std::swap(indices[i], indices[split]);
				split++;
			}
		}

		//if it is a bad partion, if they are all on one side we split it in the middle
		if(split == start || split == end){
			split = (start + end)/2;
		}

		//allocate left and right children
		int left_idx  = bvh_nodes.size(); bvh_nodes.push_back(BVHNode());
		int right_idx = bvh_nodes.size(); bvh_nodes.push_back(BVHNode());
		bvh_nodes[node_idx].left  = left_idx;
		bvh_nodes[node_idx].right = right_idx;

		build_bvh(left_idx,  start, split); //recurse on the left child
		build_bvh(right_idx, split, end); //recurse on the right chid
	}

	void build_bvh() {
		bvh_nodes.push_back(BVHNode()); // root is node 0
		build_bvh(0, 0, indices.size()); //here we start the recursion
	}
	
	// first scale and then translate the current object
	void scale_translate(double s, const Vector& t) {
		for (int i = 0; i < vertices.size(); i++) {
			vertices[i] = vertices[i] * s + t;
		}
	}

	// read an .obj file
	void readOBJ(const char* obj) {
		std::ifstream f(obj);
		if (!f) return;

		std::map<std::string, int> mtls;
		int curGroup = -1, maxGroup = -1;

		// OBJ indices are 1-based and can be negative (relative), this normalizes them
		auto resolveIdx = [](int i, int size) {
			return i < 0 ? size + i : i - 1;
		};

		auto setFaceVerts = [&](TriangleIndices& t, int i0, int i1, int i2) {
			t.vtx[0] = resolveIdx(i0, vertices.size());
			t.vtx[1] = resolveIdx(i1, vertices.size());
			t.vtx[2] = resolveIdx(i2, vertices.size());
		};
		auto setFaceUVs = [&](TriangleIndices& t, int j0, int j1, int j2) {
			t.uv[0] = resolveIdx(j0, uvs.size());
			t.uv[1] = resolveIdx(j1, uvs.size());
			t.uv[2] = resolveIdx(j2, uvs.size());
		};
		auto setFaceNormals = [&](TriangleIndices& t, int k0, int k1, int k2) {
			t.n[0] = resolveIdx(k0, normals.size());
			t.n[1] = resolveIdx(k1, normals.size());
			t.n[2] = resolveIdx(k2, normals.size());
		};

		std::string line;
		while (std::getline(f, line)) {
			// Trim trailing whitespace
			line.erase(line.find_last_not_of(" \r\t\n") + 1);
			if (line.empty()) continue;

			const char* s = line.c_str();

			if (line.rfind("usemtl ", 0) == 0) {
				std::string matname = line.substr(7);
				auto result = mtls.emplace(matname, maxGroup + 1);
				if (result.second) {
					curGroup = ++maxGroup;
				} else {
					curGroup = result.first->second;
				}
			} else if (line.rfind("vn ", 0) == 0) {
				Vector v;
				sscanf(s, "vn %lf %lf %lf", &v[0], &v[1], &v[2]);
				normals.push_back(v);
			} else if (line.rfind("vt ", 0) == 0) {
				Vector v;
				sscanf(s, "vt %lf %lf", &v[0], &v[1]);
				uvs.push_back(v);
			} else if (line.rfind("v ", 0) == 0) {
				Vector pos, col;
				if (sscanf(s, "v %lf %lf %lf %lf %lf %lf", &pos[0], &pos[1], &pos[2], &col[0], &col[1], &col[2]) == 6) {
					for (int i = 0; i < 3; i++) col[i] = std::min(1.0, std::max(0.0, col[i]));
					vertexcolors.push_back(col);
				} else {
					sscanf(s, "v %lf %lf %lf", &pos[0], &pos[1], &pos[2]);
				}
				vertices.push_back(pos);
			}
			else if (line[0] == 'f') {
				int i[4], j[4], k[4], offset, nn;
				const char* cur = s + 1;
				TriangleIndices t;
				t.group = curGroup;

				// Try each face format: v/vt/vn, v/vt, v//vn, v
				if ((nn = sscanf(cur, "%d/%d/%d %d/%d/%d %d/%d/%d%n", &i[0], &j[0], &k[0], &i[1], &j[1], &k[1], &i[2], &j[2], &k[2], &offset)) == 9) {
					setFaceVerts(t, i[0], i[1], i[2]); 
					setFaceUVs(t, j[0], j[1], j[2]); 
					setFaceNormals(t, k[0], k[1], k[2]);
				} else if ((nn = sscanf(cur, "%d/%d %d/%d %d/%d%n", &i[0], &j[0], &i[1], &j[1], &i[2], &j[2], &offset)) == 6) {
					setFaceVerts(t, i[0], i[1], i[2]); 
					setFaceUVs(t, j[0], j[1], j[2]);
				} else if ((nn = sscanf(cur, "%d//%d %d//%d %d//%d%n", &i[0], &k[0], &i[1], &k[1], &i[2], &k[2], &offset)) == 6) {
					setFaceVerts(t, i[0], i[1], i[2]); 
					setFaceNormals(t, k[0], k[1], k[2]);
				} else if ((nn = sscanf(cur, "%d %d %d%n", &i[0], &i[1], &i[2], &offset)) == 3) {
					setFaceVerts(t, i[0], i[1], i[2]);
				}
				else continue;

				indices.push_back(t);
				cur += offset;

				// Fan triangulation for polygon faces (4+ vertices)
				while (*cur && *cur != '\n') {
					TriangleIndices t2;
					t2.group = curGroup;
					if ((nn = sscanf(cur, " %d/%d/%d%n", &i[3], &j[3], &k[3], &offset)) == 3) {
						setFaceVerts(t2, i[0], i[2], i[3]); 
						setFaceUVs(t2, j[0], j[2], j[3]); 
						setFaceNormals(t2, k[0], k[2], k[3]);
					} else if ((nn = sscanf(cur, " %d/%d%n", &i[3], &j[3], &offset)) == 2) {
						setFaceVerts(t2, i[0], i[2], i[3]); 
						setFaceUVs(t2, j[0], j[2], j[3]);
					} else if ((nn = sscanf(cur, " %d//%d%n", &i[3], &k[3], &offset)) == 2) {
						setFaceVerts(t2, i[0], i[2], i[3]); 
						setFaceNormals(t2, k[0], k[2], k[3]);
					} else if ((nn = sscanf(cur, " %d%n", &i[3], &offset)) == 1) {
						setFaceVerts(t2, i[0], i[2], i[3]);
					} else { 
						cur++; 
						continue; 
					}

					indices.push_back(t2);
					cur += offset;
					i[2] = i[3]; j[2] = j[3]; k[2] = k[3];
				}
			}
		}
	}
	
	//helper function
	bool ray_hits_box(const Vector& B_min, const Vector& B_max, const Ray& ray, double t_min) const {
		//also pass the current t_min i was goig to check it later on anyways so i just did it here first too
		double t_x0 = (B_min[0] - ray.O[0]) / ray.u[0];
		double t_x1 = (B_max[0] - ray.O[0]) / ray.u[0];
		double t_xmin = std::min(t_x0, t_x1);
		double t_xmax = std::max(t_x0, t_x1);

		double t_y0 = (B_min[1] - ray.O[1]) / ray.u[1];
		double t_y1 = (B_max[1] - ray.O[1]) / ray.u[1];
		double t_ymin = std::min(t_y0, t_y1);
		double t_ymax = std::max(t_y0, t_y1);

		double t_z0 = (B_min[2] - ray.O[2]) / ray.u[2];
		double t_z1 = (B_max[2] - ray.O[2]) / ray.u[2];
		double t_zmin = std::min(t_z0, t_z1);
		double t_zmax = std::max(t_z0, t_z1);

		double t_enter = std::max(t_xmin, std::max(t_ymin, t_zmin));
		double t_exit  = std::min(t_xmax, std::min(t_ymax, t_zmax));

		return t_enter < t_exit && t_exit > 0 && t_enter < t_min;
	}


	//this was my code for lab 3 before we used the BVH nodes
	// // TODO ray-mesh intersection (labs 3 and 4)
	// bool intersect(const Ray& ray, Vector& P, double& t, Vector& N) const {
		
	// 	// lab 3 : for each triangle, compute the ray-triangle intersection with Moller-Trumbore algorithm
	// 	// lab 3 : once done, speed it up by first checking against the mesh bounding box
	// 	double t_x0 = (B_min[0] - ray.O[0])/ray.u[0];
	// 	double t_x1 = (B_max[0] - ray.O[0])/ray.u[0];
	// 	double t_x_min = std::min(t_x0, t_x1);
	// 	double t_x_max = std::max(t_x0, t_x1);

	// 	double t_y0 = (B_min[1] - ray.O[1])/ray.u[1];
	// 	double t_y1 = (B_max[1] - ray.O[1])/ray.u[1];
	// 	double t_y_min = std::min(t_y0, t_y1);
	// 	double t_y_max = std::max(t_y0, t_y1);

	// 	double t_z0 = (B_min[2] - ray.O[2])/ray.u[2];
	// 	double t_z1 = (B_max[2] - ray.O[2])/ray.u[2];
	// 	double t_z_min = std::min(t_z0, t_z1);
	// 	double t_z_max = std::max(t_z0, t_z1);

	// 	if(std::max(t_x_min, std::max(t_y_min, t_z_min)) > std::min(t_x_max, std::min(t_y_max, t_z_max))){
	// 		return false;
	// 	}

	// 	double t_min = std::numeric_limits<double>::max();
	// 	double t_temp;
	// 	Vector N_best;
	// 	bool intersected = false;
	// 	for(const TriangleIndices& triangle: this->indices){
	// 		Vector A = vertices[triangle.vtx[0]];
	// 		Vector B = vertices[triangle.vtx[1]];
	// 		Vector C = vertices[triangle.vtx[2]];
	// 		Vector e1 = B - A;
	// 		Vector e2 = C - A;
	// 		Vector N_temp = cross(e1, e2);
	// 		double den = dot(ray.u, N_temp);
	// 		t_temp = dot(A-ray.O, N_temp)/den;
	// 		if(t_temp > 0 && t_temp < t_min){
	// 			double beta_num = dot(e2, cross((A - ray.O), ray.u));
	// 			double gamma_num = -dot(e1, cross(A-ray.O, ray.u));
	// 			double beta = beta_num/den;
	// 			double gamma = gamma_num/den;
	// 			double alpha = 1 - beta - gamma;
	// 			//need to check that alpha is positive
	// 			if(alpha >= 0 && gamma >= 0 && beta >= 0){
	// 				N_best = N_temp;
	// 				t_min = t_temp;
	// 				intersected = true;
	// 			}
	// 		}
	// 	}

	// 	t = t_min;
	// 	P = ray.O + t_min*ray.u;
	// 	N_best.normalize(); //normalize before changing the referenced normal
	// 	N = N_best;

	// 	// lab 4 : recursively apply the bounding-box test from a BVH datastructure
	// 	return intersected;
	// }

	bool intersect(const Ray& ray, Vector& P, double& t, Vector& N) const {
		double t_min = std::numeric_limits<double>::max();
		Vector N_best;
		bool intersected = false;

		// usig a stack to iterate over the BVH nodes
		std::vector<int> stack;
		stack.push_back(0); // start at root
		

		//ended up doing iteratively, i was getting stuck with bugs when doing it recursively
		while (!stack.empty()) {
			int node_idx = stack.back();
			stack.pop_back();
			const BVHNode& node = bvh_nodes[node_idx];

			if (!ray_hits_box(node.B_min, node.B_max, ray, t_min)) continue; //call the helper function that checks if the ray hits the box

			if (node.left == -1) {
				// we reached a leaf node
				for (int i = node.start; i < node.end; i++) {
					const TriangleIndices& triangle = indices[i];
					Vector A = vertices[triangle.vtx[0]];
					Vector B = vertices[triangle.vtx[1]];
					Vector C = vertices[triangle.vtx[2]];
					Vector e1 = B - A;
					Vector e2 = C - A;
					Vector N_temp = cross(e1, e2);
					double den = dot(ray.u, N_temp);
					double t_temp = dot(A - ray.O, N_temp) / den;
					if (t_temp > 0 && t_temp < t_min) {
						double beta  =  dot(e2, cross(A - ray.O, ray.u)) / den;
						double gamma = -dot(e1, cross(A - ray.O, ray.u)) / den;
						double alpha = 1 - beta - gamma;
						if (alpha >= 0 && beta >= 0 && gamma >= 0) {
							N_best = alpha*normals[triangle.n[0]] + beta*normals[triangle.n[1]] + gamma*normals[triangle.n[2]];
							N_best.normalize();
							t_min = t_temp;
							intersected = true;
						}
					}
				}
			} else {
				//not a leaf node so it has children so we push them onto the stack
				stack.push_back(node.left);
				stack.push_back(node.right);
			}
		}

		if (intersected) {
			t = t_min;
			P = ray.O + t_min * ray.u;
			N_best.normalize();
			N = N_best;
		}
		return intersected;
	}


	std::vector<TriangleIndices> indices;
	std::vector<Vector> vertices;
	std::vector<Vector> normals;
	std::vector<Vector> uvs;
	std::vector<Vector> vertexcolors;
	Vector B_min;
	Vector B_max;
	std::vector<BVHNode> bvh_nodes;
};



class Scene {
public:
	Scene() {};
	void addObject(const Object* obj) {
		objects.push_back(obj);
	}

	// returns true iif there is an intersection between the ray and any object in the scene
    // if there is an intersection, also computes the point of the *nearest* intersection P, 
    // t>=0 the distance between the ray origin and P (i.e., the parameter along the ray)
    // and the unit normal N. 
	// Also returns the index of the object within the std::vector objects in object_id
	bool intersect(const Ray& ray, Vector& P, double& t, Vector& N, int &object_id) const  {

		// TODO (lab 1): iterate through the objects and check the intersections with all of them, 
		// and keep the closest intersection, i.e., the one if smallest positive value of t

		double t_min = std::numeric_limits<double>::max();
		double t_temp {0.0}; //this is the t that we pass as a parameter
		Vector P_best; //this is the P we will store
		Vector P_temp; //initialise the temp
		Vector N_best;
		Vector N_temp;
		int counter {0};
		int min_counter = 0;

		bool flag = false;
		for(int i=0; i<static_cast<int>(objects.size()); ++i){
			//iterating over the objects
			bool is_intersect = objects[i]->intersect(ray, P_temp, t_temp, N_temp);
			if(is_intersect && t_temp < t_min){
				flag = true;
				t_min = t_temp;
				P_best = P_temp;
				N_best = N_temp;
				min_counter = i;
			}

		}

		//give the values back to the references that we computed
		t = t_min;
		P = P_best;
		N = N_best;
		object_id = min_counter;
		return flag;
	}


	Vector getColor(const Ray& ray, int recursion_depth) {

		if (recursion_depth >= max_light_bounce) return Vector(0, 0, 0);

		Vector P, N;
		double t;
		int object_id;
		if (intersect(ray, P, t, N, object_id)) {
			double eps = 1e-3;

			if (objects[object_id]->mirror) {
				Ray r = Ray(P + eps*N, ray.u - 2*dot(ray.u, N)*N);
				return this->getColor(r, recursion_depth+1);
			}

			if (objects[object_id]->transparent) {
				double n1, n2;
				if(dot(ray.u, N) > 0){
					n1 = 1.5;
					n2 = 1.0;
					N = -1.0 * N;
				}else{
					n1 = 1.0;
					n2 = 1.5;
				}
				double t_N = 1 - (n1/n2)*(n1/n2)*(1 - dot(ray.u, N)*dot(ray.u, N));
				if(t_N < 0){
					Ray r = Ray(P + eps*N, ray.u - 2*dot(ray.u, N)*N);
					return this->getColor(r, recursion_depth+1);
				}
				t_N = -sqrt(t_N);
				Vector T = (n1/n2)*(ray.u - dot(ray.u, N)*N);
				Vector refraction_dir = t_N*N + T;
				return this->getColor(Ray(P - eps*N, refraction_dir), recursion_depth+1);
			}

			Vector new_P = P + eps*N;
			Vector to_light_source_vector = this->light_position - new_P;
			to_light_source_vector.normalize();

			Vector ret_colour(0, 0, 0); //initialize to black 

			Vector P_intersect;
			double t_intersect;
			Vector N_intersect;
			int n_object_id;
			if(this->intersect(Ray(new_P, to_light_source_vector), P_intersect, t_intersect, N_intersect, n_object_id)){
				if((new_P - P_intersect).norm2() > (new_P - this->light_position).norm2()){
					// not in shadow
					double attenuation = this->light_intensity/(4*M_PI*(this->light_position - P).norm2());
					Vector material = objects[object_id]->albedo/M_PI;
					double angle = std::max(0.0, dot(N, (this->light_position - P)/((this->light_position-P).norm())));
					ret_colour = attenuation*material*angle;
				}
			}else{
				double attenuation = this->light_intensity/(4*M_PI*(this->light_position - P).norm2());
				Vector material = objects[object_id]->albedo/M_PI;
				double angle = std::max(0.0, dot(N, (this->light_position - P)/((this->light_position-P).norm())));
				ret_colour = attenuation*material*angle;
			}

			double abs_min = std::min(std::abs(N.data[0]), std::min(std::abs(N.data[1]), std::abs(N.data[2])));

			double r1 = uniform(engine[0]);
			double r2 = uniform(engine[0]);
			double x = cos(2*M_PI*r1)*sqrt(1-r2);
			double y = sin(2*M_PI*r1)*sqrt(1-r2);
			double z = sqrt(r2);

			Vector T1;
			if(abs_min == std::abs(N.data[0])){
				T1 = Vector(0, -N.data[2], N.data[1]);
			}else if(abs_min == std::abs(N.data[1])){
				T1 = Vector(-N.data[2], 0, N.data[0]);
			}else{
				T1 = Vector(-N.data[1], N.data[0], 0);
			}

			T1.normalize();
			Vector T2 = cross(T1, N);
			Vector random_direction = x*T1 + y*T2 + z*N;
			random_direction.normalize();

			Ray random_ray = Ray(P + eps*N, random_direction);
			ret_colour = ret_colour + objects[object_id]->albedo * this->getColor(random_ray, recursion_depth+1);
			return ret_colour;
		}

		return Vector(0, 0, 0);
	}

	std::vector<const Object*> objects;

	Vector camera_center, light_position;
	double fov, gamma, light_intensity;
	int max_light_bounce;
};


int main() {
	int W = 512;
	int H = 512;

	for (int i = 0; i<32; i++) {
		engine[i].seed(i);
	}

	Sphere sphere_1(Vector(-14.5, 3, 0), 1.5, Vector(0.6, 0.2, 0.9));
	Sphere sphere_2(Vector(-16, 0, 0), 1.5, Vector(0.6, 0.2, 0.9));
	Sphere sphere_3(Vector(-17, -3, 0), 1.5, Vector(0.6, 0.2, 0.9), true);
	Sphere sphere_4(Vector(-17.9, -6, -1), 1.5, Vector(0.6, 0.2, 0.9));
	Sphere wall_left(Vector(-1000, 0, 0), 940, Vector(0.5, 0.8, 0.1));
	Sphere wall_right(Vector(1000, 0, 0), 940, Vector(0.9, 0.2, 0.3));
	Sphere wall_front(Vector(0, 0, -1000), 940, Vector(0.1, 0.6, 0.7));
	Sphere wall_behind(Vector(0, 0, 1000), 940, Vector(0.8, 0.2, 0.9));
	Sphere ceiling(Vector(0, 1000, 0), 940, Vector(0.3, 0.5, 0.3));
	Sphere floor(Vector(0, -1000, 0), 990, Vector(0.6, 0.5, 0.7));

	Scene scene;
	TriangleMesh cat(Vector(0.2,0.4,0.6));
	cat.readOBJ("./cat.obj");
	Vector scale(5, -10, 0);
	cat.scale_translate(0.6, scale);
	cat.find_bounds();
	cat.build_bvh();
	scene.camera_center = Vector(0, 0, 55);
	scene.light_position = Vector(-10,20,40);
	scene.light_intensity = 1.5E7;
	scene.fov = 60 * M_PI / 180.;
	scene.gamma = 2.2;    // TODO (lab 1) : play with gamma ; typically, gamma = 2.2
	scene.max_light_bounce = 5;

	// scene.addObject(&center_sphere);
	scene.addObject(&sphere_1);
	scene.addObject(&sphere_2);
	scene.addObject(&sphere_3);
	scene.addObject(&sphere_4);

	// scene.addObject(&right_sphere);

	
	scene.addObject(&wall_left);
	scene.addObject(&wall_right);
	scene.addObject(&wall_front);
	scene.addObject(&wall_behind);
	scene.addObject(&ceiling);
	scene.addObject(&floor);
	scene.addObject(&cat);

	std::vector<unsigned char> image(W * H * 3, 0);

#pragma omp parallel for schedule(dynamic, 1)
	for (int i = 0; i < H; i++) {
		for (int j = 0; j < W; j++) {
			Vector color;
			// TODO (lab 1) : correct ray_direction so that it goes through each pixel (j, i)
			// Vector ray_direction(j - W/2 + 0.5, H/2 - i -0.5, -W/(2*tan(scene.fov/2)));
			// ray_direction.normalize();

			// Ray ray(scene.camera_center, ray_direction);
			
			// TODO (lab 2) : add Monte Carlo / averaging of random ray contributions here
			// TODO (lab 2) : add antialiasing by altering the ray_direction here
			// TODO (lab 2) : add depth of field effect by altering the ray origin (and direction) here

			size_t N = 5;
			double sigma = 0.5; // tweak this
			Vector colour_sum(0,0,0);
			for(size_t k=0; k<N; ++k){
				double r1 = uniform(engine[0]);
				double r2 = uniform(engine[0]);
				double dx = sqrt(-2*log(r1)) * cos(2*M_PI*r2);
				double dy = sqrt(-2*log(r1)) * sin(2*M_PI*r2);
				Vector ray_dir(j - W/2 + 0.5 + sigma*dx, H/2 -i -0.5 + sigma*dy, -W/(2*tan(scene.fov/2)));
				ray_dir.normalize();
				Ray new_ray(scene.camera_center, ray_dir);
				colour_sum = colour_sum + scene.getColor(new_ray, 0);
			}
			colour_sum = colour_sum/N;

			image[(i * W + j) * 3 + 0] = std::min(255., std::max(0., 255. * std::pow(colour_sum[0] / 255., 1. / scene.gamma)));
			image[(i * W + j) * 3 + 1] = std::min(255., std::max(0., 255. * std::pow(colour_sum[1] / 255., 1. / scene.gamma)));
			image[(i * W + j) * 3 + 2] = std::min(255., std::max(0., 255. * std::pow(colour_sum[2] / 255., 1. / scene.gamma)));
		}
	}
	stbi_write_png("image.png", W, H, 3, &image[0], 0);

	return 0;
}