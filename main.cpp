#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>
#include <cmath>
#include <random>
#include <iostream>
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

// Class only used in labs 3 and 4 
class TriangleMesh : public Object {
public:
	TriangleMesh(const Vector& albedo, bool mirror = false, bool transparent = false) : ::Object(albedo, mirror, transparent) {};

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
	

	// TODO ray-mesh intersection (labs 3 and 4)
	bool intersect(const Ray& ray, Vector& P, double& t, Vector& N) const {
		
		// lab 3 : for each triangle, compute the ray-triangle intersection with Moller-Trumbore algorithm
		// lab 3 : once done, speed it up by first checking against the mesh bounding box
		Vector B_min(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
		Vector B_max(std::numeric_limits<double>::min(), std::numeric_limits<double>::min(), std::numeric_limits<double>::min());
		for(const auto& vertex: vertices){
			//computing the min
			if(vertex[0] < B_min[0]) B_min[0] = vertex[0];
			if(vertex[1] < B_min[0]) B_min[1] = vertex[1];
			if(vertex[2] < B_min[0]) B_min[2] = vertex[2];
			//computing the max
			if(vertex[0] > B_max[0]) B_max[0] = vertex[0];
			if(vertex[1] > B_max[1]) B_max[1] = vertex[1];
			if(vertex[2] > B_max[2]) B_max[2] = vertex[2];
		}

		double t_x_min = (B_min[0] - ray.O[0])/ray.u[0];
		double t_x_max = (B_max[0] - ray.O[0])/ray.u[0];

		double t_y_min = (B_min[1] - ray.O[1])/ray.u[1];
		double t_y_max = (B_max[1] - ray.O[1])/ray.u[1];

		double t_z_min = (B_min[2] - ray.O[2])/ray.u[2];
		double t_z_max = (B_max[2] - ray.O[2])/ray.u[2];

		if(std::max(t_x_min, std::max(t_y_min, t_z_min)) >= std::min(t_x_max, std::min(t_y_max, t_z_max))){
			return false;
		}

		double t_min = std::numeric_limits<double>::max();
		double t_temp;
		Vector N_best;
		bool intersected = false;
		for(const TriangleIndices& triangle: this->indices){
			Vector A = vertices[triangle.vtx[0]];
			Vector B = vertices[triangle.vtx[1]];
			Vector C = vertices[triangle.vtx[2]];
			Vector e1 = B - A;
			Vector e2 = C - A;
			Vector N_temp = cross(e1, e2);
			double den = dot(ray.u, N_temp);
			double beta_num = dot(e2, cross((A - ray.O), ray.u));
			double gamma_num = -dot(e1, cross(A-ray.O, ray.u));
			double beta = beta_num/den;
			double gamma = gamma_num/den;
			double alpha = 1 - beta - gamma;
			//need to check that alpha is positive
			t_temp = dot(A-ray.O, N)/dot(ray.u, N);
			if(alpha >= 0 && gamma >= 0 && beta >= 0 && t_temp < t_min){
				N_best = N_temp;
				t_min = t_temp;
			}
		}

		t = t_min;
		P = ray.O + t_min*ray.u;
		N_best.normalize(); //normalize before changing the referenced normal
		N = N_best;

		// lab 4 : recursively apply the bounding-box test from a BVH datastructure
		return intersected;
	}


	std::vector<TriangleIndices> indices;
	std::vector<Vector> vertices;
	std::vector<Vector> normals;
	std::vector<Vector> uvs;
	std::vector<Vector> vertexcolors;
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


	// return the radiance (color) along ray
	Vector getColor(const Ray& ray, int recursion_depth) {

		if (recursion_depth >= max_light_bounce) return Vector(0, 0, 0);

		// TODO (lab 1) : if intersect with ray, use the returned information to compute the color ; otherwise black 
		// in lab 1, the color only includes direct lighting with shadows

		Vector P, N;
		double t;
		int object_id;
		if (intersect(ray, P, t, N, object_id)) {
			double eps = 1e-3; // a small epsilon

			if (objects[object_id]->mirror) {
				//compute the ray that goes off the mirror
				Ray r = Ray(P + eps*N, ray.u - 2*dot(ray.u, N)*N);
				return this->getColor(r, recursion_depth+1);
				// return getColor in the reflected direction, with recursion_depth+1 (recursively)
			} // else

			if (objects[object_id]->transparent) { // optional
				double n1, n2;
				if(dot(ray.u, N) > 0){
					n1 = 1.5;
					n2 = 1.0;
					N = -1.0 * N;
				}else{ //make sure to swap the mediums if <ray.u, N> > 0
					n1 = 1.0;
					n2 = 1.5;
				}

				double t_N = 1 - (n1/n2)*(n1/n2)*(1 - dot(ray.u, N)*dot(ray.u, N));
				if(t_N < 0){
					//total reflection, behaves like a mirror
					Ray r = Ray(P + eps*N, ray.u - 2*dot(ray.u, N)*N);
					return this->getColor(r, recursion_depth +1);
				}
				t_N = -sqrt(t_N); // square root a positive value
				Vector T = (n1/n2)*(ray.u - dot(ray.u, N)*N);
				Vector refraction_dir = t_N*N + T;
				return this->getColor(Ray(P - eps*N, refraction_dir), recursion_depth+1);

				// return getColor in the refraction direction, with recursion_depth+1 (recursively)
			} // else

			// test if there is a shadow by sending a new ray
			// if there is no shadow, compute the formula with dot products etc.
			Vector new_P = P + eps*N; //shift the point upwards
			Vector to_light_source_vector = this->light_position - new_P;
			to_light_source_vector.normalize();

			Vector ret_colour;
			Vector P_intersect;
			double t_intersect;
			Vector N_intersect;
			int n_object_id;
			if(this->intersect(Ray(new_P, to_light_source_vector), P_intersect, t_intersect, N_intersect, n_object_id)){
				//have found an intersection
				//check if the distance from this intersection point is smaller than the distance to the light source
				if((new_P - P_intersect).norm2() > (new_P - this->light_position).norm2()){
					//there is no shadow so return the colour

					double attenuation = this->light_intensity/(4*M_PI*(this->light_position - P).norm2());
					Vector material = objects[object_id]->albedo/M_PI;
					double angle = std::max(0.0, dot(N, (this->light_position - P)/((this->light_position-P).norm())));
					Vector colour = attenuation*material*angle;
					ret_colour = colour;
				}
				// TODO (lab 2) : add indirect lighting component with a recursive call
				double r1 = uniform(engine[0]);
				double r2 = uniform(engine[0]);
				double x = cos(2*M_PI*r1)*sqrt(1-r2); 
				double y = sin(2*M_PI*r1)*sqrt(1-r2);
				double z = sqrt(r2);
				double abs_min = std::min(std::min(N.data[0], N.data[1]), std::min(N.data[1], N.data[2]));

				Vector T1;
				if(abs_min == abs(N.data[0])){
					T1 = Vector(0, N.data[2], -N.data[1]);

				}else if(abs_min == abs(N.data[1])){
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

	Sphere center_sphere(Vector(0, 0, 5), 5., Vector(0.8, 0.8, 0.8), true, false);
	Sphere left_sphere(Vector(-12, 0, 0), 5., Vector(0.6, 0.6, 0.8));
	Sphere right_sphere(Vector(12,0,10), 5., Vector(0.8, 0.6, 0.4));
	Sphere wall_left(Vector(-1000, 0, 0), 940, Vector(0.5, 0.8, 0.1));
	Sphere wall_right(Vector(1000, 0, 0), 940, Vector(0.9, 0.2, 0.3));
	Sphere wall_front(Vector(0, 0, -1000), 940, Vector(0.1, 0.6, 0.7));
	Sphere wall_behind(Vector(0, 0, 1000), 940, Vector(0.8, 0.2, 0.9));
	Sphere ceiling(Vector(0, 1000, 0), 940, Vector(0.3, 0.5, 0.3));
	Sphere floor(Vector(0, -1000, 0), 990, Vector(0.6, 0.5, 0.7));

	Scene scene;
	TriangleMesh cat(Vector(0.6,0.6,0.6));
	cat.readOBJ("./cat.obj");
	Vector scale(0, -10, 0);
	cat.scale_translate(0.6, scale);
	scene.camera_center = Vector(0, 0, 55);
	scene.light_position = Vector(-10,20,40);
	scene.light_intensity = 3E7;
	scene.fov = 60 * M_PI / 180.;
	scene.gamma = 2.2;    // TODO (lab 1) : play with gamma ; typically, gamma = 2.2
	scene.max_light_bounce = 5;

	// scene.addObject(&center_sphere);
	// scene.addObject(&left_sphere);
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