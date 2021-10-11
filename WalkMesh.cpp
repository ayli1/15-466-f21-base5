#include "WalkMesh.hpp"

#include "read_write_chunk.hpp"

#include <glm/gtx/norm.hpp>
#include <glm/gtx/string_cast.hpp>

#include <iostream>
#include <fstream>
#include <algorithm>
#include <string>

WalkMesh::WalkMesh(std::vector< glm::vec3 > const &vertices_, std::vector< glm::vec3 > const &normals_, std::vector< glm::uvec3 > const &triangles_)
	: vertices(vertices_), normals(normals_), triangles(triangles_) {

	//construct next_vertex map (maps each edge to the next vertex in the triangle):
	next_vertex.reserve(triangles.size()*3);
	auto do_next = [this](uint32_t a, uint32_t b, uint32_t c) {
		auto ret = next_vertex.insert(std::make_pair(glm::uvec2(a,b), c));
		assert(ret.second);
	};
	for (auto const &tri : triangles) {
		do_next(tri.x, tri.y, tri.z);
		do_next(tri.y, tri.z, tri.x);
		do_next(tri.z, tri.x, tri.y);
	}

	//DEBUG: are vertex normals consistent with geometric normals?
	for (auto const &tri : triangles) {
		glm::vec3 const &a = vertices[tri.x];
		glm::vec3 const &b = vertices[tri.y];
		glm::vec3 const &c = vertices[tri.z];
		glm::vec3 out = glm::normalize(glm::cross(b-a, c-a));

		float da = glm::dot(out, normals[tri.x]);
		float db = glm::dot(out, normals[tri.y]);
		float dc = glm::dot(out, normals[tri.z]);

		assert(da > 0.1f && db > 0.1f && dc > 0.1f);
	}
}

//project pt to the plane of triangle a,b,c and return the barycentric weights of the projected point:
glm::vec3 barycentric_weights(glm::vec3 const &a, glm::vec3 const &b, glm::vec3 const &c, glm::vec3 const &pt) {
	//TODO: implement!
	//return glm::vec3(0.25f, 0.25f, 0.5f);

	//compute barycentric weights here!
	/*float w_x, w_y, w_z;

	// Compute plane normal using two edges of the triangle
	glm::vec3 h = glm::normalize(glm::cross(c - a, b - c));

	// Project to the triangle
	// Referenced from class discussion
	glm::vec3 pp = (pt - c);
	pp -= glm::dot(h, pp) * h;

	// Calculate barycentric weights using projected point and triangle
	// Equations from: https://en.wikipedia.org/wiki/Barycentric_coordinate_system
	w_x = ((b.y - c.y) * (pp.x - c.x) + (c.x - b.x) * (pp.y - b.y)) / ((b.y - c.y) * (a.x - c.x) + (c.x - b.x) * (a.y - c.y));
	w_y = ((c.y - a.y) * (pp.x - c.x) + (a.x - c.x) * (pp.y - c.y)) / ((b.y - c.y) * (a.x - c.x) + (c.x - b.x) * (a.y - c.y));
	w_z = 1.0f - w_x - w_y;

	return glm::vec3(w_x, w_y, w_z);*/

	// From class exercise:
	//compute plane normal:
	glm::vec3 h = glm::cross(c - a, b - c);

	//compute projected, signed areas of sub-triangles formed with pt:
	float A = glm::dot(glm::cross(b - pt, c - pt), h);
	float B = glm::dot(glm::cross(c - pt, a - pt), h);
	float C = glm::dot(glm::cross(a - pt, b - pt), h);
	float S = A + B + C;

	return glm::vec3(A, B, C) / S;
}

WalkPoint WalkMesh::nearest_walk_point(glm::vec3 const &world_point) const {
	assert(!triangles.empty() && "Cannot start on an empty walkmesh");

	WalkPoint closest;
	float closest_dis2 = std::numeric_limits< float >::infinity();

	for (auto const &tri : triangles) {
		//find closest point on triangle:

		glm::vec3 const &a = vertices[tri.x];
		glm::vec3 const &b = vertices[tri.y];
		glm::vec3 const &c = vertices[tri.z];

		//get barycentric coordinates of closest point in the plane of (a,b,c):
		glm::vec3 coords = barycentric_weights(a,b,c, world_point);

		//is that point inside the triangle?
		if (coords.x >= 0.0f && coords.y >= 0.0f && coords.z >= 0.0f) {
			//yes, point is inside triangle.
			float dis2 = glm::length2(world_point - to_world_point(WalkPoint(tri, coords)));
			if (dis2 < closest_dis2) {
				closest_dis2 = dis2;
				closest.indices = tri;
				closest.weights = coords;
			}
		} else {
			//check triangle vertices and edges:
			auto check_edge = [&world_point, &closest, &closest_dis2, this](uint32_t ai, uint32_t bi, uint32_t ci) {
				glm::vec3 const &a = vertices[ai];
				glm::vec3 const &b = vertices[bi];

				//find closest point on line segment ab:
				float along = glm::dot(world_point-a, b-a);
				float max = glm::dot(b-a, b-a);
				glm::vec3 pt;
				glm::vec3 coords;
				if (along < 0.0f) {
					pt = a;
					coords = glm::vec3(1.0f, 0.0f, 0.0f);
				} else if (along > max) {
					pt = b;
					coords = glm::vec3(0.0f, 1.0f, 0.0f);
				} else {
					float amt = along / max;
					pt = glm::mix(a, b, amt);
					coords = glm::vec3(1.0f - amt, amt, 0.0f);
				}

				float dis2 = glm::length2(world_point - pt);
				if (dis2 < closest_dis2) {
					closest_dis2 = dis2;
					closest.indices = glm::uvec3(ai, bi, ci);
					closest.weights = coords;
				}
			};
			check_edge(tri.x, tri.y, tri.z);
			check_edge(tri.y, tri.z, tri.x);
			check_edge(tri.z, tri.x, tri.y);
		}
	}
	assert(closest.indices.x < vertices.size());
	assert(closest.indices.y < vertices.size());
	assert(closest.indices.z < vertices.size());
	return closest;
}


void WalkMesh::walk_in_triangle(WalkPoint const &start, glm::vec3 const &step, WalkPoint *end_, float *time_) const {
	/*
	assert(end_);
	auto &end = *end_;

	assert(time_);
	auto &time = *time_;

	glm::vec3 step_coords;
	{ //project 'step' into a barycentric-coordinates direction:
		//TODO
		step_coords = glm::vec3(0.0f);
	}
	
	//if no edge is crossed, event will just be taking the whole step:
	time = 1.0f;
	end = start;

	//figure out which edge (if any) is crossed first.
	// set time and end appropriately.
	//TODO
	*/

	//Remember: our convention is that when a WalkPoint is on an edge,
	// then wp.weights.z == 0.0f (so will likely need to re-order the indices)

	assert(end_);
	auto &end = *end_;
	assert(time_);
	auto &time = *time_;
	
	glm::vec3 const &a = vertices[start.indices.x];
	glm::vec3 const &b = vertices[start.indices.y];
	glm::vec3 const &c = vertices[start.indices.z];

	glm::vec3 step_to = barycentric_weights(a, b, c, to_world_point(start) + step);

	end.weights = step_to;
	end.indices = start.indices;
	time = 1.0f;

	//if no edge is crossed, event will just be taking the whole step:
	if ((end.weights.x >= 0.0f) &&
		(end.weights.y >= 0.0f) &&
		(end.weights.z >= 0.0f)) {
		return;
	}

	//figure out which edge (if any) is crossed first.
	// set time and end appropriately.
	
	glm::vec3 v = step_to - start.weights; 	//transform 'step' into a barycentric velocity on (a,b,c)
	glm::vec3 t = -start.weights / v;

	// Get min time component -- some times may be negative (which means you're moving
	// in opposite direction of the edge)
	for (int i = 0; i < 3; i++) {
		if ((t[i] > 0.0f) && (t[i] < time)) {
			time = t[i];
		}
	}

	std::cout << "time: " << time << std::endl;

	end.weights = start.weights + v * time;

	//Remember: our convention is that when a WalkPoint is on an edge,
	// then wp.weights.z == 0.0f (so will likely need to re-order the indices)
	if (time == t.x) {      // If min time component was x
		end.weights = glm::vec3( end.weights.y, end.weights.z, 0.0f);
		end.indices = glm::uvec3(end.indices.y, end.indices.z, end.indices.x);
	}
	else if (time == t.y) { // If min time component was y
		end.weights = glm::vec3( end.weights.z, end.weights.x, 0.0f);
		end.indices = glm::uvec3(end.indices.z, end.indices.x, end.indices.y);
	}
	else {                  // If min time component was z
		end.weights.z = 0.0f;
	}

	/*
	//TODO: transform 'step' into a barycentric velocity on (a,b,c)
	// More like transforming it into a displacement ??
	glm::vec3 v = barycentric_weights(a, b, c, step) - barycentric_weights(a, b, c, glm::vec3(0.0f));
	
	//TODO: check when/if this velocity pushes start.weights into an edge
	end = start;
	time = 1.0f;
	
	glm::vec3 time_to_edge = glm::vec3(1.0f, 1.0f, 1.0f);
	
	// For each component, if moving away from edge (negative time), 
	// or if it will take more than time = 1.0 to reach edge,
	// time for that component = 1.0
	if (v.x != 0.0f) {
		time_to_edge.x = start.weights.x / -v.x;
		if ((time_to_edge.x <= 0.0f) || (time_to_edge.x > 1.0f)) {
			time_to_edge.x = 1.0f;
		}
	}
	
	if (v.y != 0.0f) {
		time_to_edge.y = start.weights.y / -v.y;
		if ((time_to_edge.y <= 0.0f) || (time_to_edge.y > 1.0f)) {
			time_to_edge.y = 1.0f;
		}
	}
	
	if (v.z != 0.0f) {
		time_to_edge.z = start.weights.z / -v.z;
		if ((time_to_edge.z <= 0.0f) || (time_to_edge.z > 1.0f)) {
			time_to_edge.z = 1.0f;
		}
	}

	std::cout << "walk_in_triangle v: " << glm::to_string(v) << std::endl;
	
	time = std::min({time_to_edge.x, time_to_edge.y, time_to_edge.z});
	end.weights = start.weights + v * time;
	
	// Make sure that if there's a zero weight component, that
	// component is z
	if (end.weights.x == 0.0f) {
		// Shift indices and weights such that z = x, y = z, x = y
		end.indices = glm::vec3(end.indices.y, end.indices.z, 0.0f);
		end.weights = glm::vec3(end.weights.y, end.weights.z, 0.0f);
	} else if (end.weights.y == 0.0f) {
		// Shift indices and weights such that z = y, x = z, y = x
		end.indices = glm::vec3(end.indices.z, end.indices.x, end.indices.y);
		end.weights = glm::vec3(end.weights.z, end.weights.x, end.weights.y);
	}

	if (time < 1.0f) end.weights.z = 0.0f;
	std::cout << "walk_in_triangle end.weights: " << end.weights.z << std::endl;
	std::cout << "time: " << time << std::endl;*/
}

bool WalkMesh::cross_edge(WalkPoint const &start, WalkPoint *end_, glm::quat *rotation_) const {
	/*assert(end_);
	auto &end = *end_;

	assert(rotation_);
	auto &rotation = *rotation_;

	assert(start.weights.z == 0.0f); //*must* be on an edge.
	glm::uvec2 edge = glm::uvec2(start.indices);

	//check if 'edge' is a non-boundary edge:
	if (edge.x == edge.y) { //<-- TODO: use a real check, this is just here so code compiles
		//it is!

		//make 'end' represent the same (world) point, but on triangle (edge.y, edge.x, [other point]):
		//TODO

		//make 'rotation' the rotation that takes (start.indices)'s normal to (end.indices)'s normal:
		//TODO

		return true;
	} else {
		end = start;
		rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
		return false;
	}*/
	std::cout << "cross_edge start.weights: " << start.weights.z << std::endl;
	assert(start.weights.z == 0.0f);
	assert(start.indices.x <= vertices.size() && start.indices.y <= vertices.size() && start.indices.z <= vertices.size());
	assert(end_);
	auto &end = *end_;
	assert(rotation_);
	auto &rotation = *rotation_;

	//TODO: check if edge (start.indices.x, start.indices.y) has a triangle on the other side:
	//  hint: remember 'next_vertex'!
	auto f = next_vertex.find(glm::uvec2(start.indices.y, start.indices.x));
	//TODO: if there is another triangle:
	end = start;
	if (f != next_vertex.end()) {
		//  TODO: set end's weights and indices on that triangle:
		end.weights = glm::vec3(start.weights.y, start.weights.x, start.weights.z);
		end.indices = glm::uvec3(start.indices.y, start.indices.x, f->second);

		//  TODO: compute rotation that takes starting triangle's normal to ending triangle's normal:
		//  hint: look up 'glm::rotation' in the glm/gtx/quaternion.hpp header
		rotation = glm::rotation(to_world_triangle_normal(start), to_world_triangle_normal(end));

		return true; // There was another triangle
	}
	else {
		rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f); //identity quat (wxyz init order)
		return false; // There was no other triangle
	}
}


WalkMeshes::WalkMeshes(std::string const &filename) {
	std::ifstream file(filename, std::ios::binary);

	std::vector< glm::vec3 > vertices;
	read_chunk(file, "p...", &vertices);

	std::vector< glm::vec3 > normals;
	read_chunk(file, "n...", &normals);

	std::vector< glm::uvec3 > triangles;
	read_chunk(file, "tri0", &triangles);

	std::vector< char > names;
	read_chunk(file, "str0", &names);

	struct IndexEntry {
		uint32_t name_begin, name_end;
		uint32_t vertex_begin, vertex_end;
		uint32_t triangle_begin, triangle_end;
	};

	std::vector< IndexEntry > index;
	read_chunk(file, "idxA", &index);

	if (file.peek() != EOF) {
		std::cerr << "WARNING: trailing data in walkmesh file '" << filename << "'" << std::endl;
	}

	//-----------------

	if (vertices.size() != normals.size()) {
		throw std::runtime_error("Mis-matched position and normal sizes in '" + filename + "'");
	}

	for (auto const &e : index) {
		if (!(e.name_begin <= e.name_end && e.name_end <= names.size())) {
			throw std::runtime_error("Invalid name indices in index of '" + filename + "'");
		}
		if (!(e.vertex_begin <= e.vertex_end && e.vertex_end <= vertices.size())) {
			throw std::runtime_error("Invalid vertex indices in index of '" + filename + "'");
		}
		if (!(e.triangle_begin <= e.triangle_end && e.triangle_end <= triangles.size())) {
			throw std::runtime_error("Invalid triangle indices in index of '" + filename + "'");
		}

		//copy vertices/normals:
		std::vector< glm::vec3 > wm_vertices(vertices.begin() + e.vertex_begin, vertices.begin() + e.vertex_end);
		std::vector< glm::vec3 > wm_normals(normals.begin() + e.vertex_begin, normals.begin() + e.vertex_end);

		//remap triangles:
		std::vector< glm::uvec3 > wm_triangles; wm_triangles.reserve(e.triangle_end - e.triangle_begin);
		for (uint32_t ti = e.triangle_begin; ti != e.triangle_end; ++ti) {
			if (!( (e.vertex_begin <= triangles[ti].x && triangles[ti].x < e.vertex_end)
			    && (e.vertex_begin <= triangles[ti].y && triangles[ti].y < e.vertex_end)
			    && (e.vertex_begin <= triangles[ti].z && triangles[ti].z < e.vertex_end) )) {
				throw std::runtime_error("Invalid triangle in '" + filename + "'");
			}
			wm_triangles.emplace_back(
				triangles[ti].x - e.vertex_begin,
				triangles[ti].y - e.vertex_begin,
				triangles[ti].z - e.vertex_begin
			);
		}
		
		std::string name(names.begin() + e.name_begin, names.begin() + e.name_end);

		auto ret = meshes.emplace(name, WalkMesh(wm_vertices, wm_normals, wm_triangles));
		if (!ret.second) {
			throw std::runtime_error("WalkMesh with duplicated name '" + name + "' in '" + filename + "'");
		}

	}
}

WalkMesh const &WalkMeshes::lookup(std::string const &name) const {
	auto f = meshes.find(name);
	if (f == meshes.end()) {
		throw std::runtime_error("WalkMesh with name '" + name + "' not found.");
	}
	return f->second;
}
