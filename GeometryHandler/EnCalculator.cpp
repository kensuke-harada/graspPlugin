#include "EnCalculator.h"

using namespace std;
using namespace cnoid;
using namespace grasp;

void OverlapRegionCalculator::calc(const Triangle* triangle, const Boundary& _boundary, const Vector3& plane_p, const Vector3& plane_n, const Vector3& N) {
	//calculate vertex of projected triangles
	triangle_v.clear();
	for (unsigned int j = 0; j < 3; j++) {
		Vector3 p = intersectionPoint(triangle->ver[j]->pos, N, plane_p, plane_n);
		triangle_v.push_back(p);
	}

	//project the boundary
	boundary_v.clear();
	for (unsigned int i = 0; i < _boundary.size(); i++) {
		boundary_v.push_back(intersectionPoint(_boundary[i], N, plane_p, plane_n));
    removeDuplicatedPoints(boundary_v);
	}

	makeVertexList(triangle, plane_p, plane_n);

	is_overlap = 
		!(in_tri_vids.empty() && bound_vids.empty() && intersect_vids[0].empty() && intersect_vids[1].empty() && intersect_vids[2].empty() );

	if(!is_overlap) return;

	makeBoundaries(true);
	makeBoundaries(false);
}

/**
* Clear boundary data.
*/
void OverlapRegionCalculator::clear() {
	overlap_boundary.clear();
	not_overlap_boundary.clear();
}

/**
* Make vertex list of overlap region boundary .
* @param[in] triangle pointer to a target triangle
* @param[in] plane_p point on the target plane
* @param[in] plane_n normal vector of target plane
*/
void OverlapRegionCalculator::makeVertexList(const Triangle* triangle, const Vector3& plane_p, const Vector3& plane_n) {
	in_tri_vids.clear();
	out_tri_vids.clear();
	bound_vids.clear();
	intersect_vids.clear();
	intersect_pos.clear();

	// triangle vertex inside boundary
  PointVecVec triangles;
  dividePolygon(boundary_v, triangles);
  for (unsigned int j = 0; j < 3; j++) {
    bool is_in_triangle = false;
    for (unsigned int i = 0; i < triangles.size(); i++) {
      if(isInsideTriangle(triangles[i], triangle_v[j])) {
        is_in_triangle = true;
        break;
      }
    }
    if(is_in_triangle){
      in_tri_vids.push_back(j);
    }else{
      out_tri_vids.push_back(j);
    }
  }

	// boundary vertex inside triangle
  for (unsigned int j = 0; j < boundary_v.size(); j++) {
    if (isInsideTriangle(triangle_v, boundary_v[j])) {
      bound_vids.push_back(j);
    }
  }

	// intersection of a triangle and boudnary
	for (unsigned int j = 0; j < 3; j++) {
		vector<double> dist;
		vector<int> intersect_v;
		map<int, Vector3> intersect_p;
		Vector3 Pout;
		for (unsigned int k = 0; k < boundary_v.size(); k++) {
			if (OverlapRegionCalculator::twoSegmentsIntersect(boundary_v[k], boundary_v[(k+1)%boundary_v.size()], triangle_v[j], triangle_v[(j+1)%3], plane_n, Pout)) {
				intersect_v.push_back(k);
				intersect_p[k] = Pout;
				dist.push_back(norm2(Pout - triangle_v[j]));
			}
		}
		sort_by(intersect_v, dist);
		intersect_vids.push_back(intersect_v);
		intersect_pos.push_back(intersect_p);
	}
}

bool OverlapRegionCalculator::twoSegmentsIntersect(const Vector3& P1, const Vector3& P2, const Vector3& P3, const Vector3& P4, const Vector3& n, Vector3& Pout)
{
		double eps = 0.00000;

		Matrix3 A = v3(n, P2-P1, P3-P4);

		Vector3 x = A.inverse()*(P3-P1);

		Pout = P3 + x(2)*(P4-P3);

		if(x(1)>=eps && x(1)<=1-eps && x(2)>=eps && x(2)<=1-eps)
				return true;
		return false;
}

void OverlapRegionCalculator::makeBoundaries(bool is_target_overlap)
{
	vector<int> t_vids;
	vector<int> b_vids;
	vector<vector<int > > i_vids;

	t_vids = (is_target_overlap) ? in_tri_vids : out_tri_vids;
	b_vids = bound_vids;
	i_vids = intersect_vids;


	while(!(t_vids.empty() &&  i_vids[0].empty() && i_vids[1].empty() && i_vids[2].empty())){
		STATE state;
		int tvid,bvid,start_tvid;

		if(!t_vids.empty()){
			tvid = t_vids[0];	
			bvid = -1;
			state=S_TRIANGLE_VERTEX;
			start_tvid = tvid;
		}else{
			for(int n=0;n<3;n++){
				if(i_vids[n].size() !=0){
					tvid = n;
					bvid = i_vids[n][0];
					state=S_BOUNDARY_TRIANGLE;
					break;
				}
			}
		}

		bool end_flag = false;
		DIRECTION direction = D_ASCENDING;
		vector<Vector3> bound;
		while(!end_flag){
			bool has_next_point = false;
			switch(state){
			case S_TRIANGLE_VERTEX:// in vertex of triangle
				bound.push_back(triangle_v[tvid]);

				// remove target vertex
				for(vector<int>::iterator vi=t_vids.begin();vi!= t_vids.end();vi++){
					if(tvid == *vi){
						t_vids.erase(vi);
						break;
					}
				}

				if(i_vids[tvid].size() != 0){
					//next point is an intersection point
					bvid = i_vids[tvid][0];
					state = S_TRIANGLE_BOUNDARY;
				}else{
					//next point is vertex of triangle
					if(t_vids.empty()){
						end_flag = true;
					}else{
						tvid = (tvid+1)%3;
						state = S_TRIANGLE_VERTEX;
						if(tvid == start_tvid) end_flag=true;
					}
				}
				break;
			case S_TRIANGLE_BOUNDARY:
				bound.push_back(intersect_pos[tvid][bvid]);

				// remove target intersection point
				for(vector<int>::iterator vi = i_vids[tvid].begin();vi!= i_vids[tvid].end();vi++){
					if(bvid == *vi){
						i_vids[tvid].erase(vi);
						break;
					}
				}

				has_next_point = false;
				for(int n=0;n<b_vids.size();n++){
					//next point is vertex of boundary
					if(b_vids[n] == (bvid+1)%boundary_v.size()){
						bvid = (bvid+1)%boundary_v.size();
						state=S_BOUNDARY_VERTEX;
						direction = D_ASCENDING;
						has_next_point = true;
						break;
					}else if(b_vids[n] == bvid){
						state=S_BOUNDARY_VERTEX;
						direction = D_DESCENDING;
						has_next_point = true;
						break;
					}
				}
				if(!has_next_point){
					//next point is intersection point
					for(int n=0;n<3;n++){
						for(int m=0;m<i_vids[n].size();m++){
							if(i_vids[n][m] == bvid){
								tvid = n;
								state = S_BOUNDARY_TRIANGLE;
								has_next_point = true;
								break;
							}
						}
					}
				}
				if(!has_next_point) end_flag = true;
				break;
			case S_BOUNDARY_VERTEX:
				bound.push_back(boundary_v[bvid]);

				// remove target vertex of boundary
				for(vector<int>::iterator vi = b_vids.begin();vi!= b_vids.end();vi++){
					if(bvid == *vi){
						b_vids.erase(vi);
						break;
					}
				}

				int next_vid;
				if(direction == D_ASCENDING) next_vid = (bvid+1)%boundary_v.size();
				if(direction == D_DESCENDING) next_vid = (bvid==0) ? boundary_v.size()-1 : bvid-1;
				has_next_point = false;
				for(int m=0;m<b_vids.size();m++){	
					//next point is vertex of boundary
					if(b_vids[m] == next_vid){
						bvid = next_vid;
						state=S_BOUNDARY_VERTEX;
						has_next_point = true;
						break;
					}
				}
				if(!has_next_point){
					//next point is intersection point
					if(direction==D_DESCENDING) bvid = (bvid==0) ? boundary_v.size()-1 : bvid-1;
					for(int n=0;n<3;n++){
						for(int m=0;m<i_vids[n].size();m++){
							if(i_vids[n][m] == bvid){
								tvid = n;
								state = S_BOUNDARY_TRIANGLE;
								has_next_point =true;
								break;
							}
						}
					}
				}
				if(!has_next_point) end_flag = true;
				break;
			case S_BOUNDARY_TRIANGLE:
				bound.push_back(intersect_pos[tvid][bvid]);

				int b_count = 0;
				int a_count = 0;
				bool is_found = false;
				vector<int>::iterator cur_vi = i_vids[tvid].begin();
				for(vector<int>::iterator vi = i_vids[tvid].begin();vi!= i_vids[tvid].end();++vi){
					if(bvid == *vi){
						cur_vi = vi;
						is_found = true;
						continue;
					}
					if(is_found){
						a_count++;
					}else{
						b_count++;
					}
				}

				for(vector<int>::iterator vi = t_vids.begin();vi!=t_vids.end();++vi){
					if(tvid == *vi){
						b_count++;
					}
					if((tvid+1)%3 == *vi){
						a_count++;
					}
				}

				if(b_count == 0 && a_count == 0){
					i_vids[tvid].erase(cur_vi);
					end_flag=true;
					break;
				}

				if(b_count %2 == 1){
					if(cur_vi != i_vids[tvid].begin()){
						bvid = *(cur_vi-1);
						i_vids[tvid].erase(cur_vi);
						state = S_TRIANGLE_BOUNDARY;
					} else {
						i_vids[tvid].erase(cur_vi);
						state = S_TRIANGLE_VERTEX;
						if(tvid == start_tvid) end_flag=true;
					}
				} else {
					if(cur_vi+1 != i_vids[tvid].end()){
						bvid = *(cur_vi+1);
						i_vids[tvid].erase(cur_vi);
						state = S_TRIANGLE_BOUNDARY;
					} else {
						i_vids[tvid].erase(cur_vi);
						tvid = (tvid+1)%3;
						state = S_TRIANGLE_VERTEX;
						if(tvid == start_tvid) end_flag=true;
					}
				}
				break;
			}
		}
		if(is_target_overlap){
			overlap_boundary.push_back(bound);
		}else{
			not_overlap_boundary.push_back(bound);
		}
	}

	if(is_target_overlap && !b_vids.empty()){
		vector<Vector3> bound;
		//for(unsigned int j=0;j<b_vids.size();j++){
		for(int j=b_vids.size() - 1; j >= 0; j--){
			bound.push_back(boundary_v[b_vids[j]]);
		}
		overlap_boundary.push_back(bound);
	}
}

/**
* Remove duplicated points.
* @param[in] vec points
*/
void OverlapRegionCalculator::removeDuplicatedPoints(PointVec& vec) const {
  for (PointVec::iterator vi = vec.begin(); vi != vec.end()-1;) {
    Vector3 target = *vi;
    PointVec::iterator next_vi = vi + 1;
    while (next_vi != vec.end()) {
      Vector3 next = *next_vi;
      if (target == next) {
        next_vi == vec.erase(next_vi);
      } else {
        ++next_vi;
      }
      if (next_vi == vec.end()) {
        return;
      }
    }
    vi = next_vi;
  }
}

/**
* Divide polygon (point array) into triangles.
* @param[in] polygon point array
* @param[out] triangles triangle list
*/
void OverlapRegionCalculator::dividePolygon(const Boundary& polygon, PointVecVec& triangles) {
	vector<const Vector3*> points;
	for (unsigned int i = 0; i < polygon.size(); i++) {
		points.push_back(&(polygon[i]));
	}

	while (points.size() > 2) {
		//search convex vertex
		double max_dist =0;
		int target_id = 0;
		for (int j = 0; j < points.size(); j++) {
			double dist = norm2(*(points[j]));
			if (dist > max_dist) {
				target_id = j;
				max_dist = dist;
			}
		}
		int prev_id = (target_id == 0) ? points.size() - 1 : target_id - 1;
		int next_id = (target_id + 1) % points.size();

		Vector3 n_conv = cross((*(points[target_id]) - *(points[prev_id])) , (*(points[next_id]) - *(points[target_id])));

    int first_id = target_id;
    int loop_count = 0;
		while(true){
      if (first_id == target_id) loop_count++;
			prev_id = (target_id == 0) ? points.size() - 1 : target_id - 1;
			next_id = (target_id + 1) % points.size();
			Vector3 n_curr = cross((*(points[target_id]) - *(points[prev_id])) , (*(points[next_id]) - *(points[target_id])));

			//If the vertex is a reflex vertex, it isn't a candidate.  
			if (dot(n_conv, n_curr) < 0) {
				target_id = next_id;
				continue;
			}

			//check if there is a vertex in the triangle
			bool has_point_intriangle = false;
			for (int k = 0; k < points.size(); k++) {
				if (k == target_id || k == prev_id || k == next_id) continue;
				vector<Vector3> target_tri;
        target_tri.push_back(*(points[prev_id]));
        target_tri.push_back(*(points[target_id]));
        target_tri.push_back(*(points[next_id]));
        if(isInsideTriangle(target_tri,*(points[k]))){
          has_point_intriangle = true;
          break;
        }
			}

			//divide the triangle from the polygon
      if (!has_point_intriangle || (loop_count > 2)) {
				vector<Vector3> triangle;
				triangle.push_back(*(points[prev_id]));
				triangle.push_back(*(points[target_id]));
				triangle.push_back(*(points[next_id]));
				triangles.push_back(triangle);
				points.erase(points.begin()+target_id);
				break;
			}

			target_id = next_id;
		}
	}
}

/**
* Check if the point is within the triangle.
* @param[in] triangle target triangle
* @param[in] p target point
* @return ture if the point is located inside of the triangle
*/
bool OverlapRegionCalculator::isInsideTriangle(const PointVec& triangle, const Vector3& p) {
  Vector3 n = cross((triangle[1] - triangle[0]), (triangle[2] - triangle[0]));

  double sign;
  for (unsigned int k = 0; k < 3; k++) {
    Vector3 t = cross((triangle[k] - p), (triangle[(k+1)%3] - p));
    
    double tmp_sign = dot(t, n);
    
    if (tmp_sign == 0) {
      return (dot((triangle[k] - p), (triangle[(k+1)%3] - p)) <= 0);
    }
    if (k == 0) {
      sign = tmp_sign;
    } else {
      if (sign * tmp_sign < 0) {
        return  false;
      }
    }
  }
  return true;
}

/**
* Obtain area of the regions which are made up by the boundary points.
* @param boundaries point list of the boundaries
* @param n_bound    normal vector of the boundary
* @return area
*/
double OverlapRegionCalculator::getArea(const BoundaryVec& boundaries, const Vector3& n_bound) {
	double area = 0;
	for (unsigned int i = 0; i < boundaries.size(); i++) {
		for(unsigned int j = 2; j < boundaries[i].size(); j++) {
			Vector3 e1 = boundaries[i][j-1] - boundaries[i][0];
			Vector3 e2 = boundaries[i][j] - boundaries[i][0];
			Vector3 n = cross(e1, e2);
			int sign = (dot(n, n_bound) > 0) ? 1 : -1;
			area += sign * norm2(n) / 2.0;
		}
	}
	return fabs(area);
}



EnCalculator::EnCalculator() :
	obj(NULL), en(0.0), force(0.0), hmax(0.005), elastic(1.0e7), h_division_num(100) {

}

EnCalculator::~EnCalculator() {
	for (ContactTriangleVecIte ci = candidate_triangles.begin(); ci != candidate_triangles.end(); ++ci) {
		delete *ci;
	}
	clearSplittedTriangles();
}

/**
* Calculate En and contact force.
* @param[in] p_obj         contact point on the object
* @param[in] p_fing        contact point on the finger in object coordinate
* @param[in] n_fing        normal vector at contact point on the finger in object coordinate
* @param[in] boundary_fing point list of the boundaries
* @param[in] fmax          force limit
* @return                  displacement length
*/
double EnCalculator::calc(
	const Vector3& p_obj,
	const Vector3& p_fing, const Vector3& n_fing,
	const BoundaryVec& boundary_fing, double fmax) {
	return calc(p_obj, p_obj, p_fing, n_fing, boundary_fing,fmax);
}

/**
* Calculate En.
* @param[in] p_obj         contact point on the object
* @param[in] p_obj_top     closest point on the object to finger
* @param[in] p_fing        contact point on the finger in object coordinate
* @param[in] n_fing        normal vector at contact point on the finger in object coordinate
* @param[in] boundary_fing point list of the boundaries
* @param[in] fmax          force limit
* @return                  displacement length
*/
double EnCalculator::calc(
	const Vector3& p_obj, const Vector3& p_obj_top,
	const Vector3& p_fing, const Vector3& n_fing,
	const BoundaryVec& boundary_fing, double fmax) {
	p_top = p_obj_top;
	n_contact = n_fing;
	p_contact = p_obj;
	p_plane = p_fing;
	boundaries = boundary_fing;

	// precomputation
	init();
	generateCandidateTrianglesList();
	sortByDepth();
	calcOverlapArea(candidate_triangles);

	double tmp_en;
	double tmp_sum_area;
	double tmp_force;
	double diff_f = DBL_MAX;
	cnoid::Vector3 tmp_normal;

	bool is_overlap = false;

	for (unsigned int i = 0; i < h_division_num; i++) {
		double h = (hmax * (i+1)) / static_cast<double>(h_division_num);
		is_overlap = calcProc(h);
		if (force >= fmax) {
			if ( (force - fmax) > diff_f) {
				force = tmp_force;
				en = tmp_en;
				sum_area = tmp_sum_area;
				normal = tmp_normal;
				return (hmax * i / static_cast<double>(h_division_num));
			} else {
				return h;
			}
		}
		diff_f = fmax - force;
		tmp_en = en;
		tmp_force = force;
		tmp_sum_area = sum_area;
		tmp_normal = normal;
	}

	if (!is_overlap) {
		force = 0.0;
		en = 0.0;
		sum_area = 0.0;
		return 0.0;
	}
	return hmax;
}

/**
* Caluculate En when the displacment is @c h.
* @param[in] h displacemnt
* @return ture if contact region exists
*/
bool EnCalculator::calcProc(double h) {
	sum_area = 0.0;
	double sum_vol = 0.0;
	double en_denom = 0.0;
	double en_numer = 0.0;
	Vector3 sum_normal(0, 0, 0);

	ContactTriangleVec triangles;
	generateTargetTriangleList(h, triangles);
	for (ContactTriangleVecIte ci = triangles.begin(); ci != triangles.end(); ++ci) {
		double displacement = h - (*ci)->depth;
		sum_area += (*ci)->overlap_area;
		sum_normal += (*ci)->overlap_area * (*ci)->tri->normal;
		double vol = (*ci)->overlap_area * displacement;
		sum_vol += vol;
		en_numer += (*ci)->dist * vol;
	}

	if (sum_area < DBL_MIN) {
		force = 0.0;
		en = 0.0;
		return false;
	}
	force = elastic / hmax * sum_vol;
	en = en_numer / sum_vol;
	normal = unit(sum_normal);
	return true;
}

/**
* Initialize.
*/
void EnCalculator::init() {
	for (ContactTriangleVecIte ci = candidate_triangles.begin(); ci != candidate_triangles.end(); ++ci) {
		delete *ci;
	}
	candidate_triangles.clear();
	clearSplittedTriangles();
}

/**
* Select triangles which are within the depth limit @c h_max.
*/
void EnCalculator::generateCandidateTrianglesList() {
	Vector3 N = unit(n_contact);
	for (int i = 0; i < obj->nTriangles; i++) {
		const Triangle* tri = &(obj->triangles[i]);
		if (dot(tri->normal, N) > 0 ) continue;

		double len_sum = 0.0;
		bool is_within = false;
		double lengths[3];
		Vertex* vers[3];
		bool prev_len_sign;

		for (int k = 0; k < 3; k++) {
			Vector3 diff = tri->ver[k]->pos - p_top;
			double len = dot(N, diff);
			if (0 <= len && len <= hmax) {
				is_within = true;
			}
			if (k > 0 && prev_len_sign != (len < 0)) {
				is_within = true;
			}
			prev_len_sign = (len < 0);
			lengths[k] = len;
			vers[k] = tri->ver[k];
			for (int l = k; l > 0; l--) {
				if (lengths[l] < lengths[l-1]) {
					double tmp_len = lengths[l];
					Vertex* tmp_ver = vers[l];
					lengths[l] = lengths[l-1];
					vers[l] = vers[l-1];
					lengths[l-1] = tmp_len;
					vers[l-1] = tmp_ver;
				}
			}
		}

		double h = len_sum / 3.0;
		if (is_within) {
			ContactTriangle* target = new ContactTriangle(tri);
			target->depth = h;
			for (int k = 0; k < 3; k++) {
				target->sorted_depth[k] = lengths[k];
				target->sorted_ver[k] = vers[k];
			}
			candidate_triangles.push_back(target);
		}
	}
}

/**
* Sort the triangle list by the depth from contact point.
*/
void EnCalculator::sortByDepth() {
	std::sort(candidate_triangles.begin(), candidate_triangles.end(), &ContactTriangle::greater);
}

/**
* Calculate overlap regions between the object and the finger.
* @param[in,out] target_list target candidate triangle list
*/
void EnCalculator::calcOverlapArea(ContactTriangleVec& target_list, bool remove_unoverlaptri) {
	//project contact point of the object onto the boundary of the finger.
	Vector3 p_obj_proj = intersectionPoint(p_contact, n_contact, p_plane, n_contact);

	for (ContactTriangleVecIte ci = target_list.begin(); ci != target_list.end();) {
		const Triangle* tri = (*ci)->tri;
		(*ci)->overlap_area = 0.0;
		bool is_overlap = false;
		for (unsigned int i = 0; i < boundaries.size(); i++) {
			OverlapRegionCalculator orc;
			orc.calc(tri, boundaries[i], p_plane, n_contact, n_contact);
			if (orc.isOverlap()) {
				is_overlap = true;
				BoundaryVec overlap_boundary = orc.getOvelapBoundary();
				(*ci)->overlap_area += OverlapRegionCalculator::getArea(overlap_boundary, n_contact);
			}
		}
		if(!is_overlap) {
			(*ci)->overlap_area = 0.0;
			(*ci)->dist = 0;
			if (remove_unoverlaptri) {
				ci = target_list.erase(ci);
				continue;
			}
			++ci;
			continue;
		}

		//calculate center of the triangle and project it onto the boundary of the finger.
		Vector3 cot = (tri->ver[0]->pos + tri->ver[1]->pos + tri->ver[2]->pos) / 3.0;
		Vector3 cot_proj = intersectionPoint(cot, n_contact, p_plane, n_contact);

		(*ci)->dist = norm2(cot_proj - p_obj_proj);
		++ci;
	}
}

/**
* Generate contact region triangles.
* @param[in] h displacemnt
* @param[out] triangle list
*/
void EnCalculator::generateTargetTriangleList(double h, ContactTriangleVec& target_list) {
	clearSplittedTriangles();
	target_list.clear();
	
	for (ContactTriangleVecIte ci = candidate_triangles.begin(); ci != candidate_triangles.end(); ++ci) {
		if ((*ci)->sorted_depth[0] > h) break;
		
		if ((*ci)->sorted_depth[2] < h && (*ci)->sorted_depth[0] > 0) {
			target_list.push_back(*ci);
		} else {
			// split triangle
			vector<ConvexPoint> v_outside1, v_outside2, v_inside;
			for (unsigned int k = 0; k < 3; k++) {
				Vector3 p_cur = (*ci)->sorted_ver[k]->pos;
				Vector3 p_next = (*ci)->sorted_ver[(k+1)%3]->pos;
				if ((*ci)->sorted_depth[k] < 0) {
					v_outside1.push_back(ConvexPoint(p_cur, (*ci)->sorted_depth[k]));
				} else if ((*ci)->sorted_depth[k] > h) {
					v_outside2.push_back(ConvexPoint(p_cur, (*ci)->sorted_depth[k]));
				} else {
					v_inside.push_back(ConvexPoint(p_cur, (*ci)->sorted_depth[k]));
				}
				Vector3 diff = p_next - p_cur;
				Vector3 p_intersect = intersectionPoint(p_cur, diff, p_top, n_contact);
				if ((dot(diff, (p_intersect - p_cur))) > 0 && (dot(-diff, (p_intersect - p_next)) > 0)) {
					v_outside1.push_back(ConvexPoint(p_intersect, 0.0));
					v_inside.push_back(ConvexPoint(p_intersect, 0.0));
				}
				p_intersect = intersectionPoint(p_cur, diff, p_top + h * unit(n_contact), n_contact);
				if ((dot(diff, (p_intersect - p_cur))) > 0 && (dot(-diff, (p_intersect - p_next)) > 0)) {
					v_outside2.push_back(ConvexPoint(p_intersect, h));
					v_inside.push_back(ConvexPoint(p_intersect, h));
				}
			}

			makeSplittedTriangle(v_outside1, (*ci)->tri, splitted_nocandidate_triangles);
			makeSplittedTriangle(v_outside2, (*ci)->tri, splitted_nocandidate_triangles);
			makeSplittedTriangle(v_inside, (*ci)->tri, splitted_candidate_triangles);
		}
	}
	calcOverlapArea(splitted_candidate_triangles, false);
	target_list.insert(target_list.end(), splitted_candidate_triangles.begin(), splitted_candidate_triangles.end());
}

bool EnCalculator::makeSplittedTriangle(vector<ConvexPoint> p, const Triangle* ori_tri, vector<ContactTriangle*>& triangles) const {
	if (p.size() < 3) return false;
	
	sort(p.begin(), p.end());
	const int N = p.size();
	vector<ConvexPoint> convex(N);

	convex[0] = p[0];
	convex[1] = p[1];
	int count = 2;
	p.erase(p.begin());
	p.erase(p.begin());

	Vector3 p_prev = convex[0].p;
	Vector3 p_cur = convex[1].p;
	Vector3 l_base = unit(p_prev - p_cur);

	while (!p.empty()) {
		double min_d = 2.0;
		double min_depth = DBL_MAX;
		vector<ConvexPoint>::iterator min_ite = p.begin();
		for (vector<ConvexPoint>::iterator vi = p.begin(); vi != p.end(); ++vi) {
			double d = dot(l_base, unit((*vi).p - p_cur));
			if (d < min_d || (d == min_d && min_depth > (*vi).depth)) {
				min_d = d;
				min_ite = vi;
				min_depth = (*vi).depth;
			}
		}
		if (convex[count-1].depth > (*min_ite).depth) break;
		p_prev = p_cur;
		p_cur = (*min_ite).p;
		l_base = unit(p_prev - p_cur);
		convex[count++] = *min_ite;
		p.erase(min_ite);
	}

	p_prev = convex[1].p;
	p_cur = convex[0].p;
	l_base = unit(p_prev - p_cur);
	count = N - 1;
	while (!p.empty()) {
		double min_d = 2.0;
		double min_depth = DBL_MAX;
		vector<ConvexPoint>::iterator min_ite = p.begin();
		for (vector<ConvexPoint>::iterator vi = p.begin(); vi != p.end(); ++vi) {
			double d = dot(l_base, unit((*vi).p - p_cur));
			if (d < min_d || (d == min_d && min_depth > (*vi).depth)) {
				min_d = d;
				min_ite = vi;
				min_depth = (*vi).depth;
			}
		}
		p_prev = p_cur;
		p_cur = (*min_ite).p;
		l_base = unit(p_prev - p_cur);
		convex[count--] = *min_ite;
		p.erase(min_ite);
	}

	for (unsigned int i = 2; i < convex.size(); i++) {
		Triangle* tri = new Triangle();
		for (int k = 0; k < 3; k++) {
			tri->ver[k] = new VertexLink();
		}
		tri->ver[0]->pos = convex[0].p;
		tri->ver[1]->pos = convex[i-1].p;
		tri->ver[2]->pos = convex[i].p;
		Vector3 e1 (convex[i-1].p - convex[0].p);
		Vector3 e2 (convex[i].p - convex[0].p);
		if (dot(cross(e1,e2), ori_tri->normal) < 0) {
			tri->ver[1]->pos = convex[i].p;
			tri->ver[2]->pos = convex[i-1].p;
		}
		tri->normal = ori_tri->normal;
		tri->id = ori_tri->id;
		ContactTriangle* ctri = new ContactTriangle(tri);
		ctri->depth = (convex[0].depth + convex[i-1].depth + convex[i].depth)/3.0;
		ctri->splitted = true;
		triangles.push_back(ctri);

	}

	return true;
}

/**
* Clear splitted triangles
*/
void EnCalculator::clearSplittedTriangles() {
	for (ContactTriangleVecIte ci = splitted_candidate_triangles.begin(); ci != splitted_candidate_triangles.end(); ++ci) {
		for (int i = 0; i < 3; i++) {
			delete (*ci)->tri->ver[i];
		}
		delete (*ci)->tri;
	}
	splitted_candidate_triangles.clear();

	for (ContactTriangleVecIte ci = splitted_nocandidate_triangles.begin(); ci != splitted_nocandidate_triangles.end(); ++ci) {
		for (int i = 0; i < 3; i++) {
			delete (*ci)->tri->ver[i];
		}
		delete (*ci)->tri;
	}
	splitted_nocandidate_triangles.clear();
}
