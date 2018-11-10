#include "GraspEventEater.h"
#include <cnoid/SceneDrawables>
#include <cnoid/Link>
#include <boost/bind.hpp>
#include <iostream>
#include <limits>

#include "PlaceController.h"
#include "VectorMath.h"

using namespace grasp;

namespace {
	class PressedNormalCalculator {
		public:
		struct NormalDist {
			cnoid::Vector3 normal;
			double dist;
			bool has_sol;
		};

		bool compute(cnoid::Link* link, const cnoid::Vector3& press_p, cnoid::Vector3& normal) {

			if (link == NULL) return false;

			NormalDist sol;
			sol.dist = std::numeric_limits<double>::max();
			sol.normal = normal;
			sol.has_sol = false;

			cnoid::Vector3 obj_press_p = link->R().transpose()*(press_p - link->p());

			cnoid::MeshExtractor extractor;
			extractor.extract(link->collisionShape(), boost::bind(&PressedNormalCalculator::findAndComputeNormal, this, &extractor, obj_press_p, &sol));

			normal = link->R() * sol.normal;
			return sol.has_sol;
		}

		void findAndComputeNormal(cnoid::MeshExtractor* extractor, const cnoid::Vector3& press_p, NormalDist* sol) {
			const double precision = 0.001;
			cnoid::SgMeshPtr mesh = extractor->currentMesh();
			const cnoid::Affine3& T = extractor->currentTransform();
			const cnoid::SgVertexArray& vertices = *mesh->vertices();

			const int num_triangles = mesh->numTriangles();
			std::vector<cnoid::Vector3> p(3);
			cnoid::Vector3 min_p;
			cnoid::Vector3 max_p;
			cnoid::Vector3 margin = precision * cnoid::Vector3::Ones();
			for (int i = 0; i < num_triangles; ++i) {
				cnoid::SgMesh::TriangleRef tri = mesh->triangle(i);
				for (int k = 0; k < 3; k++) {
					p[k] = T * vertices[tri[k]].cast<cnoid::Affine3::Scalar>();
				}
				min_p = (p[0].cwiseMin(p[1]).cwiseMin(p[2])) - margin;
				max_p = (p[0].cwiseMax(p[1]).cwiseMax(p[2])) + margin;

				if ((min_p.array() > press_p.array()).any()) continue;
				if ((max_p.array() < press_p.array()).any()) continue;

				// triangle normal vector
				cnoid::Vector3 v1 = p[1] - p[0];
				cnoid::Vector3 v2 = p[2] - p[0];
				cnoid::Vector3 np = grasp::unit(v1.cross(v2));

				// check the distance between the target point and the plane
				double planeD = -np.dot(p[0]);
				double ppdist = fabs(np.dot(press_p) + planeD); //nx*x + ny*y + nz*z + planeD );
				if (sol->dist > ppdist) {
					sol->dist = ppdist;
					sol->normal = np;
					sol->has_sol = true;
				}
			}
		}
	};
}

GraspEventEater::GraspEventEater() {
}

GraspEventEater::~GraspEventEater() {
}

bool GraspEventEater::onButtonPressEvent(const cnoid::SceneWidgetEvent& event) {
	if (event.button() == Qt::LeftButton) {
		cnoid::SceneLink* link = NULL;
		cnoid::SceneBody* body = NULL;
		if (getBodyLink(event, &link, &body)) {
			cnoid::Vector3 pressPos = event.point();
			cnoid::Vector3 normal(0, 0, 0);
			PressedNormalCalculator normal_calc;
			if(!normal_calc.compute(link->link(), pressPos, normal)) return false;

			PlaceController::instance()->setTargetPose(pressPos, normal);

			std::cout << pressPos << std::endl;
			std::cout << normal << std::endl;
		}
	}
	return false;
}
