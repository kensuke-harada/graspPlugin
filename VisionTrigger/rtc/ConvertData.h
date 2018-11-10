#ifndef _VISIONTRIGER_CONVERTDATA_H_
#define _VISIONTRIGER_CONVERTDATA_H_

#include <vector>

#include <cnoid/EigenTypes>

namespace grasp {
	class ConvertData {
		public:
		/*
			RecognitionParam format:
			box_model_id,
			box_r00, box_r01, box_r02, box_x,
			box_r10, box_r11, box_r12, box_y,
			box_r20, box_r21, box_r22, box_z,
			camera_r00, camera_r01, camera_r02, camera_x,
			camera_r10, camera_r11, camera_r12, camera_y,
			camera_r20, camera_r21, camera_r22, camera_z,
			do_merge, use_prev_result, prev_grasped_cluster_id
		*/
		struct RecognitionParam {
			int box_model_id;
			cnoid::Vector3 box_p;
			cnoid::Matrix3 box_R;
			cnoid::Vector3 camera_p;
			cnoid::Matrix3 camera_R;
			bool do_merge;
			bool use_prev_result;
			int prev_grasped_cluster_id;
		};

		/*
			RecognitionEnvResult format:
			box_r00, box_r01, box_r02, box_x,
			box_r10, box_r11, box_r12, box_y,
			box_r20, box_r21, box_r22, box_z,
			p0_x, p0_y, p0_z,
			p1_x, p1_y, p1_z,
			...
		*/
		struct RecognitionEnvResult {
			std::vector<cnoid::Vector3> point_cloud;
			cnoid::Vector3 box_p;
			cnoid::Matrix3 box_R;
		};

		/*
			RecognitionResultExt format:
			Cluster ID, Valid flag, Finish flag, Reserve,
			r00, r01, r02, x,
			r10, r11, r12, y,
			r20, r21, r22, z,
			o_index0, o_index1, ....
		*/
		struct RecognitionResultExt {
			int cluster_id;
			cnoid::Vector3 p;
			cnoid::Matrix3 R;
			bool is_finish;
			bool valid;
			std::vector<int> outlier_indices;
		};

		static bool dblSeqToRecognitionParam(const double* data, int len, RecognitionParam& param) {
			if (len < 28) return false;
			param.box_model_id = static_cast<int>(data[0]);
			for (int i = 0; i < 3; i++) {
				param.box_p(i) = data[4*i + 4];
				param.camera_p(i) = data[4*i + 16];
				for (int j = 0; j < 3; j++) {
					param.box_R(i,j) = data[4*i + j + 1];
					param.camera_R(i,j) = data[4*i + j + 13];
				}
			}
			param.do_merge = (data[25] > 0);
			param.use_prev_result = (data[26] > 0);
			param.prev_grasped_cluster_id = static_cast<int>(data[27]);
			return true;
		}

		static void recognitionParamTodblVec(const RecognitionParam& param, std::vector<double>& data) {
			data.clear();
			data.resize(28);
			data[0] = static_cast<double>(param.box_model_id);
			for (int i = 0; i < 3; i++) {
				data[4*i + 4] = param.box_p(i);
				data[4*i + 16] = param.camera_p(i);
				for (int j = 0; j < 3; j++) {
					data[4*i + j + 1] = param.box_R(i,j);
					data[4*i + j + 13] = param.camera_R(i,j);
				}
			}
			data[25] = (param.do_merge ? 1 : -1);
			data[26] = (param.use_prev_result ? 1 : -1);
			data[27] = static_cast<double>(param.prev_grasped_cluster_id);
		}

		static bool dblSeqToEnvResult(const double* data, int len, RecognitionEnvResult& res) {
			if (len < 12) return false;
			for (int i = 0; i < 3; i++) {
				res.box_p(i) = data[4*i + 3];
				for (int j = 0; j < 3; j++) {
					res.box_R(i,j) = data[4*i + j];
				}
			}
			res.point_cloud.clear();
			int p_size = (len - 12) / 3;
			for (int i = 0; i < p_size; i++) {
				res.point_cloud.push_back(cnoid::Vector3(data[3*i + 12], data[3*i + 13], data[3*i + 14]));
			}
			return true;
		}

		static void envResultTodblVec(const RecognitionEnvResult& res, std::vector<double>& data) {
			data.clear();
			data.resize(12 + res.point_cloud.size() * 3);
			for (int i = 0; i < 3; i++) {
				for (int j = 0; j < 3; j++) {
					data[4*i + j] = res.box_R(i,j);
				}
				data[4*i + 3] = res.box_p(i);
			}
			for (size_t i = 0; i < res.point_cloud.size(); i++) {
				for (int j = 0; j < 3; j++) {
					data[12 + 3*i + j] = res.point_cloud[i](j);
				}
			}
		}

		static bool dblSeqToResult(const double* data, int len, RecognitionResultExt& res) {
			if (len < 16) return false;
			res.cluster_id = static_cast<int>(data[0]);
			res.valid = (data[1] > 0);
			res.is_finish = (data[2] > 0);
			for (int i = 0; i < 3; i++) {
				res.p(i) = data[4*i + 7];
				for (int j = 0; j < 3; j++) {
					res.R(i,j) = data[4*i + j + 4];
				}
			}
			res.outlier_indices.clear();
			int idx_size = len - 16;
			for (int i = 0; i < idx_size; i++) {
				res.outlier_indices.push_back(static_cast<int>(data[i + 16]));
			}
			return true;
		}

		static void resultTodblVec(const RecognitionResultExt& res, std::vector<double>& data) {
			data.clear();
			data.resize(16 + res.outlier_indices.size());

			data[0] = static_cast<double>(res.cluster_id);
			data[1] = (res.valid) ? 1 : -1;
			data[2] = (res.is_finish) ? 1 : -1;

			for (int i = 0; i < 3; i++) {
				data[4*i + 7] = res.p(i);
				for (int j = 0; j < 3; j++) {
					data[4*i + j + 4] = res.R(i,j);
				}
			}

			for (size_t i = 0; i < res.outlier_indices.size(); i++) {
				data[i + 16] = static_cast<double>(res.outlier_indices[i]);
			}
		}
	};
}

#endif
