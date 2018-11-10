#ifndef _GRASP_CAMERA_H_
#define _GRASP_CAMERA_H_

#include <vector>
#include <string>

#include <cnoid/Body>
#include <cnoid/JointPath>

#include <boost/shared_ptr.hpp>

#include "exportdef.h"

namespace grasp {
	class Camera;
	class FixedCamera;
	class AttachedCamera;
	typedef boost::shared_ptr<Camera> CameraPtr;
	typedef std::vector<CameraPtr>    CameraArray;
	typedef boost::shared_ptr<FixedCamera> FixedCameraPtr;
	typedef boost::shared_ptr<AttachedCamera> AttachedCameraPtr;

	/**
	 * @class CameraHandler
	 * @brief Camera handling class
	 *
	 * CameraHandler contains the default fixed camera regardless of whether invoking @c load.
	 */
	class EXCADE_API CameraHandler {
	public:
		CameraHandler();
		virtual ~CameraHandler();

		static CameraHandler* instance();

		/**
		 * @brief load camera infomation from robot YAML file
		 * @param[in] body body pointer of the robot which has camera information
		 * @param[in] filepath file path of robot YAML file
		 */
		void load(cnoid::BodyPtr body, const std::string& filepath);
		/// @deprecated use load
		void init(cnoid::BodyPtr body, const std::string& filepath);

		bool setTargetCamera(int id);
		CameraPtr getCamera(int id) const;
		CameraPtr getTargetCamera() const;
		int getTargetCameraID() const;
		cnoid::Vector3 getViewPoint() const;
		AttachedCameraPtr getTargetAttachedCamera() const;
		int size() const;
	private:
		CameraArray cameras_;
		CameraPtr target_camera_;
		int target_camera_id_;

		void addDefaultFixCamera();
		std::string getDefaultFixCameraTransMatrixFilePath() const;
	};

	/**
	 * @class CameraParameterReader
	 * @brief Generate cameras from robot YAML file
	 */
	class CameraParameterReader {
	public:
		CameraParameterReader();
		virtual ~CameraParameterReader();

		void readRobotYamlFile(cnoid::BodyPtr body, const std::string& filepath);
		const CameraArray& getCameras() const;
	private:
		CameraArray cameras_;
	};

	/**
	 * @class Camera
	 * @brief Camera base class (Abstract)
	 */
	class EXCADE_API Camera {
	public:
		Camera();
		virtual ~Camera();

		enum CameraType {UNKNOWN, HEAD_CAMERA, HANDEYE_CAMERA, FIXED_CAMERA};

		void setName(const std::string& name);
		std::string getName() const;
		void setType(const std::string& type);
		CameraType getType() const;
		bool isFixedCamera() const;
		bool isAttachedCamera() const;
		void setFocalDistance(double distance);
		double getFocalDistance() const;
		void setMinDistance(double distance);
		double getMinDistance() const;
		void setMaxDistance(double distance);
		double getMaxDistance() const;

		void setCameraPosition(const cnoid::Matrix3& R, const cnoid::Vector3& p);
		virtual cnoid::Vector3 getCameraPosition() const = 0;
		virtual cnoid::Matrix3 getCameraRotation() const = 0;
		virtual cnoid::Vector3 getViewDirection() const = 0;

		cnoid::Vector3 getOffset() const;

		/// @deprecated
		void setDirection(const cnoid::Vector3& _dir) {dir = _dir;}
		/// @deprecated
		void setPositionLocal(const cnoid::Vector3& _pos) {camera_p_ = _pos;}
		/// @deprecated
		cnoid::Vector3 getDirection() const {return dir;}
		/// @deprecated
		virtual cnoid::Vector3 getPosition() const {return getCameraPosition();}

	protected:
		std::string name_;
		CameraType type_;
		double focal_distance_;
		double min_distance_;
		double max_distance_;
		cnoid::Matrix3 camera_R_;
		cnoid::Vector3 camera_p_;

		cnoid::Vector3 dir;  /// @deprecated
	};

	/**
	 * @class FixedCamera
	 * @brief fixed camera
	 */
	class EXCADE_API FixedCamera :
		public Camera {
	public:
		FixedCamera();
		virtual ~FixedCamera();

		cnoid::Vector3 getCameraPosition() const;
		cnoid::Matrix3 getCameraRotation() const;
		cnoid::Vector3 getViewDirection() const;

		/// @deprecated
		cnoid::Vector3 getPosition() const {return getCameraPosition();}
	};

	/**
	 * @class AttachedCamera
	 * @brief Hand-eye/Head camera
	 */
	class EXCADE_API AttachedCamera :
		public Camera {
	public:
		/**
		 * @brief constructor
		 * @param[in] body BodyPtr of target robot
		 * @param[in] base A pointer to the base link
		 * @param[in] camera A pointer to the link that a camera is attached
		 */
		AttachedCamera(cnoid::BodyPtr body, cnoid::Link* base, cnoid::Link* camera);
		virtual ~AttachedCamera();

		AttachedCameraPtr copy(cnoid::BodyPtr body) const;

		cnoid::Vector3 getCameraPosition() const;
		cnoid::Matrix3 getCameraRotation() const;
		cnoid::Vector3 getViewDirection() const;

		cnoid::JointPathPtr getPath() const;
		cnoid::Link* getBaseLink() const;
		cnoid::Link* getCameraLink() const;
		cnoid::BodyPtr getBody() const;

		/// @deprecated
		cnoid::Vector3 getPosition() const {return getCameraPosition();}

	private:
		cnoid::JointPathPtr camera_path_;
		cnoid::BodyPtr body_;
	};
}  // namespace grasp

#endif /* _GRASP_CAMERA_H_ */
