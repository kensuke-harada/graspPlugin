//-*- C++ -*-

#ifndef __RRTPATH_PLANNER_H__
#define __RRTPATH_PLANNER_H__

#include <string>
#include <vector>

#include <cnoid/Body>

#include "Algorithm.h"
#include "Configuration.h"
#include "ConfigurationSpace.h"
#include "Mobility.h"
#include "../Grasp/RobotBody.h"
#include "exportdef.h"

namespace PathEngine {
	class Algorithm;
	//class Mobility;
	//class PathPlanner

	class CollisionChecker {
	public:
		explicit CollisionChecker(grasp::RobotBodyPtr body) : body_(body){;}
		bool checkCollision();
		bool checkCollision(const Configuration &pos);
		bool checkCollision(const std::vector<Configuration> &path, bool docheckstart = true, bool docheckend = true);
	private:
		grasp::RobotBodyPtr body_;
	};
    /**
     * @brief 計画経路エンジン
     *
     * 経路計画を行うプログラムはこのクラスを使用する。干渉検出などを簡易化したメソッドを持つ。
     */
    class EXCADE_API PathPlanner {

    private:
			   /**
         * @brief 使用する経路計画アルゴリズム
         */
        Algorithm* algorithm_;
        /**
         * @brief 使用する移動能力
         */
        Mobility* mobility_;

        /**
         * @brief コンフィギュレーション空間
         */
        ConfigurationSpace cspace_;
				 /**
         * @brief 経路
         */
        std::vector<Configuration> path_;

				CollisionChecker* col_checker_;
    public:

        /**
         * @brief コンストラクタ
         */
        PathPlanner(grasp::RobotBodyPtr body, unsigned int dim, const std::vector<int>& sampleDoF) : cspace_(dim) , dofsize(0) {
					algorithm_ = NULL;
					mobility_ = NULL;
					col_checker_ = NULL;
					init(body, sampleDoF);
					;}

        /**
         * @brief デストラクタ
         */
        ~PathPlanner(){
					if(mobility_ != NULL) delete mobility_;
					if(col_checker_ != NULL) delete col_checker_;
        }

				void init(grasp::RobotBodyPtr body, const std::vector<int>& sampleDoF);

				void setAlgorithm(Algorithm* algorithm) {algorithm_ = algorithm;} 
				/**
         * @brief 経路計画を行う
         * @return 計画が正常に終了した場合true、それ以外はfalseを返す
         */
        bool calcPath(std::vector<cnoid::VectorXd>& path);
				/**
         * @brief 移動能力を取得する
         * @return 移動能力
         */
        Mobility* getMobility() {return mobility_;}
			 /**
         * @brief 干渉検出を行う
         * @return 干渉している場合true, それ以外false
         */
				bool checkCollision();
        /**
         * @brief 干渉検出を行う
         * @param pos ロボットの位置
         * @return 干渉している場合true, それ以外false
         */
				bool checkCollision(const Configuration &pos);

        /**
         * @brief パスの干渉検出を行う
         * @param path パス
         * @return 一点でも干渉しているとtrue
         */
        bool checkCollision(const std::vector<Configuration> &path, bool docheckstart = true, bool docheckend = true);
	
	       /**
         * @brief コンフィギュレーション空間設定を取得する
         * @return コンフィギュレーション空間設定
         */
        ConfigurationSpace* getConfigurationSpace() { return &cspace_; }

				void setParameters();
				
				int nDOF() const {return dofsize;}

				const std::string& error_message() const {return error_message_;}

		private:
				void convertPath(std::vector<cnoid::VectorXd>& path);
				int dofsize;

				std::string error_message_;
    };
};
#endif // __RRTPATH_PLANNER_H__
