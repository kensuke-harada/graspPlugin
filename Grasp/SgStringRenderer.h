#ifndef _SgCharcterRenderer_h__
#define _SgCharcterRenderer_h__

#include <QPainter>
#include <GL/freeglut.h>

#include <boost/bind.hpp>

#include <cnoid/SceneView>
#include <cnoid/SceneGraph>
#include <cnoid/GLSceneRenderer>
#include <QGLWidget>

#include <string>

//#include "../GeometryHandler/GeometryHandle.h"


namespace grasp{

  class SgStringRenderer :
#ifdef CNOID_GE_16
		public cnoid::SgNode
#else
		public cnoid::SgCustomGLNode
#endif
	{
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW

#ifdef CNOID_GE_16
			SgStringRenderer(std::string str, cnoid::Vector3 pos) :
				cnoid::SgNode(findPolymorphicId<SgStringRenderer>()) {
#else
			SgStringRenderer(std::string str, cnoid::Vector3 pos){
				setRenderingFunction(boost::bind(&SgStringRenderer::render, this));
#endif
				displayString = str;
				position = pos;

				static bool first=true;
				if(first==true){
					first=false;
					int argc=1;
					char* argv[] = {(char*)"choreonoid"};
					glutInit(&argc, argv);
				}
			}

#ifdef CNOID_GE_16
				void render(cnoid::SceneRenderer* render){
#else
			void render(){
#endif
				glPushAttrib(GL_LIGHTING_BIT | GL_CURRENT_BIT);
				glDisable(GL_LIGHTING);
				//glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
				glColor3f( 1,0,0 );				
				glRasterPos3f(position[0], position[1], position[2]);
				for(int i=0;i<displayString.size();i++){
					glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, displayString.c_str()[i]);
				}
				glPopAttrib();
			}
			void setString(std::string str){
				displayString = str;
			}
			void setPosition(cnoid::Vector3 pos){
				position = pos;
			}
			std::string displayString;
			cnoid::Vector3 position;
			
	};
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	typedef boost::intrusive_ptr<SgStringRenderer> SgStringRendererPtr;
#else
	typedef cnoid::ref_ptr<SgStringRenderer> SgStringRendererPtr;
#endif
}

#endif
