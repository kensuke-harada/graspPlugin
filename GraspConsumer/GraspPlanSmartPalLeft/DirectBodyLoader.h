/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef EXCADE_ROBOTICS_DIRECT_BODY_LOADER_H_INCLUDED
#define EXCADE_ROBOTICS_DIRECT_BODY_LOADER_H_INCLUDED

#include <string>
#include <hrpModel/Body.h>
#include <hrpModel/ModelNodeSet.h>
//#include <Excade/SignalProxy.h>

namespace Excade {
    namespace Robotics {
  
        class DirectBodyLoaderImpl;
  
        class DirectBodyLoader
        {
        public:
            DirectBodyLoader();
            ~DirectBodyLoader();

            void setDivisionNumber(int n);
    
            hrp::BodyPtr loadModelFile(
                const std::string& filename,
                bool doTriangulation = true, bool doNormalGeneration = true, bool createColdetModel = true);

            const std::string& errorMessage();
    
            hrp::ModelNodeSetPtr modelNodeSet();

  //          SignalProxy< boost::signal<void(const std::string& message)> > sigMessage();
    
        private:
            DirectBodyLoaderImpl* impl;
        };
    }
}


#endif
