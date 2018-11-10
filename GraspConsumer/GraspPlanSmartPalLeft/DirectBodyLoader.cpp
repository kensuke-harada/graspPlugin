/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "DirectBodyLoader.h"

#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <hrpUtil/Tvmet3d.h>
#include <hrpUtil/Tvmet4d.h>
#include <hrpUtil/TriangleMeshShaper.h>
#include <hrpModel/Body.h>
#include <hrpModel/Link.h>
#include <hrpModel/Sensor.h>
#include <hrpModel/ModelNodeSet.h>
#include <hrpCollision/ColdetModel.h>


using namespace std;
using namespace boost;
using namespace hrp;
using namespace Excade;
using namespace Excade::Robotics;


namespace Excade {
    namespace Robotics {

        class DirectBodyLoaderImpl
        {
        public:
            DirectBodyLoaderImpl();
            ~DirectBodyLoaderImpl();

            BodyPtr loadModelFile(
                const std::string& filename, bool doTriangulation, bool doNormalGeneration, bool createColdetModel);

            ModelNodeSetPtr modelNodeSet;
            TriangleMeshShaper triangleMeshShaper;
            BodyPtr body;
            bool createColdetModel;
            string errorMessage;

            boost::signal<void(const std::string& message)> sigMessage;

            typedef map<string, Sensor::SensorType> SensorTypeMap;
            static SensorTypeMap sensorTypeMap;

            struct MeshInfo
            {
                MeshInfo(VrmlIndexedFaceSet* triangleMesh, const Matrix44& T) :
                    triangleMesh(triangleMesh), T(T) { }
                VrmlIndexedFaceSet* triangleMesh;
                Matrix44 T;
            };
            vector<MeshInfo> meshes;
            int numTotalVertices;
            int numTotalTriangles;

            void createBodyFromModelNodeSet();
            Link* createLink(JointNodeSetPtr jointNodeSet, const Matrix33& parentRs);
            void setJointProperties(Link* link, VrmlProtoInstancePtr jointNode, const Matrix33& parentRs);
            void setSegmentProperties(Link* link, vector<VrmlProtoInstancePtr> segmentNodes, vector<Matrix44> transforms);
            void createSensors(Link* link, std::vector<VrmlProtoInstancePtr>& sensorNodes);
            void setColdetModel(Link* link, vector<VrmlProtoInstancePtr> segmentNodes, vector<Matrix44> transforms);
            bool collectMeshes(VrmlShape* shapeNode, const Matrix44& T);
            void extractShapeNodes(VrmlNode* node, const function<bool(VrmlShape* shapeNode, const Matrix44& T)>& callback);
            bool extractShapeNodeTraverse(
                VrmlNode* node, const Matrix44& T, const function<bool(VrmlShape* shapeNode, const Matrix44& T)>& callback);
        };
    }
}


DirectBodyLoaderImpl::SensorTypeMap DirectBodyLoaderImpl::sensorTypeMap;


namespace {

    /*!
      @if jp
      transformノードで指定されたrotation,translation,scaleを計算し，4x4行列に代入する。
      計算結果は第2引数に代入する。
      @endif
    */
    void calcTransformMatrix(VrmlTransform* transform, Matrix44& out_T)
    {
        Matrix44 R;
        const SFRotation& r = transform->rotation;
        calcRodrigues(R, Vector3(r[0], r[1], r[2]), r[3]);

        const SFVec3f& center = transform->center;

        Matrix44 SR;
        const SFRotation& so = transform->scaleOrientation;
        calcRodrigues(SR, Vector3(so[0], so[1], so[2]), so[3]);

        const SFVec3f& s = transform->scale;

        Matrix44 SinvSR;
        SinvSR =
            s[0] * SR(0,0), s[0] * SR(1,0), s[0] * SR(2,0), 0.0,
            s[1] * SR(0,1), s[1] * SR(1,1), s[1] * SR(2,1), 0.0,
            s[2] * SR(0,2), s[2] * SR(1,2), s[2] * SR(2,2), 0.0,
            0.0,             0.0,           0.0,            1.0;

        const Vector4 c(center[0], center[1], center[2], 1.0);

        Matrix44 RSR(R * SR);

        out_T = RSR * SinvSR;

        const Vector4 c2(out_T * c);
        for(int i=0; i < 3; ++i){
            out_T(i, 3) -= c2(i);
        }
    
        for(int i=0; i < 3; ++i){
            out_T(i, 3) += transform->translation[i] + center[i];
        }
    }


    double getLimitValue(VrmlVariantField& field, double defaultValue)
    {
        MFFloat& values = field.mfFloat();
        if(values.empty()){
            return defaultValue;
        }
        return values[0];
    }


    void copyVrmlField(VrmlVariantField& field, string& out_s)
    {
        switch(field.typeId()){
        case SFSTRING:
            out_s = field.sfString();
            break;
        case MFSTRING:
        {
            MFString& strings = field.mfString();
            out_s = "";
            for(size_t i=0; i < strings.size(); i++){
                out_s += strings[i] + "\n";
            }
        }
        break;
        default:
            break;
        }
    }


    void copyVrmlField(VrmlVariantField& field, int& out_value)
    {
        out_value = field.sfInt32();
    }


    void copyVrmlField(VrmlVariantField& field, double& out_value)
    {
        out_value = field.sfFloat();
    }


    void copyVrmlField(VrmlVariantField& field, Vector3& out_v)
    {
        SFVec3f& v = field.sfVec3f();
        out_v = v[0], v[1], v[2];
    }


    void copyVrmlField(VrmlVariantField& field, Matrix33& out_R)
    {
        if(field.typeId() == SFROTATION){
            SFRotation& R = field.sfRotation();
            out_R = rodrigues(Vector3(R[0], R[1], R[2]), R[3]);
        
        } else if(field.typeId() == MFFLOAT){
            MFFloat& mf = field.mfFloat();
            if(mf.size() >= 9){
                out_R =
                    mf[0], mf[1], mf[2],
                    mf[3], mf[4], mf[5],
                    mf[6], mf[7], mf[8];
            }
        }
    }
}


DirectBodyLoader::DirectBodyLoader()
{
    impl = new DirectBodyLoaderImpl();
}


DirectBodyLoaderImpl::DirectBodyLoaderImpl()
{
    if(sensorTypeMap.empty()){
	sensorTypeMap["ForceSensor"]        = Sensor::FORCE;
	sensorTypeMap["Gyro"]               = Sensor::RATE_GYRO;
	sensorTypeMap["AccelerationSensor"] = Sensor::ACCELERATION;
	sensorTypeMap["PressureSensor"]     = Sensor::PRESSURE;
	sensorTypeMap["PhotoInterrupter"]   = Sensor::PHOTO_INTERRUPTER;
	sensorTypeMap["VisionSensor"]       = Sensor::VISION;
	sensorTypeMap["TorqueSensor"]       = Sensor::TORQUE;
    }

    triangleMeshShaper.sigMessage.connect(sigMessage);
}


DirectBodyLoader::~DirectBodyLoader()
{
    delete impl;
}


DirectBodyLoaderImpl::~DirectBodyLoaderImpl()
{

}


ModelNodeSetPtr DirectBodyLoader::modelNodeSet()
{
    return impl->modelNodeSet;
}


const std::string& DirectBodyLoader::errorMessage()
{
    return impl->errorMessage;
}

/*
SignalProxy< boost::signal<void(const std::string& message)> > DirectBodyLoader::sigMessage()
{
    return impl->sigMessage;
}
*/

/**
   The function for setting the division number of primitive geometries such as
   cone, cylinder and sphere.
*/
void DirectBodyLoader::setDivisionNumber(int n)
{
    impl->triangleMeshShaper.setDivisionNumber(std::max(4, n));
}


BodyPtr DirectBodyLoader::loadModelFile
(const std::string& filename, bool doTriangulation, bool doNormalGeneration, bool createColdetModel)
{
    return impl->loadModelFile(filename, doTriangulation, doNormalGeneration, createColdetModel);
}


BodyPtr DirectBodyLoaderImpl::loadModelFile
(const std::string& filename, bool doTriangulation, bool doNormalGeneration, bool createColdetModel)
{
    errorMessage.clear();
    
    modelNodeSet.reset(new ModelNodeSet());

    bool loaded = false;

    try {
	if(modelNodeSet->loadModelFile(filename)){
            loaded = true;
            if(doTriangulation || createColdetModel){
                triangleMeshShaper.setNormalGenerationMode(doNormalGeneration);
                triangleMeshShaper.apply(modelNodeSet->humanoidNode());
            }
            this->createColdetModel = createColdetModel;
            
            createBodyFromModelNodeSet();
        }
    }
    catch(ModelNodeSet::Exception& ex){
	errorMessage = ex.what();
    }

    if(!loaded){
        body = 0;
    }
    
    return body;
}


void DirectBodyLoaderImpl::createBodyFromModelNodeSet()
{
    body = 0;

    JointNodeSetPtr rootJointNodeSet = modelNodeSet->rootJointNodeSet();

    if(rootJointNodeSet){

	body = new Body();
    
	body->setModelName(modelNodeSet->humanoidNode()->defName);
    
	Matrix33 Rs(tvmet::identity<Matrix33>());
	Link* rootLink = createLink(rootJointNodeSet, Rs);
	body->setRootLink(rootLink);

	TProtoFieldMap& f = rootJointNodeSet->jointNode->fields;

	Vector3 defaultRootPos;
	copyVrmlField(f["translation"], defaultRootPos);
	Matrix33 defaultRootR;
	copyVrmlField(f["rotation"], defaultRootR);
	
	body->setDefaultRootPosition(defaultRootPos, defaultRootR);

	body->installCustomizer();
	body->initializeConfiguration();
    }
}


Link* DirectBodyLoaderImpl::createLink(JointNodeSetPtr jointNodeSet, const Matrix33& parentRs)
{
    Link* link = new Link();

    setJointProperties(link, jointNodeSet->jointNode, parentRs);

    vector<VrmlProtoInstancePtr> segmentNodes = jointNodeSet->segmentNodes;

    vector<Matrix44> transforms = jointNodeSet->transforms;

    setSegmentProperties(link, segmentNodes, transforms);

    if(createColdetModel && !segmentNodes.empty()){
        setColdetModel(link, segmentNodes, transforms);
    }
    
    // The following code adds child links from the back of the child array
    // in order to keep the original order of the children.
    // ( addChild() function of the Link class prepends a child to the child list )
    int numChildren = jointNodeSet->childJointNodeSets.size();
    for(int i = numChildren - 1; i >= 0; --i){
	JointNodeSetPtr childJointNodeSet = jointNodeSet->childJointNodeSets[i];
	Link* childLink = createLink(childJointNodeSet, link->Rs());
	link->addChild(childLink);
    }
    
    createSensors(link, jointNodeSet->sensorNodes);

    return link;
}


void DirectBodyLoaderImpl::setJointProperties(Link* link, VrmlProtoInstancePtr jointNode, const Matrix33& parentRs)
{
    link->name = jointNode->defName;

    TProtoFieldMap& jf = jointNode->fields;

    copyVrmlField(jf["jointId"], link->jointId());

    Vector3 b;
    copyVrmlField(jf["translation"], b);
    link->b = parentRs * b;

    Matrix33 R;
    copyVrmlField(jf["rotation"], R);
    link->Rs() = (parentRs * R);

    Vector3 jointAxis(0.0);
    VrmlVariantField& jointAxisField = jf["jointAxis"];
    switch(jointAxisField.typeId()){
    case SFSTRING:
    {
        SFString& axisLabel = jointAxisField.sfString();
        if(axisLabel == "X"){
            jointAxis[0] = 1.0;
        } else if(axisLabel == "Y"){
            jointAxis[1] = 1.0;
        } else if(axisLabel == "Z"){
            jointAxis[2] = 1.0;
        }
    }
    break;
    case SFVEC3F:
	copyVrmlField(jointAxisField, jointAxis);
	break;
    default:
	break;
    }

    string jointType;
    copyVrmlField(jf["jointType"], jointType);
    
    if(jointType == "fixed" ){
	link->jointType() = Link::FIXED_JOINT;
    } else if(jointType == "free" ){
	link->jointType() = Link::FREE_JOINT;
    } else if(jointType == "rotate" ){
	link->jointType() = Link::ROTATIONAL_JOINT;
    } else if(jointType == "slide" ){
	link->jointType() = Link::SLIDE_JOINT;
    } else {
	link->jointType() = Link::FREE_JOINT;
    }

    link->a = 0.0;
    link->d = 0.0;
    if(link->jointType() == Link::ROTATIONAL_JOINT){
	link->a = link->Rs() * jointAxis;
    } else if(link->jointType() == Link::SLIDE_JOINT){
	link->d = link->Rs() * jointAxis;
    }

    copyVrmlField(jf["jointValue"], link->defaultJointValue);

    copyVrmlField(jf["rotorInertia"], link->Ir);
    copyVrmlField(jf["gearRatio"], link->gearRatio);
    copyVrmlField(jf["torqueConst"], link->torqueConst);
    copyVrmlField(jf["encoderPulse"], link->encoderPulse);
    copyVrmlField(jf["rotorResistor"], link->rotorResistor);

    VrmlVariantField* field = jointNode->getField("equivalentInertia");
    if(field){
	link->Jm2 = field->sfFloat();
    } else {
	link->Jm2 = link->gearRatio * link->gearRatio * link->Ir;
    }    

    double max = numeric_limits<double>::max();

    link->q_upper()  = getLimitValue(jf["q_upper()"],  +max);
    link->q_lower()  = getLimitValue(jf["q_lower()"],  -max);
    link->dq_upper() = getLimitValue(jf["dq_upper()"], +max);
    link->dq_lower() = getLimitValue(jf["dq_lower()"], -max);
}    


void DirectBodyLoaderImpl::setSegmentProperties(Link* link, vector<VrmlProtoInstancePtr> segmentNodes, vector<Matrix44> transforms)
{
    int numSegment = segmentNodes.size();
    link->m = 0.0;

    //  Mass = Σmass                 //
    //  C = (Σmass * T * c) / Mass   //
    //  I = Σ(R * I * Rt + G)       //
    //  R = Tの回転行列               //
    //  G = y*y+z*z, -x*y, -x*z, -y*x, z*z+x*x, -y*z, -z*x, -z*y, x*x+y*y    //
    //  (x, y, z ) = T * c - C        //
    std::vector<Vector3> centerOfMassArray;
    std::vector<double> massArray;
    Vector3 linkc(0);
    Matrix33 linkI(0);
    for(int i = 0 ; i < numSegment ; ++i){
        Matrix44 T = transforms.at(i);
        TProtoFieldMap& sf = segmentNodes[i]->fields;
        Vector3 c;
        copyVrmlField(sf["centerOfMass"], c);
        double m;
        copyVrmlField(sf["mass"], m);
        Matrix33 I;
        copyVrmlField(sf["momentsOfInertia"], I);

        Matrix33 R;
        R = T(0,0), T(0,1), T(0,2), T(1,0), T(1,1), T(1,2), T(2,0), T(2,1), T(2,2);
        Vector3 T0;
        T0 = T(0,3), T(1,3), T(2,3);
        Vector3 c1(R * c + T0);
        centerOfMassArray.push_back(c1);
        massArray.push_back(m);
        linkc = c1*m + linkc * link->m;
        link->m += m;
        linkc /= link->m;
        linkI = R * I * trans(R);
    }

    for(int i = 0 ; i < numSegment ; ++i){
        Vector3 c( centerOfMassArray.at(i) );
        double x = c(0) - linkc(0);
        double y = c(1) - linkc(1);
        double z = c(2) - linkc(2);
        double m = massArray.at(i);

        linkI(0,0) += m * (y*y + z*z);
        linkI(0,1) += -m * x * y;
        linkI(0,2) += -m * x * z;
        linkI(1,0) += -m * y * x;
        linkI(1,1) += m * (z*z + x*x);
        linkI(1,2) += -m * y * z;
        linkI(2,0) += -m * z * x;
        linkI(2,1) += -m * z * y;
        linkI(2,2) += m * (x*x + y*y);
    }

    link->c = link->Rs() * linkc;
    link->I = link->Rs() * linkI * trans(link->Rs());
}
    

void DirectBodyLoaderImpl::createSensors(Link* link, std::vector<VrmlProtoInstancePtr>& sensorNodes)
{
    int numSensors = sensorNodes.size();

    for(int i=0; i < numSensors; ++i){

	VrmlProtoInstancePtr sensorNode = sensorNodes[i];
	TProtoFieldMap& f = sensorNode->fields;

	const string& name = sensorNode->defName;
	int id;
	copyVrmlField(f["sensorId"], id);

	if(id < 0){
	    throw ModelNodeSet::Exception
		(string("sensor ID is not given to sensor ") + name + "of model " + body->modelName());
	} else {

	    int sensorType = Sensor::COMMON;
	    
	    SensorTypeMap::iterator p = sensorTypeMap.find(sensorNode->proto->protoName);
	    if(p != sensorTypeMap.end()){
		sensorType = p->second;
	    } else {
		throw ModelNodeSet::Exception("Unknown type sensor node");
	    }
	    
	    Sensor* sensor = body->createSensor(link, sensorType, id, name);

	    if(sensor){

		Vector3 p;
		copyVrmlField(f["translation"], p);
		sensor->localPos = link->Rs() * p;

		Matrix33 R;
		copyVrmlField(f["rotation"], R);
		sensor->localR = link->Rs() * R;
	    }
	}
    }
}


void DirectBodyLoaderImpl::setColdetModel(Link* link, vector<VrmlProtoInstancePtr> segmentNodes, vector<Matrix44> transforms)
{
    meshes.clear();
    numTotalVertices = 0;
    numTotalTriangles = 0;

    int numSegment = segmentNodes.size();
    for(int i = 0 ; i < numSegment ; ++i){
        Matrix44 T = transforms.at(i);
        extractShapeNodeTraverse(segmentNodes[i].get(), T, bind(&DirectBodyLoaderImpl::collectMeshes, this, _1, _2));
    }

    ColdetModelPtr coldetModel(new ColdetModel());

    coldetModel->setNumVertices(numTotalVertices);
    coldetModel->setNumTriangles(numTotalTriangles);

    int vertexIndex = 0;
    int triangleIndex = 0;
    
    for(size_t i=0; i < meshes.size(); ++i){

        int vertexTop = vertexIndex;

        const Matrix33& Rs = link->Rs();
        Matrix44 Rs44;
        Rs44 = Rs(0,0), Rs(0,1), Rs(0,2), 0.0,
               Rs(1,0), Rs(1,1), Rs(1,2), 0.0,
               Rs(2,0), Rs(2,1), Rs(2,2), 0.0,
               0.0,     0.0,     0.0,     1.0;
            
        Matrix44 T(Rs44 * meshes[i].T);
        
        const MFVec3f& vertices = meshes[i].triangleMesh->coord->point;
        size_t numVertices = vertices.size();
        
        for(size_t j=0; j < numVertices; ++j){
            const SFVec3f& v0 = vertices[j];
            Vector4 v(T * Vector4(v0[0], v0[1], v0[2], 1.0));
            coldetModel->setVertex(vertexIndex++, (float)v[0], (float)v[1], (float)v[2]);
        }

        const MFInt32& indices = meshes[i].triangleMesh->coordIndex;
        const size_t numTriangles = indices.size() / 4;

        for(size_t j=0; j < numTriangles; ++j){
            coldetModel->setTriangle(
                triangleIndex++, vertexTop + indices[j*4], vertexTop + indices[j*4+1], vertexTop + indices[j*4+2]);
        }
    }
    
    coldetModel->setName(link->name.c_str());
    coldetModel->build();
    link->coldetModel() = coldetModel;
}


bool DirectBodyLoaderImpl::collectMeshes(VrmlShape* shapeNode, const Matrix44& T)
{
    VrmlIndexedFaceSet* triangleMesh = dynamic_node_cast<VrmlIndexedFaceSet>(shapeNode->geometry).get();
    if(triangleMesh){
        meshes.push_back(MeshInfo(triangleMesh, T));
        numTotalVertices += triangleMesh->coord->point.size();
        numTotalTriangles += triangleMesh->coordIndex.size() / 4;
    }

    return true;
}


/**
   @todo move this code into the ModelNodeSet class ?
*/

bool DirectBodyLoaderImpl::extractShapeNodeTraverse
(VrmlNode* node, const Matrix44& T, const function<bool(VrmlShape* shapeNode, const Matrix44& T)>& callback)
{
    bool doContinue = true;
    
    if(node->isCategoryOf(PROTO_INSTANCE_NODE)){
        VrmlProtoInstance* protoInstance = static_cast<VrmlProtoInstance*>(node);
        if(protoInstance->actualNode){
            doContinue = extractShapeNodeTraverse(protoInstance->actualNode.get(), T, callback);
        }

    } else if(node->isCategoryOf(GROUPING_NODE)) {

        VrmlGroup* groupNode = static_cast<VrmlGroup*>(node);
        
        Matrix44 T2;
        VrmlTransform* transformNode = dynamic_cast<VrmlTransform*>(groupNode);
        if(transformNode){
            Matrix44 Tlocal;
            hrp::calcTransformMatrix(transformNode, Tlocal);
            T2 = T * Tlocal;
        }

        VrmlSwitch* switchNode = dynamic_cast<VrmlSwitch*>(node);
        if(switchNode){
            int whichChoice = switchNode->whichChoice;
            if( whichChoice >= 0 && whichChoice < switchNode->countChildren() )
                extractShapeNodeTraverse(switchNode->getChild(whichChoice), (transformNode ? T2 : T), callback);
        }else{
            for(size_t i=0; i < groupNode->countChildren(); ++i){
                if(!extractShapeNodeTraverse(groupNode->getChild(i), (transformNode ? T2 : T), callback)){
                    doContinue = false;
                    break;
                }
            }
        }

    } else if(node->isCategoryOf(SHAPE_NODE)) {
        doContinue = callback(static_cast<VrmlShape*>(node), T);
    }

    return doContinue;
}
