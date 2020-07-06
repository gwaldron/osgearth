
#include "ElevationsLodNode"
#include <osgDB/ObjectWrapper>

namespace osgEarth {
   namespace Buildings
   {
      ElevationsLodNode::ElevationsLodNode() {
         ; 
      }
      ElevationsLodNode::ElevationsLodNode(const ElevationsLodNode& rhs, const osg::CopyOp& copyop)
      {
         elevationsLOD = rhs.elevationsLOD;
         xform = rhs.xform;
      }

      ElevationsLodNode& ElevationsLodNode::operator=(const ElevationsLodNode& rhs)
      {
         elevationsLOD = rhs.elevationsLOD;
         xform = rhs.xform;
         return *this;
      }


      const osg::LOD * ElevationsLodNode::getelevationsLOD() const  {
         return elevationsLOD.get(); 
      }
      void ElevationsLodNode::setelevationsLOD(osg::LOD * lod) { 
         elevationsLOD = lod; 
      }

      REGISTER_OBJECT_WRAPPER(ElevationsLodNode,
         new ElevationsLodNode,
         osgEarth::Buildings::ElevationsLodNode,
         "osg::Node osgEarth::Buildings::ElevationsLodNode")
      {
         ADD_OBJECT_SERIALIZER(elevationsLOD, osg::LOD, NULL);
         ADD_MATRIX_SERIALIZER(xform, osg::Matrix());
      }
   }
}