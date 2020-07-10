#include "InstancedModelNode"
#include <osgDB/ObjectWrapper>

namespace osgEarth 
{
   namespace Buildings
   {


      InstancedModelNode::InstancedModelNode() { 
         ; 
      }
      InstancedModelNode::InstancedModelNode(const InstancedModelNode& rhs, const osg::CopyOp& copyop)
      {
         _mapModelToInstances = rhs._mapModelToInstances;
      }
      
      InstancedModelNode& InstancedModelNode::operator=(const InstancedModelNode& rhs) {
         _mapModelToInstances = rhs._mapModelToInstances;
         return *this;
      }


      static bool check_mapModelToInstances(const InstancedModelNode& node)
      {
         return node._mapModelToInstances.size() > 0;
      }

      static bool read_mapModelToInstances(osgDB::InputStream& is, InstancedModelNode& node)
      {
         unsigned int size = 0; is >> size >> is.BEGIN_BRACKET;
         for (unsigned int i = 0; i < size; ++i)
         {
            std::string key;
            is.readWrappedString(key);
            InstancedModelNode::Instances instance;
            int matrixSize = is.readSize();
            for (int i = 0; i < matrixSize; i++)
            {
               osg::Matrixd mat;
               is >> mat;
               instance.matrices.push_back(mat);
            }
            is >> instance.minRange;
            is >> instance.maxRange;
            node._mapModelToInstances[key] = instance;
         }
         is >> is.END_BRACKET;
         return true;
      }

      static bool write_mapModelToInstances(osgDB::OutputStream& os, const InstancedModelNode& node)
      {
         unsigned int size = node._mapModelToInstances.size();
         os << size << os.BEGIN_BRACKET << std::endl;
         
         for ( InstancedModelNode::MapModelToInstances::const_iterator iter = node._mapModelToInstances.begin(); iter != node._mapModelToInstances.end(); ++iter)
         {
            os << iter->first;
            os.writeSize(iter->second.matrices.size());
            for (int i = 0; i < iter->second.matrices.size(); i++)
            {
               os << iter->second.matrices[i];
            }
            os << iter->second.minRange;
            os << iter->second.maxRange;
         }
         os << os.END_BRACKET << std::endl;
         return true;
      }

      REGISTER_OBJECT_WRAPPER(InstancedModelNode,
         new InstancedModelNode,
         osgEarth::Buildings::InstancedModelNode,
         "osg::Node osgEarth::Buildings::InstancedModelNode")
      {

         //_mapModelToInstances;
         ADD_USER_SERIALIZER(_mapModelToInstances);
      }

   }
}

