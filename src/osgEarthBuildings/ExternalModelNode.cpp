#include "BuildingCompiler"

#include "ExternalModelNode"
using namespace osgEarth;
using namespace osgEarth::Buildings;


ExternalModelNode::ExternalModelNode(const ExternalModelNode& rhs, const osg::CopyOp& copyop)
{
   vectorExternalModels = rhs.vectorExternalModels;
}


static bool checkvectorExternalModels(const ExternalModelNode& node)
{
   return node.vectorExternalModels.size() > 0;
}

static bool readvectorExternalModels(osgDB::InputStream& is, ExternalModelNode& node)
{
   unsigned int size = 0; is >> size >> is.BEGIN_BRACKET;
   for (unsigned int i = 0; i < size; ++i)
   {
      osg::ref_ptr<osg::Object> obj = is.readObject();
      ExternalModel* child = dynamic_cast<ExternalModel*>(obj.get());
      if (child) {
         node.vectorExternalModels.push_back(child);
      }
   }
   is >> is.END_BRACKET;
   return true;
}

static bool writevectorExternalModels(osgDB::OutputStream& os, const ExternalModelNode& node)
{
   unsigned int size = node.vectorExternalModels.size();
   os << size << os.BEGIN_BRACKET << std::endl;
   for (unsigned int i = 0; i < size; ++i)
   {
      os << node.vectorExternalModels[i];
   }
   os << os.END_BRACKET << std::endl;
   return true;
}

namespace ExternalModelNodeSerializer
{

   REGISTER_OBJECT_WRAPPER(ExternalModelNode,
      new ExternalModelNode,
      osgEarth::Buildings::ExternalModelNode,
      "osg::Node osgEarth::Buildings::ExternalModelNode")
   {
      ADD_USER_SERIALIZER(vectorExternalModels);
   }
}

ExternalModel::ExternalModel(const ExternalModel& rhs, const osg::CopyOp& copyop)
{
   modelName = rhs.modelName;
   xform = rhs.xform;
}

namespace ExternalModelSerializer
{
   REGISTER_OBJECT_WRAPPER(ExternalModel,
      new ExternalModel,
      osgEarth::Buildings::ExternalModel,
      "osg::Object osgEarth::Buildings::ExternalModel")
   {
      ADD_STRING_SERIALIZER(modelName, "");
      ADD_MATRIX_SERIALIZER(xform, osg::Matrixd::identity());
   }
}