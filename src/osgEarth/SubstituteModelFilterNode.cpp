#include "SubstituteModelFilterNode"
#include <osgDB/ObjectWrapper>

namespace osgEarth
{
    SubstituteModelFilterNode::SubstituteModelFilterNode()
        : _instanced(false)
        , _clustered(false)
    {

    }
    SubstituteModelFilterNode::SubstituteModelFilterNode(const SubstituteModelFilterNode& rhs, const osg::CopyOp& copyop)
    {
        _instanced = rhs._instanced;
        _clustered = rhs._clustered;
        _modelSymbolList = rhs._modelSymbolList;
    }

    static bool check_modelSymbolList(const SubstituteModelFilterNode& node)
    {
        return node.modelSymbolList().size() > 0;
    }

    static bool read_modelSymbolList(osgDB::InputStream& is, SubstituteModelFilterNode& node)
    {
        bool clustered;
        bool instanced;
        is >> clustered;
        is >> instanced;
        node.setClustered(clustered);
        node.setInstanced(instanced);

        unsigned int size = 0; is >> size >> is.BEGIN_BRACKET;
        for (unsigned int i = 0; i < size; ++i)
        {
            SubstituteModelFilterNode::ModelSymbol model;
            std::string base;
            is.readWrappedString(base);
            std::string context;
            is.readWrappedString(context);
            model.instanceURI = URI(base, context);

            is >> model.xform;
            node.modelSymbolList().push_back(model);
        }
        is >> is.END_BRACKET;
        return true;
    }

    static bool write_modelSymbolList(osgDB::OutputStream& os, const SubstituteModelFilterNode& node)
    {
        os << node.getClustered();
        os << node.getInstanced();

        unsigned int size = node.modelSymbolList().size();
        os << size << os.BEGIN_BRACKET << std::endl;

        for (SubstituteModelFilterNode::ModelSymbolList::const_iterator iter = node.modelSymbolList().begin(); iter != node.modelSymbolList().end(); ++iter)
        {

            const SubstituteModelFilterNode::ModelSymbol & model = (*iter);
            os.writeWrappedString(model.instanceURI.base());
            os.writeWrappedString(model.instanceURI.context().referrer());

            os << model.xform;
        }
        os << os.END_BRACKET << std::endl;
        return true;
    }

    REGISTER_OBJECT_WRAPPER(SubstituteModelFilterNode,
        new SubstituteModelFilterNode,
        osgEarth::SubstituteModelFilterNode,
        "osg::Node osgEarth::SubstituteModelFilterNode")
    {

        ADD_USER_SERIALIZER(_modelSymbolList);
    }
}