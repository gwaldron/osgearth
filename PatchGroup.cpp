#include "PatchGroup"

#include <osg/Array>
#include <osg/Geometry>
#include <osgDB/Registry>

#include "Patch"

namespace teng
{
using namespace std;
using namespace osg;

PatchGroup::PatchGroup()
{
    setRangeMode(LOD::PIXEL_SIZE_ON_SCREEN);
}

PatchGroup::PatchGroup(const PatchGroup& rhs, const CopyOp& copyop)
    : PagedLOD(rhs, copyop)
{
    setRangeMode(LOD::PIXEL_SIZE_ON_SCREEN);
}

PatchGroup::~PatchGroup()
{
}

void PatchGroup::getPatchExtents(Vec2d& lowerLeft, Vec2d& upperRight) const
{
    const PatchOptions* poptions
        = dynamic_cast<const PatchOptions*>(getDatabaseOptions());
    if (!poptions)
    {
        lowerLeft = Vec2d(0.0, 0.0);
        upperRight = Vec2d(1.0, 1.0);
    }
    else
    {
        poptions->getPatchExtents(lowerLeft, upperRight);
    }
}

// Child 0 is the patch for this LOD. Child 1 is a group of the 4
// patches for the next level.

// Copied and edited from PagedLOD::traverse.
void PatchGroup::traverse(NodeVisitor& nv)
{
    // set the frame number of the traversal so that external nodes can find out how active this
    // node is.
    if (nv.getFrameStamp() && 
        nv.getVisitorType()==osg::NodeVisitor::CULL_VISITOR) 
    {
        setFrameNumberOfLastTraversal(nv.getFrameStamp()->getFrameNumber());
    }

    double timeStamp = nv.getFrameStamp()?nv.getFrameStamp()->getReferenceTime():0.0;
    int frameNumber = nv.getFrameStamp()?nv.getFrameStamp()->getFrameNumber():0;
    bool updateTimeStamp = nv.getVisitorType()==osg::NodeVisitor::CULL_VISITOR;

    switch(nv.getTraversalMode())
    {
        case(NodeVisitor::TRAVERSE_ALL_CHILDREN):
            std::for_each(_children.begin(),_children.end(),NodeAcceptOp(nv));
            break;
        case(NodeVisitor::TRAVERSE_ACTIVE_CHILDREN):
        {
            Vec3 eye = nv.getViewPoint();
            Patch* patch = 0;
            if (_children.size() > 0)
                patch = dynamic_cast<Patch*>(_children[0].get());
            if (!patch)
                return;
            float epsilon = patch->getPatchError(eye);

            int lastChildTraversed = -1;
            bool needToLoadChild = false;
            // Range list is set up so that the error is [0,1] for the
            // patch  at this level and (1, 1e10) for the next level.
            for(unsigned int i=0;i<_rangeList.size();++i)
            {
                if (_rangeList[i].first <= epsilon
                    && epsilon < _rangeList[i].second)
                {
                    if (i<_children.size())
                    {
                        if (updateTimeStamp)
                        {
                            _perRangeDataList[i]._timeStamp=timeStamp;
                            _perRangeDataList[i]._frameNumber=frameNumber;
                        }

                        _children[i]->accept(nv);
                        lastChildTraversed = (int)i;
                    }
                    else
                    {
                        needToLoadChild = true;
                    }
                }
            }

            if (needToLoadChild)
            {
                unsigned int numChildren = _children.size();

                // select the last valid child.
                if (numChildren>0 && ((int)numChildren-1)!=lastChildTraversed)
                {
                    if (updateTimeStamp)
                    {
                        _perRangeDataList[numChildren-1]._timeStamp=timeStamp;
                        _perRangeDataList[numChildren-1]._frameNumber=frameNumber;
                    }
                    _children[numChildren-1]->accept(nv);
                }

                // now request the loading of the next unloaded child.
                if (!_disableExternalChildrenPaging &&
                    nv.getDatabaseRequestHandler() &&
                    numChildren<_perRangeDataList.size())
                {
                    // compute priority from where abouts in the required range the distance falls.
                    float priority = (_rangeList[numChildren].second - epsilon)/(_rangeList[numChildren].second-_rangeList[numChildren].first);
                    
                    // invert priority for PIXEL_SIZE_ON_SCREEN mode
                    priority = -priority;

                    // modify the priority according to the child's priority offset and scale.
                    priority = _perRangeDataList[numChildren]._priorityOffset + priority * _perRangeDataList[numChildren]._priorityScale;

                    if (_databasePath.empty())
                    {
                        nv.getDatabaseRequestHandler()->requestNodeFile(_perRangeDataList[numChildren]._filename,this,priority,nv.getFrameStamp(), _perRangeDataList[numChildren]._databaseRequest, _databaseOptions.get());
                    }
                    else
                    {
                        // prepend the databasePath to the child's filename.
                        nv.getDatabaseRequestHandler()->requestNodeFile(_databasePath+_perRangeDataList[numChildren]._filename,this,priority,nv.getFrameStamp(), _perRangeDataList[numChildren]._databaseRequest, _databaseOptions.get());
                    }
                }

            }
           break;
        }
        default:
            break;
    }
}

// Pseudo loader for patch group

class ReaderWriterPatchGroup : public osgDB::ReaderWriter
{
public:
    ReaderWriterPatchGroup()
    {
        supportsExtension("tengpatch", "patch pseudo loader");
    }

    virtual const char* className() const
    {
        return "patch pseudo loader";
    }

    virtual osgDB::ReaderWriter::ReadResult
    readNode(const string& fileName,
             const osgDB::ReaderWriter::Options* options) const
    {
        Vec2d lowerLeft(0.0, 1.0);
        Vec2d upperRight(1.0, 1.0);
        PatchFactory* factory = PatchFactory::instance();
        const PatchOptions* poptions
            = dynamic_cast<const PatchOptions*>(options);
        if (!poptions)
            poptions = factory->getPatchOptions();
        poptions->getPatchExtents(lowerLeft, upperRight);
        Vec2d range = upperRight - lowerLeft;
        Vec2d newRange = range * .5;
        Group* result = new Group;
        for (double x = 0; x < 1.0; x += .5)
        {
            for (double y = 0; y < 1.0; y += .5)
            {
                PatchOptions* pgroupOptions = osg::clone(poptions);
                Vec2d ll = lowerLeft + componentMultiply(Vec2d(x, y), range);
                pgroupOptions->setPatchExtents(ll, ll + newRange);
                pgroupOptions->setPatchLevel(poptions->getPatchLevel() + 1);
                Node* pgroup = pgroupOptions->createPatch(fileName);
                result->addChild(pgroup);
            }
        }
        return result;
    }
};

PatchOptions::PatchOptions()
    : _lowerLeft(0.0, 0.0), _upperRight(1.0, 1.0), _level(0), _maxLevel(4)
{
}

PatchOptions::PatchOptions(const std::string& str)
    : osgDB::Options(str), _lowerLeft(0.0, 0.0), _upperRight(1.0, 1.0),
      _level(0), _maxLevel(1)
{
}

PatchOptions::PatchOptions(const PatchOptions& rhs, const CopyOp& copyop)
    : osgDB::Options(rhs), _lowerLeft(rhs._lowerLeft),
      _upperRight(rhs._upperRight), _level(rhs._level), _maxLevel(rhs._maxLevel)
{
}

Node* PatchOptions::createPatch(const std::string& filename)
{
    PatchGroup* pgroup = new PatchGroup;
    pgroup->setDatabaseOptions(this);
    Patch* patch = new Patch;
    fillPatch(filename, patch);
    BoundingSphere bsphere = patch->getBound();
    pgroup->setCenter(bsphere.center());
    if (_level >= _maxLevel)
    {
        pgroup->addChild(patch, 0.0, 1e10);
    }
    else
    {
        pgroup->addChild(patch, 0.0, 1.0);
        pgroup->setRange(1, 1.0, 1e10);
        pgroup->setFileName(1, "foo.tengpatch");
    }
    return pgroup;
}

// Default implementation that creates a flat 81920m x 81920m plane.

void PatchOptions::fillPatch(const std::string& filename, Patch* patch)
{
    Vec2d ll, ur;
    getPatchExtents(ll, ur);
    Vec2d range = (ur - ll);
    ref_ptr<Patch::Data> data = new Patch::Data;
    Vec3Array* verts = new Vec3Array(129 * 129);
    for (int j = 0; j < 129; ++j)
        for (int i = 0; i < 129; ++i)
            (*verts)[129 * j + i]
                = Vec3((ll.x() + i * range.x() / 128.0) * 81920.0,
                       (ll.y() + j * range.y() / 128.0) * 81920.0,
                       0.0);
    data->vertexData.array = verts;
    data->vertexData.binding = Geometry::BIND_PER_VERTEX;
    Vec3Array* norms = new Vec3Array(1);
    (*norms)[0] = Vec3d(0.0, 0.0, 1.0);
    data->normalData.array = norms;
    data->normalData.binding = Geometry::BIND_OVERALL;
    Vec4Array* colors = new Vec4Array(1);
    (*colors)[0] = Vec4(1.0, 1.0, 1.0, 1.0);
    data->colorData.array = colors;
    data->colorData.binding = Geometry::BIND_OVERALL;
    patch->setData(data);
}

REGISTER_OSGPLUGIN(tengpatch, ReaderWriterPatchGroup)

PatchFactory* PatchFactory::instance()
{
    static ref_ptr<PatchFactory> factory = new PatchFactory;
    return factory.get();
}

PatchFactory::PatchFactory()
{
}

PatchFactory::~PatchFactory()
{
}

Node* PatchFactory::createPatchGroup(const std::string& filename)
{
    if (!_patchOptions.valid())
        _patchOptions = new PatchOptions;
    Node* result = _patchOptions->createPatch(filename);
    return result;
}

}
