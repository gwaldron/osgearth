#include <osgEarth/SimplePager>
#include <osgEarth/TileKey>
#include <osgEarth/Utils>
#include <osgEarth/CullingUtils>
#include <osgEarth/Metrics>
#include <osgEarth/PagedNode>
#include <osgDB/Registry>
#include <osgDB/FileNameUtils>
#include <osg/ShapeDrawable>
#include <osg/MatrixTransform>

using namespace osgEarth::Util;

#define LC "[SimplerPager] "


SimplePager::SimplePager(const osgEarth::Profile* profile):
_profile( profile ),
_rangeFactor( 6.0 ),
_additive(false),
_minLevel(0),
_maxLevel(30),
_priorityScale(1.0f),
_priorityOffset(0.0f),
_canCancel(true),
_done(false),
_mutex("SimplePager(OE)")
{
    //nop
}

void SimplePager::setEnableCancelation(bool value)
{
    _canCancel = value;
}

bool SimplePager::getEnableCancelation() const
{
    return _canCancel;
}

void SimplePager::setDone()
{
    _done = true;
}

void SimplePager::build()
{
    addChild( buildRootNode() );
}

osg::BoundingSphere SimplePager::getBounds(const TileKey& key) const
{
    int samples = 6;

    GeoExtent extent = key.getExtent();

    double xSample = extent.width() / (double)samples;
    double ySample = extent.height() / (double)samples;

    osg::BoundingSphere bs;
    for (int c = 0; c < samples+1; c++)
    {
        double x = extent.xMin() + (double)c * xSample;
        for (int r = 0; r < samples+1; r++)
        {
            double y = extent.yMin() + (double)r * ySample;
            osg::Vec3d world;

            GeoPoint samplePoint(extent.getSRS(), x, y, 0, ALTMODE_ABSOLUTE);

            GeoPoint wgs84 = samplePoint.transform(osgEarth::SpatialReference::create("epsg:4326"));
            wgs84.toWorld(world);
            bs.expandBy(world);
        }
    }
    return bs;
}

osg::ref_ptr<osg::Node> SimplePager::buildRootNode()
{
    osg::ref_ptr<osg::Group> root = new PagingManager();

    std::vector<TileKey> keys;
    _profile->getRootKeys( keys );
    osg::ref_ptr<ProgressCallback> prog = new ObserverProgressCallback(this);
    for (unsigned int i = 0; i < keys.size(); i++)
    {
        osg::ref_ptr<osg::Node> node = createPagedNode(keys[i], prog.get());
        if ( node.valid() )
            root->addChild( node );
    }

    return root;
}

osg::ref_ptr<osg::Node> SimplePager::createNode(const TileKey& key, ProgressCallback* progress)
{
    osg::BoundingSphere bounds = getBounds( key );

    osg::MatrixTransform* mt = new osg::MatrixTransform;
    mt->setMatrix(osg::Matrixd::translate( bounds.center() ) );
    osg::Geode* geode = new osg::Geode;
    osg::ShapeDrawable* sd = new osg::ShapeDrawable( new osg::Sphere(osg::Vec3f(0,0,0), bounds.radius()) );
    sd->setColor( osg::Vec4(1,0,0,1 ) );
    geode->addDrawable( sd );
    mt->addChild(geode);
    return mt;
}

osg::ref_ptr<osg::Node>
SimplePager::createPagedNode(const TileKey& key, ProgressCallback* progress)
{
    osg::BoundingSphere tileBounds = getBounds(key);
    float tileRadius = tileBounds.radius();

    // restrict subdivision to max level:
    bool hasChildren = key.getLOD() < _maxLevel;

    // Create the actual data for this tile.
    osg::ref_ptr<osg::Node> node;

    // only create real node if we are at least at the min LOD:
    if (key.getLOD() >= _minLevel)
    {
        node = createNode(key, progress);
        hasChildren = node.valid();
    }

    osg::ref_ptr<PagedNode2> pagedNode = new PagedNode2();
    pagedNode->setSceneGraphCallbacks(getSceneGraphCallbacks());

    if (node.valid())
    {
        pagedNode->addChild(node);
        fire_onCreateNode(key, node.get());
    }

    tileRadius = osg::maximum(
        tileBounds.radius(),
        static_cast<osg::BoundingSphere::value_type>(tileRadius));

    pagedNode->setCenter(tileBounds.center());
    pagedNode->setRadius(tileRadius);

    // Assume geocentric for now.
    if (true)
    {
        const GeoExtent& ccExtent = key.getExtent();
        if (ccExtent.isValid())
        {
            // if the extent is more than 90 degrees, bail
            GeoExtent geodeticExtent = ccExtent.transform(ccExtent.getSRS()->getGeographicSRS());
            if (geodeticExtent.width() < 90.0 && geodeticExtent.height() < 90.0)
            {
                // get the geocentric tile center:
                osg::Vec3d tileCenter;
                ccExtent.getCentroid(tileCenter.x(), tileCenter.y());

                osg::Vec3d centerECEF;
                const SpatialReference* mapSRS = osgEarth::SpatialReference::get("epsg:4326");
                if (mapSRS)
                {
                    ccExtent.getSRS()->transform(tileCenter, mapSRS->getGeocentricSRS(), centerECEF);
                    osg::NodeCallback* ccc = ClusterCullingFactory::create(geodeticExtent);
                    if (ccc)
                        pagedNode->addCullCallback(ccc);
                }
            }
        }
    }

    float loadRange = FLT_MAX;

    if (hasChildren)
    {
        if (getName().empty())
            pagedNode->setName(key.str());
        else
            pagedNode->setName(getName() + " " + key.str());

        // Now setup a filename on the PagedLOD that will load all of the children of this node.
        pagedNode->setPriorityScale(_priorityScale);
        //pager->setPriorityOffset(_priorityOffset);

        osg::observer_ptr<SimplePager> pager_weakptr(this);
        pagedNode->setLoadFunction(
            [pager_weakptr, key](Cancelable* c)
            {
                osg::ref_ptr<osg::Node> result;
                osg::ref_ptr<SimplePager> pager;
                if (pager_weakptr.lock(pager))
                {
                    osg::ref_ptr<ProgressCallback> progress = new ProgressCallback(c);
                    result = pager->loadKey(key, progress.get());
                }
                else
                {
                    OE_WARN << "Task canceled!" << std::endl;
                }
                return result;
            }
        );

        loadRange = (float)(tileRadius * _rangeFactor);
        pagedNode->setRefinePolicy(_additive ? pagedNode->REFINE_ADD : pagedNode->REFINE_REPLACE);
    }

    pagedNode->setMaxRange(loadRange);

    //OE_INFO << "PagedNode2: key="<<key.str()<<" hasChildren=" << hasChildren << ", range=" << loadRange << std::endl;

    return pagedNode;
}

osg::ref_ptr<osg::Node>
SimplePager::loadKey(const TileKey& key, ProgressCallback* progress)
{
    if (_done)
    {
        if (progress) progress->cancel();
        return nullptr;
    }

    osg::ref_ptr< osg::Group >  group = new osg::Group;

    for (unsigned int i = 0; i < 4; i++)
    {
        TileKey childKey = key.createChildKey( i );

        osg::ref_ptr<osg::Node> plod = createPagedNode(childKey, progress);
        if (plod.valid())
        {
            group->addChild( plod );
        }
    }
    if (group->getNumChildren() > 0)
    {
        return group;
    }
    return nullptr;
}

const osgEarth::Profile* SimplePager::getProfile() const
{
    return _profile.get();
}

void SimplePager::addCallback(Callback* callback)
{
    if (callback)
    {
        Threading::ScopedMutexLock lock(_mutex);
        _callbacks.push_back(callback);
    }
}

void SimplePager::removeCallback(Callback* callback)
{
    if (callback)
    {
        Threading::ScopedMutexLock lock(_mutex);
        for (Callbacks::iterator i = _callbacks.begin(); i != _callbacks.end(); ++i)
        {
            if (i->get() == callback)
            {
                _callbacks.erase(i);
                break;
            }
        }
    }
}

void SimplePager::fire_onCreateNode(const TileKey& key, osg::Node* node)
{
    Threading::ScopedMutexLock lock(_mutex);
    for (Callbacks::iterator i = _callbacks.begin(); i != _callbacks.end(); ++i)
        i->get()->onCreateNode(key, node);
}