#include <osgEarthUtil/SimplePager> 
#include <osgEarth/TileKey>
#include <osgDB/Registry>
#include <osgDB/FileNameUtils>
#include <osgDB/Options>
#include <osg/UserDataContainer>
#include <osg/ShapeDrawable>
#include <osg/MatrixTransform>
#include <osg/Geode>

using namespace osgEarth::Util;

#define LC "[SimplerPager] "

namespace
{
    /**
     * The master progress tracker keeps track of the current framestamp
     * for the entire paged scene graph.
     */
    struct ProgressMaster : public osg::NodeCallback
    {
        unsigned _frame;

        void operator()(osg::Node* node, osg::NodeVisitor* nv)
        {
            _frame = nv->getFrameStamp() ? nv->getFrameStamp()->getFrameNumber() : 0u;
            traverse(node, nv);
        }
    };

    /**
     * The ProgressCallback that gets passed to createNode() for the subclass
     * to use. It will report cancelation if the last reported frame number is
     * behind the current master frame number (as tracked by the ProgressMaster)
     */
    struct MyProgressCallback : public ProgressCallback
    {
        MyProgressCallback(ProgressMaster* master)
        {
            _master = master;
            _lastFrame = 0u;
        }

        // override from ProgressCallback
        bool isCanceled()
        {
            return (!_master.valid()) || (_master->_frame - _lastFrame > 1u);
        }

        // called by ProgressUpdater
        void touch(const osg::FrameStamp* stamp)
        {
            if ( stamp )
                _lastFrame = stamp->getFrameNumber();
        }

        unsigned _lastFrame;
        osg::observer_ptr<ProgressMaster> _master;
    };


    /**
    * A pseudo-loader for paged feature tiles.
    */
    struct SimplePagerPseudoLoader : public osgDB::ReaderWriter
    {
        SimplePagerPseudoLoader()
        {
            supportsExtension( "osgearth_pseudo_simple", "" );
        }

        const char* className() const
        { // override
            return "Simple Pager";
        }

        ReadResult readNode(const std::string& uri, const Options* options) const
        {
            if ( !acceptsExtension( osgDB::getLowerCaseFileExtension(uri) ) )
                return ReadResult::FILE_NOT_HANDLED;

            unsigned lod, x, y;
            sscanf( uri.c_str(), "%d_%d_%d.%*s", &lod, &x, &y );


            SimplePager* pager =
                dynamic_cast<SimplePager*>(
                    const_cast<osg::Object*>(
                        options->getUserDataContainer()->getUserObject("osgEarth::Util::SimplerPager::this")));
            
            if (pager)
            {
                SimplePager::ProgressTracker* tracker =
                    dynamic_cast<SimplePager::ProgressTracker*>(
                        const_cast<osg::Object*>(
                            options->getUserDataContainer()->getUserObject("osgEarth::Util::SimplerPager::ProgressTracker")));

                return pager->loadKey(
                    TileKey(lod, x, y, pager->getProfile()),
                    tracker);
            }

            return ReadResult::ERROR_IN_READING_FILE;
        }
    };

    REGISTER_OSGPLUGIN(osgearth_pseudo_simple, SimplePagerPseudoLoader);
}


SimplePager::ProgressTracker::ProgressTracker(osg::NodeCallback* master)
{
    setName( "osgEarth::Util::SimplerPager::ProgressTracker" );
    for(int i=0; i<4; ++i)
        _progress[i] = new MyProgressCallback( static_cast<ProgressMaster*>(master) );
}

void SimplePager::ProgressTracker::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
    for(int i=0; i<4; ++i)
        static_cast<MyProgressCallback*>(_progress[i].get())->touch( nv->getFrameStamp() );
    traverse(node, nv);
}


SimplePager::SimplePager(const osgEarth::Profile* profile):
_profile( profile ),
_rangeFactor( 6.0 ),
_additive(false),
_minLevel(0),
_maxLevel(30),
_priorityScale(1.0f),
_priorityOffset(0.0f)
{
    // required in order to pass our "this" pointer to the pseudo loader:
    this->setName( "osgEarth::Util::SimplerPager::this" );
    
    // install the master framestamp tracker:
    _progressMaster = new ProgressMaster();
    addCullCallback( _progressMaster.get() );
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

osg::Node* SimplePager::buildRootNode()
{    
    osg::Group* root = new osg::Group;

    std::vector<TileKey> keys;
    _profile->getRootKeys( keys );
    for (unsigned int i = 0; i < keys.size(); i++)
    {
        osg::Node* node = createPagedNode( keys[i], 0L );
        if ( node )
            root->addChild( node );
    }

    return root;
}

osg::Node* SimplePager::createNode(const TileKey& key, ProgressCallback* progress)
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

osg::Node* SimplePager::createPagedNode(const TileKey& key, ProgressCallback* progress)
{
    osg::BoundingSphere tileBounds = getBounds( key );
    float tileRadius = tileBounds.radius();

    // restrict subdivision to max level:
    bool hasChildren = key.getLOD() < _maxLevel;

    // Create the actual data for this tile.
    osg::ref_ptr<osg::Node> node;

    // only create real node if we are at least at the min LOD:
    if ( key.getLevelOfDetail() >= _minLevel )
    {
        node = createNode( key, progress );

        if ( node.valid() )
        {
            tileBounds = node->getBound();
        }
        else
        {
            hasChildren = false;
        }
    }

    if ( !node.valid() )
    {
        node = new osg::Group();
    }

    // notify any callbacks.
    fire_onCreateNode(key, node.get());

    tileRadius = std::max(tileBounds.radius(), tileRadius);

    osg::PagedLOD* plod = new osg::PagedLOD;
    plod->setCenter( tileBounds.center() ); 
    plod->setRadius( tileRadius );

    plod->addChild( node.get() );

    if ( hasChildren )
    {
        std::stringstream buf;
        buf << key.getLevelOfDetail() << "_" << key.getTileX() << "_" << key.getTileY() << ".osgearth_pseudo_simple";

        std::string uri = buf.str();

        // Now setup a filename on the PagedLOD that will load all of the children of this node.
        plod->setFileName(1, uri);
        plod->setPriorityOffset(1, _priorityOffset);
        plod->setPriorityScale (1, _priorityScale);
        
        // install a callback that will update the progress tracker whenever the PLOD
        // gets traversed. The children, once activated, will have access to its
        // ProgressCallback within and be able to check for cancelation, use stats, etc.
        ProgressTracker* tracker = new ProgressTracker( _progressMaster.get() );
        plod->addCullCallback( tracker );

        // assemble data to pass to the pseudoloader
        osgDB::Options* options = new osgDB::Options();
        options->getOrCreateUserDataContainer()->addUserObject( this );
        options->getOrCreateUserDataContainer()->addUserObject( tracker );
        plod->setDatabaseOptions( options );
        
        // Install an FLC if the caller provided one
        if ( _fileLocationCallback.valid() )
            options->setFileLocationCallback( _fileLocationCallback.get() );

        // Setup the min and max ranges.
        float minRange = (float)(tileRadius * _rangeFactor);

        if (!_additive)
        {
            // Replace mode, the parent is replaced by its children.
            plod->setRange( 0, minRange, FLT_MAX );
            plod->setRange( 1, 0, minRange );
        }
        else
        {
            // Additive, the parent remains and new data is added
            plod->setRange( 0, 0, FLT_MAX );
            plod->setRange( 1, 0, minRange );
        }
    }
    else
    {
        // no children, so max out the visibility range.
        plod->setRange( 0, 0, FLT_MAX );
    }

    return plod;
}


/**
* Loads the PagedLOD hierarchy for this key.
*/
osg::Node* SimplePager::loadKey(const TileKey& key, ProgressTracker* tracker)
{       
    osg::ref_ptr< osg::Group >  group = new osg::Group;

    for (unsigned int i = 0; i < 4; i++)
    {
        TileKey childKey = key.createChildKey( i );

        osg::Node* plod = createPagedNode( childKey, tracker->_progress[i].get() );
        if (plod)
        {
            group->addChild( plod );
        }
    }
    if (group->getNumChildren() > 0)
    {
        return group.release();
    }
    return 0;
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