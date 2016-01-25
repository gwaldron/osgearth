#include <osgEarthUtil/SimplePager> 
#include <osgEarth/TileKey>
#include <osgDB/Registry>
#include <osgDB/FileNameUtils>
#include <osg/ShapeDrawable>
#include <osg/MatrixTransform>
#include <osg/Geode>

using namespace osgEarth::Util;


/**
* A pseudo-loader for paged feature tiles.
*/
struct SimplePagerPseudoLoader : public osgDB::ReaderWriter
{
    SimplePagerPseudoLoader()
    {
        supportsExtension( "osgearth_pseudo_simple", "" );
    }

    const char* className()
    { // override
        return "Simple Pager";
    }

    ReadResult readNode(const std::string& uri, const Options* options) const
    {
        if ( !acceptsExtension( osgDB::getLowerCaseFileExtension(uri) ) )
            return ReadResult::FILE_NOT_HANDLED;

        unsigned lod, x, y;
        sscanf( uri.c_str(), "%d_%d_%d.%*s", &lod, &x, &y );

        SimplePager* pager = dynamic_cast< SimplePager*>(const_cast<Options*>(options)->getUserData());
        if (pager)
        {
            return pager->loadKey( TileKey( lod, x, y, pager->getProfile() ) );
        }

        return ReadResult::ERROR_IN_READING_FILE;
    }
};

REGISTER_OSGPLUGIN(osgearth_pseudo_simple, SimplePagerPseudoLoader);


SimplePager::SimplePager(const osgEarth::Profile* profile):
_profile( profile ),
    _rangeFactor( 6.0 ),
    _additive(false),
    _minLevel(0),
    _maxLevel(30)
{
    _options = new osgDB::Options;
    _options->setUserData( this );          
}

void SimplePager::build()
{
    addChild( buildRootNode() );
}

osg::BoundingSphere SimplePager::getBounds(const TileKey& key)
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
        osg::Node* node = createPagedNode( keys[i] );
        if ( node )
            root->addChild( node );
    }

    return root;
}

osg::Node* SimplePager::createNode( const TileKey& key )
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

osg::Node* SimplePager::createPagedNode( const TileKey& key )
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
        node = createNode( key );

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
        plod->setDatabaseOptions( _options.get() );

        // Setup the min and max ranges.

        // This setups a replacement mode where the parent will be completely replaced by it's children.
        float minRange = (float)(tileRadius * _rangeFactor);

        if (!_additive)
        {
            // Replace mode, the parent is replaced by it's children.
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
osg::Node* SimplePager::loadKey( const TileKey& key )
{       
    osg::ref_ptr< osg::Group >  group = new osg::Group;

    for (unsigned int i = 0; i < 4; i++)
    {
        TileKey childKey = key.createChildKey( i );

        osg::Node* plod = createPagedNode( childKey );
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

const osgEarth::Profile* SimplePager::getProfile()
{
    return _profile.get();
}