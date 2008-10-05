#include <osgEarth/TileBuilder>
#include <osgEarth/GeocentricTileBuilder>
#include <osgEarth/GeographicTileBuilder>
#include <osgEarth/PlateCarre>
#include <osg/Image>
#include <osg/Notify>
#include <osg/PagedLOD>
#include <osg/ClusterCullingCallback>
#include <osg/CoordinateSystemNode>
#include <osgDB/ReadFile>
#include <osgTerrain/Terrain>
#include <osgTerrain/TerrainTile>
#include <osgTerrain/Locator>
#include <osgTerrain/GeometryTechnique>
#include <sstream>
#include <stdlib.h>

using namespace osgEarth;


TileBuilder*
TileBuilder::create( MapConfig* map, const std::string& url_template, const osgDB::ReaderWriter::Options* options )
{
    TileBuilder* result = NULL;
    if ( map )
    {
        osg::ref_ptr<osgDB::ReaderWriter::Options> local_options = options ? 
            new osgDB::ReaderWriter::Options( *local_options ) :
            NULL;

        // transcribe proxy settings:
        if ( !map->getProxyHost().empty() )
        {
            if ( !local_options.valid() )
                local_options = new osgDB::ReaderWriter::Options();

            std::stringstream buf;
            buf << local_options->getOptionString()
                << "OSG_CURL_PROXY=" << map->getProxyHost() << " "
                << "OSG_CURL_PROXYPORT=" << map->getProxyPort();
            local_options->setOptionString( buf.str() );
        }

        osg::notify(osg::INFO) 
            << "[osgEarth] TileBuilder: options string = " 
            << (local_options.valid()? local_options->getOptionString() : "<empty>")
            << std::endl;

        if ( map->getCoordinateSystemType() == MapConfig::CSTYPE_GEOCENTRIC )
        {
            result = new GeocentricTileBuilder( map, url_template, local_options.get() );
        }
        else
        {
            result = new GeographicTileBuilder( map, url_template, local_options.get() );
        }
    }
    return result;
}

static void
addSources(const SourceConfigList& from, 
           std::vector< osg::ref_ptr<TileSource> >& to,
           const std::string& url_template,
           const osgDB::ReaderWriter::Options* global_options)
{        
    for( SourceConfigList::const_iterator i = from.begin(); i != from.end(); i++ )
    {
        SourceConfig* source = i->get();

        osg::ref_ptr<osgDB::ReaderWriter::Options> local_options = global_options ?
            new osgDB::ReaderWriter::Options( *global_options ) : 
            new osgDB::ReaderWriter::Options();

        //Setup the plugin options for the source
        for( SourceProperties::const_iterator p = source->getProperties().begin(); p != source->getProperties().end(); p++ )
        {
            local_options->setPluginData( p->first, (void*)p->second.c_str() );
        }

        //Add the source to the list
        TileSource* tile_source = new ReaderWriterTileSource( source->getDriver(), local_options.get() );
        to.push_back( tile_source );
    }
}

TileBuilder::TileBuilder(MapConfig* _map, 
                         const std::string& _url_template,
                         const osgDB::ReaderWriter::Options* options ) :
map( _map ),
url_template( _url_template )
{
    if ( map.valid() )
    {
        addSources( map->getImageSources(), image_sources, url_template, options );
        addSources( map->getHeightFieldSources(), heightfield_sources, url_template, options );
    }
}

std::string
TileBuilder::createURI( const TileKey* key )
{
    return key->getName() + "." + url_template;
    //std::stringstream buf;
    //buf << key->getTypeCode() << key->str() << "." << url_template;
    //return buf.str();
}

MapConfig*
TileBuilder::getMapConfig() const
{
    return map.get();
}


osg::Node*
TileBuilder::createCap(const double &min_lat, const double &max_lat, const osg::Vec4ub &color)
{
    double min_lon = -180.0;
    double max_lon = 180.0;

    osgTerrain::TerrainTile* tile = new osgTerrain::TerrainTile();

    osgTerrain::Locator* locator = new osgTerrain::Locator();
    locator->setCoordinateSystemType( osgTerrain::Locator::GEOCENTRIC );
    locator->setTransformAsExtents(
        osg::DegreesToRadians( min_lon ),
        osg::DegreesToRadians( min_lat ),
        osg::DegreesToRadians( max_lon ),
        osg::DegreesToRadians( max_lat ) );

    osg::HeightField *hf = new osg::HeightField();
    hf->allocate(32,32);
    for(unsigned int i=0; i<hf->getHeightList().size(); i++ ) hf->getHeightList()[i] = 0.0;

    hf->setOrigin( osg::Vec3d( min_lon, min_lat, 0.0 ) );
    hf->setXInterval( (max_lon - min_lon)/(double)(hf->getNumColumns()-1) );
    hf->setYInterval( (max_lat - min_lat)/(double)(hf->getNumRows()-1) );
    hf->setBorderWidth( 0 );

    osgTerrain::HeightFieldLayer* hf_layer = new osgTerrain::HeightFieldLayer();
    hf_layer->setLocator( locator );
    hf_layer->setHeightField( hf );

    osg::Image *image = new osg::Image();
    image->allocateImage(1,1,1, GL_RGBA, GL_UNSIGNED_BYTE);
    unsigned char *data = image->data(0,0);
    memcpy(data, color.ptr(), 4);

    osgTerrain::ImageLayer* img_layer = new osgTerrain::ImageLayer( image );
    img_layer->setLocator( locator );
    img_layer->setFilter( osgTerrain::Layer::LINEAR );
    tile->setColorLayer( 0, img_layer );

    tile->setLocator( locator );
    tile->setTerrainTechnique( new osgTerrain::GeometryTechnique() );
    tile->setElevationLayer( hf_layer );
    tile->setRequiresNormals( true );

    return tile;

}

osg::Node*
TileBuilder::createNode( const TileKey* key )
{
    osg::Group* top;
    osg::Group* tile_parent;

    //osg::notify(osg::NOTICE) << "[osgEarth] TileBuilder::createNode( " << key->str() << ")" << std::endl;

    if ( key->getLevelOfDetail() == 0 )
    {
        osg::CoordinateSystemNode* csn = new osg::CoordinateSystemNode();
        csn->setEllipsoidModel( new osg::EllipsoidModel() );
        csn->setCoordinateSystem( getProj4String() );
        csn->setFormat( "PROJ4" );

        osgTerrain::Terrain* terrain = new osgEarth::EarthTerrain;//new osgTerrain::Terrain();
        terrain->setVerticalScale( map->getVerticalScale() );
        csn->addChild( terrain );

        const osgEarth::TileGridProfile& profile = key->getProfile();
        
        

        //Extend the caps out so they slightly overlap neighboring tiles to hide seams
        double cap_offset = 0.1;
        
        //Draw a "cap" on the bottom of the earth to account for missing tiles
        if (profile.yMin() > -90)
        {
            csn->addChild(createCap(-90, profile.yMin()+cap_offset, map->getSouthCapColor()));
        }

        //Draw a "cap" on the top of the earth to account for missing tiles
        if (profile.yMax() < 90)
        {   
            csn->addChild(createCap(profile.yMax()-cap_offset, 90, map->getNorthCapColor()));
        }

        top = csn;
        tile_parent = terrain;
    }
    else
    {
        top = new osg::Group();
        top->setName( key->str() );
        tile_parent = top;
    }

    addChildren( tile_parent, key );

    return top;
}