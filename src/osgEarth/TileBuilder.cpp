#include <osgEarth/TileBuilder>
#include <osgEarth/PlateCarreTileBuilder>
#include <osgEarth/GeocentricTileBuilder>
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
            result = new PlateCarreTileBuilder( map, url_template, local_options.get() );
        }
    }
    return result;
}

static void
addSources(const SourceConfigList& from, 
           std::vector< osg::ref_ptr<PlateCarreTileSource> >& to,
           const std::string& url_template,
           const osgDB::ReaderWriter::Options* global_options)
{        
    for( SourceConfigList::const_iterator i = from.begin(); i != from.end(); i++ )
    {
        SourceConfig* source = i->get();

        osg::ref_ptr<osgDB::ReaderWriter::Options> local_options = global_options ?
            new osgDB::ReaderWriter::Options( *global_options ) : 
            new osgDB::ReaderWriter::Options();

        for( SourceProperties::const_iterator p = source->getProperties().begin(); p != source->getProperties().end(); p++ )
        {
            local_options->setPluginData( p->first, (void*)p->second.c_str() );
            PlateCarreTileSource* tile_source = new ReaderWriterPlateCarreTileSource( source->getDriver(), local_options.get() );
            to.push_back( tile_source );
        }
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
TileBuilder::createURI( const PlateCarreCellKey& key )
{
    std::stringstream buf;
    buf << key.str() << "." << url_template;
    return buf.str();
}



osg::Node*
TileBuilder::createNode( const PlateCarreCellKey& key )
{
    osg::Group* top;
    osg::Group* tile_parent;

    if ( key.getLevelOfDetail() == 0 )
    {
        osg::CoordinateSystemNode* csn = new osg::CoordinateSystemNode();
        csn->setEllipsoidModel( new osg::EllipsoidModel() );
        csn->setCoordinateSystem( getProj4String() );
        csn->setFormat( "PROJ4" );

        osgTerrain::Terrain* terrain = new osgTerrain::Terrain();
        terrain->setVerticalScale( map->getVerticalScale() );
        csn->addChild( terrain );

        top = csn;
        tile_parent = terrain;
    }
    else
    {
        top = new osg::Group();
        top->setName( key.str() );
        tile_parent = top;
    }

    tile_parent->addChild( createQuadrant( key.getSubkey( 0 ) ) );
    tile_parent->addChild( createQuadrant( key.getSubkey( 1 ) ) );

    if ( key.getLevelOfDetail() > 0 )
    {
        tile_parent->addChild( createQuadrant( key.getSubkey( 2 ) ) );
        tile_parent->addChild( createQuadrant( key.getSubkey( 3 ) ) );
    }
    return top;
}