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
TileBuilder::create( MapConfig* map, const std::string& url_template )
{
    TileBuilder* result = NULL;
    if ( map )
    {
        if ( map->getCoordinateSystemType() == MapConfig::CSTYPE_GEOCENTRIC )
        {
            result = new GeocentricTileBuilder( map, url_template );
        }
        else
        {
            result = new PlateCarreTileBuilder( map, url_template );
        }
    }
    return result;
}

static void
addSources(const SourceConfigList& from, 
           std::vector< osg::ref_ptr<PlateCarreTileSource> >& to,
           const std::string& url_template)
{        
    for( SourceConfigList::const_iterator i = from.begin(); i != from.end(); i++ )
    {
        SourceConfig* source = i->get();
        osgDB::ReaderWriter::Options* options = new osgDB::ReaderWriter::Options();
        for( SourceProperties::const_iterator p = source->getProperties().begin(); p != source->getProperties().end(); p++ )
        {
            options->setPluginData( p->first, (void*)p->second.c_str() );
            PlateCarreTileSource* tile_source = new ReaderWriterPlateCarreTileSource( source->getDriver(), options );
            to.push_back( tile_source );
        }
    }
}

TileBuilder::TileBuilder( MapConfig* _map, const std::string& _url_template ) :
map( _map ),
url_template( _url_template )
{
    if ( map.valid() )
    {
        addSources( map->getImageSources(), image_sources, url_template );
        addSources( map->getHeightFieldSources(), heightfield_sources, url_template );
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