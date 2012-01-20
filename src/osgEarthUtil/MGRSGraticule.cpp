/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2010 Pelican Mapping
 * http://osgearth.org
 *
 * osgEarth is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */
#include <osgEarthUtil/MGRSGraticule>
#include <osgEarthUtil/Formatters>

#include <osgEarthFeatures/GeometryCompiler>
#include <osgEarthFeatures/TextSymbolizer>

#include <osgEarth/ECEF>

#include <osg/BlendFunc>
#include <osg/PagedLOD>
#include <osg/Depth>
#include <osg/LogicOp>
#include <osg/MatrixTransform>
#include <osgDB/FileNameUtils>
#include <osgDB/ReaderWriter>


#define LC "[MGRSGraticule] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

#define MGRS_GRATICULE_EXTENSION "osgearthutil_mgrs_graticule"

//---------------------------------------------------------------------------

MGRSGraticuleOptions::MGRSGraticuleOptions( const Config& conf ) :
UTMGraticuleOptions( conf )
{
    mergeConfig( _conf );
}

void
MGRSGraticuleOptions::mergeConfig( const Config& conf )
{
    //todo
}

Config
MGRSGraticuleOptions::getConfig() const
{
    Config conf = UTMGraticuleOptions::newConfig();
    conf.key() = "mgrs_graticule";
    //todo
    return conf;
}

//---------------------------------------------------------------------------


MGRSGraticule::MGRSGraticule( MapNode* mapNode ) :
UTMGraticule( 0L )
{
    _mapNode = mapNode;
    init();

    if ( !_options->secondaryStyle().isSet() )
    {
        LineSymbol* line = _options->secondaryStyle()->getOrCreate<LineSymbol>();
        line->stroke()->color() = Color(Color::Yellow, 0.4f);
        line->stroke()->stipple() = 0x1111;

        AltitudeSymbol* alt = _options->secondaryStyle()->getOrCreate<AltitudeSymbol>();
        alt->verticalOffset() = 5000.0;

        TextSymbol* text = _options->secondaryStyle()->getOrCreate<TextSymbol>();
        text->fill()->color() = Color(Color::White, 0.3f);
        text->halo()->color() = Color(Color::Black, 0.1f);
        text->alignment() = TextSymbol::ALIGN_CENTER_CENTER;
    }
}

MGRSGraticule::MGRSGraticule( MapNode* mapNode, const MGRSGraticuleOptions& options ) :
UTMGraticule( 0L, options )
{
    _mapNode = mapNode;
    init();
}

osg::Group*
MGRSGraticule::buildGZDChildren( osg::Group* parent, const std::string& gzd )
{
    osg::BoundingSphere bs = parent->getBound();

    std::string uri = Stringify() << gzd << "." << getID() << "." << MGRS_GRATICULE_EXTENSION;

    osg::PagedLOD* plod = new osg::PagedLOD();
    plod->setCenter( bs.center() );
    plod->addChild( parent, 0.0, FLT_MAX );
    plod->setFileName( 1, uri );
    plod->setRange( 1, 0, bs.radius() * 10.0 );

    return plod;
}

osg::Node*
MGRSGraticule::buildSQIDTiles( const std::string& gzd )
{
    const GeoExtent& extent = _gzd[gzd];

    // parse the GZD into its components:
    unsigned zone;
    char letter;
    sscanf( gzd.c_str(), "%u%c", &zone, &letter );
    
    TextSymbol* textSym = _options->secondaryStyle()->get<TextSymbol>();
    if ( !textSym )
        textSym = _options->primaryStyle()->getOrCreate<TextSymbol>();

    AltitudeSymbol* alt = _options->secondaryStyle()->get<AltitudeSymbol>();
    double h = alt ? alt->verticalOffset()->eval() : 5000.0;

    TextSymbolizer ts( textSym );
    MGRSFormatter mgrs(MGRSFormatter::PRECISION_100000M);
    osg::Geode* textGeode = new osg::Geode();
    textGeode->getOrCreateStateSet()->setRenderBinDetails( 9999, "DepthSortedBin" );    
    textGeode->getOrCreateStateSet()->setAttributeAndModes( new osg::Depth(osg::Depth::ALWAYS,0,1,false), 1 );

    osg::Vec3d centerMap, centerECEF;
    extent.getCentroid(centerMap.x(), centerMap.y());
    extent.getSRS()->transformToECEF(centerMap, centerECEF);

    osg::Matrix local2world = ECEF::createLocalToWorld(centerECEF);
    osg::Matrix world2local;
    world2local.invert(local2world);

    FeatureList features;

    // UTM:
    if ( letter > 'B' && letter < 'Y' )
    {
        // grab the SRS for the current UTM zone:
        // TODO: AL/AA designation??
        const SpatialReference* utm = SpatialReference::create(
            Stringify() << "+proj=utm +zone=" << zone << " +north +units=m" );

        // a UTM extent that encompasses the tile extent:
        GeoExtent extentUTM = extent.transform( utm );

        // find the southern boundary of the first full SQID tile in the GZD tile.
        double southSQIDBoundary = extentUTM.yMin();
        double remainder = fmod( southSQIDBoundary, 100000.0 );
        if ( remainder > 0.0 )
            southSQIDBoundary += (100000.0 - remainder);

        std::vector<GeoExtent> sqidExtents;

        // Record the UTM extent of each SQID cell in this tile.
        // Go from the south boundary northwards:
        for( double y = southSQIDBoundary; y < extentUTM.yMax(); y += 100000.0 )
        {
            // start at the central meridian (500K) and go west:
            for( double x = 500000.0; x > extentUTM.xMin(); x -= 100000.0 )
            {
                sqidExtents.push_back( GeoExtent(utm, x-100000.0, y, x, y+100000.0) );
            }

            // then start at the central meridian and go east:
            for( double x = 500000.0; x < extentUTM.xMax(); x += 100000.0 )
            {
                sqidExtents.push_back( GeoExtent(utm, x, y, x+100000.0, y+100000.0) );
            }
        }

        // transform the four corners of the tile to UTM.
        osg::Vec3d gzdUtmSW, gzdUtmSE, gzdUtmNW, gzdUtmNE;
        extent.getSRS()->transform( osg::Vec3d(extent.xMin(),extent.yMin(),h), utm, gzdUtmSW );
        extent.getSRS()->transform( osg::Vec3d(extent.xMin(),extent.yMax(),h), utm, gzdUtmNW );
        extent.getSRS()->transform( osg::Vec3d(extent.xMax(),extent.yMin(),h), utm, gzdUtmSE );
        extent.getSRS()->transform( osg::Vec3d(extent.xMax(),extent.yMax(),h), utm, gzdUtmNE );

        for( std::vector<GeoExtent>::iterator i = sqidExtents.begin(); i != sqidExtents.end(); ++i )
        {
            GeoExtent utmEx = *i;

            // now, clamp each of the points in the UTM SQID extent to the map-space
            // boundaries of the GZD tile. (We only need to clamp in the X dimension,
            // Y geometry is allowed to overflow.) Also, skip NE, we don't need it.
            double r, xlimit;

            osg::Vec3d sw(utmEx.xMin(), utmEx.yMin(), 0);
            r = (sw.y()-gzdUtmSW.y())/(gzdUtmNW.y()-gzdUtmSW.y());
            xlimit = gzdUtmSW.x() + r * (gzdUtmNW.x() - gzdUtmSW.x());
            if ( sw.x() < xlimit ) sw.x() = xlimit;

            osg::Vec3d nw(utmEx.xMin(), utmEx.yMax(), 0);
            r = (nw.y()-gzdUtmSW.y())/(gzdUtmNW.y()-gzdUtmSW.y());
            xlimit = gzdUtmSW.x() + r * (gzdUtmNW.x() - gzdUtmSW.x());
            if ( nw.x() < xlimit ) nw.x() = xlimit;
            
            osg::Vec3d se(utmEx.xMax(), utmEx.yMin(), 0);
            r = (se.y()-gzdUtmSE.y())/(gzdUtmNE.y()-gzdUtmSE.y());
            xlimit = gzdUtmSE.x() + r * (gzdUtmNE.x() - gzdUtmSE.x());
            if ( se.x() > xlimit ) se.x() = xlimit;

            // at the northernmost GZD (lateral band X), clamp the northernmost SQIDs to the upper latitude.
            if ( letter == 'X' && nw.y() > gzdUtmNW.y() ) 
                nw.y() = gzdUtmNW.y();

            // need this in order to calculate the font size:
            double utmWidth = se.x() - sw.x();

            // now transform the corner points back into the map SRS:
            utm->transform( sw, extent.getSRS(), sw );
            utm->transform( nw, extent.getSRS(), nw );
            utm->transform( se, extent.getSRS(), se );

            // and draw valid sqid geometry.
            if ( sw.x() < se.x() )
            {
                Feature* lat = new Feature(new LineString(2));
                lat->geoInterp() = GEOINTERP_RHUMB_LINE;
                lat->getGeometry()->push_back( sw );
                lat->getGeometry()->push_back( se );
                features.push_back(lat);

                Feature* lon = new Feature(new LineString(2));
                lon->geoInterp() = GEOINTERP_GREAT_CIRCLE;
                lon->getGeometry()->push_back( sw );
                lon->getGeometry()->push_back( nw );
                features.push_back(lon);

                // and the text label:
                osg::Vec3d sqidTextMap = (nw + se) * 0.5;
                sqidTextMap.z() += 1000.0;
                osg::Vec3d sqidTextECEF;
                extent.getSRS()->transformToECEF(sqidTextMap, sqidTextECEF);
                osg::Vec3d sqidLocal;
                sqidLocal = sqidTextECEF * world2local;

                MGRSCoord mgrsCoord;
                if ( mgrs.transform(sqidTextMap, extent.getSRS(), mgrsCoord) )
                {
                    textSym->size() = utmWidth/3.0;        
                    osgText::Text* d = ts.create( mgrsCoord.sqid );

                    osg::Matrixd textLocal2World = ECEF::createLocalToWorld( sqidTextECEF );

                    d->setPosition( sqidLocal );
                    textGeode->addDrawable( d );
                }
            }
        }
    }

    osg::Group* group = new osg::Group();

    Style lineStyle;
    lineStyle.add( _options->secondaryStyle()->get<LineSymbol>() );
    lineStyle.add( _options->secondaryStyle()->get<AltitudeSymbol>() );

    GeometryCompiler compiler;
    osg::ref_ptr<Session> session = new Session( _mapNode->getMap() );
    FilterContext context( session.get(), _featureProfile.get(), extent );

    // make sure we get sufficient tessellation:
    compiler.options().maxGranularity() = 0.25;

    osg::Node* geomNode = compiler.compile(features, lineStyle, context);
    if ( geomNode ) 
        group->addChild( geomNode );

    osg::MatrixTransform* mt = new osg::MatrixTransform(local2world); //osg::Matrix::translate(centerECEF)); //ECEF::createLocalToWorld(centerECEF));
    mt->addChild(textGeode);
    group->addChild( mt );

    return group;
}


//---------------------------------------------------------------------------

namespace osgEarth { namespace Util
{
    // OSG Plugin for loading subsequent graticule levels
    class MGRSGraticuleFactory : public osgDB::ReaderWriter
    {
    public:
        virtual const char* className()
        {
            supportsExtension( MGRS_GRATICULE_EXTENSION, "osgEarth MGRS graticule" );
            return "osgEarth MGRS graticule LOD loader";
        }

        virtual bool acceptsExtension(const std::string& extension) const
        {
            return osgDB::equalCaseInsensitive(extension, MGRS_GRATICULE_EXTENSION);
        }

        virtual ReadResult readNode(const std::string& uri, const Options* options) const
        {        
            std::string ext = osgDB::getFileExtension( uri );
            if ( !acceptsExtension( ext ) )
                return ReadResult::FILE_NOT_HANDLED;

            // the graticule definition is formatted: LEVEL_ID.MARKER.EXTENSION
            std::string def = osgDB::getNameLessExtension( uri );
            
            std::string idStr = osgDB::getFileExtension(def);
            unsigned id;
            sscanf(idStr.c_str(), "%u", &id);

            std::string gzd = osgDB::getNameLessExtension(def);

            // look up the graticule referenced in the location name:
            MGRSGraticule* graticule = 0L;
            {
                Threading::ScopedMutexLock lock( UTMGraticule::s_graticuleMutex );
                UTMGraticule::UTMGraticuleRegistry::iterator i = UTMGraticule::s_graticuleRegistry.find(id);
                if ( i != UTMGraticule::s_graticuleRegistry.end() )
                    graticule = dynamic_cast<MGRSGraticule*>( i->second.get() );
            }

            osg::Node* result = graticule->buildSQIDTiles( gzd );
            return result ? ReadResult(result) : ReadResult::ERROR_IN_READING_FILE;
        }
    };
    REGISTER_OSGPLUGIN(MGRS_GRATICULE_EXTENSION, MGRSGraticuleFactory)

} } // namespace osgEarth::Util


