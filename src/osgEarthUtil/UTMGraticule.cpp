/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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
#include <osgEarthUtil/UTMGraticule>

#include <osgEarthFeatures/GeometryCompiler>
#include <osgEarthFeatures/TextSymbolizer>

#include <osgEarth/Registry>
#include <osgEarth/NodeUtils>
#include <osgEarth/Utils>
#include <osgEarth/CullingUtils>
#include <osgEarth/ThreadingUtils>

#include <OpenThreads/Mutex>
#include <OpenThreads/ScopedLock>
#include <osg/PagedLOD>
#include <osg/Depth>
#include <osg/Program>
#include <osg/ClipNode>
#include <osg/ClipPlane>
#include <osgDB/FileNameUtils>

#define LC "[UTMGraticule] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

REGISTER_OSGEARTH_LAYER(utm_graticule, UTMGraticule);

//---------------------------------------------------------------------------

void
UTMData::rebuild(const Profile* profile)
{
    // build the base Grid Zone Designator (GZD) loolup table. This is a table
    // that maps the GZD string to its extent.
    static std::string s_gzdRows( "CDEFGHJKLMNPQRSTUVWX" );
    const SpatialReference* geosrs = profile->getSRS()->getGeographicSRS();

    // build the lateral zones:
    for( unsigned zone = 0; zone < 60; ++zone )
    {
        for( unsigned row = 0; row < s_gzdRows.size(); ++row )
        {
            double yMaxExtra = row == s_gzdRows.size()-1 ? 4.0 : 0.0; // extra 4 deg for row X

            GeoExtent cellExtent(
                geosrs,
                -180.0 + double(zone)*6.0,
                -80.0  + row*8.0,
                -180.0 + double(zone+1)*6.0,
                -80.0  + double(row+1)*8.0 + yMaxExtra );

            _gzd[ Stringify() << (zone+1) << s_gzdRows[row] ] = cellExtent;
        }        
    }

    // the polar zones (UPS):
    _gzd["1Y"] = GeoExtent( geosrs, -180.0,  84.0,   0.0,  90.0 );
    _gzd["1Z"] = GeoExtent( geosrs,    0.0,  84.0, 180.0,  90.0 );
    _gzd["1A"] = GeoExtent( geosrs, -180.0, -90.0,   0.0, -80.0 );
    _gzd["1B"] = GeoExtent( geosrs,    0.0, -90.0, 180.0, -80.0 );

    // replace the "exception" zones in Norway and Svalbard
    _gzd["31V"] = GeoExtent( geosrs, 0.0, 56.0, 3.0, 64.0 );
    _gzd["32V"] = GeoExtent( geosrs, 3.0, 56.0, 12.0, 64.0 );
    _gzd["31X"] = GeoExtent( geosrs, 0.0, 72.0, 9.0, 84.0 );
    _gzd["33X"] = GeoExtent( geosrs, 9.0, 72.0, 21.0, 84.0 );
    _gzd["35X"] = GeoExtent( geosrs, 21.0, 72.0, 33.0, 84.0 );
    _gzd["37X"] = GeoExtent( geosrs, 33.0, 72.0, 42.0, 84.0 );

    // ..and remove the non-existant zones:
    _gzd.erase( "32X" );
    _gzd.erase( "34X" );
    _gzd.erase( "36X" );
}

osg::Group*
UTMData::buildGZDTile(const std::string& name, const GeoExtent& extent, const Style& style, const FeatureProfile* featureProfile, const Map* map)
{
    osg::Group* group = new osg::Group();

    Style lineStyle;
    lineStyle.add( const_cast<LineSymbol*>(style.get<LineSymbol>()) );
    lineStyle.add( const_cast<AltitudeSymbol*>(style.get<AltitudeSymbol>()) );

    bool hasText = style.get<TextSymbol>() != 0L;

    GeometryCompiler compiler;
    osg::ref_ptr<Session> session = new Session(map);
    FilterContext context( session.get(), featureProfile, extent );

    // make sure we get sufficient tessellation:
    compiler.options().maxGranularity() = 1.0;

    FeatureList features;

    // longitudinal line:
    LineString* lon = new LineString(2);
    lon->push_back( osg::Vec3d(extent.xMin(), extent.yMax(), 0) );
    lon->push_back( osg::Vec3d(extent.xMin(), extent.yMin(), 0) );
    Feature* lonFeature = new Feature(lon, extent.getSRS());
    lonFeature->geoInterp() = GEOINTERP_GREAT_CIRCLE;
    features.push_back( lonFeature );

    // latitudinal line:
    LineString* lat = new LineString(2);
    lat->push_back( osg::Vec3d(extent.xMin(), extent.yMin(), 0) );
    lat->push_back( osg::Vec3d(extent.xMax(), extent.yMin(), 0) );
    Feature* latFeature = new Feature(lat, extent.getSRS());
    latFeature->geoInterp() = GEOINTERP_RHUMB_LINE;
    features.push_back( latFeature );

    // top lat line at 84N
    if ( extent.yMax() == 84.0 )
    {
        LineString* lat = new LineString(2);
        lat->push_back( osg::Vec3d(extent.xMin(), extent.yMax(), 0) );
        lat->push_back( osg::Vec3d(extent.xMax(), extent.yMax(), 0) );
        Feature* latFeature = new Feature(lat, extent.getSRS());
        latFeature->geoInterp() = GEOINTERP_RHUMB_LINE;
        features.push_back( latFeature );
    }

    osg::Node* geomNode = compiler.compile(features, lineStyle, context);
    if ( geomNode ) 
        group->addChild( geomNode );

    // get the geocentric tile center:
    osg::Vec3d tileCenter;
    extent.getCentroid( tileCenter.x(), tileCenter.y() );

    const SpatialReference* ecefSRS = extent.getSRS()->getECEF();
    
    osg::Vec3d centerECEF;
    extent.getSRS()->transform( tileCenter, ecefSRS, centerECEF );

    if ( hasText )
    {
        osg::Vec3d west, east;
        extent.getSRS()->transform( osg::Vec3d(extent.xMin(),tileCenter.y(),0), ecefSRS, west );
        extent.getSRS()->transform( osg::Vec3d(extent.xMax(),tileCenter.y(),0), ecefSRS, east );

        const TextSymbol* textSym_in = style.get<TextSymbol>();
        osg::ref_ptr<TextSymbol> textSym = textSym_in ? new TextSymbol(*textSym_in) : new TextSymbol();
        textSym->size() = (west-east).length() / 3.0;

        TextSymbolizer ts(textSym.get());
        
        osg::Geode* textGeode = new osg::Geode();        
        osg::Drawable* d = ts.create(name);
        d->getOrCreateStateSet()->setRenderBinToInherit();
        textGeode->addDrawable(d);
        Registry::shaderGenerator().run(textGeode, Registry::stateSetCache());

        osg::Matrixd centerL2W;
        ecefSRS->createLocalToWorld( centerECEF, centerL2W );
        osg::MatrixTransform* mt = new osg::MatrixTransform(centerL2W);
        mt->addChild(textGeode);
       
        group->addChild(mt);
    }

    //group = buildGZDChildren( group, name );

    group = ClusterCullingFactory::createAndInstall( group, centerECEF )->asGroup();

    return group;
}

//---------------------------------------------------------------------------


UTMGraticule::UTMGraticule() :
VisibleLayer(&_optionsConcrete),
_options(&_optionsConcrete)
{
    init();
}

UTMGraticule::UTMGraticule(const UTMGraticuleOptions& options) :
VisibleLayer(&_optionsConcrete),
_options(&_optionsConcrete),
_optionsConcrete(options)
{
    init();
}

void
UTMGraticule::dirty()
{
    rebuild();
}

void
UTMGraticule::init()
{
    VisibleLayer::init();

    // make the shared depth attr:
    this->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, 0);

    // force it to render after the terrain.
    this->getOrCreateStateSet()->setRenderBinDetails(1, "RenderBin");
}

void
UTMGraticule::addedToMap(const Map* map)
{
    _map = map;
    rebuild();
}

void
UTMGraticule::removedFromMap(const Map* map)
{
    _map = 0L;
}

osg::Node*
UTMGraticule::getOrCreateNode()
{
    if (_root.valid() == false)
    {
        _root = new osg::Group();

        // install the range callback for clip plane activation
        _root->addCullCallback( new RangeUniformCullCallback() );

        rebuild();
    }

    return _root.get();
}

void
UTMGraticule::rebuild()
{
    if (_root.valid() == false)
        return;

    osg::ref_ptr<const Map> map;
    if (!_map.lock(map))
        return;

    // clear everything out
    _root->removeChildren( 0, _root->getNumChildren() );

    // requires a geocentric map
    if ( !map->isGeocentric() )
    {
        OE_WARN << LC << "Projected map mode is not yet supported" << std::endl;
        return;
    }

    const Profile* mapProfile = map->getProfile();

    _profile = Profile::create(
        mapProfile->getSRS(),
        mapProfile->getExtent().xMin(),
        mapProfile->getExtent().yMin(),
        mapProfile->getExtent().xMax(),
        mapProfile->getExtent().yMax(),
        8, 4 );

    _featureProfile = new FeatureProfile(_profile->getSRS());

    //todo: do this right..
    osg::StateSet* set = this->getOrCreateStateSet();
    set->setMode( GL_LIGHTING, 0 );
    set->setMode( GL_BLEND, 1 );

    // set up default options if the caller did not supply them
    if ( !options().gzdStyle().isSet() )
    {
        options().gzdStyle() = Style();

        LineSymbol* line = options().gzdStyle()->getOrCreate<LineSymbol>();
        line->stroke()->color() = Color::Gray;
        line->stroke()->width() = 1.0;
        line->tessellation() = 20;

        TextSymbol* text = options().gzdStyle()->getOrCreate<TextSymbol>();
        text->fill()->color() = Color(Color::White, 0.3f);
        text->halo()->color() = Color(Color::Black, 0.2f);
        text->alignment() = TextSymbol::ALIGN_CENTER_CENTER;
    }
    
    // rebuild the graph:
    osg::Group* top = _root.get();

    // Horizon clipping plane.
    osg::ClipPlane* cp = _clipPlane.get();
    if ( cp == 0L )
    {
        osg::ClipNode* clipNode = new osg::ClipNode();
        osgEarth::Registry::shaderGenerator().run( clipNode );
        cp = new osg::ClipPlane( 0 );
        clipNode->addClipPlane( cp );
        _root->addChild(clipNode);
        top = clipNode;
    }
    top->addCullCallback( new ClipToGeocentricHorizon(_profile->getSRS(), cp) );
    
    // intialize the UTM sector tables for this profile.
    _utmData.rebuild(_profile.get());

    // now build the lateral tiles for the GZD level.
    for( UTMData::SectorTable::iterator i = _utmData.sectorTable().begin(); i != _utmData.sectorTable().end(); ++i )
    {
        osg::Node* tile = _utmData.buildGZDTile(i->first, i->second, options().gzdStyle().get(), _featureProfile.get(), map.get());
        if ( tile )
            _root->addChild( tile );
    }
}
