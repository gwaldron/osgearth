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
#include <osgEarthUtil/MGRSGraticule>
#include <osgEarthUtil/MGRSFormatter>

#include <osgEarthFeatures/GeometryCompiler>
#include <osgEarthFeatures/TextSymbolizer>
#include <osgEarthFeatures/FeatureSource>

#include <osgEarthAnnotation/FeatureNode>

#include <osgEarth/ECEF>
#include <osgEarth/Registry>
#include <osgEarth/CullingUtils>
#include <osgEarth/Utils>
#include <osgEarth/PagedNode>

#include <osg/BlendFunc>
#include <osg/PagedLOD>
#include <osg/Depth>
#include <osg/LogicOp>
#include <osg/MatrixTransform>
#include <osg/ClipNode>
#include <osgDB/FileNameUtils>
#include <osgDB/ReaderWriter>
#include <osgDB/WriteFile>

#include <osgEarthDrivers/feature_ogr/OGRFeatureOptions>


#define LC "[MGRSGraticule] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;
using namespace osgEarth::Annotation;

#define MGRS_GRATICULE_PSEUDOLOADER_EXTENSION "osgearthutil_mgrs_graticule"


REGISTER_OSGEARTH_LAYER(mgrs_graticule, MGRSGraticule);

//---------------------------------------------------------------------------


MGRSGraticule::MGRSGraticule() :
VisibleLayer(&_optionsConcrete),
_options(&_optionsConcrete)
{
    init();
}

MGRSGraticule::MGRSGraticule(const MGRSGraticuleOptions& options) :
VisibleLayer(&_optionsConcrete),
_options(&_optionsConcrete),
_optionsConcrete(options)
{
    init();
}

void
MGRSGraticule::dirty()
{
    rebuild();
}

void
MGRSGraticule::init()
{
    VisibleLayer::init();

    osg::StateSet* ss = this->getOrCreateStateSet();

    // make the shared depth attr:
    ss->setMode( GL_DEPTH_TEST, 0 );
    ss->setMode( GL_LIGHTING, 0 );
    ss->setMode( GL_BLEND, 1 );

    // force it to render after the terrain.
    ss->setRenderBinDetails(1, "RenderBin");
}

void
MGRSGraticule::addedToMap(const Map* map)
{
    _map = map;
    rebuild();
}

void
MGRSGraticule::removedFromMap(const Map* map)
{
    _map = 0L;
}

osg::Node*
MGRSGraticule::getOrCreateNode()
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

namespace
{
    void findPointClosestTo(const Feature* f, const osg::Vec3d& p1, osg::Vec3d& out)
    {
        out = p1;
        double minLen2 = DBL_MAX;
        const Geometry* g = f->getGeometry();
        ConstGeometryIterator iter(f->getGeometry(), false);
        while (iter.hasMore())
        {
            const Geometry* part = iter.next();
            for (Geometry::const_iterator i = part->begin(); i != part->end(); ++i)
            {
                osg::Vec3d p(i->x(), i->y(), 0);
                double len2 = (p1 - p).length2();
                if (len2 < minLen2)
                {
                    minLen2 = len2, out = *i;
                }
            }
        }
    }

    
    struct Geom10km : public PagedNode
    {
        Geom10km()
        {
            setRangeMode(osg::LOD::PIXEL_SIZE_ON_SCREEN);
            setRange(880);
            setAdditive(false);
        }

        void setupData(Feature* feature, const Style* style)
        {
            _feature = feature;
            _style = *style;
            _style.getOrCreate<LineSymbol>()->stroke()->stipple() = 0x1111;
            setNode(build());
        }

        bool hasChild() const 
        {
            return false;
        }

        osg::Node* build()
        {
            GeoExtent extent(_feature->getSRS(), _feature->getGeometry()->getBounds());
            double lon, lat;
            extent.getCentroid(lon, lat);
            osg::ref_ptr<const SpatialReference> utm = _feature->getSRS()->createUTMFromLonLat(lon, lat);

            double x0 = _feature->getDouble("easting");
            double y0 = _feature->getDouble("northing");

            osg::ref_ptr<MultiGeometry> grid = new MultiGeometry();

            // south-north lines:
            for (double x=x0; x<=x0+100000.0; x += 10000.0)
            {
                LineString* ls = new LineString();
                ls->push_back(osg::Vec3d(x, y0, 0));
                ls->push_back(osg::Vec3d(x, y0+100000.0, 0));
                grid->getComponents().push_back(ls);
            }
            
            // west-east lines:
            for (double y=y0; y<=y0+100000.0; y+=10000.0)
            {
                LineString* ls = new LineString();
                ls->push_back(osg::Vec3d(x0, y, 0));
                ls->push_back(osg::Vec3d(x0+100000.0, y, 0));
                grid->getComponents().push_back(ls);
            }

            //OE_INFO << utm->getName() << std::endl;
            osg::ref_ptr<Feature> f = new Feature(grid.get(), utm);
            f->transform(_feature->getSRS());

            osg::ref_ptr<Geometry> croppedGeom;
            if (f->getGeometry()->crop(extent.bounds(), croppedGeom))
            {
                f->setGeometry(croppedGeom.get());
            }

            GeometryCompilerOptions gco;
            gco.shaderPolicy() = ShaderPolicy::SHADERPOLICY_INHERIT;
            return new FeatureNode(f.get(), _style, gco);
        }

        Feature* _feature;
        Style _style;
    };


    //! Geometry for a single SQID 100km cell and its children
    struct SQID100kmGeom : public PagedNode
    {
        SQID100kmGeom(const std::string& name)
        {
            setName(name);
            setRangeMode(osg::LOD::PIXEL_SIZE_ON_SCREEN);
            setRange(880);
            setAdditive(true);
        }

        void setupData(Feature* feature, const Style* style)
        {
            _feature = feature;
            _style = *style;
            _style.getOrCreate<LineSymbol>()->stroke()->color().set(1,1,0,1);
            _style.getOrCreate<LineSymbol>()->stroke()->stipple().clear();
            setNode( build() );
        }

        osg::Node* loadChild()
        {
            Geom10km* child = new Geom10km();
            child->setupData(_feature, &_style);
            child->setupPaging();
            return child;
        }

        bool hasChild() const
        {
            return true;
        }

        osg::BoundingSphere getChildBound() const
        {
            osg::ref_ptr<Geom10km> child = new Geom10km();
            child->setupData(_feature, &_style);
            return child->getBound();
        }

        osg::Node* build()
        {
            GeometryCompilerOptions gco;
            gco.shaderPolicy() = ShaderPolicy::SHADERPOLICY_INHERIT;
            FeatureNode* node = new FeatureNode(_feature, _style, gco);
            return node;
        }

        osg::ref_ptr<Feature> _feature;
        Style _style;
    };


    //! All SQID 100km goemetry from a single UTM cell combines into one geometry
    struct SQID100kmGeomCombined : public PagedNode
    {
        SQID100kmGeomCombined(const std::string& name, const osg::BoundingSphere& bs)
        {
            setName(name);
            _bs = bs;
            setAdditive(false);
            setRangeMode(osg::LOD::PIXEL_SIZE_ON_SCREEN);
            setRange(3200);
        }

        void setupData(const FeatureList& sqidFeatures, const Style* sqidStyle)
        {
            _sqidFeatures = sqidFeatures;
            _sqidStyle = sqidStyle;
            setNode(build());
        }

        osg::Node* loadChild()
        {
            osg::Group* group = new osg::Group();

            for (FeatureList::const_iterator f = _sqidFeatures.begin(); f != _sqidFeatures.end(); ++f)
            {
                Feature* feature = f->get();
                SQID100kmGeom* geom = new SQID100kmGeom(getName());
                geom->setupData(feature, _sqidStyle);
                geom->setupPaging();
                group->addChild(geom);                
            }

            return group;
        }

        osg::BoundingSphere getChildBound() const
        {
            return getChild(0)->getBound();
        }

        osg::Node* build()
        {
            GeometryCompilerOptions gco;
            gco.shaderPolicy() = ShaderPolicy::SHADERPOLICY_INHERIT;
            
            // remove the text since we'll make text elsewhere
            Style style = *_sqidStyle;
            style.remove<TextSymbol>();

            return new FeatureNode(0L, _sqidFeatures, style, gco);
        }

        osg::BoundingSphere _bs;
        FeatureList _sqidFeatures;
        const Style* _sqidStyle;
    };


    struct GZDGeom : public PagedNode
    {
        const Style* _sqidStyle;
        FeatureList  _sqidFeatures;

        GZDGeom(const std::string& name)
        {
            setName(name);     
            setRangeMode(osg::LOD::PIXEL_SIZE_ON_SCREEN);
            setRange(640);
            setAdditive(false);
        }

        osg::Node* loadChild()
        {
#if 0
            osg::Group* group = new osg::Group();

            for (FeatureList::const_iterator f = _sqidFeatures.begin(); f != _sqidFeatures.end(); ++f)
            {
                Feature* feature = f->get();
                SQID100kmGeom* geom = new SQID100kmGeom(getName());
                geom->setupData(feature, _sqidStyle);
                geom->setupPaging();
                group->addChild(geom);                
            }

            return group;
#else
            SQID100kmGeomCombined* child = new SQID100kmGeomCombined(getName(), getBound());
            child->setupData(_sqidFeatures, _sqidStyle);
            child->setupPaging();
            return child;
#endif
        }

        osg::BoundingSphere getChildBound() const
        {
            return getChild(0)->getBound();
        }

        void setupData(const Feature* gzdFeature, const Style& gzdStyle,
                       const FeatureList& sqidFeatures, const Style& sqidStyle,
                       const FeatureProfile* prof, const Map* map)
        {
            setNode(build(gzdFeature, gzdStyle, prof, map));
            _sqidFeatures = sqidFeatures;
            _sqidStyle = &sqidStyle;
        }

        osg::Node* build(const Feature* f, const Style& style, const FeatureProfile* prof, const Map* map)
        {
            osg::Group* group = new osg::Group();

            Style lineStyle;
            lineStyle.add( const_cast<LineSymbol*>(style.get<LineSymbol>()) );
            lineStyle.add( const_cast<AltitudeSymbol*>(style.get<AltitudeSymbol>()) );
            
            GeoExtent extent(f->getSRS(), f->getGeometry()->getBounds());

            GeometryCompiler compiler;
            osg::ref_ptr<Session> session = new Session(map);
            FilterContext context( session.get(), prof, extent );

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

            Registry::shaderGenerator().run(group, Registry::stateSetCache());
    
            return ClusterCullingFactory::createAndInstall(group, centerECEF);
        }
    };


    struct GZDText : public PagedNode
    {
        const Style* _sqidStyle;
        FeatureList  _sqidFeatures;
        osg::BoundingSphere _bs;

        GZDText(const std::string& name, const osg::BoundingSphere& bs)
        {
            setName(name);     
            _bs = bs;
            _additive = false;    
            setRangeMode(osg::LOD::PIXEL_SIZE_ON_SCREEN);
            setRange(880);
        }

        bool hasChild() const
        {
            return _sqidStyle->has<TextSymbol>();
        }

        osg::Node* loadChild()
        {
            return buildSQID();
        }

        osg::BoundingSphere getChildBound() const
        {
            return _bs;
        }

        void setupData(const Feature* gzdFeature, const Style& gzdStyle,
                       const FeatureList& sqidFeatures, const Style& sqidStyle)
        {
            setNode(buildGZD(gzdFeature, gzdStyle));
            _sqidFeatures = sqidFeatures;
            _sqidStyle = &sqidStyle;
        }

        osg::Node* buildGZD(const Feature* f, const Style& style)
        {
            if (style.has<TextSymbol>() == false)
                return 0L;

            const TextSymbol* textSymPrototype = style.get<TextSymbol>();

            GeoExtent extent(f->getSRS(), f->getGeometry()->getBounds());

            osg::ref_ptr<TextSymbol> textSym = textSymPrototype ? new TextSymbol(*textSymPrototype) : new TextSymbol();

            if (textSym->size().isSet() == false)
                textSym->size() = 32.0f;
            if (textSym->alignment().isSet() == false)
                textSym->alignment() = textSym->ALIGN_LEFT_BASE_LINE;
        
            TextSymbolizer symbolizer( textSym );
            osgText::Text* drawable = symbolizer.create(getName());
            drawable->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
            drawable->getOrCreateStateSet()->setRenderBinToInherit();

            const SpatialReference* ecef = f->getSRS()->getECEF();
            osg::Vec3d positionECEF;
            extent.getSRS()->transform( osg::Vec3d(extent.xMin(),extent.yMin(),0), ecef, positionECEF );

            Registry::shaderGenerator().run(drawable, Registry::stateSetCache());
        
            osg::Matrixd L2W;
            ecef->createLocalToWorld( positionECEF, L2W );
            osg::MatrixTransform* mt = new osg::MatrixTransform(L2W);
            mt->addChild(drawable); 

            return ClusterCullingFactory::createAndInstall(mt, positionECEF);
        }

        osg::Node* buildSQID()
        {
            const TextSymbol* textSymPrototype = _sqidStyle->get<TextSymbol>();
            osg::ref_ptr<TextSymbol> textSym = textSymPrototype ? new TextSymbol(*textSymPrototype) : new TextSymbol();

            if (textSym->size().isSet() == false)
                textSym->size() = 24.0f;
            if (textSym->alignment().isSet() == false)
                textSym->alignment() = textSym->ALIGN_LEFT_BASE_LINE;
        
            TextSymbolizer symbolizer( textSym );

            osg::Group* group = new osg::Group();

            GeoExtent fullExtent;

            for (FeatureList::const_iterator f = _sqidFeatures.begin(); f != _sqidFeatures.end(); ++f)
            {
                const Feature* feature = f->get();
                std::string sqid = feature->getString("MGRS");
                osgText::Text* drawable = symbolizer.create(sqid);
                drawable->setCharacterSizeMode(drawable->SCREEN_COORDS);
                drawable->getOrCreateStateSet()->setRenderBinToInherit();
            
                GeoExtent extent(feature->getSRS(), feature->getGeometry()->getBounds());

                const SpatialReference* ecef = feature->getSRS()->getECEF();
                osg::Vec3d LL;
                findPointClosestTo(feature, osg::Vec3d(extent.xMin(), extent.yMin(), 0), LL);
                osg::Vec3d positionECEF;
                extent.getSRS()->transform(LL, ecef, positionECEF );
        
                osg::Matrixd L2W;
                ecef->createLocalToWorld( positionECEF, L2W );
                osg::MatrixTransform* mt = new osg::MatrixTransform(L2W);
                mt->addChild(drawable);

                group->addChild(mt);

                fullExtent.expandToInclude(extent);
            }
            
            Registry::shaderGenerator().run(group, Registry::stateSetCache());
            return group;
        }
    };
}

void
MGRSGraticule::rebuild()
{
    if (_root.valid() == false)
        return;

    osg::ref_ptr<const Map> map;
    if (!_map.lock(map))
        return;

    // Set up some reasonable default styling for a caller that did not
    // set styles in the options.
    setUpDefaultStyles();
    
    // clear everything out and start over
    _root->removeChildren( 0, _root->getNumChildren() );

    // requires a geocentric map
    if ( !map->isGeocentric() )
    {
        OE_WARN << LC << "Projected map mode is not supported" << std::endl;
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

    // Build the GZD tiles
    osgEarth::Drivers::OGRFeatureOptions gzd_ogr;
    gzd_ogr.url() = "H:/data/nga/mgrs/MGRS_GZD_WorldWide.shp";
    gzd_ogr.buildSpatialIndex() = true;

    osg::ref_ptr<FeatureSource> gzd_fs = FeatureSourceFactory::create(gzd_ogr);
    if (!gzd_fs.valid())
    {
        setStatus(Status::ResourceUnavailable, "Cannot access GZD dataset");
        return;
    }
    if (gzd_fs->open().isError())
    {
        setStatus(Status::ResourceUnavailable, "Cannot open GZD feature source");
        return;
    }

    osgEarth::Drivers::OGRFeatureOptions sqid_ogr;
    sqid_ogr.url() = "H:/data/nga/mgrs/MGRS_100kmSQ_ID/WGS84/ALL_SQID.shp";
    sqid_ogr.buildSpatialIndex() = true;

    osg::ref_ptr<FeatureSource> sqid_fs = FeatureSourceFactory::create(sqid_ogr);
    if (sqid_fs.valid() && sqid_fs->open().isOK())
    {
        // read in all SQID data
        typedef std::map<std::string, FeatureList> Table;
        Table table;

        FeatureList sqids;
        osg::ref_ptr<FeatureCursor> sqid_cursor = sqid_fs->createFeatureCursor();
        if (sqid_cursor.valid() && sqid_cursor->hasMore())
            sqid_cursor->fill(sqids);

        for (FeatureList::iterator i = sqids.begin(); i != sqids.end(); ++i)
            table[i->get()->getString("gzd")].push_back(i->get());

        GeometryCompilerOptions gcOpt;
        gcOpt.shaderPolicy() = ShaderPolicy::SHADERPOLICY_INHERIT;
        gcOpt.optimizeStateSharing() = false;

        osg::Group* geomTop = new osg::Group();
        top->addChild(geomTop);

        osg::Group* textTop = new osg::Group();
        top->addChild(textTop);

        osg::ref_ptr<FeatureCursor> gzd_cursor = gzd_fs->createFeatureCursor();
        while (gzd_cursor.valid() && gzd_cursor->hasMore())
        {
            osg::ref_ptr<Feature> feature = gzd_cursor->nextFeature();
            std::string gzd = feature->getString("gzd");
            if (!gzd.empty())
            {
                GZDGeom* geom = new GZDGeom(gzd);
                geom->setupData(feature.get(), options().gzdStyle().get(), table[gzd], options().sqidStyle().get(), _featureProfile.get(), map.get());
                geom->setupPaging();
                geomTop->addChild(geom);

                GZDText* text = new GZDText(gzd, geom->getBound());
                text->setupData(feature.get(), options().gzdStyle().get(), table[gzd], options().sqidStyle().get());
                text->setupPaging();
                textTop->addChild(text);
            }
        }
    }
    else
    {
        OE_WARN << LC << "SQID data file not opened" << std::endl;
    }
}

osg::Node*
MGRSGraticule::buildGZDTextTile(const std::string& name, const Feature* f, const Style& style)
{    
    const TextSymbol* textSymPrototype = style.get<TextSymbol>();
    if ( textSymPrototype )
    {
        GeoExtent extent(f->getSRS(), f->getGeometry()->getBounds());

        osg::ref_ptr<TextSymbol> textSym = textSymPrototype ? new TextSymbol(*textSymPrototype) : new TextSymbol();

        if (textSym->size().isSet() == false)
            textSym->size() = 32.0f;
        if (textSym->alignment().isSet() == false)
            textSym->alignment() = textSym->ALIGN_LEFT_BOTTOM;
        
        TextSymbolizer symbolizer( textSym );
        osgText::Text* drawable = symbolizer.create(name);
        drawable->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
        drawable->getOrCreateStateSet()->setRenderBinToInherit();

        const SpatialReference* ecef = f->getSRS()->getECEF();
        osg::Vec3d positionECEF;
        extent.getSRS()->transform( osg::Vec3d(extent.xMin(),extent.yMin(),0), ecef, positionECEF );

        Registry::shaderGenerator().run(drawable, Registry::stateSetCache());
        
        osg::Matrixd L2W;
        ecef->createLocalToWorld( positionECEF, L2W );
        osg::MatrixTransform* mt = new osg::MatrixTransform(L2W);
        mt->addChild(drawable); 

        return ClusterCullingFactory::createAndInstall(mt, positionECEF);
        //return mt;
    }
    else
    {
        OE_WARN << LC << "No text symbol found for GZD style\n";
    }

    return 0L;
    //group = ClusterCullingFactory::createAndInstall( group, centerECEF )->asGroup();
    //return group;
}

osg::Node*
MGRSGraticule::buildGZDGeomTile(const std::string& name, const Feature* f, const Style& style, const Map* map)
{
    osg::Group* group = new osg::Group();

    Style lineStyle;
    lineStyle.add( const_cast<LineSymbol*>(style.get<LineSymbol>()) );
    lineStyle.add( const_cast<AltitudeSymbol*>(style.get<AltitudeSymbol>()) );

    bool hasText = style.get<TextSymbol>() != 0L;

    GeoExtent extent(f->getSRS(), f->getGeometry()->getBounds());

    GeometryCompiler compiler;
    osg::ref_ptr<Session> session = new Session(map);
    FilterContext context( session.get(), _featureProfile.get(), extent );

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

    Registry::shaderGenerator().run(group, Registry::stateSetCache());
    
    return ClusterCullingFactory::createAndInstall(group, centerECEF);
}

void
MGRSGraticule::setUpDefaultStyles()
{
    if (!options().gzdStyle().isSet())
    {
        LineSymbol* line = options().gzdStyle()->getOrCreate<LineSymbol>();
        line->stroke()->color() = Color::Gray;
        line->stroke()->width() = 1.0;
        line->tessellation() = 20;

        TextSymbol* text = options().gzdStyle()->getOrCreate<TextSymbol>();
        text->fill()->color() = Color(Color::White, 0.3f);
        text->halo()->color() = Color(Color::Black, 0.2f);
        text->alignment() = TextSymbol::ALIGN_CENTER_CENTER;
    }

    if (!options().sqidStyle().isSet())
    {
        LineSymbol* line = options().sqidStyle()->getOrCreate<LineSymbol>();
        line->stroke()->color() = Color::Gray;
        line->stroke()->stipplePattern() = 0x1111;

        TextSymbol* text = options().sqidStyle()->getOrCreate<TextSymbol>();
        text->fill()->color() = Color(Color::White, 0.3f);
        text->halo()->color() = Color(Color::Black, 0.1f);
        text->alignment() = TextSymbol::ALIGN_CENTER_CENTER;
    }
}

osg::Node*
MGRSGraticule::buildSQIDTiles( const std::string& gzd )
{
    const GeoExtent& extent = _utmData.sectorTable()[gzd];

    // parse the GZD into its components:
    unsigned zone;
    char letter;
    sscanf( gzd.c_str(), "%u%c", &zone, &letter );
    
    const TextSymbol* textSymFromOptions = options().sqidStyle()->get<TextSymbol>();
    if ( !textSymFromOptions )
        textSymFromOptions = options().sqidStyle()->get<TextSymbol>();

    // copy it since we intend to alter it
    osg::ref_ptr<TextSymbol> textSym = 
        textSymFromOptions ? new TextSymbol(*textSymFromOptions) :
        new TextSymbol();

    double h = 0.0;

    TextSymbolizer ts( textSym );
    MGRSFormatter mgrs(MGRSFormatter::PRECISION_100000M);
    osg::Geode* textGeode = new osg::Geode();

    const SpatialReference* ecefSRS = extent.getSRS()->getECEF();
    osg::Vec3d centerMap, centerECEF;
    extent.getCentroid(centerMap.x(), centerMap.y());
    extent.getSRS()->transform(centerMap, ecefSRS, centerECEF);

    osg::Matrix local2world;
    ecefSRS->createLocalToWorld( centerECEF, local2world );
    osg::Matrix world2local;
    world2local.invert(local2world);

    FeatureList features;

    std::vector<GeoExtent> sqidExtents;

    // UTM:
    if ( letter > 'B' && letter < 'Y' )
    {
        // grab the SRS for the current UTM zone:
        // TODO: AL/AA designation??
        const SpatialReference* utm = SpatialReference::create(
            Stringify() << "+proj=utm +zone=" << zone << " +north +units=m" );

        // transform the four corners of the tile to UTM.
        osg::Vec3d gzdUtmSW, gzdUtmSE, gzdUtmNW, gzdUtmNE;
        extent.getSRS()->transform( osg::Vec3d(extent.xMin(),extent.yMin(),h), utm, gzdUtmSW );
        extent.getSRS()->transform( osg::Vec3d(extent.xMin(),extent.yMax(),h), utm, gzdUtmNW );
        extent.getSRS()->transform( osg::Vec3d(extent.xMax(),extent.yMin(),h), utm, gzdUtmSE );
        extent.getSRS()->transform( osg::Vec3d(extent.xMax(),extent.yMax(),h), utm, gzdUtmNE );

        // find the southern boundary of the first full SQID tile in the GZD tile.
        double southSQIDBoundary = gzdUtmSW.y(); //extentUTM.yMin();
        double remainder = fmod( southSQIDBoundary, 100000.0 );
        if ( remainder > 0.0 )
            southSQIDBoundary += (100000.0 - remainder);

        // find the min/max X for this cell in UTM:
        double xmin = extent.yMin() >= 0.0 ? gzdUtmSW.x() : gzdUtmNW.x();
        double xmax = extent.yMin() >= 0.0 ? gzdUtmSE.x() : gzdUtmNE.x();

        // Record the UTM extent of each SQID cell in this tile.
        // Go from the south boundary northwards:
        for( double y = southSQIDBoundary; y < gzdUtmNW.y(); y += 100000.0 )
        {
            // start at the central meridian (500K) and go west:
            for( double x = 500000.0; x > xmin; x -= 100000.0 )
            {
                sqidExtents.push_back( GeoExtent(utm, x-100000.0, y, x, y+100000.0) );
            }

            // then start at the central meridian and go east:
            for( double x = 500000.0; x < xmax; x += 100000.0 )
            {
                sqidExtents.push_back( GeoExtent(utm, x, y, x+100000.0, y+100000.0) );
            }
        }

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
                Feature* lat = new Feature(new LineString(2), extent.getSRS());
                lat->geoInterp() = GEOINTERP_RHUMB_LINE;
                lat->getGeometry()->push_back( sw );
                lat->getGeometry()->push_back( se );
                features.push_back(lat);

                Feature* lon = new Feature(new LineString(2), extent.getSRS());
                lon->geoInterp() = GEOINTERP_GREAT_CIRCLE;
                lon->getGeometry()->push_back( sw );
                lon->getGeometry()->push_back( nw );
                features.push_back(lon);

                // and the text label:
                osg::Vec3d sqidTextMap = (nw + se) * 0.5;
                sqidTextMap.z() += 1000.0;
                osg::Vec3d sqidTextECEF;
                extent.getSRS()->transform(sqidTextMap, ecefSRS, sqidTextECEF);
                osg::Vec3d sqidLocal;
                sqidLocal = sqidTextECEF * world2local;

                MGRSCoord mgrsCoord;
                if ( mgrs.transform( GeoPoint(extent.getSRS(),sqidTextMap,ALTMODE_ABSOLUTE), mgrsCoord) )
                {
                    textSym->size() = utmWidth/3.0;        
                    osgText::Text* d = ts.create( mgrsCoord.sqid );

                    osg::Matrixd textLocal2World;
                    ecefSRS->createLocalToWorld( sqidTextECEF, textLocal2World );

                    d->setPosition( sqidLocal );
                    textGeode->addDrawable( d );
                }
            }
        }
    }

    else if ( letter == 'A' || letter == 'B' )
    {
        // SRS for south polar region UPS projection. This projection has (0,0) at the
        // south pole, with +X extending towards 90 degrees E longitude and +Y towards
        // 0 degrees longitude.
        const SpatialReference* ups = SpatialReference::create(
            "+proj=stere +lat_ts=-90 +lat_0=-90 +lon_0=0 +k_0=1 +x_0=0 +y_0=0");

        osg::Vec3d gtemp;
        double r = GeoMath::distance(-osg::PI_2, 0.0, -1.3962634, 0.0); // -90 => -80 latitude
        double r2 = r*r;

        if ( letter == 'A' )
        {
            for( double x = 0.0; x < 1200000.0; x += 100000.0 )
            {
                double yminmax = sqrt( r2 - x*x );
                Feature* f = new Feature( new LineString(2), extent.getSRS() );
                f->geoInterp() = GEOINTERP_GREAT_CIRCLE;
                osg::Vec3d p0, p1;
                ups->transform( osg::Vec3d(-x, -yminmax, 0), extent.getSRS(), p0 );
                ups->transform( osg::Vec3d(-x,  yminmax, 0), extent.getSRS(), p1 );
                f->getGeometry()->push_back( p0 );
                f->getGeometry()->push_back( p1 );
                features.push_back( f );
            }

            for( double y = -1100000.0; y < 1200000.0; y += 100000.0 )
            {
                double xmax = sqrt( r2 - y*y );
                Feature* f = new Feature( new LineString(2), extent.getSRS() );
                f->geoInterp() = GEOINTERP_GREAT_CIRCLE;
                osg::Vec3d p0, p1;
                ups->transform( osg::Vec3d(-xmax, y, 0), extent.getSRS(), p0 );
                ups->transform( osg::Vec3d(    0, y, 0), extent.getSRS(), p1 );
                f->getGeometry()->push_back( p0 );
                f->getGeometry()->push_back( p1 );
                features.push_back( f );
            }

            for( double x = -1200000.0; x < 0.0; x += 100000.0 )
            {
                for( double y = -1200000.0; y < 1200000.0; y += 100000.0 )
                {
                    osg::Vec3d sqidTextMap;
                    ups->transform( osg::Vec3d(x+50000.0, y+50000.0, 0), extent.getSRS(), sqidTextMap);
                    if ( sqidTextMap.y() < -80.0 )
                    {
                        sqidTextMap.z() += 1000.0;
                        osg::Vec3d sqidTextECEF;
                        extent.getSRS()->transform(sqidTextMap, ecefSRS, sqidTextECEF);
                        osg::Vec3d sqidLocal = sqidTextECEF * world2local;

                        MGRSCoord mgrsCoord;
                        if ( mgrs.transform( GeoPoint(extent.getSRS(),sqidTextMap,ALTMODE_ABSOLUTE), mgrsCoord) )
                        {
                            textSym->size() = 33000.0;
                            osgText::Text* d = ts.create( mgrsCoord.sqid );
                            osg::Matrixd textLocal2World;
                            ecefSRS->createLocalToWorld( sqidTextECEF, textLocal2World );
                            d->setPosition( sqidLocal );
                            textGeode->addDrawable( d );
                        }
                    }
                }
            }
        }

        else if ( letter == 'B' )
        {
            for( double x = 100000.0; x < 1200000.0; x += 100000.0 )
            {
                double yminmax = sqrt( r2 - x*x );
                Feature* f = new Feature( new LineString(2), extent.getSRS() );
                f->geoInterp() = GEOINTERP_GREAT_CIRCLE;
                osg::Vec3d p0, p1;
                ups->transform( osg::Vec3d(x, -yminmax, 0), extent.getSRS(), p0 );
                ups->transform( osg::Vec3d(x,  yminmax, 0), extent.getSRS(), p1 );
                f->getGeometry()->push_back( p0 );
                f->getGeometry()->push_back( p1 );
                features.push_back( f );
            }

            for( double y = -1100000.0; y < 1200000.0; y += 100000.0 )
            {
                double xmax = sqrt( r2 - y*y );
                Feature* f = new Feature( new LineString(2), extent.getSRS() );
                f->geoInterp() = GEOINTERP_GREAT_CIRCLE;
                osg::Vec3d p0, p1;
                ups->transform( osg::Vec3d(    0, y, 0), extent.getSRS(), p0 );
                ups->transform( osg::Vec3d( xmax, y, 0), extent.getSRS(), p1 );
                f->getGeometry()->push_back( p0 );
                f->getGeometry()->push_back( p1 );
                features.push_back( f );
            }

            for( double x = 0.0; x < 1200000.0; x += 100000.0 )
            {
                for( double y = -1200000.0; y < 1200000.0; y += 100000.0 )
                {
                    osg::Vec3d sqidTextMap;
                    ups->transform( osg::Vec3d(x+50000.0, y+50000.0, 0), extent.getSRS(), sqidTextMap);
                    if ( sqidTextMap.y() < -80.0 )
                    {
                        sqidTextMap.z() += 1000.0;
                        osg::Vec3d sqidTextECEF;
                        extent.getSRS()->transform(sqidTextMap, ecefSRS, sqidTextECEF);
                        //extent.getSRS()->transformToECEF(sqidTextMap, sqidTextECEF);
                        osg::Vec3d sqidLocal = sqidTextECEF * world2local;

                        MGRSCoord mgrsCoord;
                        if ( mgrs.transform( GeoPoint(extent.getSRS(),sqidTextMap,ALTMODE_ABSOLUTE), mgrsCoord) )
                        {
                            textSym->size() = 33000.0;
                            osgText::Text* d = ts.create( mgrsCoord.sqid );
                            osg::Matrixd textLocal2World;
                            ecefSRS->createLocalToWorld( sqidTextECEF, textLocal2World );
                            d->setPosition( sqidLocal );
                            textGeode->addDrawable( d );
                        }
                    }
                }
            }
        }
    }

    else if ( letter == 'Y' || letter == 'Z' )
    {
        // SRS for north polar region UPS projection. This projection has (0,0) at the
        // south pole, with +X extending towards 90 degrees E longitude and +Y towards
        // 180 degrees longitude.
        const SpatialReference* ups = SpatialReference::create(
            "+proj=stere +lat_ts=90 +lat_0=90 +lon_0=0 +k_0=1 +x_0=0 +y_0=0");

        osg::Vec3d gtemp;
        double r = GeoMath::distance(osg::PI_2, 0.0, 1.46607657, 0.0); // 90 -> 84 latitude
        double r2 = r*r;

        if ( letter == 'Y' )
        {
            for( double x = 0.0; x < 700000.0; x += 100000.0 )
            {
                double yminmax = sqrt( r2 - x*x );
                Feature* f = new Feature( new LineString(2), extent.getSRS() );
                f->geoInterp() = GEOINTERP_GREAT_CIRCLE;
                osg::Vec3d p0, p1;
                ups->transform( osg::Vec3d(-x, -yminmax, 0), extent.getSRS(), p0 );
                ups->transform( osg::Vec3d(-x,  yminmax, 0), extent.getSRS(), p1 );
                f->getGeometry()->push_back( p0 );
                f->getGeometry()->push_back( p1 );
                features.push_back( f );
            }

            for( double y = -600000.0; y < 700000.0; y += 100000.0 )
            {
                double xmax = sqrt( r2 - y*y );
                Feature* f = new Feature( new LineString(2), extent.getSRS() );
                f->geoInterp() = GEOINTERP_GREAT_CIRCLE;
                osg::Vec3d p0, p1;
                ups->transform( osg::Vec3d(-xmax, y, 0), extent.getSRS(), p0 );
                ups->transform( osg::Vec3d(    0, y, 0), extent.getSRS(), p1 );
                f->getGeometry()->push_back( p0 );
                f->getGeometry()->push_back( p1 );
                features.push_back( f );
            }

            for( double x = -700000.0; x < 0.0; x += 100000.0 )
            {
                for( double y = -700000.0; y < 700000.0; y += 100000.0 )
                {
                    osg::Vec3d sqidTextMap;
                    ups->transform( osg::Vec3d(x+50000.0, y+50000.0, 0), extent.getSRS(), sqidTextMap);
                    if ( sqidTextMap.y() > 84.0 )
                    {
                        sqidTextMap.z() += 1000.0;
                        osg::Vec3d sqidTextECEF;
                        extent.getSRS()->transform(sqidTextMap, ecefSRS, sqidTextECEF);
                        //extent.getSRS()->transformToECEF(sqidTextMap, sqidTextECEF);
                        osg::Vec3d sqidLocal = sqidTextECEF * world2local;

                        MGRSCoord mgrsCoord;
                        if ( mgrs.transform( GeoPoint(extent.getSRS(),sqidTextMap,ALTMODE_ABSOLUTE), mgrsCoord) )
                        {
                            textSym->size() = 33000.0;
                            osgText::Text* d = ts.create( mgrsCoord.sqid );
                            osg::Matrixd textLocal2World;
                            ecefSRS->createLocalToWorld( sqidTextECEF, textLocal2World );
                            d->setPosition( sqidLocal );
                            textGeode->addDrawable( d );
                        }
                    }
                }
            }
        }

        else if ( letter == 'Z' )
        {
            for( double x = 100000.0; x < 700000.0; x += 100000.0 )
            {
                double yminmax = sqrt( r2 - x*x );
                Feature* f = new Feature( new LineString(2), extent.getSRS() );
                f->geoInterp() = GEOINTERP_GREAT_CIRCLE;
                osg::Vec3d p0, p1;
                ups->transform( osg::Vec3d(x, -yminmax, 0), extent.getSRS(), p0 );
                ups->transform( osg::Vec3d(x,  yminmax, 0), extent.getSRS(), p1 );
                f->getGeometry()->push_back( p0 );
                f->getGeometry()->push_back( p1 );
                features.push_back( f );
            }

            for( double y = -600000.0; y < 700000.0; y += 100000.0 )
            {
                double xmax = sqrt( r2 - y*y );
                Feature* f = new Feature( new LineString(2), extent.getSRS() );
                f->geoInterp() = GEOINTERP_GREAT_CIRCLE;
                osg::Vec3d p0, p1;
                ups->transform( osg::Vec3d(    0, y, 0), extent.getSRS(), p0 );
                ups->transform( osg::Vec3d( xmax, y, 0), extent.getSRS(), p1 );
                f->getGeometry()->push_back( p0 );
                f->getGeometry()->push_back( p1 );
                features.push_back( f );
            }

            for( double x = 0.0; x < 700000.0; x += 100000.0 )
            {
                for( double y = -700000.0; y < 700000.0; y += 100000.0 )
                {
                    osg::Vec3d sqidTextMap;
                    ups->transform( osg::Vec3d(x+50000.0, y+50000.0, 0), extent.getSRS(), sqidTextMap);
                    if ( sqidTextMap.y() > 84.0 )
                    {
                        sqidTextMap.z() += 1000.0;
                        osg::Vec3d sqidTextECEF;
                        extent.getSRS()->transform(sqidTextMap, ecefSRS, sqidTextECEF);
                        //extent.getSRS()->transformToECEF(sqidTextMap, sqidTextECEF);
                        osg::Vec3d sqidLocal = sqidTextECEF * world2local;

                        MGRSCoord mgrsCoord;
                        if ( mgrs.transform( GeoPoint(extent.getSRS(),sqidTextMap,ALTMODE_ABSOLUTE), mgrsCoord) )
                        {
                            textSym->size() = 33000.0;
                            osgText::Text* d = ts.create( mgrsCoord.sqid );
                            osg::Matrixd textLocal2World;
                            ecefSRS->createLocalToWorld( sqidTextECEF, textLocal2World );
                            d->setPosition( sqidLocal );
                            textGeode->addDrawable( d );
                        }
                    }
                }
            }
        }
    }

    osg::Group* group = new osg::Group();

    osg::ref_ptr<const Map> map;
    if (_map.lock(map))
    {
        Style lineStyle;
        lineStyle.add( options().sqidStyle()->get<LineSymbol>() );

        GeometryCompiler compiler;
        osg::ref_ptr<Session> session = new Session(map.get());
        FilterContext context( session.get(), _featureProfile.get(), extent );

        // make sure we get sufficient tessellation:
        compiler.options().maxGranularity() = 0.25;

        osg::Node* geomNode = compiler.compile(features, lineStyle, context);
        if ( geomNode ) 
            group->addChild( geomNode );

        osg::MatrixTransform* mt = new osg::MatrixTransform(local2world);
        mt->addChild(textGeode);
        group->addChild( mt );

        Registry::shaderGenerator().run(textGeode, Registry::stateSetCache());
    }

    return group;
}

//---------------------------------------------------------------------------

namespace osgEarth { namespace Util
{
    // OSG Plugin for loading subsequent graticule levels
    class MGRSGraticulePseudoLoader : public osgDB::ReaderWriter
    {
    public:
        MGRSGraticulePseudoLoader()
        {
            supportsExtension( MGRS_GRATICULE_PSEUDOLOADER_EXTENSION, "osgEarth MGRS graticule" );
        }

        const char* className() const
        {
            return "osgEarth MGRS graticule LOD loader";
        }

        bool acceptsExtension(const std::string& extension) const
        {
            return osgDB::equalCaseInsensitive(extension, MGRS_GRATICULE_PSEUDOLOADER_EXTENSION);
        }

        ReadResult readNode(const std::string& uri, const Options* options) const
        {        
            std::string ext = osgDB::getFileExtension( uri );
            if ( !acceptsExtension( ext ) )
                return ReadResult::FILE_NOT_HANDLED;

            if ( !options )
            {
                OE_WARN << LC << "INTERNAL ERROR: MGRSGraticule object not present in Options (1)\n";
                return ReadResult::ERROR_IN_READING_FILE;
            }

            osg::ref_ptr<MGRSGraticule> graticule;
            if (!OptionsData<MGRSGraticule>::lock(options, "osgEarth.MGRSGraticule", graticule))
            {
                OE_WARN << LC << "INTERNAL ERROR: MGRSGraticule object not present in Options (2)\n";
                return ReadResult::ERROR_IN_READING_FILE;
            }

            std::string def = osgDB::getNameLessExtension(uri);
            std::string gzd = osgDB::getNameLessExtension(def);
            
            osg::Node* result = graticule->buildSQIDTiles( gzd );

            return result ? ReadResult(result) : ReadResult::ERROR_IN_READING_FILE;
        }
    };
    REGISTER_OSGPLUGIN(MGRS_GRATICULE_PSEUDOLOADER_EXTENSION, MGRSGraticulePseudoLoader);

} } // namespace osgEarth::Util


