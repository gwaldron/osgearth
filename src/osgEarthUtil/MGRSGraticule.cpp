/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2019 Pelican Mapping
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
#include <osgEarthUtil/UTMLabelingEngine>

#include <osgEarthFeatures/TextSymbolizer>
#include <osgEarthFeatures/TessellateOperator>

#include <osgEarthAnnotation/FeatureNode>

#include <osgEarth/Registry>
#include <osgEarth/PagedNode>
#include <osgEarth/Endian>
#include <osgEarth/GLUtils>
#include <osgEarth/Text>

#include <osg/Depth>
//#include <osgDB/WriteFile>

#include <osgEarthDrivers/feature_ogr/OGRFeatureOptions>


#define LC "[MGRSGraticule] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;
using namespace osgEarth::Annotation;

REGISTER_OSGEARTH_LAYER(mgrs_graticule, MGRSGraticule);

#ifndef GL_CLIP_DISTANCE0
#define GL_CLIP_DISTANCE0 0x3000
#endif

// whether to use SCREEN_COORDS to auto-size the text
#define USE_SCREEN_COORDS


//#define DEBUG_MODE

//---------------------------------------------------------------------------

MGRSGraticuleOptions::MGRSGraticuleOptions(const ConfigOptions& conf) :
VisibleLayerOptions(conf),
_useDefaultStyles(true)
{
    _sqidURI.init(URI("../data/mgrs_sqid.bin", conf.referrer()));
    _styleSheet = new StyleSheet();
    fromConfig(_conf);
}

Config
MGRSGraticuleOptions::getConfig() const
{
    Config conf = VisibleLayerOptions::getConfig();
    conf.set("sqid_data", _sqidURI);
    conf.set("styles", _styleSheet);
    conf.set("use_default_styles", _useDefaultStyles);
    return conf;
}

void
MGRSGraticuleOptions::fromConfig(const Config& conf)
{
    conf.get("sqid_data", _sqidURI);
    conf.get("styles", _styleSheet);
    conf.get("use_default_styles", _useDefaultStyles);
}

//---------------------------------------------------------------------------

namespace
{
    void simplify(Vec3dVector& vec)
    {
        int count = vec.size();
        for (int i = 1; i < vec.size()-1; ++i)
        {
            osg::Vec3d& p0 = vec[i-1];
            osg::Vec3d& p1 = vec[i];
            osg::Vec3d& p2 = vec[i+1];

            osg::Vec3d a = p1 - p0; a.normalize();
            osg::Vec3d b = p2 - p1; b.normalize();
            if (a*b > 0.60)
            {
                vec.erase(vec.begin()+i);
                --i;
            }
        }
    }

    //! Generates the binary SQID file.
    void writeSQIDfile(const URI& uri)
    {
        osgEarth::Drivers::OGRFeatureOptions sqid_ogr;
        sqid_ogr.url() = "H:/data/nga/mgrs/MGRS_100kmSQ_ID/WGS84/ALL_SQID.shp";
        sqid_ogr.buildSpatialIndex() = false;

        osg::ref_ptr<FeatureSource> sqid_fs = FeatureSourceFactory::create(sqid_ogr);
        if (sqid_fs.valid() && sqid_fs->open().isOK())
        {
            // Read source data into an array:
            FeatureList sqids;
            osg::ref_ptr<FeatureCursor> sqid_cursor = sqid_fs->createFeatureCursor(0L);
            if (sqid_cursor.valid() && sqid_cursor->hasMore())
                sqid_cursor->fill(sqids);

            // Open the output stream:
            std::ofstream out(uri.full().c_str(), std::ostream::out | std::ostream::binary);
            out.imbue(std::locale::classic());

            // We will need a local XY SRS for geometry simplification:
            const SpatialReference* xysrs = SpatialReference::get("spherical-mercator");

            u_int count = OE_ENCODE_LONG(sqids.size());
            out.write(reinterpret_cast<const char*>(&count), sizeof(u_int));

            for (FeatureList::iterator i = sqids.begin(); i != sqids.end(); ++i)
            {
                Feature* f = i->get();

                std::string gzd = f->getString("GZD");
                out.write(gzd.c_str(), 3);

                std::string sqid = f->getString("100kmSQ_ID");
                out.write(sqid.c_str(), 2);

                char easting = (char)(f->getDouble("EASTING")/100000.0);
                out.put(easting);

                char northing = (char)(f->getDouble("NORTHING")/100000.0);
                out.put(northing);

                //GeoExtent extent(f->getSRS(), f->getGeometry()->getBounds());

                // Transform into a local XY SRS for simplification:
                if (f->getGeometry()->size() > 0)
                {
                    f->transform(xysrs);
                    simplify(f->getGeometry()->asVector());
                    f->transform(xysrs->getGeographicSRS());
                }

                // TODO: deal with MultiGeometries!

                Geometry* g = f->getGeometry();

                u_short numPoints = OE_ENCODE_SHORT((u_short)g->size());
                out.write(reinterpret_cast<const char*>(&numPoints), sizeof(u_short));
                                   
                for (Geometry::const_iterator p = g->begin(); p != g->end(); ++p)
                {
                    uint64_t x = OE_ENCODE_DOUBLE(p->x());
                    out.write(reinterpret_cast<const char*>(&x), sizeof(uint64_t));

                    uint64_t y = OE_ENCODE_DOUBLE(p->y());
                    out.write(reinterpret_cast<const char*>(&y), sizeof(uint64_t));
                }
            }
            out.flush();
            out.close();

            OE_WARN << "Wrote SQIDs to " << uri.full() << std::endl;
        }
    }

    bool readSQIDfile(const URI& uri, FeatureList& output)
    {
        output.clear();

        std::ifstream fin(uri.full().c_str(), std::ostream::in | std::ostream::binary);
        fin.imbue(std::locale::classic());

        if (fin.eof() || fin.is_open() == false)
            return false;

        u_int count;
        fin.read(reinterpret_cast<char*>(&count), sizeof(u_int));
        count = OE_DECODE_LONG(count);

        const SpatialReference* wgs84 = SpatialReference::get("wgs84");

        for (u_int i = 0; i < count; ++i)
        {
            char gzd[4]; gzd[3] = 0;
            fin.read(gzd, 3);

            char sqid[3]; sqid[2] = 0;
            fin.read(sqid, 2);

            double easting = (double)fin.get() * 100000.0;

            double northing = (double)fin.get() * 100000.0;

            u_short numPoints;
            fin.read(reinterpret_cast<char*>(&numPoints), sizeof(u_short));
            numPoints = OE_DECODE_SHORT(numPoints);

            if (numPoints > 16384)
            {
                OE_WARN << LC << "sqid bin file is corrupt.. abort!" << std::endl;
                return false;
            }

            osgEarth::Symbology::Ring* line = new osgEarth::Symbology::Ring();
            for (u_short n = 0; n < numPoints; ++n)
            {
                uint64_t x;
                fin.read(reinterpret_cast<char*>(&x), sizeof(uint64_t));

                uint64_t y;
                fin.read(reinterpret_cast<char*>(&y), sizeof(uint64_t));

                line->push_back(osg::Vec3d(OE_DECODE_DOUBLE(x), OE_DECODE_DOUBLE(y), 0));
            }

            if (line->getTotalPointCount() > 0)
            {
                osg::ref_ptr<Feature> feature = new Feature(line, wgs84);
                feature->set("gzd", std::string(gzd));
                feature->set("sqid", std::string(sqid));
                feature->set("easting", easting);
                feature->set("northing", northing);
                output.push_back(feature.get());
            }
            else
            {
                OE_INFO << LC << "Empty SQID geom at " << gzd << " " << sqid << std::endl;
            }
        }
        return true;
    }

    struct LocalStats : public osg::Object
    {
        META_Object(osgEarth, LocalStats);
        LocalStats() : osg::Object(), _gzdNode(0), _gzdText(0), _sqidText(0), _geomCell(0), _geomGrid(0) { }
        unsigned _gzdNode, _gzdText, _sqidText, _geomCell, _geomGrid;
        LocalStats(const LocalStats& rhs, const osg::CopyOp&) { }
    };

    struct LocalRoot : public osg::Group
    {
#ifdef DEBUG_MODE
        void traverse(osg::NodeVisitor& nv)
        {
            if (nv.getVisitorType() == nv.CULL_VISITOR)
            {
                osg::UserDataContainer* udc = nv.getOrCreateUserDataContainer();
                LocalStats* stats = new LocalStats();
                stats->setName("stats");
                unsigned index = udc->addUserObject(stats);

                osg::Group::traverse(nv);

                Registry::instance()->startActivity("GZDGeom", Stringify() << stats->_gzdNode);
                Registry::instance()->startActivity("GZDText", Stringify() << stats->_gzdText);
                Registry::instance()->startActivity("SQIDText", Stringify() << stats->_sqidText);
                Registry::instance()->startActivity("GeomCell", Stringify() << stats->_geomCell);
                Registry::instance()->startActivity("GeomGrid", Stringify() << stats->_geomGrid);

                udc->removeUserObject(index);
            }
            else
            {
                osg::Group::traverse(nv);
            }
        }
#endif
    };

#define STATS(nv) (dynamic_cast<LocalStats*>(nv.getUserDataContainer()->getUserObject("stats")))
}

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

    // disable the depth buffer
    ss->setAttributeAndModes(
        new osg::Depth(osg::Depth::ALWAYS, 0.f, 1.f, false),
        osg::StateAttribute::ON);
    
    // activate horizon clipping
    ss->setMode(GL_CLIP_DISTANCE0, 1);

    // blending on to support transculency
    ss->setMode(GL_BLEND, 1);

    // force it to render after the terrain.
    ss->setRenderBinDetails(1, "RenderBin");

    _root = new LocalRoot();

    GLUtils::setLighting(_root->getOrCreateStateSet(), osg::StateAttribute::OFF);

    // install the range callback for clip plane activation
    _root->addCullCallback( new RangeUniformCullCallback() );

    if (getEnabled() == true)
    {
        rebuild();
    }

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
MGRSGraticule::getNode() const
{
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

    struct GeomCell : public PagedNode
    {
        double _size;        
        osg::ref_ptr<Feature> _feature;
        Style _style;
        bool _hasChild;
        const MGRSGraticuleOptions* _options;
    
        GeomCell(double size);
        void setupData(Feature* feature, const MGRSGraticuleOptions* options);
        osg::Node* loadChild();
        bool hasChild() const;
        osg::BoundingSphere getChildBound() const;
        osg::Node* build();

        void traverse(osg::NodeVisitor&);
    };


    struct GeomGrid : public PagedNode
    {
        double _size;
        const MGRSGraticuleOptions* _options;
        osg::ref_ptr<Feature> _feature;
        Style _style;
        GeoExtent _extent;
        osg::ref_ptr<const SpatialReference> _utm;

        GeomGrid(double size)
        {
            _size = size;
            setRangeMode(osg::LOD::PIXEL_SIZE_ON_SCREEN);
            setRange(1600);
            setAdditive(false);
        }

        void setupData(Feature* feature, const MGRSGraticuleOptions* options)
        {
            _feature = feature;
            _options = options;
            std::string styleName = Stringify() << (int)(_size*0.1);
            _style = *options->styleSheet()->getStyle(styleName, true);
            setNode(build());
        }

        osg::Node* loadChild()
        {
            osg::Group* group = new osg::Group();

            double x0 = _feature->getDouble("easting");
            double y0 = _feature->getDouble("northing");
            double interval = _size * 0.1;

            for (double x = x0; x < x0 + _size; x += interval)
            {
                for (double y = y0; y < y0 + _size; y += interval)
                {
                    osg::ref_ptr<LineString> polygon = new LineString();
                    polygon->push_back(x, y);
                    polygon->push_back(x+interval, y);
                    polygon->push_back(x+interval, y+interval);
                    polygon->push_back(x, y+interval);
                    polygon->push_back(x, y);

                    osg::ref_ptr<Feature> f = new Feature(polygon.get(), _utm.get());
                    f->transform(_feature->getSRS());

                    osg::ref_ptr<Geometry> croppedGeom;
                    if (f->getGeometry()->crop(_extent.bounds(), croppedGeom))
                    {
                        f->setGeometry(croppedGeom.get());
                        f->set("easting", x);
                        f->set("northing", y);
                        GeomCell* child = new GeomCell(interval);
                        child->setupData(f.get(), _options);
                        child->setupPaging();
                        group->addChild(child);
                    }                 
                }
            }
            
            return group;
        }

        osg::BoundingSphere getChildBound() const
        {
            return getChild(0)->getBound();
        }

        bool hasChild() const 
        {
            return true;
        }

        osg::Node* build()
        {
            _extent = GeoExtent(_feature->getSRS(), _feature->getGeometry()->getBounds());
            double lon, lat;
            _extent.getCentroid(lon, lat);
            _utm = _feature->getSRS()->createUTMFromLonLat(lon, lat);

            double x0 = _feature->getDouble("easting");
            double y0 = _feature->getDouble("northing");

            double interval = _size * 0.1;

            osg::ref_ptr<MultiGeometry> grid = new MultiGeometry();

            // south-north lines:
            for (double x=x0; x<=x0+_size; x += interval)
            {
                LineString* ls = new LineString();
                ls->push_back(osg::Vec3d(x, y0, 0));
                ls->push_back(osg::Vec3d(x, y0+_size, 0));
                grid->getComponents().push_back(ls);
            }
            
            // west-east lines:
            for (double y=y0; y<=y0+_size; y+=interval)
            {
                LineString* ls = new LineString();
                ls->push_back(osg::Vec3d(x0, y, 0));
                ls->push_back(osg::Vec3d(x0+_size, y, 0));
                grid->getComponents().push_back(ls);
            }

            osg::ref_ptr<Feature> f = new Feature(grid.get(), _utm.get());
            f->transform(_feature->getSRS());

            osg::ref_ptr<Geometry> croppedGeom;
            if (f->getGeometry()->crop(_extent.bounds(), croppedGeom))
            {
                f->setGeometry(croppedGeom.get());
            }

            GeometryCompilerOptions gco;
            gco.shaderPolicy() = SHADERPOLICY_INHERIT;
            FeatureNode* node = new FeatureNode(f.get(), _style, gco);
            
            return node;
        }
        
#ifdef DEBUG_MODE
        void traverse(osg::NodeVisitor& nv)
        {
            if (nv.getVisitorType() == nv.CULL_VISITOR)
                STATS(nv)->_geomGrid++;
            PagedNode::traverse(nv);
        }
#endif
    };
    


    GeomCell::GeomCell(double size) : _size(size)
    {
        setRangeMode(osg::LOD::PIXEL_SIZE_ON_SCREEN);
        setRange(880);
        setAdditive(true);
    }

    void GeomCell::setupData(Feature* feature, const MGRSGraticuleOptions* options)
    {
        _feature = feature;
        _options = options;
        std::string styleName = Stringify() << (int)(_size);
        _style = *options->styleSheet()->getStyle(styleName, true);
        setNode( build() );
    }

    osg::Node* GeomCell::loadChild()
    {
        GeomGrid* child = new GeomGrid(_size);
        child->setupData(_feature.get(), _options);
        child->setupPaging();
        return child;
    }

    bool GeomCell::hasChild() const
    {
        std::string sizeStr = Stringify() << (int)(_size/10);
        return _options->styleSheet()->getStyle(sizeStr, false) != 0L;
    }

    osg::BoundingSphere GeomCell::getChildBound() const
    {
        osg::ref_ptr<GeomGrid> child = new GeomGrid(_size);
        child->setupData(_feature.get(), _options);
        return child->getBound();
    }

    osg::Node* GeomCell::build()
    {
        GeometryCompilerOptions gco;
        gco.shaderPolicy() = SHADERPOLICY_INHERIT;
        FeatureNode* node = new FeatureNode(_feature.get(), _style, gco);
        return node;
    }

    void GeomCell::traverse(osg::NodeVisitor& nv)
    {
#ifdef DEBUG_MODE
        if (nv.getVisitorType() == nv.CULL_VISITOR)
            STATS(nv)->_geomCell++;
#endif
        PagedNode::traverse(nv);
    }


    //! Geometry for a single SQID 100km cell and its children
    struct SQID100kmCell : public PagedNode
    {
        osg::ref_ptr<Feature> _feature;
        const MGRSGraticuleOptions* _options;
        Style _style;

        SQID100kmCell(const std::string& name)
        {
            setName(name);
            setRangeMode(osg::LOD::PIXEL_SIZE_ON_SCREEN);
            setRange(880);
            setAdditive(true);
        }

        void setupData(Feature* feature, const MGRSGraticuleOptions* options)
        {
            _feature = feature;
            _options = options;
            std::string styleName("100000");
            _style = *options->styleSheet()->getStyle(styleName, true);
            setNode( build() );
        }

        osg::Node* loadChild()
        {
            GeomGrid* child = new GeomGrid(100000.0);
            child->setupData(_feature.get(), _options);
            child->setupPaging();
            return child;
        }

        bool hasChild() const
        {
            return _options->styleSheet()->getStyle("10000", false) != 0L;
        }

        osg::BoundingSphere getChildBound() const
        {
            GeomGrid* child = new GeomGrid(100000.0);
            child->setupData(_feature.get(), _options);
            return child->getBound();
        }

        osg::Node* build()
        {
            GeometryCompilerOptions gco;
            gco.shaderPolicy() = SHADERPOLICY_INHERIT;
            FeatureNode* node = new FeatureNode(_feature.get(), _style, gco);
            return node;
        }
    };


    //! All SQID 100km goemetry from a single UTM GZD cell combined into one geometry
    struct SQID100kmGrid : public PagedNode
    {
        osg::BoundingSphere _bs;
        FeatureList _sqidFeatures;
        Style _style;
        const MGRSGraticuleOptions* _options;

        SQID100kmGrid(const std::string& name, const osg::BoundingSphere& bs)
        {
            setName(name);
            _bs = bs;
            setAdditive(false);
            setRangeMode(osg::LOD::PIXEL_SIZE_ON_SCREEN);
            setRange(3200);
        }

        void setupData(const FeatureList& sqidFeatures, const MGRSGraticuleOptions* options)
        {
            _sqidFeatures = sqidFeatures;
            _options = options;

            std::string styleName("100000");
            _style = *options->styleSheet()->getStyle(styleName, true);

            // remove any text symbology; that gets built elsewhere.
            _style.remove<TextSymbol>();
           
            setNode(build());
        }

        osg::Node* loadChild()
        {
            osg::Group* group = new osg::Group();

            for (FeatureList::const_iterator f = _sqidFeatures.begin(); f != _sqidFeatures.end(); ++f)
            {
                Feature* feature = f->get();
                SQID100kmCell* geom = new SQID100kmCell(feature->getString("sqid"));
                geom->setupData(feature, _options);
                geom->setupPaging();
                group->addChild(geom);
                
                GeoExtent extent(feature->getSRS(), feature->getGeometry()->getBounds());
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
            gco.shaderPolicy() = SHADERPOLICY_INHERIT;
            return new FeatureNode(_sqidFeatures, _style, gco);
        }
    };


    struct GZDGeom : public PagedNode
    {
        Style _sqidStyle;
        FeatureList _sqidFeatures;
        const MGRSGraticuleOptions* _options;

        GZDGeom(const std::string& name)
        {
            setName(name);     
            setRangeMode(osg::LOD::PIXEL_SIZE_ON_SCREEN);
            setRange(640);
            setAdditive(false);
        }

        bool hasChild() const
        {
            return _options->styleSheet()->getStyle("100000", false) != 0L;
        }

        osg::Node* loadChild()
        {
            SQID100kmGrid* child = new SQID100kmGrid(getName(), getBound());
            child->setupData(_sqidFeatures, _options);
            child->setupPaging();
            return child;
        }

        osg::BoundingSphere getChildBound() const
        {
            return getChild(0)->getBound();
        }

        void setupData(const Feature* gzdFeature,
                       const FeatureList& sqidFeatures, 
                       const MGRSGraticuleOptions* options,
                       const FeatureProfile* prof, 
                       const Map* map)
        {
            _options = options;
            setNode(build(gzdFeature, prof, map));
            _sqidFeatures = sqidFeatures;
        }

        osg::Node* build(const Feature* f, const FeatureProfile* prof, const Map* map)
        {
            osg::Group* group = new osg::Group();

            // Extract just the line and altitude symbols:
            const Style& gzdStyle = *_options->styleSheet()->getStyle("gzd");
            Style lineStyle;
            lineStyle.add( const_cast<LineSymbol*>(gzdStyle.get<LineSymbol>()) );
            lineStyle.add( const_cast<AltitudeSymbol*>(gzdStyle.get<AltitudeSymbol>()) );
            
            GeoExtent extent(f->getSRS(), f->getGeometry()->getBounds());

            GeometryCompiler compiler;
            osg::ref_ptr<Session> session = new Session(map);
            FilterContext context( session.get(), prof, extent );

            // make sure we get sufficient tessellation:
            compiler.options().maxGranularity() = 1.0;

            // line shaders are at the root of the geometry tree, so don't install any
            compiler.options().shaderPolicy() = SHADERPOLICY_INHERIT;

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

            const SpatialReference* ecefSRS = extent.getSRS()->getGeocentricSRS();
    
            osg::Vec3d centerECEF;
            extent.getSRS()->transform( tileCenter, ecefSRS, centerECEF );

            return ClusterCullingFactory::createAndInstall(group, centerECEF);
        }
        
#ifdef DEBUG_MODE
        void traverse(osg::NodeVisitor& nv)
        {
            if (nv.getVisitorType() == nv.CULL_VISITOR)
                STATS(nv)->_gzdNode++;
            PagedNode::traverse(nv);
        }
#endif
    };


    struct SQIDTextGrid : public osg::Group
    {
        SQIDTextGrid(const std::string& name, FeatureList& features, const MGRSGraticuleOptions* options)
        {
            setName(name);

            const Style& style = *options->styleSheet()->getStyle("100000", true);
            if (style.has<TextSymbol>() == false)
                return;

            const TextSymbol* textSymPrototype = style.get<TextSymbol>();
            osg::ref_ptr<TextSymbol> textSym = new TextSymbol(*style.get<TextSymbol>());

            if (textSym->size().isSet() == false)
                textSym->size() = 24.0f;

            if (textSym->alignment().isSet() == false)
                textSym->alignment() = textSym->ALIGN_LEFT_BASE_LINE;
        
            TextSymbolizer symbolizer( textSym.get() );
            
            GeoExtent fullExtent;

            for (FeatureList::const_iterator f = features.begin(); f != features.end(); ++f)
            {
                const Feature* feature = f->get();
                std::string sqid = feature->getString("sqid");
                osgText::Text* drawable = new osgEarth::Text(sqid);
                symbolizer.apply(drawable);
#ifdef USE_SCREEN_COORDS
                drawable->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
#else
                drawable->setCharacterSizeMode(osgText::Text::OBJECT_COORDS);
                drawable->setCharacterSize(13000);
#endif
                drawable->getOrCreateStateSet()->setRenderBinToInherit();
            
                GeoExtent extent(feature->getSRS(), feature->getGeometry()->getBounds());

                const SpatialReference* ecef = feature->getSRS()->getGeocentricSRS();

                osg::Vec3d LL;
                findPointClosestTo(feature, osg::Vec3d(extent.xMin(), extent.yMin(), 0), LL);
                osg::Vec3d positionECEF;
                extent.getSRS()->transform(LL, ecef, positionECEF );
        
                osg::Matrixd L2W;
                ecef->createLocalToWorld( positionECEF, L2W );
                osg::MatrixTransform* mt = new osg::MatrixTransform(L2W);
                mt->addChild(drawable);

                addChild(mt);

                fullExtent.expandToInclude(extent);
            }

            OE_DEBUG << LC << "Created " << features.size() << " text elements for " << getName() << std::endl;
            
            //Registry::shaderGenerator().run(this, Registry::stateSetCache());
        }
        
#ifdef DEBUG_MODE
        void traverse(osg::NodeVisitor& nv)
        {
            if (nv.getVisitorType() == nv.CULL_VISITOR)
                STATS(nv)->_sqidText++;
            osg::Group::traverse(nv);
        }
#endif
    };


    struct GZDText : public PagedNode
    {
        const MGRSGraticuleOptions* _options;
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
            const Style* s = _options->styleSheet()->getStyle("100000", false);
            return s && s->has<TextSymbol>();
        }

        osg::Node* loadChild()
        {
            return new SQIDTextGrid(getName(), _sqidFeatures, _options);
        }

        osg::BoundingSphere getChildBound() const
        {
            return _bs;
        }

        void setupData(const Feature* gzdFeature, const FeatureList& sqidFeatures, const MGRSGraticuleOptions* options)
        {
            _options = options;
            setNode(buildGZD(gzdFeature));
            _sqidFeatures = sqidFeatures;
        }

        osg::Node* buildGZD(const Feature* f)
        {
            Style style = *_options->styleSheet()->getStyle("gzd", true);

            const TextSymbol* textSymPrototype = style.get<TextSymbol>();

            GeoExtent extent(f->getSRS(), f->getGeometry()->getBounds());

            osg::ref_ptr<TextSymbol> textSym = textSymPrototype ? new TextSymbol(*textSymPrototype) : new TextSymbol();

            if (textSym->size().isSet() == false)
                textSym->size() = 32.0f;
            if (textSym->alignment().isSet() == false)
                textSym->alignment() = textSym->ALIGN_LEFT_BASE_LINE;
        
            TextSymbolizer symbolizer( textSym.get() );
            osgText::Text* drawable = new osgEarth::Text(getName());
            symbolizer.apply(drawable);

#ifdef USE_SCREEN_COORDS
            drawable->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
            //drawable->setPosition(osg::Vec3(0,0,0));
            //drawable->setCharacterSize(36);
#else
            drawable->setCharacterSizeMode(osgText::Text::OBJECT_COORDS);
            drawable->setCharacterSize(130000);
#endif
            drawable->getOrCreateStateSet()->setRenderBinToInherit();

            const SpatialReference* ecef = f->getSRS()->getGeocentricSRS();
            osg::Vec3d positionECEF;
            extent.getSRS()->transform( osg::Vec3d(extent.xMin(),extent.yMin(),0), ecef, positionECEF );
        
            osg::Matrixd L2W;
            ecef->createLocalToWorld( positionECEF, L2W );
            osg::MatrixTransform* mt = new osg::MatrixTransform(L2W);
            mt->addChild(drawable); 

            return ClusterCullingFactory::createAndInstall(mt, positionECEF);
        }

#ifdef DEBUG_MODE
        void traverse(osg::NodeVisitor& nv)
        {
            if (nv.getVisitorType() == nv.CULL_VISITOR)
                STATS(nv)->_gzdText++;
            PagedNode::traverse(nv);
        }
#endif
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
    if (options().useDefaultStyles() == true)
    {
        setUpDefaultStyles();
    }
    
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

#if 0
    // Uncomment to write out a SQID data file
    writeSQIDfile(options().sqidData().get());
#endif
    
    FeatureList sqids;
    if (readSQIDfile(options().sqidData().get(), sqids))
    {        
        typedef std::map<std::string, FeatureList> Table;
        Table table;

        for (FeatureList::iterator i = sqids.begin(); i != sqids.end(); ++i)
        {
            table[i->get()->getString("gzd")].push_back(i->get());
        }

        // Root of the geometry tree
        osg::Group* geomTop = new osg::Group();
        top->addChild(geomTop);

        // Root of the text tree
        osg::Group* textTop = new osg::Group();
        osg::StateSet* textSS = textTop->getOrCreateStateSet();
        //TextSymbolizer::insta(textSS);
        top->addChild(textTop);

        // build the GZD feature set
        FeatureList gzdFeatures;
        loadGZDFeatures(map->getSRS()->getGeographicSRS(), gzdFeatures);
        osg::ref_ptr<FeatureListCursor> gzd_cursor = new FeatureListCursor(gzdFeatures);

        unsigned count = 0u;

        while (gzd_cursor.valid() && gzd_cursor->hasMore())
        {
            osg::ref_ptr<Feature> feature = gzd_cursor->nextFeature();
            std::string gzd = feature->getString("gzd");
            if (!gzd.empty())
            {
                GZDGeom* geom = new GZDGeom(gzd);
                geom->setupData(feature.get(), table[gzd], &options(), _featureProfile.get(), map.get());
                geom->setupPaging();
                geomTop->addChild(geom);

                GZDText* text = new GZDText(gzd, geom->getBound());
                text->setupData(feature.get(), table[gzd], &options());
                text->setupPaging();
                textTop->addChild(text);

                ++count;
            }
            else
            {
                OE_WARN << LC << "INTERNAL ERROR: GZD empty!" << std::endl;
            }
        }

        // Install the UTM grid labeler
        UTMLabelingEngine* labeler = new UTMLabelingEngine(_map->getSRS());
        _root->addChild(labeler);

        // Figure out the maximum labeling resolution
        StyleSheet* ss = _options->styleSheet().get();
        double maxRes = 100000.0;
        if (ss->getStyle("10000", false)) maxRes = 10000.0;
        if (ss->getStyle("1000", false)) maxRes = 1000.0;
        if (ss->getStyle("100", false)) maxRes = 100.0;
        if (ss->getStyle("10", false)) maxRes = 10.0;
        if (ss->getStyle("1", false)) maxRes = 1.0;
        labeler->setMaxResolution(maxRes);

        // Set labeler styles
        const Style* xEdgeStyle = ss->getStyle("xedge", false);
        const Style* yEdgeStyle = ss->getStyle("yedge", false);
        if (xEdgeStyle && yEdgeStyle)
            labeler->setStyles(*xEdgeStyle, *yEdgeStyle);

        osg::ref_ptr<StateSetCache> sscache = new StateSetCache();
        sscache->optimize(geomTop);
        sscache->optimize(textTop);
    }
    else
    {
        OE_WARN << LC << "SQID data file not opened" << std::endl;
    }
}

// Algorithmically builds the world GZD cells
void
MGRSGraticule::loadGZDFeatures(const SpatialReference* geosrs, FeatureList& output) const
{
    std::map<std::string, GeoExtent> _gzd;

    // build the base Grid Zone Designator (GZD) loolup table. This is a table
    // that maps the GZD string to its extent.
    static std::string s_gzdRows( "CDEFGHJKLMNPQRSTUVWX" );

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

            _gzd[ Stringify() << std::setfill('0') << std::setw(2) << (zone+1) << s_gzdRows[row] ] = cellExtent;
        }        
    }

    // the polar zones (UPS):
    _gzd["01Y"] = GeoExtent( geosrs, -180.0,  84.0,   0.0,  90.0 );
    _gzd["01Z"] = GeoExtent( geosrs,    0.0,  84.0, 180.0,  90.0 );
    _gzd["01A"] = GeoExtent( geosrs, -180.0, -90.0,   0.0, -80.0 );
    _gzd["01B"] = GeoExtent( geosrs,    0.0, -90.0, 180.0, -80.0 );

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

    // Now go through the table and create features for these things
    for (std::map<std::string, GeoExtent>::const_iterator i = _gzd.begin(); i != _gzd.end(); ++i)
    {
        const std::string& gzd = i->first;
        const GeoExtent& extent = i->second;

        Vec3dVector points;

        TessellateOperator::tessellateGeo(
            osg::Vec3d(extent.west(), extent.south(), 0),
            osg::Vec3d(extent.east(), extent.south(), 0), 
            20, GEOINTERP_RHUMB_LINE, points);
        points.resize(points.size()-1);

        TessellateOperator::tessellateGeo(
            osg::Vec3d(extent.east(), extent.south(), 0),
            osg::Vec3d(extent.east(), extent.north(), 0), 
            20, GEOINTERP_GREAT_CIRCLE, points);
        points.resize(points.size()-1);

        TessellateOperator::tessellateGeo(
            osg::Vec3d(extent.east(), extent.north(), 0),
            osg::Vec3d(extent.west(), extent.north(), 0), 
            20, GEOINTERP_RHUMB_LINE, points);
        points.resize(points.size()-1);

        TessellateOperator::tessellateGeo(
            osg::Vec3d(extent.west(), extent.north(), 0),
            osg::Vec3d(extent.west(), extent.south(), 0), 
            20, GEOINTERP_GREAT_CIRCLE, points);

        osg::ref_ptr<LineString> line = new LineString(&points);
        Feature* feature = new Feature(line.get(), geosrs);
        std::string gzd_padded = gzd.length() < 3 ? ("0" + gzd) : gzd;
        feature->set("gzd", gzd_padded);
        output.push_back(feature);
    }
}

void
MGRSGraticule::setUpDefaultStyles()
{
    float alpha = 0.35f;

    StyleSheet* styles = options().styleSheet().get();
    if (styles)
    {
        // GZD
        if (styles->getStyle("gzd", false) == 0L)
        {
            Style style("gzd");
            LineSymbol* line = style.getOrCreate<LineSymbol>();
            line->stroke()->color().set(1, 0, 0, 0.25);
            line->stroke()->width() = 4.0;
            line->tessellation() = 20;
            TextSymbol* text = style.getOrCreate<TextSymbol>();
            text->fill()->color() = Color::White;
            text->halo()->color() = Color::Black;
            text->alignment() = TextSymbol::ALIGN_LEFT_BOTTOM;
            styles->addStyle(style);
        }

        // SQID 100km (support "sqid" as an alias for "100000")
        const Style* sqid = styles->getStyle("sqid", false);
        if (sqid)
        {
            Style alias(*sqid);
            alias.setName("100000");
            styles->addStyle(alias);
        }

        if (styles->getStyle("100000", false) == 0L)
        {
            Style style("100000");
            LineSymbol* line = style.getOrCreate<LineSymbol>();
            line->stroke()->color().set(1,1,0,alpha);
            line->stroke()->width() = 3;
            TextSymbol* text = style.getOrCreate<TextSymbol>();
            text->fill()->color() = Color::White;
            text->halo()->color() = Color::Black;
            text->alignment() = TextSymbol::ALIGN_LEFT_BOTTOM;
            styles->addStyle(style);
        }

        // 10km
        if (styles->getStyle("10000", false) == 0L)
        {
            Style style("10000");
            LineSymbol* line = style.getOrCreate<LineSymbol>();
            line->stroke()->color().set(0,1,0,alpha);
            line->stroke()->width() = 2;
            styles->addStyle(style);
        }

        // 1km
        if (styles->getStyle("1000", false) == 0L)
        {
            Style style("1000");
            LineSymbol* line = style.getOrCreate<LineSymbol>();
            line->stroke()->color().set(.5,.5,1,alpha);
            line->stroke()->width() = 2;
            styles->addStyle(style);
        }

        // 100m
        if (styles->getStyle("100", false) == 0L)
        {
            Style style("100");
            LineSymbol* line = style.getOrCreate<LineSymbol>();
            line->stroke()->color().set(1,1,1,alpha);
            line->stroke()->width() = 1;
            styles->addStyle(style);
        }

        // 10m
        if (styles->getStyle("10", false) == 0L)
        {
            Style style("10");
            LineSymbol* line = style.getOrCreate<LineSymbol>();
            line->stroke()->color().set(1,1,1,alpha);
            line->stroke()->width() = 1;
            styles->addStyle(style);
        }

        // 1m
        if (styles->getStyle("1", false) == 0L)
        {
            Style style("1");
            LineSymbol* line = style.getOrCreate<LineSymbol>();
            line->stroke()->color().set(1,1,1,alpha);
            line->stroke()->width() = 0.5;
            styles->addStyle(style);
        }
    }
}
