/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2014 Pelican Mapping
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

#include "AerodromeRenderer"
#include "Common"
#include "AerodromeFeatureOptions"
#include "AerodromeNode"
#include "LightBeaconNode"
#include "LightIndicatorNode"
#include "LinearFeatureNode"
#include "PavementNode"
#include "RunwayNode"
#include "RunwayThresholdNode"
#include "StartupLocationNode"
#include "StopwayNode"
#include "TaxiwayNode"
#include "TaxiwaySignNode"
#include "TerminalNode"
#include "WindsockNode"

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/MatrixTransform>
#include <osg/Depth>
#include <osg/Texture2D>
#include <osg/PolygonOffset>
#include <osgDB/WriteFile>
#include <osgUtil/Optimizer>
#include <osgUtil/Tessellator>
#include <osgEarth/ECEF>
#include <osgEarth/ElevationQuery>
#include <osgEarth/Registry>
#include <osgEarth/Tessellator>
#include <osgEarthFeatures/BuildGeometryFilter>
#include <osgEarthFeatures/Feature>
#include <osgEarthFeatures/FeatureSource>
#include <osgEarthFeatures/GeometryCompiler>
#include <osgEarthFeatures/TransformFilter>
#include <osgEarthFeatures/FilterContext>
#include <osgEarthSymbology/MeshConsolidator>
#include <osgEarthSymbology/StyleSheet>


using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;
using namespace osgEarth::Aerodrome;

#define LC "[AerodromeRenderer] "

#define DEPTH_RANGE_MAX 1.0

// 
#define ORDER_PAVEMENT      0
#define ORDER_TAXIWAY       1
#define ORDER_RUNWAY        2
#define ORDER_STOPWAY       3
#define ORDER_LINEARFEATURE 4


// macro to set a render order and to force a negative polygon offset on ground-co-planar components.
#define SET_ORDER(NODE, NUMBER) \
    (NODE).getOrCreateStateSet()->setRenderBinDetails( (NUMBER)+_baseRenderBinNum, "RenderBin" ); \
    (NODE).getOrCreateStateSet()->setAttributeAndModes(new osg::PolygonOffset(-1, -1), osg::StateAttribute::ON)

// macro to set a render order and to force a negative polygon offset on ground-co-planar components
// for use with reverse Z buffer (Vantage style - clip control, znear: 1, far: 0, depth test: greater or gequal)
#define SET_ORDER_REV(NODE, NUMBER) \
   (NODE).getOrCreateStateSet()->setRenderBinDetails( (NUMBER)+_baseRenderBinNum, "RenderBin" ); \
   (NODE).getOrCreateStateSet()->setAttributeAndModes(new osg::PolygonOffset(1, 1), osg::StateAttribute::ON | osg::StateAttribute::PROTECTED)

AerodromeRenderer::AerodromeRenderer()
  : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN),
    _baseRenderBinNum(1)
{
 setUseReverseZBuffer(false);
}


AerodromeRenderer::~AerodromeRenderer()
{
  
}

void AerodromeRenderer::initialize(const Map* map, const osgDB::Options* options)
{
    _map = map;
    _dbOptions = new osgDB::Options( *options );
    _dbOptions->setObjectCacheHint( (osgDB::Options::CacheHintOptions)(osgDB::Options::CACHE_IMAGES | osgDB::Options::CACHE_NODES) );

    // Global session with a resource cache
    _session = new Session(map, 0L, 0L, options );
    _session->setResourceCache( new ResourceCache() );
}

void
AerodromeRenderer::apply(AerodromeNode& node)
{
    // don't rerender if unnecessary 
    if (node.getRendered())
        return;

    node.setRendered(true);

    if (node.getBoundary() && node.getBoundary()->getFeature()->getGeometry())
    {
        osg::ref_ptr<BoundaryNode> boundary = node.getBoundary();
        osg::ref_ptr<Geometry> featureGeom = boundary->getFeature()->getGeometry();

        // create localizations for this aerodrome
        createLocalizations(featureGeom->getBounds(), boundary.get());
    }
    else
    {
        // node does not have a boundary or geometry so do not render
        return;
    }

    traverse(node);
}

void
AerodromeRenderer::apply(LightBeaconNode& node)
{
    osg::ref_ptr<osgEarth::Features::Feature> feature = node.getFeature();

    osg::Node* geom;

    if (node.getOptions().modelOptions().size() > 0)
        geom = featureModelRenderer(feature.get(), node.getOptions().modelOptions());
    else
        geom = defaultFeatureRenderer(feature.get(), Color::Yellow);

    if (geom)
    {
        node.addChild(geom);
    }
}

void
AerodromeRenderer::apply(LightIndicatorNode& node)
{
    osg::ref_ptr<osgEarth::Features::Feature> feature = node.getFeature();

    osg::Node* geom;

    if (node.getOptions().modelOptions().size() > 0)
        geom = featureModelRenderer(feature.get(), node.getOptions().modelOptions());
    else
        geom = defaultFeatureRenderer(feature.get(), Color::Red);

    if (geom)
    {
        node.addChild(geom);
    }
}

void
AerodromeRenderer::apply(LinearFeatureNode& node)
{
    osg::ref_ptr<osgEarth::Features::Feature> feature = osg::clone( node.getFeature(), osg::CopyOp::DEEP_COPY_ALL );

    osg::ref_ptr<osg::Node> geom;

    GeometryIterator iter( feature->getGeometry() );
    while( iter.hasMore() )
    {
        Geometry* part = iter.next();

        for(unsigned i=0; i<part->size(); ++i)
        {
            (*part)[i].z() = _elevation;
        }

        osg::ref_ptr<osg::Vec3Array> verts = new osg::Vec3Array();
        transformAndLocalize( part->asVector(), feature->getSRS(), verts.get(), 0L );
        for(int j=0; j<verts->size(); ++j)
            (*part)[j] = (*verts)[j];
    }

    Feature* linearFeature = new Feature(feature->getGeometry(), 0L);

    // setup the style
    Style style;
    style.getOrCreate<LineSymbol>()->stroke()->color() = Color(0.75f, 0.65f, 0.15f, 0.8f);
    style.getOrCreate<LineSymbol>()->stroke()->width() = 1.0f;
    style.getOrCreate<LineSymbol>()->stroke()->widthUnits() = Units::METERS;

    style.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_NONE;

    // use the BuildGeometryFilter to render the linear feature
    BuildGeometryFilter filter( style );

    FeatureList workingSet;
    workingSet.push_back(linearFeature);

    FilterContext context;
    osg::Node* filterNode = filter.push( workingSet, context );

    if (filterNode)
    {
        osgUtil::Optimizer optimizer;
        optimizer.optimize( filterNode, optimizer.MERGE_GEOMETRY );

        osg::MatrixTransform* mt = new osg::MatrixTransform();
        mt->setMatrix(_local2world);
        mt->addChild(filterNode);

        geom = mt;
    }

    if ( geom.valid() )
    {
       setDepthState(geom);
       geom->setName(node.icao() + "_LINEAR_FEATURES");
        node.addChild( geom );
    }

    setBinAndOffset(node, ORDER_LINEARFEATURE);
}

void
AerodromeRenderer::apply(PavementNode& node)
{
    osg::ref_ptr<osgEarth::Features::Feature> feature = node.getFeature();

    osg::ref_ptr<osg::Node> geom;

    if (node.getOptions().textureOptions().isSet() && node.getOptions().textureOptions()->url().isSet())
    {
        geom = featureSingleTextureRenderer(feature.get(), node.getOptions().textureOptions()->url().value(), node.getOptions().textureOptions()->length().isSet() ? node.getOptions().textureOptions()->length().value() : 10.0);
    }
    else
    {
        geom = defaultFeatureRenderer(feature.get(), Color(0.6, 0.6, 0.6, 1.0));
    }

     if (geom.valid())
    {
        setDepthState(geom);
        //geom->getOrCreateStateSet()->setRenderBinDetails(0, "RenderBin");
        geom->setName(node.icao() + "_PAVEMENT");
        node.addChild(geom);
    }

    setBinAndOffset(node, ORDER_PAVEMENT);
}

void
AerodromeRenderer::apply(RunwayNode& node)
{
    osg::ref_ptr<osgEarth::Features::Feature> feature = node.getFeature();

    osg::ref_ptr<osg::Node> geom;

    osg::ref_ptr<osg::Vec3dArray> geomPoints = feature->getGeometry()->createVec3dArray();
    if (geomPoints.valid() && geomPoints->size() == 4)
    {
        std::vector<osg::Vec3d> featurePoints;
        for (int i=0; i < geomPoints->size(); i++)
        {
            featurePoints.push_back(osg::Vec3d((*geomPoints)[i].x(), (*geomPoints)[i].y(), _elevation));
        }
     
        //osg::Vec3Array* normals = new osg::Vec3Array();
        osg::Vec3Array* verts = new osg::Vec3Array();
        transformAndLocalize(featurePoints, feature->getSRS(), verts, 0L);

        for(int i=0; i<verts->size(); ++i)
            (*verts)[i].z() = 0.0;

        osg::Geometry* geometry = new osg::Geometry();
        geometry->setVertexArray( verts );

        osg::Vec3Array* normals = new osg::Vec3Array();
        normals->push_back( osg::Vec3(0.0f, 0.0f, 1.0f) );
        geometry->setNormalArray( normals );
        geometry->setNormalBinding( osg::Geometry::BIND_OVERALL );

        osg::Vec4Array* colors = new osg::Vec4Array;

        if (node.getOptions().textureOptions().isSet() && node.getOptions().textureOptions()->url().isSet())
        {
            osg::Image* tex = node.getOptions().textureOptions()->url()->getImage(_dbOptions.get());
            if (tex)
            {
                osg::Texture2D* _texture = new osg::Texture2D(tex);
                _texture->setWrap(_texture->WRAP_S, _texture->CLAMP_TO_EDGE);
                _texture->setWrap(_texture->WRAP_T, _texture->REPEAT);
                //_texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::NEAREST); 
                //_texture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::NEAREST);
                _texture->setResizeNonPowerOfTwoHint(false);
                geometry->getOrCreateStateSet()->setTextureAttributeAndModes(0, _texture, osg::StateAttribute::ON);

                osg::Vec2Array* tcoords = new osg::Vec2Array(4);

                float side1 = ((*verts)[1] - (*verts)[0]).length();
                float side2 = ((*verts)[2] - (*verts)[1]).length();

                float width = osg::minimum(side1, side2);
                float scale = width / tex->getPixelAspectRatio();

                float length = osg::maximum(side1, side2);
                float repeat = length / scale;

                if (side1 > side2)
                {
                    (*tcoords)[3].set(0.0f,0.0f);
                    (*tcoords)[0].set(1.0f,0.0f);
                    (*tcoords)[1].set(1.0f,repeat);
                    (*tcoords)[2].set(0.0f,repeat);
                }
                else
                {
                    (*tcoords)[0].set(0.0f,0.0f);
                    (*tcoords)[1].set(1.0f,0.0f);
                    (*tcoords)[2].set(1.0f,repeat);
                    (*tcoords)[3].set(0.0f,repeat);
                }
            
                geometry->setTexCoordArray(0,tcoords);
            }
            else
            {
                OE_WARN << LC << "Error reading texture file: " << node.getOptions().textureOptions()->url()->full() << std::endl;
            }



            colors->push_back(osg::Vec4(1.0f,1.0f,1.0f,1.0f));
        }
        else
        {
            colors->push_back(osg::Vec4(0.4f,0.4f,0.4f,1.0f));
        }

        geometry->setColorArray(colors);
        geometry->setColorBinding(osg::Geometry::BIND_OVERALL);

        geometry->addPrimitiveSet( new osg::DrawArrays( GL_POLYGON, 0, verts->size() ) );

        geometry->setName(node.icao() + "_RUNWAY");

        //osgEarth::Tessellator tess;
        //tess.tessellateGeometry(*geometry);

        osg::ref_ptr<osg::Geode> geode = new osg::Geode;
        geode->addDrawable(geometry);

        //Registry::shaderGenerator().run(geode.get(), "osgEarth.AerodromeRenderer");

        osg::MatrixTransform* mt = new osg::MatrixTransform();
        mt->setMatrix(_local2world);
        mt->addChild(geode);

        geom = mt;
    }
    else
    {
        geom = defaultFeatureRenderer(feature.get(), Color(0.4, 0.4, 0.4, 1.0));
    }

    if (geom.valid())
    {
        setDepthState(geom);
        node.addChild(geom);
    }

    setBinAndOffset(node, ORDER_RUNWAY);
}


void
AerodromeRenderer::apply(RunwayThresholdNode& node)
{
    // We don't necessarily want to runway thresholds (metadata)
    //osg::ref_ptr<osgEarth::Features::Feature> feature = node.getFeature();
    //osg::Node* geom = defaultFeatureRenderer(feature.get(), Color::White);
    //if (geom)
    //    node.addChild(geom);
}

void
AerodromeRenderer::apply(StartupLocationNode& node)
{
    // We don't necessarily want to render statup locations (metadata)
    //osg::ref_ptr<osgEarth::Features::Feature> feature = node.getFeature();
    //osg::Node* geom = defaultFeatureRenderer(feature.get(), Color::White);
    //if (geom)
    //    node.addChild(geom);
}

void
AerodromeRenderer::apply(StopwayNode& node)
{
   osg::ref_ptr<const Map> refMap;
   if (!_map.lock(refMap))
   {
      return;
   }
   
    osg::ref_ptr<osgEarth::Features::Feature> feature = node.getFeature();

    osg::ref_ptr<osg::Node> geom;

    osg::ref_ptr<osg::Vec3dArray> geomPoints = feature->getGeometry()->createVec3dArray();
    if (geomPoints.valid() && geomPoints->size() == 4)
    {
        std::vector<osg::Vec3d> featurePoints;
        for (int i=0; i < geomPoints->size(); i++)
        {
            featurePoints.push_back(osg::Vec3d((*geomPoints)[i].x(), (*geomPoints)[i].y(), _elevation));
        }
     
        //osg::Vec3Array* normals = new osg::Vec3Array();
        osg::Vec3Array* verts = new osg::Vec3Array();
        transformAndLocalize(featurePoints, feature->getSRS(), verts, 0L);

        for(int i=0; i<verts->size(); ++i)
            (*verts)[i].z() = 0.0;

        osg::Geometry* geometry = new osg::Geometry();
        geometry->setVertexArray( verts );

        osg::Vec3Array* normals = new osg::Vec3Array();
        normals->push_back( osg::Vec3(0.0f, 0.0f, 1.0f) );
        geometry->setNormalArray( normals );
        geometry->setNormalBinding( osg::Geometry::BIND_OVERALL );

        osg::Vec4Array* colors = new osg::Vec4Array;

        if (node.getOptions().textureOptions().isSet() && node.getOptions().textureOptions()->url().isSet())
        {
            osg::Image* tex = node.getOptions().textureOptions()->url()->getImage(_dbOptions.get());
            if (tex)
            {
                osg::Texture2D* _texture = new osg::Texture2D(tex);
                _texture->setWrap(_texture->WRAP_S, _texture->CLAMP_TO_EDGE);
                _texture->setWrap(_texture->WRAP_T, _texture->REPEAT);
                //_texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::NEAREST); 
                //_texture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::NEAREST);
                _texture->setResizeNonPowerOfTwoHint(false);
                geometry->getOrCreateStateSet()->setTextureAttributeAndModes(0, _texture, osg::StateAttribute::ON);

                osg::Vec2Array* tcoords = new osg::Vec2Array(4);

                osg::Vec3d refPoint = transformAndLocalize(node.getReferencePoint(), refMap->getSRS());

                float side1 = (refPoint - (((*verts)[1] + (*verts)[0]) / 2.0)).length();
                float side2 = (refPoint - (((*verts)[2] + (*verts)[1]) / 2.0)).length();
                float side3 = (refPoint - (((*verts)[3] + (*verts)[2]) / 2.0)).length();
                float side4 = (refPoint - (((*verts)[0] + (*verts)[3]) / 2.0)).length();

                if (side1 < side2 && side1 < side3 && side1 < side4)
                {
                    float width = ((*verts)[1] - (*verts)[0]).length();
                    float scale = width / tex->getPixelAspectRatio();
                    float length = ((*verts)[0] - (*verts)[3]).length();
                    float repeat = length / scale;

                    (*tcoords)[2].set(0.0f,0.0f);
                    (*tcoords)[3].set(1.0f,0.0f);
                    (*tcoords)[0].set(1.0f,repeat);
                    (*tcoords)[1].set(0.0f,repeat);
                }
                else if (side2 < side3 && side2 < side4)
                {
                    float width = ((*verts)[2] - (*verts)[1]).length();
                    float scale = width / tex->getPixelAspectRatio();
                    float length = ((*verts)[1] - (*verts)[0]).length();
                    float repeat = length / scale;

                    (*tcoords)[3].set(0.0f,0.0f);
                    (*tcoords)[0].set(1.0f,0.0f);
                    (*tcoords)[1].set(1.0f,repeat);
                    (*tcoords)[2].set(0.0f,repeat);
                }
                else if (side3 < side4)
                {
                    float width = ((*verts)[3] - (*verts)[2]).length();
                    float scale = width / tex->getPixelAspectRatio();
                    float length = ((*verts)[2] - (*verts)[1]).length();
                    float repeat = length / scale;

                    (*tcoords)[0].set(0.0f,0.0f);
                    (*tcoords)[1].set(1.0f,0.0f);
                    (*tcoords)[2].set(1.0f,repeat);
                    (*tcoords)[3].set(0.0f,repeat);
                }
                else
                {
                    float width = ((*verts)[0] - (*verts)[3]).length();
                    float scale = width / tex->getPixelAspectRatio();
                    float length = ((*verts)[3] - (*verts)[2]).length();
                    float repeat = length / scale;

                    (*tcoords)[1].set(0.0f,0.0f);
                    (*tcoords)[2].set(1.0f,0.0f);
                    (*tcoords)[3].set(1.0f,repeat);
                    (*tcoords)[0].set(0.0f,repeat);
                }

                geometry->setTexCoordArray(0,tcoords);
            }
            else
            {
                OE_WARN << LC << "Error reading texture file: " << node.getOptions().textureOptions()->url()->full() << std::endl;
            }



            colors->push_back(osg::Vec4(1.0f,1.0f,1.0f,1.0f));
        }
        else
        {
            colors->push_back(osg::Vec4(0.4f,0.4f,0.4f,1.0f));
        }

        geometry->setColorArray(colors);
        geometry->setColorBinding(osg::Geometry::BIND_OVERALL);

        geometry->addPrimitiveSet( new osg::DrawArrays( GL_POLYGON, 0, verts->size() ) );

        geometry->setName(node.icao() + "_STOPWAY");

        //osgEarth::Tessellator tess;
        //tess.tessellateGeometry(*geometry);

        osg::ref_ptr<osg::Geode> geode = new osg::Geode;
        geode->addDrawable(geometry);

        //Registry::shaderGenerator().run(geode.get(), "osgEarth.AerodromeRenderer");

        osg::MatrixTransform* mt = new osg::MatrixTransform();
        mt->setMatrix(_local2world);
        mt->addChild(geode);

        geom = mt;
    }
    else
    {
        geom = defaultFeatureRenderer(feature.get(), Color::White);
    }

    if (geom.valid())
    {
        setDepthState(geom);
        //geom->getOrCreateStateSet()->setRenderBinDetails(0, "RenderBin");
        node.addChild(geom);
    }

    setBinAndOffset(node, ORDER_STOPWAY);
}

void
AerodromeRenderer::apply(TaxiwayNode& node)
{
    osg::ref_ptr<osgEarth::Features::Feature> feature = node.getFeature();

    osg::ref_ptr<osg::Node> geom;

    if (node.getOptions().textureOptions().isSet() && node.getOptions().textureOptions()->url().isSet())
    {
        geom = featureSingleTextureRenderer(feature.get(), node.getOptions().textureOptions()->url().value(), node.getOptions().textureOptions()->length().isSet() ? node.getOptions().textureOptions()->length().value() : 10.0);
    }
    else
    {
        geom = defaultFeatureRenderer(feature.get(), Color(0.6, 0.6, 0.6, 1.0));
    }

    if (geom.valid())
    {
        setDepthState(geom);
        //geom->getOrCreateStateSet()->setRenderBinDetails(0, "RenderBin");
        geom->setName(node.icao() + "_TAXIWAY");
        node.addChild(geom);
    }

    setBinAndOffset(node, ORDER_TAXIWAY);
}

void
AerodromeRenderer::apply(TaxiwaySignNode& node)
{
    osg::ref_ptr<osgEarth::Features::Feature> feature = node.getFeature();

    osg::Node* geom;

    if (node.getOptions().modelOptions().size() > 0)
        geom = featureModelRenderer(feature.get(), node.getOptions().modelOptions());
    else
        geom = defaultFeatureRenderer(feature.get(), Color::Yellow);

    if (geom)
    {
        //geom->getOrCreateStateSet()->setRenderBinDetails(1, "RenderBin");
        node.addChild(geom);
    }
}

void
AerodromeRenderer::apply(TerminalNode& node)
{
    osg::ref_ptr<osgEarth::Features::Feature> feature = node.getFeature();

    osg::Node* geom;
    
    if (node.getOptions().skinsUrl().isSet())
    {
        ResourceLibrary* reslib = new ResourceLibrary( "terminal_resources", node.getOptions().skinsUrl().value() );

        // a style for the building data:
        Style buildingStyle;
        buildingStyle.setName( "buildings" );

        // Extrude the shapes into 3D buildings.
        ExtrusionSymbol* extrusion = buildingStyle.getOrCreate<ExtrusionSymbol>();
        extrusion->height() = 25.0;
        extrusion->flatten() = true;
        extrusion->wallStyleName() = "building-wall";
        extrusion->roofStyleName() = "building-roof";

        PolygonSymbol* poly = buildingStyle.getOrCreate<PolygonSymbol>();
        poly->fill()->color() = Color::White;

        // Clamp the buildings to the terrain.
        AltitudeSymbol* alt = buildingStyle.getOrCreate<AltitudeSymbol>();
        alt->clamping() = AltitudeSymbol::CLAMP_NONE;
        alt->verticalOffset() = _elevation;

        // a style for the wall textures:
        Style wallStyle;
        wallStyle.setName( "building-wall" );
        SkinSymbol* wallSkin = wallStyle.getOrCreate<SkinSymbol>();
        wallSkin->library() = "terminal_resources";

        for (std::vector<std::string>::const_iterator it = node.getOptions().wallTags().begin(); it != node.getOptions().wallTags().end(); ++it)
            wallSkin->addTag( *it );

        if (wallSkin->tags().size() == 0)
          wallSkin->addTag("building");

        //wallSkin->randomSeed() = 1;

        // a style for the rooftop textures:
        Style roofStyle;
        roofStyle.setName( "building-roof" );
        SkinSymbol* roofSkin = roofStyle.getOrCreate<SkinSymbol>();
        roofSkin->library() = "terminal_resources";

        for (std::vector<std::string>::const_iterator it = node.getOptions().roofTags().begin(); it != node.getOptions().roofTags().end(); ++it)
            roofSkin->addTag( *it );

        if (roofSkin->tags().size() == 0)
          roofSkin->addTag("rooftop");

        //roofSkin->randomSeed() = 1;
        roofSkin->isTiled() = true;

        // assemble a stylesheet and add our styles to it:
        osg::ref_ptr<StyleSheet> styleSheet = new StyleSheet();
        styleSheet->addResourceLibrary(reslib);
        styleSheet->addStyle( wallStyle );
        styleSheet->addStyle( roofStyle );

        geom = defaultFeatureRenderer(feature.get(), buildingStyle, styleSheet.get());
    }
    else
    {
        geom = defaultFeatureRenderer(feature.get(), Color::Red, 25.0);
    }

    if (geom)
    {
        //geom->getOrCreateStateSet()->setRenderBinDetails(1, "RenderBin");
        node.addChild(geom);
    }

}

void
AerodromeRenderer::apply(WindsockNode& node)
{
    osg::ref_ptr<osgEarth::Features::Feature> feature = node.getFeature();

    osg::Node* geom;

    if (node.getOptions().modelOptions().size() > 0)
        geom = featureModelRenderer(feature.get(), node.getOptions().modelOptions());
    else
        geom = defaultFeatureRenderer(feature.get(), Color::Orange);

    if (geom)
    {
        //geom->getOrCreateStateSet()->setRenderBinDetails(1, "RenderBin");
        node.addChild(geom);
    }
}

void
AerodromeRenderer::apply(LightBeaconGroup& group)
{
    traverse(group);
}

void
AerodromeRenderer::apply(LightIndicatorGroup& group)
{
    traverse(group);
}

void
AerodromeRenderer::apply(LinearFeatureGroup& group)
{
    traverse(group);
    group.getOrCreateStateSet()->setAttributeAndModes( new osg::Depth(osg::Depth::LEQUAL, 0.0, DEPTH_RANGE_MAX, false) );
    //group.getOrCreateStateSet()->setRenderBinDetails(0, "RenderBin");
}

void
AerodromeRenderer::apply(PavementGroup& group)
{
    traverse(group);
}

void
AerodromeRenderer::apply(RunwayGroup& group)
{
    traverse(group);
}

void
AerodromeRenderer::apply(RunwayThresholdGroup& group)
{
    traverse(group);
}

void
AerodromeRenderer::apply(StartupLocationGroup& group)
{
    traverse(group);
}

void
AerodromeRenderer::apply(StopwayGroup& group)
{
    traverse(group);
}

void
AerodromeRenderer::apply(TaxiwayGroup& group)
{
    traverse(group);
}

void
AerodromeRenderer::apply(TaxiwaySignGroup& group)
{
    traverse(group);
}

void
AerodromeRenderer::apply(TerminalGroup& group)
{
    traverse(group);
}

void
AerodromeRenderer::apply(WindsockGroup& group)
{
    traverse(group);
}

void
AerodromeRenderer::apply(osg::Group& node)
{
    if (dynamic_cast<AerodromeNode*>(&node))
        apply(static_cast<AerodromeNode&>(node));
    else if (dynamic_cast<LightBeaconNode*>(&node))
        apply(static_cast<LightBeaconNode&>(node));
    else if (dynamic_cast<LightIndicatorNode*>(&node))
        apply(static_cast<LightIndicatorNode&>(node));
    else if (dynamic_cast<LinearFeatureNode*>(&node))
        apply(static_cast<LinearFeatureNode&>(node));
    else if (dynamic_cast<PavementNode*>(&node))
        apply(static_cast<PavementNode&>(node));
    else if (dynamic_cast<RunwayNode*>(&node))
        apply(static_cast<RunwayNode&>(node));
    else if (dynamic_cast<RunwayThresholdNode*>(&node))
        apply(static_cast<RunwayThresholdNode&>(node));
    else if (dynamic_cast<StartupLocationNode*>(&node))
        apply(static_cast<StartupLocationNode&>(node));
    else if (dynamic_cast<StopwayNode*>(&node))
        apply(static_cast<StopwayNode&>(node));
    else if (dynamic_cast<TaxiwayNode*>(&node))
        apply(static_cast<TaxiwayNode&>(node));
    else if (dynamic_cast<TaxiwaySignNode*>(&node))
        apply(static_cast<TaxiwaySignNode&>(node));
    else if (dynamic_cast<TerminalNode*>(&node))
        apply(static_cast<TerminalNode&>(node));
    else if (dynamic_cast<WindsockNode*>(&node))
        apply(static_cast<WindsockNode&>(node));
    else if (dynamic_cast<LightBeaconGroup*>(&node))
        apply(static_cast<LightBeaconGroup&>(node));
    else if (dynamic_cast<LightIndicatorGroup*>(&node))
        apply(static_cast<LightIndicatorGroup&>(node));
    else if (dynamic_cast<LinearFeatureGroup*>(&node))
        apply(static_cast<LinearFeatureGroup&>(node));
    else if (dynamic_cast<PavementGroup*>(&node))
        apply(static_cast<PavementGroup&>(node));
    else if (dynamic_cast<RunwayGroup*>(&node))
        apply(static_cast<RunwayGroup&>(node));
    else if (dynamic_cast<RunwayThresholdGroup*>(&node))
        apply(static_cast<RunwayThresholdGroup&>(node));
    else if (dynamic_cast<StartupLocationGroup*>(&node))
        apply(static_cast<StartupLocationGroup&>(node));
    else if (dynamic_cast<StopwayGroup*>(&node))
        apply(static_cast<StopwayGroup&>(node));
    else if (dynamic_cast<TaxiwayGroup*>(&node))
        apply(static_cast<TaxiwayGroup&>(node));
    else if (dynamic_cast<TaxiwaySignGroup*>(&node))
        apply(static_cast<TaxiwaySignGroup&>(node));
    else if (dynamic_cast<TerminalGroup*>(&node))
        apply(static_cast<TerminalGroup&>(node));
    else if (dynamic_cast<WindsockGroup*>(&node))
        apply(static_cast<WindsockGroup&>(node));
    else
        traverse(node);
}

template <typename T, typename Y>
osgEarth::Symbology::Geometry* AerodromeRenderer::combineGeometries(T& group)
{
    osg::ref_ptr<Geometry> combGeom = 0L;
    for (int c=0; c < group.getNumChildren(); c++)
    {
        Y* childNode = dynamic_cast<Y*>(group.getChild(c));
        if (childNode)
        {
            osg::ref_ptr<osgEarth::Features::Feature> feature = childNode->getFeature();
            Polygon* featurePoly = dynamic_cast<Polygon*>(feature->getGeometry());
            if (featurePoly)
            {
                if (!combGeom.valid())
                {
                    combGeom = new Polygon(*featurePoly);
                }
                else
                {
                    osg::ref_ptr<Geometry> copy = combGeom.get();
                    combGeom = 0L;

                    if (!featurePoly->geounion(copy, combGeom))
                    {
                        
                        OE_WARN << LC << "GEOS union failed. (" << childNode->icao() << ")" << std::endl;
                    }
                }
            }
        }
    }

    return combGeom.release();
}

osg::Node*
AerodromeRenderer::featureSingleTextureRenderer(osgEarth::Features::Feature* feature, const osgEarth::URI& uri, float length)
{
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;

    osg::ref_ptr<osg::Node> geom;
    GeometryIterator iter( feature->getGeometry(), false );
    while( iter.hasMore() )
    {
        Geometry* polygon = iter.next();

        osg::Geometry* geometry = new osg::Geometry();
            
        std::vector<osg::Vec3d> featurePoints;
        std::vector<unsigned>   partOffset;
        unsigned offset = 0;

        GeometryIterator polygonIter( polygon, true );
        while( polygonIter.hasMore() )
        {
            Geometry* part = polygonIter.next();

            osg::ref_ptr<osg::Vec3dArray> partVerts = part->createVec3dArray();
            if ( partVerts.valid() && partVerts->size() > 2 )
            {
                partOffset.push_back(offset);
                offset += partVerts->size();

                for (int i=0; i < partVerts->size(); i++)
                {
                    featurePoints.push_back(osg::Vec3d((*partVerts)[i].x(), (*partVerts)[i].y(), _elevation));
                }
            }
        }
        partOffset.push_back(featurePoints.size());

        // transform all verts into the local coordinate frame:
        osg::Vec3Array* verts = new osg::Vec3Array();
        geometry->setVertexArray( verts );
        transformAndLocalize(featurePoints, feature->getSRS(), verts, 0L);

        // re-initialize all Z values to zero:
        for(int i=0; i<verts->size(); ++i)
            (*verts)[i].z() = 0.0;

        // set up all the texture coordinates:
        osg::Vec2Array* tcoords = new osg::Vec2Array(verts->size());
        for (int i=0; i < verts->size(); i++)
        {
            float tcX = ((*verts)[i].x() - _localMin.x()) / length;
            float tcY = ((*verts)[i].y() - _localMin.y()) / length;

            (*tcoords)[i].set(tcX, tcY);
        }            
        geometry->setTexCoordArray(0,tcoords);

        for(int i=0; i<partOffset.size()-1; ++i)
        {
            geometry->addPrimitiveSet( new osg::DrawArrays(GL_LINE_LOOP, partOffset[i], partOffset[i+1]-partOffset[i]) );
        }

        osgEarth::Tessellator tess;
        bool tessOK = tess.tessellateGeometry(*geometry);

        if ( !tessOK )
        {
            osgUtil::Tessellator tess2;
            tess2.setTessellationType( osgUtil::Tessellator::TESS_TYPE_GEOMETRY );
            tess2.setWindingType( osgUtil::Tessellator::TESS_WINDING_POSITIVE );
            tess2.retessellatePolygons( *geometry );
        }

        // remove the color/normal arrays; don't need them
        geometry->setColorArray(0L);
        geometry->setNormalArray(0L);

        if ( !tessOK )
        {
            // The OSG tessellator generates strips and fans, which we don't want because
            // the geometry merge optimization won't work at its full potential
            MeshConsolidator::convertToTriangles( *geometry );
        }

        geode->addDrawable(geometry);
    }

    if ( geode->getNumDrawables() > 0 )
    {
        unsigned before = geode->getNumDrawables();
        osgUtil::Optimizer optimizer;
        optimizer.optimize( geode, optimizer.MERGE_GEOMETRY );
        unsigned after = geode->getNumDrawables();

        OE_DEBUG << "Merge from " << before << " to " << after << std::endl;

        osg::Vec4Array* colors = new osg::Vec4Array;
        colors->push_back(osg::Vec4(1.0f,1.0f,1.0f,1.0f));
        
        osg::Vec3Array* normals = new osg::Vec3Array();
        normals->push_back( osg::Vec3(0.0f, 0.0f, 1.0f) );

        for(unsigned i=0; i<geode->getNumDrawables(); ++i)
        {
            geode->getDrawable(i)->asGeometry()->setNormalArray( normals );
            geode->getDrawable(i)->asGeometry()->setNormalBinding( osg::Geometry::BIND_OVERALL );
        
            geode->getDrawable(i)->asGeometry()->setColorArray(colors);
            geode->getDrawable(i)->asGeometry()->setColorBinding(osg::Geometry::BIND_OVERALL);
        }

        osg::Image* tex = uri.getImage(_dbOptions.get());
        if (tex)
        {
            osg::Texture2D* _texture = new osg::Texture2D(tex);     
            _texture->setWrap(_texture->WRAP_S, _texture->REPEAT);
            _texture->setWrap(_texture->WRAP_T, _texture->REPEAT);
            _texture->setResizeNonPowerOfTwoHint(false);
            geode->getOrCreateStateSet()->setTextureAttributeAndModes(0, _texture, osg::StateAttribute::ON);
        }            
        else
        {
            OE_WARN << LC << "Error reading texture file: " << uri.full() << std::endl;
        }
        
        osg::MatrixTransform* mt = new osg::MatrixTransform();
        mt->setMatrix(_local2world);
        mt->addChild(geode);

        geom = mt;
    }

    return geom.release();
}

osg::Node*
AerodromeRenderer::featureModelRenderer(osgEarth::Features::Feature* feature, const ModelOptionsSet& modelOptions)
{
    Style style;

    ModelSymbol* model = style.getOrCreate<ModelSymbol>();
    model->scale()->setLiteral( 1.0 );
    model->placement() = model->PLACEMENT_VERTEX;

    for (ModelOptionsSet::const_iterator i = modelOptions.begin(); i != modelOptions.end(); i++)
    {
        if (!i->selector().isSet() ||
            (i->selector()->attr().isSet() && i->selector()->value().isSet() && feature->getString(i->selector()->attr().value()) == i->selector()->value().value()))
        {
            // construct url string w/ scale in necessary
            float scale = i->scale().isSet() ? i->scale().get() : 1.0f;
            std::string url = i->url().isSet() ? i->url().value().full() : "";
            std::string modelUrl = url + (scale != 1.0f ? "." + toString(scale) + ".scale" : "");
            model->url()->setLiteral(modelUrl);

            std::string heading = i->headingAttr().isSet() ? feature->getString(i->headingAttr().value()) : "";
            if (heading.length() > 0)
                model->heading() = heading;
        }
    }

    style.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_NONE;
    style.getOrCreate<AltitudeSymbol>()->verticalOffset() = _elevation;

    return defaultFeatureRenderer(feature, style);
}

osg::Node*
AerodromeRenderer::defaultFeatureRenderer(osgEarth::Features::Feature* feature, float height)
{
    //random color
    float r = (float)rand() / (float)RAND_MAX;
    float g = (float)rand() / (float)RAND_MAX;
    float b = (float)rand() / (float)RAND_MAX;

    return defaultFeatureRenderer(feature, Color(r, g, b, 1.0), height);
}

osg::Node*
AerodromeRenderer::defaultFeatureRenderer(osgEarth::Features::Feature* feature, const Color& color, float height)
{
    Style style;

    if (feature->getGeometry()->getType() == osgEarth::Symbology::Geometry::TYPE_POLYGON)
    {
        style.getOrCreate<PolygonSymbol>()->fill()->color() = color;
    }
    else if (feature->getGeometry()->getType() == osgEarth::Symbology::Geometry::TYPE_POINTSET)
    {
        style.getOrCreate<PointSymbol>()->fill()->color() = color;
        style.getOrCreate<PointSymbol>()->size() = 2.0f;
    }
    else
    {
        style.getOrCreate<LineSymbol>()->stroke()->color() = color;
        style.getOrCreate<LineSymbol>()->stroke()->width() = 1.0f;
        style.getOrCreate<LineSymbol>()->stroke()->widthUnits() = Units::METERS;
    }

    if (height > 0.0)
    {
        style.getOrCreate<ExtrusionSymbol>()->height() = height;
    }

    style.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_NONE;
    style.getOrCreate<AltitudeSymbol>()->verticalOffset() = _elevation;

    return defaultFeatureRenderer(feature, style);
}

osg::Node*
AerodromeRenderer::defaultFeatureRenderer(osgEarth::Features::Feature* feature, const Style& style, StyleSheet* styleSheet)
{
    if (feature)
    {
        if (styleSheet)
            _session->setStyles(styleSheet);

        GeoExtent extent(feature->getSRS(), feature->getGeometry()->getBounds());
        osg::ref_ptr<FeatureProfile> profile = new FeatureProfile( extent );
        FilterContext context(_session.get(), profile.get(), extent );

        // disable shader generation so we can do it later, once.
        GeometryCompilerOptions go;
        go.shaderPolicy() = osgEarth::SHADERPOLICY_INHERIT;

        GeometryCompiler compiler( go );
        osg::ref_ptr< Feature > clone = new Feature(*feature, osg::CopyOp::DEEP_COPY_ALL);
        return compiler.compile( clone.get(), (clone->style().isSet() ? *clone->style() : style), context );
    }

    return 0L;
}

void
AerodromeRenderer::createLocalizations(const osgEarth::Bounds& bounds, BoundaryNode* boundary)
{
   osg::ref_ptr<const Map> refMap;
   if (_map.lock(refMap))
   {
      // use the center of the bounds as an anchor for localization
      GeoPoint anchor(boundary->getFeature()->getSRS(), bounds.center().x(), bounds.center().y());
      anchor = anchor.transform(refMap->getSRS());

      // get a common elevation for the aerodrome
      if (boundary && boundary->hasElevation())
      {
         _elevation = boundary->elevation();
      }
      else
      {
         ElevationQuery eq(refMap.get());
         eq.getElevation(anchor, _elevation);
         OE_WARN << LC << "No elevation data in boundary; using an elevation query" << std::endl;
      }

      // create a w2l matrix for the aerodrome
      GeoPoint p(anchor.getSRS(), anchor.x(), anchor.y(), _elevation, ALTMODE_ABSOLUTE);
      p.createLocalToWorld(_local2world);
      _world2local.invert(_local2world);

      // find the local min point (lower-left), used for calculating site-wide texture coords
      GeoPoint vert(boundary->getFeature()->getSRS(), osg::Vec3d(bounds.xMin(), bounds.yMin(), _elevation), osgEarth::ALTMODE_ABSOLUTE);

      osg::Vec3d world;
      vert.toWorld(world);
      _localMin = world * _world2local;
      _localMin.z() = 0.0;
   }
}

void
AerodromeRenderer::transformAndLocalize(const std::vector<osg::Vec3d>& input,
                                        const SpatialReference*        inputSRS,
                                        osg::Vec3Array*                output_verts,
                                        osg::Vec3Array*                output_normals)
{
    output_verts->reserve( output_verts->size() + input.size() );

    if ( output_normals )
        output_normals->reserve( output_verts->size() );

    for (int i=0; i < input.size(); i++)
    {
        GeoPoint vert(inputSRS, input[i], osgEarth::ALTMODE_ABSOLUTE);

        osg::Vec3d world;
        vert.toWorld(world);
        osg::Vec3d local = world * _world2local;
        local.z() = 0.0;
        output_verts->push_back(local);
    }
}

osg::Vec3d
AerodromeRenderer::transformAndLocalize(const osg::Vec3d& input, const SpatialReference* inputSRS)
{
    GeoPoint vert(inputSRS, input, osgEarth::ALTMODE_ABSOLUTE);

    osg::Vec3d world;
    vert.toWorld(world);
    osg::Vec3d local = world * _world2local;
    local.z() = 0.0;

    return local;
}


void AerodromeRenderer::setBinAndOffset(AerodromeFeatureNode& node, int bin)
{
   if (_useReverseZBuffer)
   {
      SET_ORDER_REV(node, bin);
   }
   else
   {
      SET_ORDER(node, bin);
   }
}

void AerodromeRenderer::setDepthState(osg::ref_ptr<osg::Node> geom)
{
   if (_useReverseZBuffer)
   {
      // I would have thought this was DEPTH_RANGE_MAX to 0, but it makes the 
      // objects way too close in depth buffer (do nSight capture with false -> true
      // to turn z writes on)
      // I think the reverse z buffer projection matrix remaps 0 to 1 -> 1 to 0
      geom->getOrCreateStateSet()->setAttributeAndModes(new osg::Depth(osg::Depth::GEQUAL, 0.0, DEPTH_RANGE_MAX, false), osg::StateAttribute::ON | osg::StateAttribute::PROTECTED);
   }
   else
   {
      geom->getOrCreateStateSet()->setAttributeAndModes(new osg::Depth(osg::Depth::LEQUAL, 0.0, DEPTH_RANGE_MAX, false), osg::StateAttribute::ON);
   }
}

void AerodromeRenderer::setUseReverseZBuffer(bool useReverseZ)
{
   _useReverseZBuffer = useReverseZ;
}

bool AerodromeRenderer::getUseReverseZBuffer()
{
   return _useReverseZBuffer;
}
