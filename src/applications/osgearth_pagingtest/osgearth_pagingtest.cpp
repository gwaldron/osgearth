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
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

#include <osg/Notify>
#include <osgGA/StateSetManipulator>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgEarth/Map>
#include <osgEarth/MapNode>
#include <osgEarthUtil/ExampleResources>
#include <osgEarthUtil/EarthManipulator>
#include <osgEarthAnnotation/FeatureNode>
#include <osg/PagedLOD>
#include <osgDB/FileNameUtils>
#include <osgDB/ReaderWriter>
#include <osgEarth/SpatialReference>
#include <osgEarthUtil/SimplePager>

#include <osg/ShapeDrawable>

#include <osgEarthSymbology/Style>
#include <osgEarthSymbology/Query>
#include <osgEarthFeatures/ConvertTypeFilter>

#include <osgEarthDrivers/gdal/GDALOptions>
#include <osgEarthDrivers/feature_ogr/OGRFeatureOptions>
#include <osgEarthDrivers/agglite/AGGLiteOptions>
#include <osgEarthDrivers/model_feature_geom/FeatureGeomModelOptions>

#include <osgEarthDrivers/feature_mapnikvectortiles/MVTFeatureOptions>

#include <osgDB/WriteFile>

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Drivers;
using namespace osgEarth::Symbology;
using namespace osgEarth::Annotation;
using namespace osgEarth::Util;

osg::Vec4
randomColor()
{
    float r = (float)rand() / (float)RAND_MAX;
    float g = (float)rand() / (float)RAND_MAX;
    float b = (float)rand() / (float)RAND_MAX;
    return osg::Vec4(r,g,b,1.0f);
}

class BoxSimplePager : public SimplePager
{
public:
    BoxSimplePager(const osgEarth::Profile* profile):
      SimplePager( profile )
    {
    }

    virtual osg::Node* createNode(const TileKey& key, ProgressCallback*)
    {        
        osg::BoundingSphere bounds = getBounds( key );

        osg::MatrixTransform* mt = new osg::MatrixTransform;
        mt->setMatrix(osg::Matrixd::translate( bounds.center() ) );
        osg::Geode* geode = new osg::Geode;
        osg::ShapeDrawable* sd = new osg::ShapeDrawable( new osg::Box(osg::Vec3f(0,0,0), bounds.radius(), bounds.radius(), bounds.radius()) );
        sd->setColor( randomColor() );
        geode->addDrawable( sd );
        mt->addChild(geode);
        return mt;
    }
};

class FeaturePager : public SimplePager
{
public:
    FeaturePager( FeatureSource* features, const Style& style, MapNode* mapNode):
      SimplePager( features->getFeatureProfile()->getProfile() ),
          _features( features ),
          _style( style ),
          _mapNode( mapNode )
      {
          setMinLevel( features->getFeatureProfile()->getFirstLevel());
          setMaxLevel( features->getFeatureProfile()->getMaxLevel());
      }
      
      virtual osg::Node* createNode(const TileKey& key, ProgressCallback* progress)
      {
          // Get features for this key
          osgEarth::Symbology::Query query;
          query.tileKey() = key;          

          osg::ref_ptr< FeatureCursor > cursor = _features->createFeatureCursor( query );
          if (cursor)
          {              
              FeatureList features;
              cursor->fill( features );

              Style style = _style;

              // See if we have a style for a given lod, otherwise use the default style
              StyleLODMap::iterator itr = _styleMap.find( key.getLevelOfDetail() );
              if (itr != _styleMap.end())
              {
                  style = itr->second;
              }

              FeatureNode* featureNode = new FeatureNode(_mapNode, features, style, GeometryCompilerOptions(), _styleSheet.get() );
              return featureNode;
          }
          else
          {
              // We got nothing, so return nothing.
              return 0;
          }
      }


      // Set a style per lod
      void setLODStyle(unsigned int lod, const Style& style)
      {
          _styleMap[lod] = style;
      }

      StyleSheet* getStyleSheet() const
      {
          return _styleSheet;
      }

      void setStyleSheet(StyleSheet* styleSheet)
      {
          _styleSheet = styleSheet;
      }

      typedef std::map<unsigned int, Style > StyleLODMap;
      StyleLODMap _styleMap;

      osg::ref_ptr < FeatureSource > _features;
      osg::ref_ptr < StyleSheet > _styleSheet;
      Style _style;
      osg::ref_ptr< MapNode > _mapNode;
};

Style getStyle(const osg::Vec4& color, double extrusionHeight)
{
    Style style;
    style.getOrCreate<PolygonSymbol>()->fill()->color() = color;
    style.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;
    style.getOrCreate<AltitudeSymbol>()->technique() = AltitudeSymbol::TECHNIQUE_DRAPE;
    /*
    if (extrusionHeight > 0)
    {
        style.getOrCreate<ExtrusionSymbol>()->height() = extrusionHeight;
    }
    */
    return style;
}

//
// NOTE: run this sample from the repo/tests directory.
//
int main(int argc, char** argv)
{
    osg::ArgumentParser arguments(&argc,argv);

    osgViewer::Viewer viewer(arguments);

    // Tell the database pager to not modify the unref settings
    viewer.getDatabasePager()->setUnrefImageDataAfterApplyPolicy( false, false );

    // install our default manipulator (do this before calling load)
    viewer.setCameraManipulator( new EarthManipulator(arguments) );

    osg::Group* root = new osg::Group;

    osg::Node* node = MapNodeHelper().load( arguments, &viewer );
    root->addChild( node );

    MapNode* mapNode = MapNode::findMapNode( node );

    bool additive = false;
    if (arguments.read("--additive"))
    {
        additive = true;
    }

    bool spheres = false;
    if (arguments.read("--spheres"))
    {
        spheres = true;
    }

    bool boxes = false;
    if (arguments.read("--boxes"))
    {
        boxes = true;
    }

    bool features = false;
    if (arguments.read("--features"))
    {
        features = true;
    }

    if (!spheres && !boxes && !features)
    {
        spheres = true;
    }

    SimplePager* pager = 0;

    if (spheres)
    {
        pager = new SimplePager( mapNode->getMap()->getProfile() );
        pager->setAdditive( additive );    
        pager->build();    
        root->addChild( pager );

    }
    else if (boxes)
    {
        pager = new BoxSimplePager( mapNode->getMap()->getProfile() );
        pager->setAdditive( additive );    
        pager->build();    
        root->addChild( pager );

    }
    else if (features)
    {
        std::vector< std::string > filenames;
        for(int pos=1;pos<arguments.argc();++pos)
        {
            if (!arguments.isOption(pos))
            {
                std::string filename = arguments[ pos ];
                if (osgDB::getFileExtension( filename ) == "mbtiles")
                {
                    filenames.push_back( filename );                
                }
            }
        }

        for (unsigned int i = 0; i < filenames.size(); i++)
        {
            OE_NOTICE << "Loading " << filenames[i] << std::endl;


            MVTFeatureOptions featureOpt;
            featureOpt.url() = filenames[i];

            osg::ref_ptr< FeatureSource > features = FeatureSourceFactory::create( featureOpt );
            Status s = features->open();
            if (s.isError()) {
                OE_WARN << s.message() << "\n";
                return -1;
            }

            FeaturePager* featurePager = new FeaturePager(features, getStyle(randomColor(), 0.0), mapNode);

            // Style 13 is where the full resolution data comes, in so use a fancy textured and extruded style
            // a style for the building data:
            Style buildingStyle;
            buildingStyle.setName( "buildings" );

            // Extrude the shapes into 3D buildings.
            ExtrusionSymbol* extrusion = buildingStyle.getOrCreate<ExtrusionSymbol>();
            extrusion->heightExpression() = 50.0;
            extrusion->flatten() = true;
            extrusion->wallStyleName() = "building-wall";
            extrusion->roofStyleName() = "building-roof";

            // a style for the wall textures:
            Style wallStyle;
            wallStyle.setName( "building-wall" );
            SkinSymbol* wallSkin = wallStyle.getOrCreate<SkinSymbol>();
            wallSkin->library() = "us_resources";
            wallSkin->addTag( "building" );
            wallSkin->randomSeed() = 1;

            // a style for the rooftop textures:
            Style roofStyle;
            roofStyle.setName( "building-roof" );
            SkinSymbol* roofSkin = roofStyle.getOrCreate<SkinSymbol>();
            roofSkin->library() = "us_resources";
            roofSkin->addTag( "rooftop" );
            roofSkin->randomSeed() = 1;
            roofSkin->isTiled() = true;

            // assemble a stylesheet and add our styles to it:
            StyleSheet* styleSheet = new StyleSheet();
            styleSheet->addStyle( buildingStyle );
            styleSheet->addStyle( wallStyle );
            styleSheet->addStyle( roofStyle );

            // load a resource library that contains the building textures.
            ResourceLibrary* reslib = new ResourceLibrary( "us_resources", "../data/resources/textures_us/catalog.xml" );
            styleSheet->addResourceLibrary( reslib );


            // Give the FeaturePager it's overall stylesheet.
            featurePager->setStyleSheet( styleSheet );

            featurePager->setLODStyle(14, buildingStyle );

            featurePager->setAdditive( additive );    
            featurePager->build();    
            root->addChild( featurePager );

        }
    }

    viewer.setSceneData( root );
       
    
    return viewer.run();
}
