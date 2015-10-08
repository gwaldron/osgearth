/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2015 Pelican Mapping
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

#include <osg/ShapeDrawable>

#include <osgEarthSymbology/Style>
#include <osgEarthSymbology/Query>
#include <osgEarthFeatures/ConvertTypeFilter>

#include <osgEarthDrivers/gdal/GDALOptions>
#include <osgEarthDrivers/feature_ogr/OGRFeatureOptions>
#include <osgEarthDrivers/agglite/AGGLiteOptions>
#include <osgEarthDrivers/model_feature_geom/FeatureGeomModelOptions>
#include <osgEarthDrivers/model_feature_stencil/FeatureStencilModelOptions>

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


class SimplePager : public osg::Group
{
public:
    SimplePager(const osgEarth::Profile* profile):
      _profile( profile ),
          _rangeFactor( 6.0 ),
          _additive(false),
          _minLevel(0),
          _maxLevel(30)
      {
          _options = new osgDB::Options;
          _options->setUserData( this );          
      }

      void build()
      {
          addChild( buildRootNode() );
      }

      bool getAdditive() { return _additive; }
      void setAdditive(bool additive) { _additive = additive; }

      unsigned int getMinLevel() const { return _minLevel; }
      void setMinLevel(unsigned int minLevel) { _minLevel = minLevel; }

      unsigned int getMaxLevel() const { return _maxLevel; }
      void setMaxLevel(unsigned int maxLevel) { _maxLevel = maxLevel; }

      /**
      * Gets the bounding sphere for a given TileKey.
      */
      osg::BoundingSphered getBounds(const TileKey& key)
      {
          int samples = 6;

          GeoExtent extent = key.getExtent();

          double xSample = extent.width() / (double)samples;
          double ySample = extent.height() / (double)samples;

          osg::BoundingSphered bs;
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

      /**
       * The root node is a special node.
       */
      osg::Node* buildRootNode()
      {
          osg::Group* root = new osg::Group;

          std::vector<TileKey> keys;
          _profile->getRootKeys( keys );
          for (unsigned int i = 0; i < keys.size(); i++)
          {
              root->addChild( createPagedNode( keys[i] ) );
          }

          return root;
      }

      /**
       * Creates a node for the given TileKey.  Doesn't do anything with paging, just gets the raw data.
       */
      virtual osg::Node* createNode( const TileKey& key )
      {
          osg::BoundingSphered bounds = getBounds( key );

          osg::MatrixTransform* mt = new osg::MatrixTransform;
          mt->setMatrix(osg::Matrixd::translate( bounds.center() ) );
          osg::Geode* geode = new osg::Geode;
          osg::ShapeDrawable* sd = new osg::ShapeDrawable( new osg::Sphere(osg::Vec3f(0,0,0), bounds.radius()) );
          sd->setColor( randomColor() );
          geode->addDrawable( sd );
          mt->addChild(geode);
          return mt;
      }

      osg::PagedLOD* createPagedNode( const TileKey& key )
      {
          osg::BoundingSphered tileBounds = getBounds( key );

          osg::PagedLOD* plod = new osg::PagedLOD;
          plod->setCenter( tileBounds.center() );
          plod->setRadius( tileBounds.radius() );

          // Create the actual data for this tile.
          osg::Node* node = createNode( key );
          if (!node && key.getLevelOfDetail() < _minLevel)
          {              
              // If we couldn't create any data, just create an empty group.  That's ok.
              node = new osg::Group;
          }
          if (!node) return 0;
          plod->addChild( node );

          std::stringstream buf;
          buf << key.getLevelOfDetail() << "_" << key.getTileX() << "_" << key.getTileY() << ".osgearth_pseudo_simple";

          std::string uri = buf.str();

          // Now setup a filename on the PagedLOD that will load all of the children of this node.
          plod->setFileName(1, uri);
          plod->setDatabaseOptions( _options.get() );

          // Setup the min and max ranges.

          // This setups a replacement mode where the parent will be completely replaced by it's children.
          float minRange = (float)(tileBounds.radius() * _rangeFactor);

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

          return plod;
      }

 
      /**
       * Loads the PagedLOD hierarchy for this key.
       */
      osg::Node* loadKey( const TileKey& key )
      {       
          osg::ref_ptr< osg::Group >  group = new osg::Group;

          for (unsigned int i = 0; i < 4; i++)
          {
              TileKey childKey = key.createChildKey( i );

              osg::PagedLOD* plod = createPagedNode( childKey );
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

      const osgEarth::Profile* getProfile()
      {
          return _profile.get();
      }

      bool _additive;
      double _rangeFactor;
      unsigned int _minLevel;
      unsigned int _maxLevel;
      osg::ref_ptr< const osgEarth::Profile > _profile;
      osg::ref_ptr< osgDB::Options > _options;
};

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


class BoxSimplePager : public SimplePager
{
public:
    BoxSimplePager(const osgEarth::Profile* profile):
      SimplePager( profile )
    {
    }

    virtual osg::Node* createNode( const TileKey& key )
    {        
        osg::BoundingSphered bounds = getBounds( key );

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
      
      virtual osg::Node* createNode( const TileKey& key )
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
            features->initialize();
            features->getFeatureProfile();


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
