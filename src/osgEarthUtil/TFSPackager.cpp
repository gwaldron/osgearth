/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
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
#include <osgEarthUtil/TFSPackager>

#include <osgEarth/Registry>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>

#define LC "[TFSPackager] "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;
using namespace osgEarth::Util;

/******************************************************************************************/
class FeatureTileVisitor;
class FeatureTile;

typedef std::list< osgEarth::Features::FeatureID > FeatureIDList;

class FeatureTile : public osg::Referenced
{
public:
    FeatureTile( TileKey& key ):
      _key( key ),
          _isSplit( false )
      {        
      }

      const GeoExtent& getExtent() const
      {
          return _key.getExtent();
      }

      const TileKey& getKey() const
      {
          return _key;
      }

      bool getIsSplit() const { return _isSplit;}

      void split()
      {
          if (!_isSplit)
          {
              for (unsigned int i = 0; i < 4; ++i)
              {                
                  _children[i] = new FeatureTile(_key.createChildKey( i ) );
              }
              _isSplit = true;
          }
      }

      void accept( FeatureTileVisitor* v);

      void traverse( FeatureTileVisitor* v )
      {
          if (_isSplit)
          {
              for (unsigned int i = 0; i < 4; ++i)
              {
                  _children[i]->accept( v );
              }
          }
      }

      FeatureIDList& getFeatures()
      {
          return _features;
      }


private:    
    FeatureIDList _features;
    TileKey _key;   
    osg::ref_ptr<FeatureTile> _children[4];
    bool _isSplit;
};

class FeatureTileVisitor : public osg::Referenced
{
public:
    virtual void traverse(FeatureTile* tile)
    {
        tile->traverse( this );
    }
};

void FeatureTile::accept(FeatureTileVisitor* v)
{    
    v->traverse( this );
}

/******************************************************************************************/

class AddFeatureVisitor : public FeatureTileVisitor
{
public:
    AddFeatureVisitor( Feature* feature, int maxFeatures, int firstLevel, int maxLevel, CropFilter::Method cropMethod):
      _feature( feature ),
          _maxFeatures( maxFeatures ),      
          _maxLevel( maxLevel ),
          _firstLevel( firstLevel ),
          _added(false),
          _numAdded( 0 ),
          _levelAdded(-1),
          _cropMethod( cropMethod )
      {

      }

      virtual void traverse( FeatureTile* tile)
      {        
          if (_added && _cropMethod != CropFilter::METHOD_CROPPING) return;

          GeoExtent featureExtent(_feature->getSRS(), _feature->getGeometry()->getBounds());
          if (featureExtent.intersects( tile->getExtent()))
          {
              //If the node contains the feature, and it doesn't contain the max number of features add it.  If it's already full then 
              //split it.
              if (tile->getKey().getLevelOfDetail() >= (unsigned int)_firstLevel && 
                  (tile->getFeatures().size() < (unsigned int)_maxFeatures || tile->getKey().getLevelOfDetail() == _maxLevel || tile->getKey().getLevelOfDetail() == _levelAdded))
              {
                  if (_levelAdded < 0 || _levelAdded == tile->getKey().getLevelOfDetail())
                  {
                      osg::ref_ptr< Feature > clone = new Feature( *_feature, osg::CopyOp::DEEP_COPY_ALL );
                      FeatureList features;
                      features.push_back( clone );

                      CropFilter cropFilter(_cropMethod);
                      FilterContext context(0);
                      context.extent() = tile->getExtent();
                      cropFilter.push( features, context );


                      if (!features.empty() && clone->getGeometry() && clone->getGeometry()->size() > 0)
                      {
                          //tile->getFeatures().push_back( clone );
                          tile->getFeatures().push_back( clone->getFID() );
                          _added = true;
                          _levelAdded = tile->getKey().getLevelOfDetail();
                          _numAdded++;                   
                      }
                  }

                  if (_cropMethod == CropFilter::METHOD_CROPPING)
                  {
                      tile->traverse( this );
                  }
              }
              else
              {   
                  tile->split();
                  tile->traverse( this );
              }

          }


      }

      int _levelAdded;

      bool _added;
      int _maxFeatures;
      int _firstLevel;
      int _maxLevel;
      int _numAdded;

      CropFilter::Method _cropMethod;



      osg::ref_ptr< Feature > _feature;
};


/******************************************************************************************/
class WriteFeaturesVisitor : public FeatureTileVisitor
{
public:
    WriteFeaturesVisitor(FeatureSource* features, const std::string& dest, const std::string &layer, CropFilter::Method cropMethod):
      _dest( dest ),
          _layer( layer ),
          _features( features ),
          _cropMethod( cropMethod )
      {

      }

      virtual void traverse( FeatureTile* tile)
      {
          if (tile->getFeatures().size() > 0)
          {   
              osg::ref_ptr< osgEarth::SpatialReference > wgs84 = osgEarth::SpatialReference::create("epsg:4326");
              //Actually load up the features
              FeatureList features;
              for (FeatureIDList::const_iterator i = tile->getFeatures().begin(); i != tile->getFeatures().end(); i++)
              {
                  Feature* f = _features->getFeature( *i );                  

                  if (f)
                  {
                      //Reproject the feature to WGS84 if it's not already
                      if (!f->getSRS()->isEquivalentTo( wgs84 ) )
                      {
                          f->getSRS()->transform(f->getGeometry()->asVector(), wgs84 );
                          f->setSRS( wgs84 );
                      }

                      features.push_back( f );
                  }
                  else
                  {
                      OE_NOTICE << "couldn't get feature " << *i << std::endl;
                  }
              }

              //Need to do the cropping again since these are brand new features coming from the feature source.
              CropFilter cropFilter(_cropMethod);
              FilterContext context(0);
              context.extent() = tile->getExtent();
              cropFilter.push( features, context );

              std::string contents = Feature::featuresToGeoJSON( features );
              std::stringstream buf;
              int x =  tile->getKey().getTileX();
              unsigned int numRows, numCols;
              tile->getKey().getProfile()->getNumTiles(tile->getKey().getLevelOfDetail(), numCols, numRows);
              int y  = numRows - tile->getKey().getTileY() - 1;

              buf << osgDB::concatPaths( _dest, _layer ) << "/" << tile->getKey().getLevelOfDetail() << "/" << x << "/" << y << ".json";
              std::string filename = buf.str();
              //OE_NOTICE << "Writing " << features.size() << " features to " << filename << std::endl;

              if ( !osgDB::fileExists( osgDB::getFilePath(filename) ) )
                  osgDB::makeDirectoryForFile( filename );


              std::fstream output( filename.c_str(), std::ios_base::out );
              if ( output.is_open() )
              {
                  output << contents;
                  output.flush();
                  output.close();                
              }            
          }
          tile->traverse( this );        
      }

      osg::ref_ptr< FeatureSource > _features;
      std::string _dest;
      std::string _layer;
      CropFilter::Method _cropMethod;
};



/******************************************************************************************/

TFSPackager::TFSPackager():
_firstLevel( 0 ),
    _maxLevel( 10 ),
    _maxFeatures( 300 ),
    _method( CropFilter::METHOD_CENTROID )
{
}

void
    TFSPackager::package( FeatureSource* features, const std::string& destination, const std::string& layername, const std::string& description )
{
    osg::ref_ptr< osgEarth::SpatialReference > wgs84 = osgEarth::SpatialReference::create("epsg:4326");

    //Transform to lat/lon extents
    GeoExtent extent = features->getFeatureProfile()->getExtent().transform( wgs84.get() );

    osg::ref_ptr< const osgEarth::Profile > profile = osgEarth::Profile::create(extent.getSRS(), extent.xMin(), extent.yMin(), extent.xMax(), extent.yMax(), 1, 1);


    TileKey rootKey = TileKey(0, 0, 0, profile );    


    osg::ref_ptr< FeatureTile > root = new FeatureTile( rootKey );
    //Loop through all the features and try to insert them into the quadtree
    osg::ref_ptr< FeatureCursor > cursor = features->createFeatureCursor( _query );
    int added = 0;
    int failed = 0;
    int skipped = 0;
    int highestLevel = 0;

    while (cursor.valid() && cursor->hasMore())
    {        
        osg::ref_ptr< Feature > feature = cursor->nextFeature();

        //Reproject the feature to WGS84 if it's not already
        if (!feature->getSRS()->isEquivalentTo( wgs84 ) )
        {
            feature->getSRS()->transform(feature->getGeometry()->asVector(), wgs84 );
            feature->setSRS( wgs84 );
        }

        if (feature->getGeometry() && feature->getGeometry()->getBounds().valid() && feature->getGeometry()->size() > 0)
        {

            AddFeatureVisitor v(feature.get(), _maxFeatures, _firstLevel, _maxLevel, _method);
            root->accept( &v );
            if (!v._added)
            {
                OE_NOTICE << "Failed to add feature " << feature->getFID() << std::endl;
                failed++;
            }
            else
            {                
                if (highestLevel < v._levelAdded)
                {
                    highestLevel = v._levelAdded;
                }
                added++;
                OE_DEBUG << "Added " << added << std::endl;
            }   
        }
        else
        {
            OE_NOTICE << "Skipping feature " << feature->getFID() << " with null or invalid geometry" << std::endl;
            skipped++;
        }
    }   
    OE_NOTICE << "Added=" << added << "Skipped=" << skipped << " Failed=" << failed << std::endl;

    WriteFeaturesVisitor write(features, destination, layername, _method);
    root->accept( &write );

    //Write out the meta doc
    TFSLayer layer;
    layer.setTitle( layername );
    layer.setAbstract( description );
    layer.setFirstLevel( _firstLevel );
    layer.setMaxLevel( highestLevel );
    layer.setExtent( profile->getExtent() );
    TFSReaderWriter::write( layer, osgDB::concatPaths(destination, layername) + "/tfs.xml");

}

