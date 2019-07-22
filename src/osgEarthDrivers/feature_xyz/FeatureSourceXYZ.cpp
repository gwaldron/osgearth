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
#include "XYZFeatureOptions"

#include <osgEarth/Registry>
#include <osgEarth/XmlUtils>
#include <osgEarth/FileUtils>

#include <osgEarthFeatures/FeatureSource>
#include <osgEarthFeatures/Filter>
#include <osgEarthFeatures/FilterContext>
#include <osgEarthFeatures/MVT>
#include <osgEarthFeatures/OgrUtils>
#include <osgEarthFeatures/FeatureCursor>

#include <osgEarthUtil/TFS>

#include <osg/Notify>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <list>
#include <stdio.h>
#include <stdlib.h>

#include <ogr_api.h>

#ifdef WIN32
#include <windows.h>
#endif

#define LC "[XYZ FeatureSource] "

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Features;
using namespace osgEarth::Drivers;

#define OGR_SCOPED_LOCK GDAL_SCOPED_LOCK

/**
* A FeatureSource that reads features from a templated URL
* 
*/
class XYZFeatureSource : public FeatureSource
{
public:
    XYZFeatureSource(const XYZFeatureOptions& options ) :
      FeatureSource( options ),
          _options     ( options )      
      {                
      }

      /** Destruct the object, cleaning up and OGR handles. */
      virtual ~XYZFeatureSource()
      {               
          //nop
      }

      //override
      Status initialize(const osgDB::Options* readOptions)
      { 
          // make a local copy of the read options.
          _readOptions = Registry::cloneOrCreateOptions(readOptions);

          FeatureProfile* fp = 0L;

          // Try to get the results from the settings instead
          if ( !_options.profile().isSet())
          {
              return Status::Error(Status::ConfigurationError, "XYZ driver requires an explicit profile");
          }

          if (!_options.minLevel().isSet() || !_options.maxLevel().isSet())
          {
              return Status::Error(Status::ConfigurationError, "XYZ driver requires a min and max level");
          }

          _template = _options.url()->full();

          _rotateStart = _template.find("[");
          _rotateEnd   = _template.find("]");
          if ( _rotateStart != std::string::npos && _rotateEnd != std::string::npos && _rotateEnd-_rotateStart > 1 )
          {
              _rotateString  = _template.substr(_rotateStart, _rotateEnd-_rotateStart+1);
              _rotateChoices = _template.substr(_rotateStart+1, _rotateEnd-_rotateStart-1);
          }


          osg::ref_ptr<const Profile> profile = Profile::create( *_options.profile() );    
          fp = new FeatureProfile(profile->getExtent());
          fp->setTiled( true );
          fp->setFirstLevel( *_options.minLevel() );
          fp->setMaxLevel( *_options.maxLevel() );
          fp->setProfile( profile.get() );
          if ( _options.geoInterp().isSet() )
              fp->geoInterp() = _options.geoInterp().get();

          setFeatureProfile(fp);

          return Status::OK();
      }


      bool getFeatures( const std::string& buffer, const TileKey& key, const std::string& mimeType, FeatureList& features )
      {            
          if (mimeType == "application/x-protobuf" || mimeType == "binary/octet-stream" || mimeType == "application/octet-stream")
          {
              std::stringstream in(buffer);
              return MVT::read(in, key, features);
          }
          else
          {            
              // find the right driver for the given mime type
              OGR_SCOPED_LOCK;

              // find the right driver for the given mime type
              OGRSFDriverH ogrDriver =
                  isJSON(mimeType) ? OGRGetDriverByName( "GeoJSON" ) :
                  isGML(mimeType)  ? OGRGetDriverByName( "GML" ) :
                  0L;

              // fail if we can't find an appropriate OGR driver:
              if ( !ogrDriver )
              {
                  OE_WARN << LC << "Error reading TFS response; cannot grok content-type \"" << mimeType << "\""
                      << std::endl;
                  return false;
              }

              OGRDataSourceH ds = OGROpen( buffer.c_str(), FALSE, &ogrDriver );

              if ( !ds )
              {
                  OE_WARN << LC << "Error reading TFS response" << std::endl;
                  return false;
              }

              // read the feature data.
              OGRLayerH layer = OGR_DS_GetLayer(ds, 0);
              if ( layer )
              {
                  const SpatialReference* srs = getFeatureProfile()->getSRS();

                  OGR_L_ResetReading(layer);                                
                  OGRFeatureH feat_handle;
                  while ((feat_handle = OGR_L_GetNextFeature( layer )) != NULL)
                  {
                      if ( feat_handle )
                      {
                          osg::ref_ptr<Feature> f = OgrUtils::createFeature( feat_handle, getFeatureProfile() );
                          if ( f.valid() && !isBlacklisted(f->getFID()) )
                          {
                              features.push_back( f.release() );
                          }
                          OGR_F_Destroy( feat_handle );
                      }
                  }
              }

              // Destroy the datasource
              OGR_DS_Destroy( ds );
          }

          return true;
      }


      std::string getExtensionForMimeType(const std::string& mime)
      {
          //OGR is particular sometimes about the extension of files when it's reading them so it's good to have
          //the temp file have an appropriate extension
          if ((mime.compare("text/xml") == 0) ||
              (mime.compare("text/xml; subtype=gml/2.1.2") == 0) ||
              (mime.compare("text/xml; subtype=gml/3.1.1") == 0)
              )
          {
              return ".xml";
          }        
          else if ((mime.compare("application/json") == 0) ||
              (mime.compare("json") == 0) ||            

              (mime.compare("application/x-javascript") == 0) ||
              (mime.compare("text/javascript") == 0) ||
              (mime.compare("text/x-javascript") == 0) ||
              (mime.compare("text/x-json") == 0)                 
              )
          {
              return ".json";
          }        
          return "";
      }

      bool isGML( const std::string& mime ) const
      {
          return
              startsWith(mime, "text/xml");
      }


      bool isJSON( const std::string& mime ) const
      {
          return
              (mime.compare("application/json") == 0)         ||
              (mime.compare("json") == 0)                     ||            

              (mime.compare("application/x-javascript") == 0) ||
              (mime.compare("text/javascript") == 0)          ||
              (mime.compare("text/x-javascript") == 0)        ||
              (mime.compare("text/x-json") == 0);
      }

      URI createURL(const Symbology::Query& query)
      {     
          if (query.tileKey().isSet())
          {
              const TileKey &key = query.tileKey().get();
              unsigned int tileX = key.getTileX();
              unsigned int tileY = key.getTileY();
              unsigned int level = key.getLevelOfDetail();

              if (_options.invertY() == false)
              {                
                  unsigned int numRows, numCols;
                  key.getProfile()->getNumTiles(key.getLevelOfDetail(), numCols, numRows);
                  tileY  = numRows - tileY - 1;            
              }


              std::string location = _template;

              // support OpenLayers template style:
              replaceIn( location, "${x}", Stringify() << tileX );
              replaceIn( location, "${y}", Stringify() << tileY );
              replaceIn( location, "${z}", Stringify() << key.getLevelOfDetail() );

              // failing that, legacy osgearth style:
              replaceIn( location, "{x}", Stringify() << tileX );
              replaceIn( location, "{y}", Stringify() << tileY );
              replaceIn( location, "{z}", Stringify() << key.getLevelOfDetail() );

              std::string cacheKey;

              if ( !_rotateChoices.empty() )
              {
                  cacheKey = location;
                  unsigned index = (++_rotate_iter) % _rotateChoices.size();
                  replaceIn( location, _rotateString, Stringify() << _rotateChoices[index] );
              }


              URI uri( location, _options.url()->context() );
              if ( !cacheKey.empty() )
              {
                  uri.setCacheKey(Cache::makeCacheKey(location, "uri"));
              }

              return uri;
          }
          return URI();
      }

      FeatureCursor* createFeatureCursor(const Symbology::Query& query, ProgressCallback* progress)
      {
          FeatureCursor* result = 0L;

          URI uri =  createURL( query );
          if (uri.empty()) return 0;

          OE_DEBUG << LC << uri.full() << std::endl;

          // read the data:
          ReadResult r = uri.readString( _readOptions.get(), progress );

          const std::string& buffer = r.getString();
          const Config&      meta   = r.metadata();

          bool dataOK = false;

          FeatureList features;
          if ( !buffer.empty() )
          {
              // Get the mime-type from the metadata record if possible
              std::string mimeType = r.metadata().value( IOMetadata::CONTENT_TYPE );
              //If the mimetype is empty then try to set it from the format specification
              if (mimeType.empty())
              {
                  if (_options.format().value() == "json") mimeType = "json";
                  else if (_options.format().value().compare("gml") == 0) mimeType = "text/xml";
                  else if (_options.format().value().compare("pbf") == 0) mimeType = "application/x-protobuf";
              }
              dataOK = getFeatures( buffer, *query.tileKey(), mimeType, features );
          }

          if ( dataOK )
          {
              OE_DEBUG << LC << "Read " << features.size() << " features" << std::endl;
          }

          //If we have any filters, process them here before the cursor is created
          if (getFilters() && !getFilters()->empty() && !features.empty())
          {
              FilterContext cx;
              cx.setProfile(getFeatureProfile());
              cx.extent() = query.tileKey()->getExtent();

              for (FeatureFilterChain::const_iterator i = getFilters()->begin(); i != getFilters()->end(); ++i)
              {
                  FeatureFilter* filter = i->get();
                  cx = filter->push(features, cx);
              }
          }

          // If we have any features and we have an fid attribute, override the fid of the features
          if (_options.fidAttribute().isSet())
          {
              for (FeatureList::iterator itr = features.begin(); itr != features.end(); ++itr)
              {
                  std::string attr = itr->get()->getString(_options.fidAttribute().get());                
                  FeatureID fid = as<long>(attr, 0);
                  itr->get()->setFID( fid );
              }
          }

          //result = new FeatureListCursor(features);
          result = dataOK ? new FeatureListCursor( features ) : 0L;

          return result;
      }

      virtual bool supportsGetFeature() const
      {
          return false;
      }

      virtual Feature* getFeature( FeatureID fid )
      {
          // not supported for TFS.
          return 0;
      }

      virtual bool isWritable() const
      {
          return false;
      }

      virtual const FeatureSchema& getSchema() const
      {
          //TODO:  Populate the schema from the DescribeFeatureType call
          return _schema;
      }

      virtual osgEarth::Symbology::Geometry::Type getGeometryType() const
      {
          return Geometry::TYPE_UNKNOWN;
      }



private:
    const XYZFeatureOptions         _options;    
    FeatureSchema                   _schema;    
    osg::ref_ptr<osgDB::Options>    _readOptions;    
    std::string                     _template;
    std::string                     _rotateChoices;
    std::string                     _rotateString;
    std::string::size_type          _rotateStart, _rotateEnd;
    OpenThreads::Atomic             _rotate_iter;

};


class XYZFeatureSourceFactory : public FeatureSourceDriver
{
public:
    XYZFeatureSourceFactory()
    {
        supportsExtension( "osgearth_feature_xyz", "XYZ feature driver for osgEarth" );
    }

    virtual const char* className() const
    {
        return "XYZ Feature Reader";
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        return ReadResult( new XYZFeatureSource( getFeatureSourceOptions(options) ) );
    }
};

REGISTER_OSGPLUGIN(osgearth_feature_xyz, XYZFeatureSourceFactory)

