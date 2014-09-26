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

#include <osgEarth/TileSource>
#include <osgEarth/FileUtils>
#include <osgEarth/ImageUtils>
#include <osgEarth/Registry>

#include <osg/Notify>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osg/ImageStream>
#include <osg/ImageSequence>

#include <sstream>
#include <iomanip>
#include <string.h>

#include "RefreshOptions"

using namespace osgEarth;
using namespace osgEarth::Drivers;

#define LC "[Refresh driver] "

/*
 * LoadImageOperation is a simple operation that simply loads an image in a background thread
 */
class LoadImageOperation : public osg::Operation
{
public:
    LoadImageOperation(const std::string& filename):
      _filename(filename),
          _done(false)
      {
      }

      void operator()(osg::Object*)
      {
          //Try to load the image a few times.  If you happen to be writing the file at the
          //same time as the plugin tries to load it it can fail.
          unsigned int maxTries = 5;
          for (unsigned int i = 0; i < maxTries; i++)
          {
              _image = osgDB::readImageFile( _filename );                                     
              if (_image.valid()) break;              
          }
          _done = true;
      }

      bool _done;
      osg::ref_ptr< osg::Image > _image;
      std::string _filename;
};



/*
 * RefreshImage is a special ImageStream that reloads an image from a filename and
 * updates it's internal image data.
 */
class RefreshImage : public osg::ImageStream
{
public:

    RefreshImage(const std::string& filename, double time):
      _filename(filename),
          _time(time),
          _lastUpdateTime(0),
          osg::ImageStream()
      {                    
          osg::ref_ptr< osg::Image > image = osgDB::readImageFile( filename );
          if (image.valid()) copyImage( image.get() );
      }      


      /**
       * Tell OpenSceneGraph that we require an update call
       */
      virtual bool requiresUpdateCall() const { return true; }

      ~RefreshImage()
      {
      }

      static osg::OperationsThread* getOperationsThread() 
      {          
          //Create an Operations Thread.  This thread is static and is not deleted b/c 
          //there are issues with calling cancel on static threads on Windows.  The thread itself will die
          //cleanly, but the application will hang waiting for the cancel call in OpenThreads to return.
          //This is something that needs to be fixed in OpenThreads so we can maintain a static threadpool.
          static osg::OperationsThread* _thread = 0;
          static OpenThreads::Mutex _mutex;

          if (!_thread)
          {
              OpenThreads::ScopedLock< OpenThreads::Mutex > lock(_mutex);
              if (!_thread)
              {
                  _thread = new osg::OperationsThread;
                  _thread->start();
              }            
          }    
          return _thread;                          

      }

      //If the loadImageOp is complete then update the contents of this image with the new pixels
      void updateImage() 
      {
          if (_loadImageOp.valid() && _loadImageOp->_done)
          {              
              osg::ref_ptr< osg::Image > image = _loadImageOp->_image.get();
              if (image.valid())
              {
                  copyImage( image.get() );
              }
              _lastUpdateTime = osg::Timer::instance()->time_s();
              _loadImageOp = 0;
          }
      }


      /**
       * Copies the contents of the given image into this image.
       */
      void copyImage( osg::Image* image)
      {
          if (image)
          {
              unsigned char* data = new unsigned char[ image->getTotalSizeInBytes() ];
              memcpy(data, image->data(), image->getTotalSizeInBytes());
              setImage(image->s(), image->t(), image->r(), image->getInternalTextureFormat(), image->getPixelFormat(), image->getDataType(), data, osg::Image::USE_NEW_DELETE, image->getPacking());                            
          }
      }

      /** update method for osg::Image subclasses that update themselves during the update traversal.*/
      virtual void update(osg::NodeVisitor* nv)
      {                               
          updateImage();
          double time = osg::Timer::instance()->time_s();
          osg::Timer_t ticks = osg::Timer::instance()->tick();          
          //If we've let enough time elapse and we're not waiting on an existing load image operation then add one to the queue
          if (!_loadImageOp.valid() && (time - _lastUpdateTime > _time))
          {
              std::stringstream ss;
              std::string file = ss.str();              
              _loadImageOp = new LoadImageOperation(_filename);
              getOperationsThread()->add( _loadImageOp.get() );
          }
      }

      std::string _filename;
      double _time;
      double _lastUpdateTime;
      osg::ref_ptr< LoadImageOperation > _loadImageOp;      
};


class RefreshSource : public TileSource
{
public:
    RefreshSource(const TileSourceOptions& options) : TileSource(options), _options(options)
    {
    }


    Status initialize(const osgDB::Options* dbOptions)
    {        
        setProfile( osgEarth::Registry::instance()->getGlobalGeodeticProfile() );
        return STATUS_OK;
    }


    osg::Image* createImage(
        const TileKey&        key,
        ProgressCallback*     progress )
    {        
        return new RefreshImage( _options.url()->full(), *_options.frequency());     
    }

    bool isDynamic() const
    {
        //Tell osgEarth that this is a dynamic image
        return true;
    }

    virtual int getPixelsPerTile() const
    {
        return 256;
    }

    virtual std::string getExtension()  const 
    {
        return osgDB::getFileExtension( _options.url()->full() );
    }

private:
    const RefreshOptions      _options;
    
};




class ReaderWriterRefresh : public TileSourceDriver
{
public:
    ReaderWriterRefresh()
    {
        supportsExtension( "osgearth_refresh", "Refresh" );
    }

    virtual const char* className()
    {
        return "ReaderWriterRefresh";
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        return new RefreshSource( getTileSourceOptions(options) );
    }
};

REGISTER_OSGPLUGIN(osgearth_refresh, ReaderWriterRefresh)

