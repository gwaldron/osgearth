#ifndef OSGEARTH_WCS_PLUGIN_WCS11SOURCE_H_
#define OSGEARTH_WCS_PLUGIN_WCS11SOURCE_H_ 1

#include <osgEarth/PlateCarre>
#include <osgEarth/HTTPClient>
#include <osg/Image>
#include <osg/Shape>
#include <string>

using namespace osgEarth;

class WCS11Source : public osg::Referenced // : public PlateCarreTileSource
{
public:
    WCS11Source();
    
    osg::HeightField* createHeightField( const PlateCarreCellKey& key );
    
    //osg::Image* createImage( const PlateCarreCellKey& key );
        
    //std::string createURI( const PlateCarreCellKey& key ) const;

private:
    std::string prefix, map_file, coverage, cov_format, osg_format;

    HTTPRequest* createRequest( const PlateCarreCellKey& key ) const;
};

#endif // OSGEARTH_WCS_PLUGIN_WCS11SOURCE_H_