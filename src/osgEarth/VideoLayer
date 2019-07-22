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

#ifndef OSGEARTH_VIDEO_LAYER_H
#define OSGEARTH_VIDEO_LAYER_H 1

#include <osgEarth/Common>
#include <osgEarth/ImageLayer>
#include <osg/Texture2D>

namespace osgEarth
{  
    /**
     * Initialization and serialization options for a video layer
     */
    class OSGEARTH_EXPORT VideoLayerOptions : public ImageLayerOptions
    {
    public:
        /** Constructs new video layer options. */
        VideoLayerOptions();
        
        /** Deserializes new video layer options. */
        VideoLayerOptions(const ConfigOptions& options);

        // Constructs new options with a layer name
        VideoLayerOptions(const std::string& name);

        /** dtor */
        virtual ~VideoLayerOptions()
        {
        }

    public:

        optional<URI>& url() { return _url; }
        const optional<URI>& url() const { return _url; }


    public:
        virtual Config getConfig() const;
        virtual void mergeConfig( const Config& conf );
        
    private:
        void fromConfig( const Config& conf );
        void setDefaults();

        optional<URI> _url;
    };


    /**
     * A layer that displays a video texture on the earth.
     */
    class OSGEARTH_EXPORT VideoLayer : public osgEarth::ImageLayer
    {
    public:
        META_Layer(osgEarth, VideoLayer, VideoLayerOptions, video);

        VideoLayer();        
        
        VideoLayer( const VideoLayerOptions& options );

        osg::Texture2D* getTexture() const { return _texture.get(); }

    public: // ImageLayer:

        void init();

        const Status& open();

        virtual osg::Texture* createTexture(const TileKey& key, ProgressCallback* progress, osg::Matrixf& textureMatrix);

    protected:

        osg::ref_ptr< osg::Texture2D > _texture;
    };
} // namespace osgEarth

#endif // OSGEARTH_VIDEO_LAYER_H
