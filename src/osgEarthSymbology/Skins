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

#ifndef OSGEARTHSYMBOLOGY_SKIN_RESOURCE_H
#define OSGEARTHSYMBOLOGY_SKIN_RESOURCE_H 1

#include <osgEarthSymbology/Common>
#include <osgEarthSymbology/Resource>
#include <osgEarthSymbology/Symbol>
#include <osgEarth/URI>
#include <osg/TexEnv>
#include <osg/Image>

namespace osgEarth { namespace Symbology
{
    using namespace osgEarth;

    /**
     * A resource that points to a "skin", which is a Texture image paired with
     * a collection of metadata that describes its suitability for use in a scene.
     */
    class OSGEARTHSYMBOLOGY_EXPORT SkinResource : public Resource
    {
    public:
        /** Constructs a new skin resource. */
        SkinResource( const Config& conf =Config() );

        /** dtor */
        virtual ~SkinResource() { }

        /**
         * Creates a new StateSet containing a Texture based on this Skin.
         */
        osg::StateSet* createStateSet( const osgDB::Options* dbOptions ) const;

        /**
         * Creates an image for this SkinResource.
         */
        osg::ref_ptr<osg::Image> createImage( const osgDB::Options* options ) const;
        
        osg::Texture* createTexture(const osgDB::Options* readOptions) const;

        osg::Texture* createTexture(osg::Image* image) const;

        /** 
         * A key string that can uniquely identify this object for the purposes
         * of creating its state set (e.g., for a cache)
         */
        std::string getUniqueID() const;

    public:
        /** inline image (not serializable) */
        osg::ref_ptr<osg::Image>& image() { return _image; }
        const osg::ref_ptr<osg::Image>& image() const { return _image; }

        /** Source location of the actual texture image.  */
        optional<URI>& imageURI() { return _imageURI; }
        const optional<URI>& imageURI() const { return _imageURI; }

        /** Real-world width of the image, in meters */
        optional<float>& imageWidth() { return _imageWidth; }
        const optional<float>& imageWidth() const { return _imageWidth; }

        /** Real-world height of the image, in meters */
        optional<float>& imageHeight() { return _imageHeight; }
        const optional<float>& imageHeight() const { return _imageHeight; }

        /** Minimum acceptable real-world object height (meters) for which this image would make an appropriate texture */
        optional<float>& minObjectHeight() { return _minObjHeight; }
        const optional<float>& minObjectHeight() const { return _minObjHeight; }

        /** Maximum acceptable real-world object height (meters) for which this image would make an appropriate texture */
        optional<float>& maxObjectHeight() { return _maxObjHeight; }
        const optional<float>& maxObjectHeight() const { return _maxObjHeight; }

        /** Whether this image is suitable for use as a vertically repeating texture */
        optional<bool>& isTiled() { return _isTiled; }
        const optional<bool>& isTiled() const { return _isTiled; }

        /** Image offset within a source atlas (S dimension [0..1]) */
        optional<float>& imageBiasS() { return _imageBiasS; }
        const optional<float>& imageBiasS() const { return _imageBiasS; }

        /** Image offset (pixels) within a source atlas (T dimension [0..1]) */
        optional<float>& imageBiasT() { return _imageBiasT; }
        const optional<float>& imageBiasT() const { return _imageBiasT; }

        /** Image layer index within a source atlas (R dimension) */
        optional<unsigned>& imageLayer() { return _imageLayer; }
        const optional<unsigned>& imageLayer() const { return _imageLayer; }

        /** Image scalke factor within a source atlas (S dimension) */
        optional<float>& imageScaleS() { return _imageScaleS; }
        const optional<float>& imageScaleS() const { return _imageScaleS; }

        /** Image scalke factor within a source atlas (T dimension) */
        optional<float>& imageScaleT() { return _imageScaleT; }
        const optional<float>& imageScaleT() const { return _imageScaleT; }

        /** GL texture application mode */
        optional<osg::TexEnv::Mode>& texEnvMode() { return _texEnvMode; }
        const optional<osg::TexEnv::Mode>& texEnvMode() const { return _texEnvMode; }

        /** The maximum allowable size of a texture (in either dimension) that uses this image. */
        optional<unsigned> maxTextureSpan() { return _maxTexSpan; }
        const optional<unsigned> maxTextureSpan() const { return _maxTexSpan; }

        /** Whether this image is suitable for texture atlasing */
        optional<bool>& atlasHint() { return _atlasHint; }
        const optional<bool>& atlasHint() const { return _atlasHint; }

        /** Options string to pass in when reading the image */
        optional<std::string>& readOptions() { return _readOptions; }
        const optional<std::string>& readOptions() const { return _readOptions; }

    public: // serialization methods

        virtual Config getConfig() const;
        void mergeConfig( const Config& conf );

    protected:

        osg::StateSet* createStateSet( osg::Image* image ) const;        

    protected:

        optional<std::string>       _name;
        optional<URI>               _imageURI;
        optional<float>             _imageWidth;
        optional<float>             _imageHeight;
        optional<float>             _minObjHeight;
        optional<float>             _maxObjHeight;
        optional<bool>              _isTiled;
        optional<osg::TexEnv::Mode> _texEnvMode;
        optional<unsigned>          _maxTexSpan;
        optional<float>             _imageBiasS;
        optional<float>             _imageBiasT;
        optional<unsigned>          _imageLayer;
        optional<float>             _imageScaleS;
        optional<float>             _imageScaleT;
        optional<bool>              _atlasHint;
        optional<std::string>       _readOptions;

        osg::ref_ptr<osg::Image>    _image;
    };


    /**
     * Query object that you can use to search for applicable Skin resources from the 
     * ResourceLibrary.
     */
    class OSGEARTHSYMBOLOGY_EXPORT SkinSymbol : public Taggable<Symbol>
    {
    public:
        META_Object(osgEarthSymbology, SkinSymbol);

        SkinSymbol(const SkinSymbol& rhs,const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY);
        SkinSymbol( const Config& conf =Config() );

        /** dtor */
        virtual ~SkinSymbol() { }

    public: // query parameters

        /** Name of the resource library to use with this symbol. */
        optional<std::string>& library() { return _library; }
        const optional<std::string>& library() const { return _library; }

        /** Object height in meters (must fall in the skin's min/max object height range to be accepted) */
        optional<float>& objectHeight() { return _objHeight; }
        const optional<float>& objectHeight() const { return _objHeight; }

        /** Minimum acceptable real-world object height for which this image would make an appropriate texture */
        optional<float>& minObjectHeight() { return _minObjHeight; }
        const optional<float>& minObjectHeight() const { return _minObjHeight; }

        /** Maximum acceptable real-world object height for which this image would make an appropriate texture */
        optional<float>& maxObjectHeight() { return _maxObjHeight; }
        const optional<float>& maxObjectHeight() const { return _maxObjHeight; }

        /** Whether this image is suitable for use as a vertically repeating texture */
        optional<bool>& isTiled() { return _isTiled; }
        const optional<bool>& isTiled() const { return _isTiled; }

        /** Random number seed value (for picking a candidate) */
        optional<unsigned>& randomSeed() { return _randomSeed; }
        const optional<unsigned>& randomSeed() const { return _randomSeed; }

        /** Name of a specific skin in the catalog */
        optional<StringExpression>& name() { return _name; }
        const optional<StringExpression>& name() const { return _name; }

    public:
        void mergeConfig(const Config& conf);
        Config getConfig() const;
        static void parseSLD(const Config& c, class Style& style);

    protected:
        optional<std::string> _library;
        optional<float>       _objHeight;
        optional<float>       _minObjHeight;
        optional<float>       _maxObjHeight;
        optional<bool>        _isTiled;
        optional<unsigned>    _randomSeed;
        optional<StringExpression> _name;
    };

    typedef std::vector< osg::ref_ptr<SkinResource> > SkinResourceVector;


} } // namespace osgEarth::Symbology

#endif // OSGEARTHSYMBOLOGY_SKIN_RESOURCE_H
