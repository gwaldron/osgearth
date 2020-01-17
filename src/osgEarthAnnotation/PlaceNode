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
#ifndef OSGEARTH_ANNOTATION_PLACE_NODE_H
#define OSGEARTH_ANNOTATION_PLACE_NODE_H 1

#include <osgEarthAnnotation/GeoPositionNode>
#include <osgEarthSymbology/Style>
#include <osgEarth/ScreenSpaceLayout>

namespace osgEarth { namespace Annotation
{
    using namespace osgEarth;
    using namespace osgEarth::Symbology;

    /** 
     * A PlaceNode combines an 2D icon, a label, support for mouse interaction
     * and a popup control (eventually)
     */
    class OSGEARTHANNO_EXPORT PlaceNode : public GeoPositionNode
    {
    public:
        META_AnnotationNode(osgEarthAnnotation, PlaceNode);

        //! Constructs a new empty place node.
        PlaceNode();

        //! Construct new node with text content and default styling.
        PlaceNode(const std::string& text,
                  const Style&       style =Style(),
                  osg::Image*        image =0L);
        
        //! Construct new node with position, text content and default styling.
        PlaceNode(const GeoPoint&    position,
                  const std::string& text  ="",
                  const Style&       style =Style(),
                  osg::Image*        image =0L);

        //! Deserialize a place node.
        PlaceNode(
            const Config& conf,
            const osgDB::Options* readOptions );

        /**
         * Image to use for the icon
         */
        void setIconImage(osg::Image* image);
        osg::Image* getIconImage() const { return _image.get(); }

        /**
         * Text label content
         */
        void setText( const std::string& text );
        virtual const std::string& getText() const { return _text; }

        /**
         * Style (for text and placement)
         */
        void setStyle(const Style& style );
        void setStyle(const Style& style, const osgDB::Options* readOptions);
        const Style& getStyle() const { return _style; }

    public: // GeoPositionNode override

        virtual void setPriority( float value );
        virtual void setDynamic( bool value );

        virtual Config getConfig() const;

        virtual void dirty();

    protected: // AnnotationNode override
        
        virtual bool supportsRenderBinDetails() const { return false; }

    protected:

        virtual ~PlaceNode() { }

        osg::ref_ptr<osg::Image>            _image;
        std::string                         _text;
        Style                               _style;
        osg::Group*                         _geode;
        osg::Geometry*                      _imageDrawable;
        osg::Drawable*                      _bboxDrawable;
        osgText::Text*                      _textDrawable;
        osg::ref_ptr<const osgDB::Options>  _readOptions;
        osg::ref_ptr<ScreenSpaceLayoutData> _dataLayout;

        /** rotation of the label **/
        float                    _labelRotationRad;
        bool                     _followFixedCourse;
        GeoPoint                 _geoPointLoc;
        GeoPoint                 _geoPointProj;

        //void init();
        void updateLayoutData();
        void construct();
        void compile();

    public:

        void setConfig(const Config& conf);

    private:

        PlaceNode(const PlaceNode& rhs, const osg::CopyOp& op =osg::CopyOp::DEEP_COPY_ALL) : GeoPositionNode(rhs, op) { }

    private:
        static osg::observer_ptr<osg::StateSet> s_geodeStateSet;
        static osg::observer_ptr<osg::StateSet> s_imageStateSet;

        osg::ref_ptr<osg::StateSet> _geodeStateSet, _imageStateSet;
    };

} } // namespace osgEarth::Annotation

#endif //OSGEARTH_ANNOTATION_PLACE_NODE_H
