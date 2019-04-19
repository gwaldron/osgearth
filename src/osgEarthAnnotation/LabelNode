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

#ifndef OSGEARTH_ANNOTATION_LABEL_NODE_H
#define OSGEARTH_ANNOTATION_LABEL_NODE_H 1

#include <osgEarthAnnotation/GeoPositionNode>
#include <osgEarthSymbology/Style>
#include <osgEarth/MapNode>
#include <osgEarth/ScreenSpaceLayout>
#include <osg/Geode>

namespace osgEarth { namespace Annotation
{
    using namespace osgEarth;
    using namespace osgEarth::Symbology;

    /**
     * Text labeling node.
     */
    class OSGEARTHANNO_EXPORT LabelNode : public GeoPositionNode
    {
    public:
        META_AnnotationNode( osgEarthAnnotation, LabelNode );

        //! Construct a new empty label node.
        LabelNode();

        //! Construct new node with text content and default styling.
        LabelNode(const std::string& text,
                  const Style&       style =Style());

        LabelNode(const GeoPoint&    position,
                  const std::string& text  ="",
                  const Style&       style =Style());

        //! Deserialize a label node
        LabelNode(
            const Config& conf,
            const osgDB::Options* dbOptions );

        /**
         * Sets the text content.
         */
        void setText( const std::string& text );
        const std::string& text() const { return _text; }
        virtual const std::string& getText() const { return text(); }

        /**
         * Gets a copy of the text style.
         */
        const Style& getStyle() const { return _style; }

        /**
         * Sets a new text style
         */
        void setStyle( const Style& style );

    public: // GeoPositionNode override

        virtual void setPriority(float value);

        virtual void setDynamic( bool value );

        virtual Config getConfig() const;

        virtual void dirty();

    protected: // AnnotationNode override
        
        virtual bool supportsRenderBinDetails() const { return false; }

    protected:

        virtual ~LabelNode() { }

        std::string              _text;
        Style                    _style;
        osg::ref_ptr<osg::Geode> _geode;
        osg::ref_ptr<ScreenSpaceLayoutData> _dataLayout;

        /** rotation of the label **/
        float                    _labelRotationRad;
        bool                     _followFixedCourse;
        GeoPoint                 _geoPointLoc;
        GeoPoint                 _geoPointProj;

        /** Copy constructor */
        LabelNode(const LabelNode& rhs, const osg::CopyOp& op = osg::CopyOp::DEEP_COPY_ALL) { }

        void updateLayoutData();

    private:
        static osg::observer_ptr<osg::StateSet> s_geodeStateSet;
        void construct();
        void compile();

    };

} } // namespace osgEarth::Annotation

#endif // OSGEARTH_ANNOTATION_LABEL_NODE_H
