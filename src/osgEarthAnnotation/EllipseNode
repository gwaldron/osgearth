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

#ifndef OSGEARTH_ANNOTATION_ELLIPSE_NODE_H
#define OSGEARTH_ANNOTATION_ELLIPSE_NODE_H 1

#include <osgEarthAnnotation/LocalGeometryNode>
#include <osgEarthSymbology/Style>
#include <osgEarth/Units>
#include <osgEarth/MapNode>

namespace osgEarth { namespace Annotation
{	
    using namespace osgEarth;
    using namespace osgEarth::Symbology;

    /** 
     * Renders an ellipse that can drape on a MapNode terrain.    
     */
    class OSGEARTHANNO_EXPORT EllipseNode : public LocalGeometryNode
    {
    public:
        META_AnnotationNode( osgEarthAnnotation, EllipseNode );

        //! Constuct a new default ellipsoid.
        EllipseNode();

        //! Set all the parameters together
        //! @param position    Location of the annotation, in map coordinates
        //! @param radiusMajor Length of 1/2 the major axis of the ellipse
        //! @param radiusMinor Length of 1/2 the minor axis of the ellipse
        //! @param rotation    Rotation angle of the ellipse
        //! @param style       Style defining how the annotation will look
        //! @param arcStart    Optional start of arc (default to 0.0 degrees)
        //! @param arcStop     Optional end of arc (default to 360.0 degrees)
        //! @param pie         Optionally make pie shape instead of arc
        void set(
            const GeoPoint&         position,
            const Distance&         radiusMajor,
            const Distance&         radiusMinor,
            const Angle&            rotationAngle,
            const Style&            style,
            const Angle&            arcStart = Angle(0.0, Units::DEGREES),
            const Angle&            arcEnd = Angle(360.0, Units::DEGREES),
            const bool              pie = false);

        /**
         * Gets the major radius
         */
        const Distance& getRadiusMajor() const;

        /**
         * Gets the minor radius
         */
        const Distance& getRadiusMinor() const;

        /**
         * Sets the major radius
         */
        void setRadiusMajor( const Distance& radiusMajor );

        /**
         * Sets the minor radius
         */
        void setRadiusMinor( const Distance& radiusMinor );

        /**
         * Sets the major and minor radii
         */
        void setRadii( const Distance& radiusMajor, const Distance& radiusMinor );

        /*
         * Gets the rotation angle
         */
        const Angle& getRotationAngle() const;

        /**
         * Sets the rotation angle
         */
        void setRotationAngle(const Angle& rotationAngle);

        /**
         * Gets the number of segments
         */
        unsigned int getNumSegments() const;

        /**
         * Sets the number of segments
         */
        void setNumSegments(unsigned int numSegments );

        /**
         * Gets the start degrees of this (arc) circle
         */
        const Angle& getArcStart(void) const;

        /**
         * Sets the start degrees of this (arc) circle
         */
        void setArcStart(const Angle& arcStart);

        /**
         * Gets the end degrees of this (arc) circle
         */
        const Angle& getArcEnd(void) const;

        /**
         * Sets the end degrees of this (arc) circle
         */
        void setArcEnd(const Angle& arcEnd);

        /**
         * Gets the pie flag
         */
        const bool& getPie(void) const;

        /**
         * Sets the pie flag
         */
        void setPie(const bool& pie);

    public:

        EllipseNode(const Config& conf, const osgDB::Options* dbOptions);

        virtual Config getConfig() const;

    protected:

        virtual ~EllipseNode() { }

    private:
        EllipseNode(const EllipseNode& rhs, const osg::CopyOp& op) { }

        void construct();

        void buildGeometry();
        
        Angle _rotationAngle;
        Distance _radiusMajor;
        Distance _radiusMinor;
        Angle _arcStart;
        Angle _arcEnd;
        bool  _pie;
        unsigned int _numSegments;
    };

} } // namespace osgEarth::Annotation

#endif // OSGEARTH_ANNOTATION_ELLIPSE_NODE_H
