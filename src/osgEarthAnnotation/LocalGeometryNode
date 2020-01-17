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
#ifndef OSGEARTH_ANNO_LOCAL_GEOMETRY_NODE_H
#define OSGEARTH_ANNO_LOCAL_GEOMETRY_NODE_H 1

#include <osgEarthAnnotation/GeoPositionNode>
#include <osgEarth/MapNode>
#include <osgEarth/GeometryClamper>
#include <osgEarthSymbology/Geometry>
#include <osgEarthSymbology/Style>

namespace osgEarth { namespace Annotation
{	
    using namespace osgEarth;
    using namespace osgEarth::Symbology;

    /**
     * Simple node that renders geometry into a scene graph node. The geometry
     * is in a local tangent plane that may be positioned somewhere on the 
     * Map's surface.
     *
     * As a GeoPositionNode, the LGN's main location is controlled by the 
     * setPosition method. Terrain following is controlled by using a GeoPoint
     * with an altitude value relative to the terrain.
     */
    class OSGEARTHANNO_EXPORT LocalGeometryNode : public GeoPositionNode
    {
    public:
        //! Construct a new empty LocalGeometryNode
        LocalGeometryNode();
        
        //! Construct a new empty LocalGeometryNode and assign it a geoemtry and style
        LocalGeometryNode( 
            Geometry*    geom, 
            const Style& style );

        /**
         * Gets the Style used to create thenode.
         */
        const Style& getStyle() const { return _style; }

        /**
         * Sets a new style for the node
         */
        void setStyle(const Style& style);

        /**
         * Gets the geometry used to create this node (if applicable)
         */
        const Geometry* getGeometry() const { return _geom.get(); }

        /**
         * Sets new geometry for this node.
         */
        void setGeometry( Geometry* geom );


    public: // GeoPositionNode

        virtual void setPosition(const GeoPoint&);

    public: // AnnotationNode

        virtual void setMapNode(MapNode*);

    public:

        /**
         * Constructs an LGN from a serialized Config.
         */
        LocalGeometryNode(
            const Config&         conf,
            const osgDB::Options* dbOptions);

        virtual Config getConfig() const;

    public: // osg::Node

        virtual void traverse(osg::NodeVisitor&);

    protected:

        virtual ~LocalGeometryNode() { }

        Style                        _style;
        osg::ref_ptr<osg::Node>      _node;
        osg::ref_ptr<Geometry>       _geom;
        bool                         _clampInUpdateTraversal;
        bool                         _perVertexClampingEnabled;
        
        typedef TerrainCallbackAdapter<LocalGeometryNode> ClampCallback;
        osg::ref_ptr<ClampCallback> _clampCallback;
        GeometryClamper::LocalData _clamperData;

        void compileGeometry();
        void togglePerVertexClamping();
        void reclamp();

    public:
        void onTileAdded(
            const TileKey&          key, 
            osg::Node*              graph, 
            TerrainCallbackContext& context);

        virtual void clamp(
            osg::Node*     graph,
            const Terrain* terrain);

    private:

        void construct();

        GeoPoint _lastPosition;
    };

} } // namespace osgEarth::Annotation

#endif // OSGEARTH_ANNO_LOCAL_GEOMETRY_NODE_H
