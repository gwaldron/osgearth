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

#ifndef OSGEARTH_GEOMETRY_CLAMPER
#define OSGEARTH_GEOMETRY_CLAMPER 1

#include <osgEarth/Common>
#include <osgEarth/SpatialReference>
#include <osgEarth/Terrain>
#include <osgUtil/LineSegmentIntersector>
#include <osg/NodeVisitor>
#include <osg/fast_back_stack>

namespace osgEarth
{
    /**
     * Utility that takes existing OSG geometry and modifies it so that
     * it "conforms" with a terrain patch.
     * TODO: Consider relocating this into Annotation namespace
     */
    class OSGEARTH_EXPORT GeometryClamper : public osg::NodeVisitor
    {
    public:
        class GeometryData {
            osg::ref_ptr<osg::Vec3Array> _verts;
            osg::ref_ptr<osg::FloatArray> _altitudes;
            friend class GeometryClamper;
        };

        typedef std::map<osg::Array*, GeometryData> LocalData;      

    public:
        //! Construct a geometry clamper, passing in a data structure managed
        //! by the caller.
        GeometryClamper(LocalData& data);

        virtual ~GeometryClamper() { }

        //! Terrain graph the clamper should use to sample in-memory terrain
        void setTerrainPatch(osg::Node* node) { _terrainPatch = node; }
        osg::Node* getTerrainPatch() const { return _terrainPatch.get(); }

        //! SRS of the terrain model in memory
        void setTerrainSRS(const SpatialReference* srs) { _terrainSRS = srs; }
        const SpatialReference* getTerrainSRS() const   { return _terrainSRS.get(); }

        //! Whether to incorporate (add) the original vertex's altitude to the result
        //! of the clamping operation. Default=true. If false, the vert's height information 
        //! is ignored.
        void setUseVertexZ(bool value) { _useVertexZ = value; }
        bool getUseVertexZ() const     { return _useVertexZ; }

        void setScale(float scale) { _scale = scale; }
        float getScale() const     { return _scale; }

        void setOffset(float offset) { _offset = offset; }
        float getOffset() const      { return _offset; }

        //! Whether to revert a previous clamping operation (default=false)
        void setRevert(bool value) { _revert = value; }

    public: // osg::NodeVisitor

        void apply( osg::Drawable& );
        void apply( osg::Transform& );

    protected:

        LocalData&                           _localData;
        osg::ref_ptr<osg::Node>              _terrainPatch;
        osg::ref_ptr<const SpatialReference> _terrainSRS;
        bool                                 _useVertexZ;
        bool                                 _revert;
        float                                _scale;
        float                                _offset;
        osg::fast_back_stack<osg::Matrixd>   _matrixStack;
        osg::ref_ptr<osgUtil::LineSegmentIntersector> _lsi;
    };


    class GeometryClamperCallback : public osgEarth::TerrainCallback
    {
    public:
        GeometryClamperCallback();

        virtual ~GeometryClamperCallback() { }

        /** Access to configure the underlying clamper */
        GeometryClamper& getClamper()             { return _clamper; }
        const GeometryClamper& getClamper() const { return _clamper; }

    public: // TerrainCallback
        
        virtual void onTileAdded(
            const TileKey&          key, 
            osg::Node*              tile, 
            TerrainCallbackContext& context);

    protected:
        GeometryClamper _clamper;
    };

} // namespace osgEarth

#endif // OSGEARTH_GEOMETRY_CLAMPER
