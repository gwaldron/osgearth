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

#ifndef OSGEARTHFEATURES_EXTRUDE_GEOMETRY_FILTER_H
#define OSGEARTHFEATURES_EXTRUDE_GEOMETRY_FILTER_H 1

#include <osgEarthFeatures/Common>
#include <osgEarthFeatures/Filter>
#include <osgEarthSymbology/Expression>
#include <osgEarthSymbology/Style>
#include <osg/Geode>
#include <vector>
#include <list>

namespace osgEarth {
    namespace Symbology {
        class ResourceLibrary;
    }
}

namespace osgEarth { namespace Features 
{
    using namespace osgEarth;
    using namespace osgEarth::Symbology;
    
    class FeatureSourceIndex;
    class FeatureIndexBuilder;

    /**
     * Extrudes footprint geometry into 3D geometry
     */
    class OSGEARTHFEATURES_EXPORT ExtrudeGeometryFilter : public FeaturesToNodeFilter
    {
    public:
        struct HeightCallback : public osg::Referenced
        {
            virtual float operator()( Feature* input, const FilterContext& cx ) =0;
        };

    public:

        /** Constructs a new filter that will extrude footprints */
        ExtrudeGeometryFilter();

        virtual ~ExtrudeGeometryFilter() { }

        /**
         * Sets the style that will govern the geometry generation.
         */
        void setStyle( const Style& style );

        /**
         * Pushes a list of features through the filter.
         */
        osg::Node* push( FeatureList& input, FilterContext& context );

    public: // properties

        /**
         * Sets the maximum wall angle that doesn't require a new normal vector
         */
        void setWallAngleThreshold(float value) { _wallAngleThresh_deg = value; }
        float getWallAngleThreshold() const { return _wallAngleThresh_deg; }

        /**
         * Sets whether to render a bottom top. Useful for creating stencil volumes.
         */
        void setMakeStencilVolume( bool value ) { _makeStencilVolume = value; }
        
        /**
         * Sets the expression to evaluate when setting a feature name.
         * NOTE: setting this forces geometry-merging to OFF
         */
        void setFeatureNameExpr( const StringExpression& expr ) { _featureNameExpr = expr; }
        const StringExpression& getFeatureNameExpr() const { return _featureNameExpr; }

        /**
         * Whether to merge geometry for better rendering performance
         */
        void setMergeGeometry(bool value) { _mergeGeometry = value; }
        bool getMergeGeometry() const { return _mergeGeometry; }


    protected:

        // A Corner is one vertex in the source geometry, extrude from base to roof.
        struct Corner
        {
            osg::Vec3d base, roof;
            float      roofTexU, roofTexV;
            double     offsetX;
            float      wallTexHeightAdjusted;
            bool       isFromSource;
            float      cosAngle;
            float      height;
        };
        typedef std::list<Corner> Corners; // use a list to prevent iterator invalidation

        // A Face joins to Corners.
        struct Face
        {
            Corner left;
            Corner right;
            double widthM;
        };
        typedef std::vector<Face> Faces;
        
        // An Elevation is a series of related Faces.
        struct Elevation
        {
            Faces  faces;
            double texHeightAdjustedM;

            unsigned getNumPoints() const {
                return faces.size() * 6;
            }
        };
        typedef std::vector<Elevation> Elevations;

        // A Structure is a collection of related Elevations.
        struct Structure
        {
            Elevations elevations;
            bool       isPolygon;
            osg::Vec3d baseCentroid;
            float      verticalOffset;

            unsigned getNumPoints() const {
                unsigned c = 0;
                for(Elevations::const_iterator e = elevations.begin(); e != elevations.end(); ++e ) {
                    c += e->getNumPoints();
                }
                return c;
            }
        };

        // a set of geodes indexed by stateset pointer, for pre-sorting geodes based on 
        // their texture usage
        typedef std::map<osg::StateSet*, osg::ref_ptr<osg::Geode> > SortedGeodeMap;
        SortedGeodeMap                 _geodes;
        SortedGeodeMap                 _lineGroups;
        osg::ref_ptr<osg::StateSet>    _noTextureStateSet;

        bool                           _mergeGeometry;
        float                          _wallAngleThresh_deg;
        float                          _cosWallAngleThresh;
        StringExpression               _featureNameExpr;
        osg::ref_ptr<HeightCallback>   _heightCallback;
        optional<NumericExpression>    _heightExpr;
        bool                           _makeStencilVolume;

        Style                          _style;
        bool                           _styleDirty;
        bool                           _gpuClamping;

        osg::ref_ptr<const ExtrusionSymbol> _extrusionSymbol;
        osg::ref_ptr<const PolygonSymbol>   _polySymbol;
        osg::ref_ptr<const SkinSymbol>      _wallSkinSymbol;
        osg::ref_ptr<const PolygonSymbol>   _wallPolygonSymbol;
        osg::ref_ptr<const SkinSymbol>      _roofSkinSymbol;
        osg::ref_ptr<const PolygonSymbol>   _roofPolygonSymbol;
        osg::ref_ptr<const LineSymbol>      _outlineSymbol;
        osg::ref_ptr<ResourceLibrary>       _wallResLib;
        osg::ref_ptr<ResourceLibrary>       _roofResLib;

        void reset( const FilterContext& context );
        
        void addDrawable( 
            osg::Drawable*       drawable, 
            osg::StateSet*       stateSet, 
            const std::string&   name,
            Feature*             feature,
            FeatureIndexBuilder* index);
        
        bool process( 
            FeatureList&     input,
            FilterContext&   context );
        
        bool buildStructure(const Geometry*         input,
                            double                  height,
                            bool                    flatten,
                            float                   verticalOffset,
                            const SkinResource*     wallSkin,
                            const SkinResource*     roofSkin,
                            Structure&              out_structure,
                            FilterContext&          cx );

        bool buildWallGeometry(const Structure&     structure,
                               osg::Geometry*       walls,
                               const osg::Vec4&     wallColor,
                               const osg::Vec4&     wallBaseColor,
                               const SkinResource*  wallSkin);

        bool buildRoofGeometry(const Structure&     structure,
                               osg::Geometry*       roof,
                               const osg::Vec4&     roofColor,
                               const SkinResource*  roofSkin);

        osg::Drawable* buildOutlineGeometry(const Structure& structure);
    };

} } // namespace osgEarth::Features

#endif // OSGEARTHFEATURES_BUILD_GEOMETRY_FILTER_H
