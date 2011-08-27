/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2010 Pelican Mapping
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
#include <osgEarthFeatures/ExtrudeGeometryFilter>
#include <osgEarthSymbology/MeshSubdivider>
#include <osgEarthSymbology/MeshConsolidator>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/ClusterCullingCallback>
#include <osgUtil/Tessellator>
#include <osgUtil/Optimizer>
#include <osgUtil/SmoothingVisitor>
#include <osg/Version>
#include <osgEarth/Version>

#define LC "[ExtrudeGeometryFilter] "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;


ExtrudeGeometryFilter::ExtrudeGeometryFilter() :
_maxAngle_deg       ( 5.0 ),
_mergeGeometry      ( false ),
_height             ( 10.0 ),
_flatten            ( true ),
_wallAngleThresh_deg( 60.0 ),
_color              ( osg::Vec4f(1, 1, 1, 1) )
{
    reset();
}

void
ExtrudeGeometryFilter::reset()
{
    //_geode = new osg::Geode();
    _cosWallAngleThresh = cos( _wallAngleThresh_deg );
}

void
ExtrudeGeometryFilter::setPropertiesFromStyle( const Style& style )
{
    const ExtrusionSymbol* extrusion = style.get<ExtrusionSymbol>();
    if ( extrusion )
    {
        if ( extrusion->height().isSet() )
            setExtrusionHeight( *extrusion->height() );
        if ( extrusion->heightExpression().isSet() )
            setExtrusionExpr( *extrusion->heightExpression() );
        if ( extrusion->heightReference() == ExtrusionSymbol::HEIGHT_REFERENCE_MSL )
            setHeightOffsetExpression( NumericExpression("[__max_z]") );
        setFlatten( *extrusion->flatten() );
    }

    const PolygonSymbol* polygon = style.get<PolygonSymbol>();
    if ( polygon )
    {
        setColor( polygon->fill()->color() );
    }

    _wallSkinSymbol = style.get<SkinSymbol>();
}

#define USE_TEX 1

bool
ExtrudeGeometryFilter::extrudeGeometry(const Geometry*         input,
                                       double                  height,
                                       double                  heightOffset,
                                       bool                    flatten,
                                       osg::Geometry*          walls,
                                       osg::Geometry*          topCap,
                                       osg::Geometry*          bottomCap,
                                       const osg::Vec4&        color,
                                       const SkinResource*     skin,
                                       FilterContext&          cx )
{
    bool made_geom = false;

    double tex_width_m   = skin ? *skin->imageWidth() : 1.0;
    double tex_height_m  = skin ? *skin->imageHeight() : 1.0;
    bool   tex_repeats_y = skin ? *skin->repeatsVertically() : false;
    bool   useColor      = !skin || skin->texEnvMode() != osg::TexEnv::DECAL;

    bool isPolygon = input->getComponentType() == Geometry::TYPE_POLYGON;

    unsigned pointCount = input->getTotalPointCount();
    unsigned numVerts = 2 * pointCount;

    // create all the OSG geometry components
    osg::Vec3Array* verts = new osg::Vec3Array( numVerts );
    walls->setVertexArray( verts );

    osg::Vec2Array* texcoords = 0L;
    if ( skin )
    { 
        texcoords = new osg::Vec2Array( numVerts );
        walls->setTexCoordArray( 0, texcoords );
    }

    osg::Vec4Array* colors = 0L;
    if ( useColor )
    {
        colors = new osg::Vec4Array( 1 );
        (*colors)[0] = color;
        walls->setColorArray( colors );
        walls->setColorBinding( osg::Geometry::BIND_OVERALL );
    }

    osg::Vec3Array* topVerts = NULL;
    osg::Vec4Array* topColors = NULL;
    if ( topCap )
    {
        topVerts = new osg::Vec3Array( pointCount );
        topCap->setVertexArray( topVerts );

        //todo: use colors for cap? depends on whether there's a roof texture.
        topColors = new osg::Vec4Array( 1 );
        (*topColors)[0] = color;
        topCap->setColorArray( topColors );
        topCap->setColorBinding( osg::Geometry::BIND_OVERALL );
    }

    osg::Vec3Array* bottomVerts = NULL;
    if ( bottomCap )
    {
        bottomVerts = new osg::Vec3Array( pointCount );
        bottomCap->setVertexArray( bottomVerts );
    }

    unsigned wallVertPtr   = 0;
    unsigned topVertPtr    = 0;
    unsigned bottomVertPtr = 0;

    double targetLen = -DBL_MAX;
    osg::Vec3d minLoc(DBL_MAX, DBL_MAX, DBL_MAX);
    double minLoc_len = DBL_MAX;
    osg::Vec3d maxLoc(0,0,0);
    double maxLoc_len = 0;

    // Initial pass over the geometry does two things:
    // 1: Calculate the minimum Z across all parts.
    // 2: Establish a "target length" for extrusion
    ConstGeometryIterator zfinder( input );
    while( zfinder.hasMore() )
    {
        const Geometry* geom = zfinder.next();
        for( Geometry::const_iterator m = geom->begin(); m != geom->end(); ++m )
        {
            // Find the minimum Z
            osg::Vec3d m_world = cx.toWorld( *m );
            if ( cx.isGeocentric() )
            {
                osg::Vec3d p_vec = m_world;
                osg::Vec3d e_vec = p_vec;
                e_vec.normalize();
                p_vec = p_vec + (e_vec * height);

                double m_world_len = m_world.length();
                if ( m_world_len < minLoc_len )
                {
                    minLoc = m_world;
                    minLoc_len = m_world_len;
                }

                if ( m_world_len > maxLoc_len )
                {
                    maxLoc = m_world;
                    maxLoc_len = m_world_len;
                }

                double p_ex_len = p_vec.length();
                if ( p_ex_len > targetLen )
                    targetLen = p_ex_len;
            }
            else
            {
                if ( m_world.z() + height > targetLen )
                    targetLen = m_world.z() + height;

                if (m_world.z() < minLoc.z())
                    minLoc = m_world;

                if ( m_world.z() > maxLoc.z())
                    maxLoc = m_world;
            }
        }
    }

    // apply the height offsets
    height -= heightOffset;
    targetLen -= heightOffset;

    // now generate the extruded geometry.
    ConstGeometryIterator iter( input );
    while( iter.hasMore() )
    {
        const Geometry* part = iter.next();

        double tex_height_m_adj = tex_height_m;

        unsigned wallPartPtr = wallVertPtr;
        unsigned topPartPtr  = topVertPtr;
        unsigned bottomPartPtr = bottomVertPtr;
        double   partLen = 0.0;
        double   maxHeight = 0.0;

        if ( cx.isGeocentric() )
            maxHeight = targetLen - minLoc_len;
        else
            maxHeight = targetLen - minLoc.z();

        //Adjust the texture height so it is a multiple of the maximum height
        double div = osg::round(maxHeight / tex_height_m);
        if (div == 0) div = 1; //Prevent divide by zero
        tex_height_m_adj = maxHeight / div;

        osg::DrawElementsUInt* idx = new osg::DrawElementsUInt( GL_TRIANGLES );

        for( Geometry::const_iterator m = part->begin(); m != part->end(); ++m )
        {
            osg::Vec3d m_world = cx.toWorld( *m );

            // calculate our extrusion vector for this point.
            osg::Vec3d extrudeVec;
            if ( cx.isGeocentric() )
            {
                osg::Vec3d p_vec = m_world;
                if ( flatten )
                {
                    double p_len = p_vec.length();
                    double ratio = targetLen/p_len;
                    p_vec *= ratio;
                }
                else
                {
                    osg::Vec3d unitVec = p_vec;
                    unitVec.normalize();
                    p_vec = p_vec + unitVec*height;
                }

                extrudeVec = p_vec * cx.referenceFrame();
            }
            else
            {
                if ( flatten )
                    extrudeVec.set( m_world.x(), m_world.y(), targetLen );
                else
                    extrudeVec.set( m_world.x(), m_world.y(), m_world.z() + height );

                extrudeVec = extrudeVec * cx.referenceFrame();
            }

            // build the top and bottom caps
            if ( topCap )
            {
                //(*topColors)[topVertPtr]  = color;
                (*topVerts)[topVertPtr++] = extrudeVec;
            }
            if ( bottomCap )
            {
                (*bottomVerts)[bottomVertPtr++] = *m;
            }
             
            partLen += wallVertPtr > wallPartPtr ?
                (extrudeVec - (*verts)[wallVertPtr-2]).length() :
                0.0;

            double h;
            if ( tex_repeats_y )
                h = -(extrudeVec - *m).length();
            else
                h = -tex_height_m_adj;

            int p;

            p = wallVertPtr; // ++
            (*verts)[p] = extrudeVec;
            //if ( useColor )
            //    (*colors)[p] = color;
            if ( skin )
                (*texcoords)[p].set( partLen/tex_width_m, 0.0f );

            p = wallVertPtr+1; // ++
            (*verts)[p] = *m;
            //if ( useColor )
            //    (*colors)[p] = color;
            if ( skin )
                (*texcoords)[p].set( partLen/tex_width_m, h/tex_height_m_adj );

            // form the 2 triangles
            if ( (m+1) == part->end() )
            {
                if ( isPolygon )
                {
                    // end of the wall; loop around to close it off.
                    idx->push_back(wallVertPtr); 
                    idx->push_back(wallVertPtr+1);
                    idx->push_back(wallPartPtr);

                    idx->push_back(wallVertPtr+1);
                    idx->push_back(wallPartPtr+1);
                    idx->push_back(wallPartPtr);
                }
                else
                {
                    //nop - no elements required at the end of a line
                }
            }
            else
            {
                idx->push_back(wallVertPtr); 
                idx->push_back(wallVertPtr+1);
                idx->push_back(wallVertPtr+2); 

                idx->push_back(wallVertPtr+1);
                idx->push_back(wallVertPtr+3);
                idx->push_back(wallVertPtr+2);
            }

            wallVertPtr += 2;
            made_geom = true;
        }

        walls->addPrimitiveSet( idx );

        if ( topCap )
        {
            topCap->addPrimitiveSet( new osg::DrawArrays(
                osg::PrimitiveSet::LINE_LOOP,
                topPartPtr, topVertPtr - topPartPtr ) );
        }
        if ( bottomCap )
        {
            // reverse the bottom verts:
            int len = bottomVertPtr - bottomPartPtr;
            for( int i=bottomPartPtr; i<len/2; i++ )
                std::swap( (*bottomVerts)[i], (*bottomVerts)[bottomPartPtr+(len-1)-i] );

            bottomCap->addPrimitiveSet( new osg::DrawArrays(
                osg::PrimitiveSet::LINE_LOOP,
                bottomPartPtr, bottomVertPtr - bottomPartPtr ) );
        }
    }

    return made_geom;
}

bool
ExtrudeGeometryFilter::pushFeature( Feature* input, ResourceLibrary* reslib, FilterContext& context )
{
    GeometryIterator iter( input->getGeometry(), false );
    while( iter.hasMore() )
    {
        Geometry* part = iter.next();

        osg::ref_ptr<osg::Geometry> walls = new osg::Geometry();
        //walls->setUseVertexBufferObjects(true);
        
        osg::ref_ptr<osg::Geometry> rooflines = 0L;
        
        if ( part->getType() == Geometry::TYPE_POLYGON )
        {
            rooflines = new osg::Geometry();
            //rooflines->setUseVertexBufferObjects(true);

            // prep the shapes by making sure all polys are open:
            static_cast<Polygon*>(part)->open();
        }

        // calculate the extrusion height:
        float height;

        if ( _heightCallback.valid() )
        {
            height = _heightCallback->operator()(input, context);
        }
        else if ( _heightAttr.isSet() )
        {
            height = input->getDouble(*_heightAttr, _height);
        }
        else if ( _heightExpr.isSet() )
        {
            height = input->eval( _heightExpr.mutable_value() );
        }
        else
        {
            height = _height;
        }

        // calculate the height offset from the base:
        float offset = 0.0;
        if ( _heightOffsetExpr.isSet() )
        {
            offset = input->eval( _heightOffsetExpr.mutable_value() );
        }

        osg::StateSet* wallStateSet = 0L;

        // calculate the wall texturing:
        SkinResource* wallSkin = 0L;
        if ( _wallSkinSymbol.valid() )
        {
            if ( reslib )
            {
                SkinSymbol querySymbol( *_wallSkinSymbol.get() );
                querySymbol.objectHeight() = height;
                SkinResourceVector candidates;
                reslib->getSkins( &querySymbol, candidates );

                if ( candidates.size() == 1 )
                {
                    wallSkin = candidates[0].get();
                }
                else if ( candidates.size() > 1 )
                {
                    // select on at random:
                    int index = ::rand() % candidates.size();
                    wallSkin = candidates[index].get();
                }
            }

            else
            {
                //TODO: simple single texture?
            }
        }

        // Create the extruded geometry!
        if ( extrudeGeometry( part, height, offset, _flatten, walls.get(), rooflines.get(), 0L, _color, wallSkin, context ) )
        {      
            if ( wallSkin )
            {
                wallStateSet = context.resourceCache()->getStateSet( wallSkin );

#if 0
                if ( wall_ss )
                {
                    walls->setStateSet( wall_ss );
                }
#endif
                //wallGeodeKey = wall_ss;
            }

#if 0 // note: this was originally in there to prevent rooftop textures from sliding down to the walls in osgGIS.
            else
            {
                //There is no skin, so disable texturing for the walls to prevent other textures from being applied to the walls
                if ( !_noTextureStateSet.valid() )
                {
                    _noTextureStateSet = new osg::StateSet();
                    _noTextureStateSet->setTextureMode(0, GL_TEXTURE_2D, osg::StateAttribute::OFF);
                }

                walls->setStateSet( _noTextureStateSet.get() );
            }
#endif

            // generate per-vertex normals, altering the geometry as necessary to avoid
            // smoothing around sharp corners
#if OSG_MIN_VERSION_REQUIRED(2,9,9)
            //Crease angle threshold wasn't added until
            osgUtil::SmoothingVisitor::smooth(
                *walls.get(), 
                osg::DegreesToRadians(_wallAngleThresh_deg) );            
#else
            osgUtil::SmoothingVisitor::smooth(*walls.get());            
#endif

            // tessellate and add the roofs if necessary:
            if ( rooflines.valid() )
            {
                osgUtil::Tessellator tess;
                tess.setTessellationType( osgUtil::Tessellator::TESS_TYPE_GEOMETRY );
                tess.setWindingType( osgUtil::Tessellator::TESS_WINDING_ODD ); //POSITIVE );
                tess.retessellatePolygons( *(rooflines.get()) );

                // generate default normals (no crease angle necessary; they are all pointing up)
                // TODO do this manually; probably faster
                osgUtil::SmoothingVisitor::smooth( *rooflines.get() );

                // texture the rooflines if necessary
                //applyOverlayTexturing( rooflines.get(), input, env );
                
                // reorganize the drawable into a single triangle set.
                // TODO: probably deprecate this once we start texturing rooftops
                // NOTE: MC is now called elsewhere.
                //MeshConsolidator::run( *rooflines.get() );

                // mark this geometry as DYNAMIC because otherwise the OSG optimizer will destroy it.
                // TODO: why??
                rooflines->setDataVariance( osg::Object::DYNAMIC );
            }

            //applyFragmentName( new_fragment, input, env );

            std::string name;
            if ( !_featureNameExpr.empty() )
                name = input->eval( _featureNameExpr );

            // find the geode for the active stateset:
            osg::Geode* geode = _geodes[wallStateSet].get();
            if ( !geode ) {
                geode = new osg::Geode();
                geode->setStateSet( wallStateSet );
                _geodes[wallStateSet] = geode;
            }

            geode->addDrawable( walls.get() );
            if ( !name.empty() )
                walls->setName( name );

            if ( rooflines.valid() )
            {
                // For now, sort the rooftops into the "state-set-less" geode. Later these will
                // sort into other geodes based on rooftop texturing
                osg::Geode* noTexGeode = _geodes[0L].get();
                if ( !noTexGeode ) {
                    noTexGeode = new osg::Geode();
                    _geodes[0L] = noTexGeode;
                }
                
                noTexGeode->addDrawable( rooflines.get() );
                if ( !name.empty() )
                    rooflines->setName( name );
            }
        }   
    }


    return true;
}

namespace 
{
    struct EnableVBO : public osg::NodeVisitor 
    {
        EnableVBO() : osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN ) { }

        void accept( osg::Geode& geode )
        {
            for( unsigned i=0; i<geode.getNumDrawables(); ++i )
            {
                osg::Geometry* geom = geode.getDrawable(i)->asGeometry();
                if ( geom )
                {
                    geom->setUseVertexBufferObjects( true );
                }
            }
            traverse( geode );
        }
    };
}

osg::Node*
ExtrudeGeometryFilter::push( FeatureList& input, FilterContext& context )
{
    reset();

    // establish the active resource library, if applicable.
    const StyleSheet* sheet = context.getSession()->styles();
    ResourceLibrary*  lib = 0L;

    if (sheet != 0L                            &&
        _wallSkinSymbol.valid()                &&
        _wallSkinSymbol->libraryName().isSet() )
    {
        lib = sheet->getResourceLibrary( *_wallSkinSymbol->libraryName() );
        if ( !lib )
        {
            OE_WARN << LC << "Unable to load resource library '" << *_wallSkinSymbol->libraryName() << "'"
                << "; extruded geometry will not be textured." << std::endl;
        }
    }

    // push all the features through the extruder.
    bool ok = true;
    for( FeatureList::iterator i = input.begin(); i != input.end(); i++ )
    {
        pushFeature( i->get(), lib, context );
    }
   
    // NOTE: **** GW commented out to test texturing ****
#if 1
    // convert everything to triangles and combine drawables.    
    if ( _featureNameExpr.empty() )
    {
        for( SortedGeodeMap::iterator i = _geodes.begin(); i != _geodes.end(); ++i )
            MeshConsolidator::run( *i->second.get() );
    }
#endif

    // combines geometries where the statesets are the same.
    osg::Group* group = new osg::Group();
    for( SortedGeodeMap::iterator i = _geodes.begin(); i != _geodes.end(); ++i )
        group->addChild( i->second.get() );
    _geodes.clear();

    OE_INFO << LC << "Sorted geometry into " << group->getNumChildren() << " groups" << std::endl;

    //TODO
    // running this after the MC reduces the primitive set count by a huge amount, but I
    // have not figured out why yet.
    osgUtil::Optimizer o;
    o.optimize( group, osgUtil::Optimizer::MERGE_GEOMETRY ); // | osgUtil::Optimizer::SHARE_DUPLICATE_STATE );

    return group;
    //return _geode.release();
}
