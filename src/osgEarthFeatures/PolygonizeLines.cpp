/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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
#include <osgEarthFeatures/PolygonizeLines>
#include <osgEarthFeatures/FeatureSourceIndexNode>
#include <osgEarthFeatures/Session>
#include <osgEarthSymbology/MeshConsolidator>
#include <osgEarth/VirtualProgram>
#include <osgEarth/Utils>
#include <osgEarth/CullingUtils>

#define LC "[PolygonizeLines] "

using namespace osgEarth::Features;

#define OV(p) "("<<p.x()<<","<<p.y()<<")"

#define ATTR_LOCATION osg::Drawable::ATTRIBUTE_7

namespace
{
    typedef std::pair<osg::Vec3,osg::Vec3> Segment;
    
    // Given two rays (point + direction vector), find the intersection
    // of those rays in 2D space and put the result in [out]. Return true
    // if they intersect, false if they do not.
    bool interesctRays(const osg::Vec3& p0, const osg::Vec3& pd, // point, dir
                       const osg::Vec3& q0, const osg::Vec3& qd, // point, dir
                       const osg::Vec3& normal,                  // normal at control point
                       osg::Vec3&       out)
    {
        // make the conversion quats:
        osg::Quat toLocal, toWorld;
        toLocal.makeRotate( normal, osg::Vec3(0,0,1) );
        toWorld.makeRotate( osg::Vec3(0,0,1), normal );

        // convert the inputs:
        osg::Vec3 p0r = p0; //toLocal*p0; //(p0-cp);
        osg::Vec3 pdr = toLocal*pd;
        osg::Vec3 q0r = q0; //toLocal*q0; //(q0-cp);
        osg::Vec3 qdr = toLocal*qd;

        // this epsilon will cause us to skip invalid or colinear rays.
        const float epsilon = 0.001f;

        float det = pdr.y()*qdr.x()-pdr.x()*qdr.y();
        if ( osg::equivalent(det, 0.0f, epsilon) )
            return false;

        float u = (qdr.x()*(q0r.y()-p0r.y())+qdr.y()*(p0r.x()-q0r.x()))/det;
        if ( u < epsilon )
            return false;

        float v = (pdr.x()*(q0r.y()-p0r.y())+pdr.y()*(p0r.x()-q0r.x()))/det;
        if ( v < epsilon )
            return false;

        out = /*cp +*/ (toWorld * (p0r + pdr*u));

        return true;
    }

    // Rotate the directional vector [in] counter-clockwise by [angle] radians
    // and return the result in [out].
    inline void rotate(const osg::Vec3& in, float angle, const osg::Vec3& normal, osg::Vec3& out)
    {
        osg::Quat rot( angle, normal );
        out = rot * in;
    }

    // Add two triangles to an EBO vector; [side] controls the winding
    // direction.
    inline void addTris(std::vector<unsigned>& ebo, unsigned i, unsigned prev_i, unsigned current, float side)
    {
        if ( side < 0.0f )
        {
            ebo.push_back( i-1 );
            ebo.push_back( i );
            ebo.push_back( prev_i );
            ebo.push_back( prev_i );
            ebo.push_back( i );
            ebo.push_back( current );
        }
        else
        {
            ebo.push_back( i-1 );
            ebo.push_back( prev_i );
            ebo.push_back( i );
            ebo.push_back( prev_i );
            ebo.push_back( current );
            ebo.push_back( i );
        }
    }

    // Add a triangle to an EBO vector; [side] control the winding
    // direction.
    inline void addTri(std::vector<unsigned>& ebo, unsigned i0, unsigned i1, unsigned i2, float side)
    {
        ebo.push_back( i0 );
        ebo.push_back( side < 0.0f ? i1 : i2 );
        ebo.push_back( side < 0.0f ? i2 : i1 );
    }
}


PolygonizeLinesOperator::PolygonizeLinesOperator(const Stroke& stroke) :
_stroke( stroke )
{
    //nop
}


osg::Geometry*
PolygonizeLinesOperator::operator()(osg::Vec3Array* verts, 
                                    osg::Vec3Array* normals,
                                    bool            twosided) const
{
    // number of verts on the original line.
    unsigned lineSize = verts->size();

    // cannot generate a line with less than 2 verts.
    if ( lineSize < 2 )
        return 0L;

    float width            = Distance(*_stroke.width(), *_stroke.widthUnits()).as(Units::METERS);
    float halfWidth        = 0.5f * width;
    float maxRoundingAngle = asin( _stroke.roundingRatio().get() );
    float minPixelSize     = _stroke.minPixels().getOrUse( 0.0f );
    bool  autoScale        = minPixelSize > 0.0f;

    osg::Geometry* geom  = new osg::Geometry();
    geom->setUseVertexBufferObjects( true );
    geom->setUseDisplayList( false );

    // Add the input verts to the geometry. This forms the "spine" of the
    // polygonized line. We need the spine so we can affect proper clamping,
    // texturing and vertex attribution.
    geom->setVertexArray( verts );

    // Set up the normals array
    if ( !normals )
    {
        normals = new osg::Vec3Array(verts->size());
        normals->assign( normals->size(), osg::Vec3(0,0,1) );
    }
    geom->setNormalArray( normals );
    geom->setNormalBinding( osg::Geometry::BIND_PER_VERTEX );

    // Set up the buffering vector attribute array.
    osg::Vec3Array* spine = 0L;
    if ( autoScale )
    {
        spine = new osg::Vec3Array( *verts );
        geom->setVertexAttribArray    ( ATTR_LOCATION, spine );
        geom->setVertexAttribBinding  ( ATTR_LOCATION, osg::Geometry::BIND_PER_VERTEX );
        geom->setVertexAttribNormalize( ATTR_LOCATION, false );
    }

    // initialize the texture coordinates.
    float spineLen = 0.0f;
    osg::Vec2Array* tverts = new osg::Vec2Array( lineSize );
    geom->setTexCoordArray( 0, tverts );
    (*tverts)[0].set( 0.5f, 0.0f );
    for( unsigned i=1; i<lineSize; ++i )
    {
        Segment   seg   ( (*verts)[i-1], (*verts)[i] );  // current segment.
        osg::Vec3 dir = seg.second - seg.first;
        spineLen += dir.length();
        (*tverts)[i].set( 0.5f, spineLen * 1.0f/width );
    }

    // triangulate the points into a mesh.
    std::vector<unsigned> ebo;

    // buffer the left side:
    unsigned  i;
    osg::Vec3 prevBufVert;
    osg::Vec3 prevBufVec;
    unsigned  prevBufVertPtr;
    unsigned  eboPtr = 0;
    osg::Vec3 prevDir;
    
    osg::Vec3 up(0,0,1);

    // iterate over both "sides" of the center line:
    int firstside = 0;
    int lastside  = twosided ? 1 : 0;
    
    const float RIGHT_SIDE = 1.0f;
    const float LEFT_SIDE = -1.0f;

    for( int ss=firstside; ss<=lastside; ++ss )
    {
        float side = ss == 0 ? RIGHT_SIDE : LEFT_SIDE;

        // iterate over each line segment.
        for( i=0; i<lineSize-1; ++i )
        {
            // establish the normal for this point, which will help us calculate a
            // 3D buffering vector.
            const osg::Vec3& normal = (*normals)[i];

            // calculate the directional vector of this segment.
            Segment   seg   ( (*verts)[i], (*verts)[i+1] );
            osg::Vec3 dir = seg.second - seg.first;
            dir.normalize();

            // the buffering vector is orthogonal to the direction vector and the normal;
            // flip it depending on the current side.
            osg::Vec3 bufVecUnit = (dir ^ up) * side;
            bufVecUnit.normalize();

            // scale the buffering vector to half the stroke width.
            osg::Vec3 bufVec = bufVecUnit * halfWidth;

            // calculate the starting buffered vertex
            osg::Vec3 bufVert = (*verts)[i] + bufVec;

            if ( i == 0 )
            {
                // first vert-- no corner to check, just make the buffered vert.
                verts->push_back( bufVert );
                prevBufVert = bufVert;
                prevBufVertPtr = verts->size() - 1;

                // first tex coord:
                // TODO: revisit. I believe we have them going x = [-1..1] instead of [0..1] -gw
                tverts->push_back( osg::Vec2f(1.0*side, (*tverts)[i].y()) );

                // first normal
                normals->push_back( (*normals)[i] );

                // buffering vector.
                if ( spine ) spine->push_back( (*verts)[i] );

                // render the front end-cap.
                if ( _stroke.lineCap() == Stroke::LINECAP_ROUND )
                {
                    float angle = osg::PI_2;
                    int steps = (int)ceil(angle/maxRoundingAngle);
                    float step = angle/(float)steps;
                    osg::Vec3 circlevec = verts->back() - (*verts)[i];

                    for( int j=1; j<=steps; ++j )
                    {
                        osg::Vec3 v;
                        float a = step * (float)j;
                        //rotate( circlevec, -(side)*a, (*normals)[i], v );
                        rotate( circlevec, -(side)*a, up, v );

                        verts->push_back( (*verts)[i] + v );
                        addTri( ebo, i, verts->size()-2, verts->size()-1, side );
                        tverts->push_back( osg::Vec2f(1.0*side, (*tverts)[i].y()) );
                        normals->push_back( (*normals)[i] );
                        if ( spine ) spine->push_back( (*verts)[i] );
                    }
                }
                else if ( _stroke.lineCap() == Stroke::LINECAP_SQUARE )
                {
                    float cornerWidth = sqrt(2.0*halfWidth*halfWidth);

                    verts->push_back( verts->back() - dir*halfWidth );
                    addTri( ebo, i, verts->size()-2, verts->size()-1, side );
                    tverts->push_back( osg::Vec2f(1.0*side, (*tverts)[i].y()) );
                    normals->push_back( normals->back() );
                    if ( spine ) spine->push_back( (*verts)[i] );

                    verts->push_back( (*verts)[i] - dir*halfWidth );
                    addTri( ebo, i, verts->size()-2, verts->size()-1, side );
                    tverts->push_back( osg::Vec2f(1.0*side, (*tverts)[i].y()) );
                    normals->push_back( (*normals)[i] );
                    if ( spine ) spine->push_back( (verts->back() - (*verts)[i]) * sqrt(2.0f) );
                }
            }
            else
            {
                // does the new segment turn create a reflex angle (>180deg)?
                osg::Vec3f cp = prevDir ^ dir;
                float z = cp.z();
                bool isOutside = side == LEFT_SIDE ? z <= 0.0 : z >= 0.0;
                bool isInside = !isOutside;

                // if this is an inside angle (or we're using mitered corners)
                // calculate the corner point by finding the convergance of the two
                // vectors enimating from the previous and next buffered points.
                if ( isInside || _stroke.lineJoin() == Stroke::LINEJOIN_MITRE )
                {
                    bool addedVertex = false;
                    {
                        osg::Vec3 nextBufVert = seg.second + bufVec;

                        OE_DEBUG 
                            << "\n"
                            << "point " << i << " : \n"
                            << "seg f: " << seg.first.x() << ", " << seg.first.y() << "\n"
                            << "seg s: " << seg.second.x() << ", " << seg.second.y() << "\n"
                            << "pnt 1: " << prevBufVert.x() << ", " << prevBufVert.y() << "\n"
                            << "ray 1: " << prevDir.x() << ", " << prevDir.y() << "\n"
                            << "pnt 2: " << nextBufVert.x() << ", " << nextBufVert.y() << "\n"
                            << "ray 2: " << -dir.x() << ", " << -dir.y() << "\n"
                            << "bufvec: " << bufVec.x() << ", " << bufVec.y() << "\n"
                            << "\n";

                        // find the 2D intersection of these two vectors. Check for the 
                        // special case of colinearity.
                        osg::Vec3 isect;

                        if ( interesctRays(prevBufVert, prevDir, nextBufVert, -dir, up, isect) )//(*normals)[i], isect) )
                        {
                            verts->push_back(isect);
                            addedVertex = true;
                        }
                    }
                    
                    if ( !addedVertex )
                    {
                        verts->push_back(bufVert);
                    }

                    // now that we have the current buffered point, build triangles
                    // for *previous* segment.
                    //if ( addedVertex )
                    addTris( ebo, i, prevBufVertPtr, verts->size()-1, side );
                    tverts->push_back( osg::Vec2f(1.0*side, (*tverts)[i].y()) );
                    normals->push_back( (*normals)[i] );

                    if ( spine ) spine->push_back( (*verts)[i] );
                }

                else if ( _stroke.lineJoin() == Stroke::LINEJOIN_ROUND )
                {
                    // for a rounded corner, first create the first rim point:
                    osg::Vec3 start = (*verts)[i] + prevBufVec;

                    verts->push_back( start );
                    addTris( ebo, i, prevBufVertPtr, verts->size()-1, side );
                    tverts->push_back( osg::Vec2f(1.0*side, (*tverts)[i].y()) );
                    normals->push_back( (*normals)[i] );
                    if ( spine ) spine->push_back( (*verts)[i] );

                    // insert the edge-rounding points:
                    float angle = acosf( (prevBufVec * bufVec)/(halfWidth*halfWidth) );
                    int steps = (int)ceil(angle/maxRoundingAngle);
                    float step = angle/(float)steps;
                    osg::Vec3 circlevec = start - (*verts)[i];
                    for( int j=1; j<=steps; ++j )
                    {
                        osg::Vec3 v;
                        float a = step * (float)j;
                        rotate( circlevec, side*a, up, v );
                        //rotate( circlevec, side*a, (*normals)[i], v );

                        verts->push_back( (*verts)[i] + v );
                        addTri( ebo, i, verts->size()-1, verts->size()-2, side );
                        tverts->push_back( osg::Vec2f(1.0*side, (*tverts)[i].y()) );
                        normals->push_back( (*normals)[i] );

                        if ( spine ) spine->push_back( (*verts)[i] );
                    }
                }

                // record these for the next segment.
                prevBufVert    = verts->back();
                prevBufVertPtr = verts->size() - 1;
            }

            // record these for the next segment.
            prevDir    = dir;
            prevBufVec = bufVec;
        }

        // record the final point data.
        verts->push_back( (*verts)[i] + prevBufVec );
        addTris( ebo, i, prevBufVertPtr, verts->size()-1, side );
        tverts->push_back( osg::Vec2f(1.0*side, (*tverts)[i].y()) );
        normals->push_back( (*normals)[i] );
        if ( spine ) spine->push_back( (*verts)[i] );

        if ( _stroke.lineCap() == Stroke::LINECAP_ROUND )
        {
            float angle = osg::PI_2;
            int steps = (int)ceil(angle/maxRoundingAngle);
            float step = angle/(float)steps;
            osg::Vec3 circlevec = verts->back() - (*verts)[i];

            // tessellate half of a rounded end camp:
            for( int j=1; j<=steps; ++j )
            {
                osg::Vec3 v;
                float a = step * (float)j;
                //rotate( circlevec, (side)*a, (*normals)[i], v );
                rotate( circlevec, (side)*a, up, v );
                verts->push_back( (*verts)[i] + v );
                addTri( ebo, i, verts->size()-1, verts->size()-2, side );
                tverts->push_back( osg::Vec2f(1.0*side, (*tverts)[i].y()) );
                normals->push_back( (*normals)[i] );
                if ( spine ) spine->push_back( (*verts)[i] );
            }
        }
        else if ( _stroke.lineCap() == Stroke::LINECAP_SQUARE )
        {
            float cornerWidth = sqrt(2.0*halfWidth*halfWidth);

            verts->push_back( verts->back() + prevDir*halfWidth );
            addTri( ebo, i, verts->size()-1, verts->size()-2, side );
            tverts->push_back( osg::Vec2f(1.0*side, (*tverts)[i].y()) );
            normals->push_back( normals->back() );
            if ( spine ) spine->push_back( (*verts)[i] );

            verts->push_back( (*verts)[i] + prevDir*halfWidth );
            addTri( ebo, i, verts->size()-1, verts->size()-2, side );
            tverts->push_back( osg::Vec2f(1.0*side, (*tverts)[i].y()) );
            normals->push_back( (*normals)[i] );
            if ( spine ) spine->push_back( (*verts)[i] );
        }
    }

    // copy the ebo into a primitive set of appropriate size:
    osg::DrawElements* primset =
        verts->size() > 0xFFFF ? (osg::DrawElements*)new osg::DrawElementsUInt  ( GL_TRIANGLES ) :
        verts->size() > 0xFF   ? (osg::DrawElements*)new osg::DrawElementsUShort( GL_TRIANGLES ) :
                                 (osg::DrawElements*)new osg::DrawElementsUByte ( GL_TRIANGLES );

    primset->reserveElements( ebo.size() );
    for(i=0; i<ebo.size(); ++i )
        primset->addElement( ebo[i] );
    geom->addPrimitiveSet( primset );

    // generate colors
    {
        osg::Vec4Array* colors = new osg::Vec4Array( verts->size() );
        colors->assign( colors->size(), _stroke.color() );
        geom->setColorArray( colors );
        geom->setColorBinding( osg::Geometry::BIND_PER_VERTEX );
    }
     
#if 0
    //TESTING
    osg::Image* image = osgDB::readImageFile("E:/data/textures/road.jpg");
    osg::Texture2D* tex = new osg::Texture2D(image);
    tex->setWrap( osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE );
    tex->setWrap( osg::Texture::WRAP_T, osg::Texture::REPEAT );
    geom->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex, 1);
#endif

    return geom;
}


#define SHADER_NAME "osgEarth::PolygonizeLinesAutoScale"

namespace
{
    struct PixelSizeVectorCullCallback : public osg::NodeCallback
    {
        PixelSizeVectorCullCallback(osg::StateSet* stateset)
        {
            _frameNumber = 0;
            _pixelSizeVectorUniform = new osg::Uniform(osg::Uniform::FLOAT_VEC4, "oe_PixelSizeVector");
            stateset->addUniform( _pixelSizeVectorUniform.get() );
        }

        void operator()(osg::Node* node, osg::NodeVisitor* nv)
        {
            osgUtil::CullVisitor* cv = Culling::asCullVisitor(nv);

            // temporary patch to prevent uniform overwrite -gw
            if ( nv->getFrameStamp() && (int)nv->getFrameStamp()->getFrameNumber() > _frameNumber )
            {
                _pixelSizeVectorUniform->set( cv->getCurrentCullingSet().getPixelSizeVector() );    
                _frameNumber = nv->getFrameStamp()->getFrameNumber();
            }

            traverse(node, nv);
        }

        osg::ref_ptr<osg::Uniform> _pixelSizeVectorUniform;
        int _frameNumber;        
    };

    class PixelScalingGeode : public osg::Geode
    {
    public:
        void traverse(osg::NodeVisitor& nv)
        {
            osgUtil::CullVisitor* cv = 0L;
            if (nv.getVisitorType() == nv.CULL_VISITOR &&
                (cv = Culling::asCullVisitor(nv)) != 0L &&
                cv->getCurrentCamera() )
            {
                osg::ref_ptr<osg::StateSet>& ss = _stateSets.get( cv->getCurrentCamera() );
                if ( !ss.valid() )
                    ss = new osg::StateSet();

                ss->getOrCreateUniform("oe_PixelSizeVector", osg::Uniform::FLOAT_VEC4)->set(
                    cv->getCurrentCullingSet().getPixelSizeVector() );
            }

            osg::Geode::traverse( nv );
        }

        PerObjectFastMap<osg::Camera*, osg::ref_ptr<osg::StateSet> > _stateSets;
    };
}


void
PolygonizeLinesOperator::installShaders(osg::Node* node) const
{
    if ( !node )
        return;

    float minPixels = _stroke.minPixels().getOrUse( 0.0f );
    if ( minPixels <= 0.0f )
        return;

    osg::StateSet* stateset = node->getOrCreateStateSet();

    VirtualProgram* vp = VirtualProgram::getOrCreate(stateset);

    // bail if already installed.
    if ( vp->getName().compare( SHADER_NAME ) == 0 )
        return;

    vp->setName( SHADER_NAME );

    const char* vs =
        "#version " GLSL_VERSION_STR "\n"
        GLSL_DEFAULT_PRECISION_FLOAT "\n"
        "attribute vec3 oe_polyline_center; \n"
        "uniform float oe_polyline_scale;  \n"
        "uniform float oe_polyline_min_pixels; \n"
        "uniform vec4 oe_PixelSizeVector; \n"

        "void oe_polyline_scalelines(inout vec4 vertex_model4) \n"
        "{ \n"
        "   const float epsilon = 0.0001; \n"

        "   vec4 center = vec4(oe_polyline_center, 1.0); \n"
        "   vec3 vector = vertex_model4.xyz - center.xyz; \n"
        
        "   float r = length(vector); \n"

        "   float activate  = step(epsilon, r*oe_polyline_min_pixels);\n"
        "   float pixelSize = max(epsilon, 2.0*abs(r/dot(center, oe_PixelSizeVector))); \n"
        "   float min_scale = max(oe_polyline_min_pixels/pixelSize, 1.0); \n"
        "   float scale     = mix(1.0, max(oe_polyline_scale, min_scale), activate); \n"

        "   vertex_model4.xyz = center.xyz + vector*scale; \n"
        "} \n";

    vp->setFunction( "oe_polyline_scalelines", vs, ShaderComp::LOCATION_VERTEX_MODEL, 0.5f );
    vp->addBindAttribLocation( "oe_polyline_center", ATTR_LOCATION );

    // add the default scaling uniform.
    // good way to test:
    //    osgearth_viewer earthfile --uniform oe_polyline_scale 1.0 10.0
    osg::Uniform* scaleU = new osg::Uniform(osg::Uniform::FLOAT, "oe_polyline_scale");
    scaleU->set( 1.0f );
    stateset->addUniform( scaleU, 1 );

    // the default "min pixels" uniform.
    osg::Uniform* minPixelsU = new osg::Uniform(osg::Uniform::FLOAT, "oe_polyline_min_pixels");
    minPixelsU->set( minPixels );
    stateset->addUniform( minPixelsU, 1 );

    // this will install and update the oe_PixelSizeVector uniform.
    node->addCullCallback( new PixelSizeVectorCullCallback(stateset) );
}


//------------------------------------------------------------------------


PolygonizeLinesFilter::PolygonizeLinesFilter(const Style& style) :
_style( style )
{
    //nop
}


osg::Node*
PolygonizeLinesFilter::push(FeatureList& input, FilterContext& cx)
{
    // compute the coordinate localization matrices.
    computeLocalizers( cx );

    // establish some things
    bool                    makeECEF   = false;
    const SpatialReference* featureSRS = 0L;
    const SpatialReference* mapSRS     = 0L;

    if ( cx.isGeoreferenced() )
    {
        makeECEF   = cx.getSession()->getMapInfo().isGeocentric();
        featureSRS = cx.extent()->getSRS();
        mapSRS     = cx.getSession()->getMapInfo().getProfile()->getSRS();
    }

    // The operator we'll use to make lines into polygons.
    const LineSymbol* line = _style.get<LineSymbol>();
    PolygonizeLinesOperator polygonize( line ? (*line->stroke()) : Stroke() );

    // Geode to hold all the geometries.
    osg::Geode* geode = new PixelScalingGeode(); //osg::Geode();

    // iterate over all features.
    for( FeatureList::iterator i = input.begin(); i != input.end(); ++i )
    {
        Feature* f = i->get();

        // iterate over all the feature's geometry parts. We will treat
        // them as lines strings.
        GeometryIterator parts( f->getGeometry(), false );
        while( parts.hasMore() )
        {
            Geometry* part = parts.next();

            // skip empty geometry
            if ( part->size() == 0 )
                continue;

            // transform the geometry into the target SRS and localize it about 
            // a local reference point.
            osg::Vec3Array* verts   = new osg::Vec3Array();
            osg::Vec3Array* normals = new osg::Vec3Array();
            transformAndLocalize( part->asVector(), featureSRS, verts, normals, mapSRS, _world2local, makeECEF );

            // turn the lines into polygons.
            osg::Geometry* geom = polygonize( verts, normals );

            // install.
            geode->addDrawable( geom );

            // record the geometry's primitive set(s) in the index:
            if ( cx.featureIndex() )
                cx.featureIndex()->tagDrawable( geom, f );
        }
    }

    // attempt to combine geometries for better performance
    MeshConsolidator::run( *geode );

    // GPU performance optimization:
    VertexCacheOptimizer vco;
    geode->accept( vco );

    // If we're auto-scaling, we need a shader
    polygonize.installShaders( geode );

    return delocalize( geode );
}
