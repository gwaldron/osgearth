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

#include <osgEarthAnnotation/AnnotationUtils>
#include <osgEarthSymbology/Color>
#include <osgEarthSymbology/MeshSubdivider>
#include <osgEarthFeatures/TextSymbolizer>
#include <osgEarth/ThreadingUtils>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/CullingUtils>
#include <osgEarth/DrapeableNode>
#include <osgEarth/ClampableNode>
#include <osgEarth/GLUtils>
#include <osgEarth/Text>

#include <osgText/Text>
#include <osg/Depth>
#include <osg/BlendFunc>
#include <osg/CullFace>
#include <osg/MatrixTransform>
#include <osg/LightModel>
#include <osg/Projection>

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Features;

osgText::String::Encoding
AnnotationUtils::convertTextSymbolEncoding (const TextSymbol::Encoding encoding) {
    osgText::String::Encoding text_encoding = osgText::String::ENCODING_UNDEFINED;

    switch(encoding)
    {
    case TextSymbol::ENCODING_ASCII: text_encoding = osgText::String::ENCODING_ASCII; break;
    case TextSymbol::ENCODING_UTF8: text_encoding = osgText::String::ENCODING_UTF8; break;
    case TextSymbol::ENCODING_UTF16: text_encoding = osgText::String::ENCODING_UTF16; break;
    case TextSymbol::ENCODING_UTF32: text_encoding = osgText::String::ENCODING_UTF32; break;
    default: text_encoding = osgText::String::ENCODING_UNDEFINED; break;
    }

    return text_encoding;
}

namespace
{
    static int nextPowerOf2(int x)
    {
        --x;
        x |= x >> 1;
        x |= x >> 2;
        x |= x >> 4;
        x |= x >> 8;
        x |= x >> 16;
        return x+1;
    }
}

osgText::Text*
AnnotationUtils::createTextDrawable(const std::string& text,
                                    const TextSymbol*  symbol,
                                    const osg::BoundingBox& box)
{
    osgText::Text* drawable = new osgEarth::Text();

    osgText::String::Encoding text_encoding = osgText::String::ENCODING_UNDEFINED;
    if ( symbol && symbol->encoding().isSet() )
    {
        text_encoding = convertTextSymbolEncoding(symbol->encoding().value());
    }

    drawable->setText( text, text_encoding );

    TextSymbolizer symbolizer(symbol);
    symbolizer.apply(drawable, 0L, 0L, &box);

    // osgText::Text turns on depth writing by default, even if you turned it off.
    drawable->setEnableDepthWrites( false );
    
    return drawable;
}

osg::Geometry*
AnnotationUtils::createImageGeometry(osg::Image*       image,
                                     const osg::Vec2s& pixelOffset,
                                     unsigned          textureUnit,
                                     double            heading,
                                     double            scale)
{
    if ( !image )
        return 0L;

    osg::Texture2D* texture = new osg::Texture2D();
    texture->setFilter(osg::Texture::MIN_FILTER,osg::Texture::LINEAR_MIPMAP_LINEAR);
    texture->setFilter(osg::Texture::MAG_FILTER,osg::Texture::LINEAR);
    texture->setResizeNonPowerOfTwoHint(false);
    texture->setImage( image );

    // set up the decoration.
    osg::StateSet* dstate = new osg::StateSet;
    dstate->setDataVariance(osg::Object::DYNAMIC);
    dstate->setMode(GL_CULL_FACE,osg::StateAttribute::OFF);
    GLUtils::setLighting(dstate, osg::StateAttribute::OFF);
    dstate->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON);

    // set up the geoset.
    osg::Geometry* geom = new osg::Geometry();
    geom->setUseVertexBufferObjects(true);
    geom->setStateSet(dstate);

    float s = Registry::instance()->getDevicePixelRatio() * scale * image->s();
    float t = Registry::instance()->getDevicePixelRatio() * scale * image->t();

    float x0 = (float)pixelOffset.x() - s/2.0;
    float y0 = (float)pixelOffset.y() - t/2.0;

    osg::Vec3Array* verts = new osg::Vec3Array(4);
    (*verts)[0].set( x0,     y0,     0 );
    (*verts)[1].set( x0 + s, y0,     0 );
    (*verts)[2].set( x0 + s, y0 + t, 0 );
    (*verts)[3].set( x0,     y0 + t, 0 );

    if (heading != 0.0)
    {
        osg::Matrixd rot;
        rot.makeRotate( heading, 0.0, 0.0, 1.0);
        for (unsigned int i = 0; i < 4; i++)
        {
            (*verts)[i] = rot * (*verts)[i];
        }
    }
    geom->setVertexArray(verts);

    bool flip = image->getOrigin() == osg::Image::TOP_LEFT;
    
    osg::Vec2Array* tcoords = new osg::Vec2Array(4);    
    (*tcoords)[0].set(0.0, flip? 1.0: 0.0);
    (*tcoords)[1].set(1.0, flip? 1.0: 0.0);
    (*tcoords)[2].set(1.0, flip? 0.0: 1.0);
    (*tcoords)[3].set(0.0, flip? 0.0: 1.0);
    geom->setTexCoordArray(textureUnit,tcoords);

    osg::Vec4Array* colors = new osg::Vec4Array(osg::Array::BIND_OVERALL, 1);
    (*colors)[0].set(1.0f,1.0f,1.0,1.0f);
    geom->setColorArray(colors);

    GLushort indices[] = {0,1,2,0,2,3};
    geom->addPrimitiveSet( new osg::DrawElementsUShort( GL_TRIANGLES, 6, indices ) );

    return geom;
}

osg::Node*
AnnotationUtils::createSphere( float r, const osg::Vec4& color, float maxAngle )
{
    osg::Geometry* geom = new osg::Geometry();
    geom->setUseVertexBufferObjects(true);

    osg::Vec3Array* v = new osg::Vec3Array();
    v->reserve(6);
    v->push_back( osg::Vec3(0,0,r) ); // top
    v->push_back( osg::Vec3(0,0,-r) ); // bottom
    v->push_back( osg::Vec3(-r,0,0) ); // left
    v->push_back( osg::Vec3(r,0,0) ); // right
    v->push_back( osg::Vec3(0,r,0) ); // back
    v->push_back( osg::Vec3(0,-r,0) ); // front
    geom->setVertexArray(v);
    if ( v->getVertexBufferObject() )
       v->getVertexBufferObject()->setUsage(GL_STATIC_DRAW_ARB);

    osg::DrawElementsUByte* b = new osg::DrawElementsUByte(GL_TRIANGLES);
    b->reserve(24);
    b->push_back(0); b->push_back(3); b->push_back(4);
    b->push_back(0); b->push_back(4); b->push_back(2);
    b->push_back(0); b->push_back(2); b->push_back(5);
    b->push_back(0); b->push_back(5); b->push_back(3);
    b->push_back(1); b->push_back(3); b->push_back(5);
    b->push_back(1); b->push_back(4); b->push_back(3);
    b->push_back(1); b->push_back(2); b->push_back(4);
    b->push_back(1); b->push_back(5); b->push_back(2);
    geom->addPrimitiveSet( b );

    osg::Vec3Array* n = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX);
    n->reserve(6);
    n->push_back( osg::Vec3( 0, 0, 1) );
    n->push_back( osg::Vec3( 0, 0,-1) );
    n->push_back( osg::Vec3(-1, 0, 0) );
    n->push_back( osg::Vec3( 1, 0, 0) );
    n->push_back( osg::Vec3( 0, 1, 0) );
    n->push_back( osg::Vec3( 0,-1, 0) );
    geom->setNormalArray(n);

    MeshSubdivider ms;
    ms.run( *geom, osg::DegreesToRadians(maxAngle), GEOINTERP_GREAT_CIRCLE );

    osg::Vec4Array* c = new osg::Vec4Array(osg::Array::BIND_OVERALL, 1);
    (*c)[0] = color;
    geom->setColorArray( c );

    osg::Geode* geode = new osg::Geode();
    geode->addDrawable( geom );

    // need 2-pass alpha so you can view it properly from below.
    if ( color.a() < 1.0f )
      return installTwoPassAlpha( geode );
    else
      return geode;
}

osg::Node*
AnnotationUtils::createHemisphere( float r, const osg::Vec4& color, float maxAngle )
{
    osg::Geometry* geom = new osg::Geometry();
    geom->setUseVertexBufferObjects(true);

    osg::Vec3Array* v = new osg::Vec3Array();
    v->reserve(5);
    v->push_back( osg::Vec3(0,0,r) ); // top
    v->push_back( osg::Vec3(-r,0,0) ); // left
    v->push_back( osg::Vec3(r,0,0) ); // right
    v->push_back( osg::Vec3(0,r,0) ); // back
    v->push_back( osg::Vec3(0,-r,0) ); // front
    geom->setVertexArray(v);
    if ( v->getVertexBufferObject() )
       v->getVertexBufferObject()->setUsage(GL_STATIC_DRAW_ARB);

    osg::DrawElementsUByte* b = new osg::DrawElementsUByte(GL_TRIANGLES);
    b->reserve(12);
    b->push_back(0); b->push_back(2); b->push_back(3);
    b->push_back(0); b->push_back(3); b->push_back(1);
    b->push_back(0); b->push_back(1); b->push_back(4);
    b->push_back(0); b->push_back(4); b->push_back(2);
    geom->addPrimitiveSet( b );

    osg::Vec3Array* n = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX);
    n->reserve(5);
    n->push_back( osg::Vec3(0,0,1) );
    n->push_back( osg::Vec3(-1,0,0) );
    n->push_back( osg::Vec3(1,0,0) );
    n->push_back( osg::Vec3(0,1,0) );
    n->push_back( osg::Vec3(0,-1,0) );
    geom->setNormalArray(n);

    MeshSubdivider ms;
    ms.run( *geom, osg::DegreesToRadians(maxAngle), GEOINTERP_GREAT_CIRCLE );

    osg::Vec4Array* c = new osg::Vec4Array(osg::Array::BIND_OVERALL, 1);
    (*c)[0] = color;
    geom->setColorArray( c );

    osg::Geode* geode = new osg::Geode();
    geode->addDrawable( geom );

    // need 2-pass alpha so you can view it properly from below.
    if ( color.a() < 1.0f )
      return installTwoPassAlpha( geode );
    else
      return geode;
}

// constructs an ellipsoidal mesh
osg::Geometry*
AnnotationUtils::createEllipsoidGeometry(float xRadius,
                                         float yRadius,
                                         float zRadius,
                                         const osg::Vec4f& color,
                                         float maxAngle,
                                         float minLat,
                                         float maxLat,
                                         float minLon,
                                         float maxLon)
{
    osg::Geometry* geom = new osg::Geometry();
    geom->setUseVertexBufferObjects(true);

    float latSpan = maxLat - minLat;
    float lonSpan = maxLon - minLon;
    float aspectRatio = lonSpan/latSpan;

    int latSegments = osg::maximum( 6, (int)ceil(latSpan / maxAngle) );
    int lonSegments = osg::maximum( 3, (int)ceil(latSegments * aspectRatio) );

    float segmentSize = latSpan/latSegments; // degrees

    osg::Vec3Array* verts = new osg::Vec3Array();
    verts->reserve( latSegments * lonSegments );

#if 0
    bool genTexCoords = false; // TODO: optional
    osg::Vec2Array* texCoords = 0;
    if (genTexCoords)
    {
        texCoords = new osg::Vec2Array();
        texCoords->reserve( latSegments * lonSegments );
        geom->setTexCoordArray( 0, texCoords );
    }
#endif

    osg::Vec3Array* normals = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX);
    normals->reserve( latSegments * lonSegments );
    geom->setNormalArray( normals );

    osg::DrawElementsUShort* el = new osg::DrawElementsUShort( GL_TRIANGLES );
    el->reserve( latSegments * lonSegments * 6 );

    for( int y = 0; y <= latSegments; ++y )
    {
        float lat = minLat + segmentSize * (float)y;
        for( int x = 0; x < lonSegments; ++x )
        {
            float lon = minLon + segmentSize * (float)x;

            float u = osg::DegreesToRadians( lon );
            float v = osg::DegreesToRadians( lat );
            float cos_u = cosf(u);
            float sin_u = sinf(u);
            float cos_v = cosf(v);
            float sin_v = sinf(v);

            verts->push_back(osg::Vec3(
                xRadius * cos_u * cos_v,
                yRadius * sin_u * cos_v,
                zRadius * sin_v ));

#if 0
            if (genTexCoords)
            {
                double s = (lon + 180) / 360.0;
                double t = (lat + 90.0) / 180.0;
                texCoords->push_back( osg::Vec2(s, t ) );
            }
#endif

            normals->push_back( verts->back() );
            normals->back().normalize();

            if ( y < latSegments )
            {
                int x_plus_1 = x < lonSegments-1 ? x+1 : 0;
                int y_plus_1 = y+1;
                el->push_back( y*lonSegments + x );
                el->push_back( y*lonSegments + x_plus_1 );
                el->push_back( y_plus_1*lonSegments + x );
                el->push_back( y*lonSegments + x_plus_1 );
                el->push_back( y_plus_1*lonSegments + x_plus_1 );
                el->push_back( y_plus_1*lonSegments + x );
            }
        }
    }

    osg::Vec4Array* c = new osg::Vec4Array(osg::Array::BIND_OVERALL, 1);
    (*c)[0] = color;
    geom->setColorArray( c );

    geom->setVertexArray( verts );
    geom->addPrimitiveSet( el );

    return geom;
}

osg::Node*
AnnotationUtils::createEllipsoid(float xRadius,
                                 float yRadius,
                                 float zRadius,
                                 const osg::Vec4f& color,
                                 float maxAngle,
                                 float minLat,
                                 float maxLat,
                                 float minLon,
                                 float maxLon)
{
    osg::Geode* geode = new osg::Geode();
    geode->addDrawable( createEllipsoidGeometry(xRadius, yRadius, zRadius, color, maxAngle, minLat, maxLat, minLon, maxLon) );

    if ( color.a() < 1.0f )
    {
        geode->getOrCreateStateSet()->setRenderingHint( osg::StateSet::TRANSPARENT_BIN );
    }

    bool solid = (maxLat-minLat >= 180.0f && maxLon-minLon >= 360.0f);

    if ( solid )
    {
        geode->getOrCreateStateSet()->setAttributeAndModes( new osg::CullFace(osg::CullFace::BACK), 1 );
    }

    else if ( color.a() < 1.0f )
    {
        //geode->getOrCreateStateSet()->setAttributeAndModes( new osg::CullFace(), 0 );
        return installTwoPassAlpha(geode);
    }

    return geode;
}

osg::Node*
AnnotationUtils::createFullScreenQuad( const osg::Vec4& color )
{
    osg::Geometry* geom = new osg::Geometry();
    geom->setUseVertexBufferObjects(true);

    osg::Vec3Array* v = new osg::Vec3Array();
    v->reserve(4);
    v->push_back( osg::Vec3(0,0,0) );
    v->push_back( osg::Vec3(1,0,0) );
    v->push_back( osg::Vec3(1,1,0) );
    v->push_back( osg::Vec3(0,1,0) );
    geom->setVertexArray(v);
    if ( v->getVertexBufferObject() )
        v->getVertexBufferObject()->setUsage(GL_STATIC_DRAW_ARB);

    osg::DrawElementsUByte* b = new osg::DrawElementsUByte(GL_TRIANGLES);
    b->reserve(6);
    b->push_back(0); b->push_back(1); b->push_back(2);
    b->push_back(2); b->push_back(3); b->push_back(0);
    geom->addPrimitiveSet( b );

    osg::Vec4Array* c = new osg::Vec4Array(osg::Array::BIND_OVERALL, 1);
    (*c)[0] = color;
    geom->setColorArray( c );

    osg::Geode* geode = new osg::Geode();
    geode->addDrawable( geom );

    osg::StateSet* s = geom->getOrCreateStateSet();
    GLUtils::setLighting(s, 0);
    s->setMode(GL_DEPTH_TEST,0);
    s->setMode(GL_CULL_FACE,0);
    s->setAttributeAndModes( new osg::Depth(osg::Depth::ALWAYS, 0, 1, false), 1 );

    osg::MatrixTransform* xform = new osg::MatrixTransform( osg::Matrix::identity() );
    xform->setReferenceFrame( osg::Transform::ABSOLUTE_RF );
    xform->addChild( geode );

    osg::Projection* proj = new osg::Projection( osg::Matrix::ortho(0,1,0,1,0,-1) );
    proj->addChild( xform );

    return proj;
}

osg::Node*
AnnotationUtils::installTwoPassAlpha(osg::Node* node)
{
  // first, get the whole thing under a depth-sorted bin:
  osg::Group* g1 = new osg::Group();
  g1->getOrCreateStateSet()->setRenderingHint( osg::StateSet::TRANSPARENT_BIN );
  g1->getOrCreateStateSet()->setAttributeAndModes( new osg::BlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA), 1);

#ifdef OSG_GL_FIXED_FUNCTION_AVAILABLE
  // for semi-transpareny items, we want the lighting to "shine through"
  osg::LightModel* lm = new osg::LightModel();
  lm->setTwoSided( true );
  g1->getOrCreateStateSet()->setAttributeAndModes( lm );
#endif

  // next start a traversal order bin so we draw in the proper order:
  osg::Group* g2 = new osg::Group();
  g2->getOrCreateStateSet()->setRenderBinDetails(0, "TraversalOrderBin");
  g1->addChild( g2 );

  // next, create a group for the first pass (backfaces only):
  osg::Group* backPass = new osg::Group();
  backPass->getOrCreateStateSet()->setAttributeAndModes( new osg::CullFace(osg::CullFace::FRONT), 1 );
  backPass->getOrCreateStateSet()->setAttributeAndModes( new osg::Depth(osg::Depth::LEQUAL,0,1,false), 1);
  g2->addChild( backPass );

  // and a group for the front-face pass:
  osg::Group* frontPass = new osg::Group();
  frontPass->getOrCreateStateSet()->setAttributeAndModes( new osg::CullFace(osg::CullFace::BACK), 1 );
  g2->addChild( frontPass );

  // finally, attach the geometry to both passes.
  backPass->addChild( node );
  frontPass->addChild( node );

  return g1;
}


bool
AnnotationUtils::styleRequiresAlphaBlending( const Style& style )
{
    if (style.has<PolygonSymbol>() &&
        style.get<PolygonSymbol>()->fill().isSet() &&
        style.get<PolygonSymbol>()->fill()->color().a() < 1.0)
    {
        return true;
    }

    if (style.has<LineSymbol>() &&
        style.get<LineSymbol>()->stroke().isSet() &&
        style.get<LineSymbol>()->stroke()->color().a() < 1.0 )
    {
        return true;
    }

    if (style.has<PointSymbol>() &&
        style.get<PointSymbol>()->fill().isSet() &&
        style.get<PointSymbol>()->fill()->color().a() < 1.0 )
    {
        return true;
    }

    return false;
}


void
AnnotationUtils::getAltitudePolicy(const Style& style, AltitudePolicy& out)
{
    out.sceneClamping = false;
    out.gpuClamping   = false;
    out.draping       = false;

    // conditions where clamping is not yet compatible
    bool compatible =
        !style.has<ExtrusionSymbol>();      // backwards-compability

    if ( compatible )
    {
        const AltitudeSymbol* alt = style.get<AltitudeSymbol>();
        if ( alt )
        {
            if (alt->clamping() == AltitudeSymbol::CLAMP_TO_TERRAIN ||
                alt->clamping() == AltitudeSymbol::CLAMP_RELATIVE_TO_TERRAIN )
            {
                out.sceneClamping = alt->technique() == AltitudeSymbol::TECHNIQUE_SCENE;
                out.gpuClamping   = alt->technique() == AltitudeSymbol::TECHNIQUE_GPU;
                out.draping       = alt->technique() == AltitudeSymbol::TECHNIQUE_DRAPE;

                // for instance/markers, GPU clamping falls back on SCENE clamping.
                if (out.gpuClamping && style.has<InstanceSymbol>())
                {
                    out.gpuClamping   = false;
                    out.sceneClamping = true;
                }
            }
        }
    }
}

osg::Node*
AnnotationUtils::installOverlayParent(osg::Node* node, const Style& style)
{
    AnnotationUtils::AltitudePolicy ap;

    AnnotationUtils::getAltitudePolicy( style, ap );

    // Draped (projected) geometry
    if ( ap.draping )
    {
        DrapeableNode* drapable = new DrapeableNode();
        drapable->addChild( node );
        node = drapable;
    }

    // gw - not sure whether is makes sense to support this for LocalizedNode
    // GPU-clamped geometry
    else if ( ap.gpuClamping )
    {
        ClampableNode* clampable = new ClampableNode();
        clampable->addChild( node );
        node = clampable;
    }

    return node;
}

