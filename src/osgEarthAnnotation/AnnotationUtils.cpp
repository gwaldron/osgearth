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

#include <osgEarthAnnotation/AnnotationUtils>
#include <osgEarthSymbology/Color>
#include <osgEarthSymbology/MeshSubdivider>
#include <osgEarth/ThreadingUtils>
#include <osgEarth/Registry>
#include <osgText/Text>
#include <osg/Depth>
#include <osg/MatrixTransform>

using namespace osgEarth;
using namespace osgEarth::Annotation;

const std::string&
AnnotationUtils::PROGRAM_NAME()
{
  static std::string s = "osgEarthAnnotation::Program";
  return s;
}

const std::string&
AnnotationUtils::UNIFORM_HIGHLIGHT()
{
   static std::string s = "highlight";
   return s;
}


const std::string&
AnnotationUtils::UNIFORM_IS_TEXT()
{
  static std::string s = "is_text";
  return s;
}

const std::string&
AnnotationUtils::UNIFORM_FADE()
{
  static std::string s ="fade";
  return s;
}

osg::Drawable* 
AnnotationUtils::createTextDrawable(const std::string& text,
                                    const TextSymbol*  symbol,
                                    const osg::Vec3&   positionOffset )
                                    
{
    osgText::Text* t = new osgText::Text();
    osgText::String::Encoding text_encoding = osgText::String::ENCODING_UNDEFINED;
    if ( symbol && symbol->encoding().isSet() )
    {
        switch(symbol->encoding().value())
        {
        case TextSymbol::ENCODING_ASCII: text_encoding = osgText::String::ENCODING_ASCII; break;
        case TextSymbol::ENCODING_UTF8: text_encoding = osgText::String::ENCODING_UTF8; break;
        case TextSymbol::ENCODING_UTF16: text_encoding = osgText::String::ENCODING_UTF16; break;
        case TextSymbol::ENCODING_UTF32: text_encoding = osgText::String::ENCODING_UTF32; break;
        default: text_encoding = osgText::String::ENCODING_UNDEFINED; break;
        }
    }

    t->setText( text, text_encoding );

    if ( symbol && symbol->pixelOffset().isSet() )
    {
        t->setPosition( osg::Vec3(
            positionOffset.x() + symbol->pixelOffset()->x(),
            positionOffset.y() + symbol->pixelOffset()->y(),
            positionOffset.z() ) );
    }
    else
    {
        t->setPosition( positionOffset );
    }

    t->setAutoRotateToScreen( false );
    t->setCharacterSizeMode( osgText::Text::OBJECT_COORDS );
    t->setCharacterSize( symbol && symbol->size().isSet() ? *symbol->size() : 16.0f );
    t->setColor( symbol && symbol->fill().isSet() ? symbol->fill()->color() : Color::White );

    osgText::Font* font = 0L;
    if ( symbol && symbol->font().isSet() )
        font = osgText::readFontFile( *symbol->font() );
    if ( !font )
        font = Registry::instance()->getDefaultFont();
    if ( font )
        t->setFont( font );

    if ( symbol )
    {
        // they're the same enum.
        osgText::Text::AlignmentType at = (osgText::Text::AlignmentType)symbol->alignment().value();
        t->setAlignment( at );
    }

    if ( symbol && symbol->halo().isSet() )
    {
        t->setBackdropColor( symbol->halo()->color() );
        t->setBackdropType( osgText::Text::OUTLINE );
    }
    else if ( !symbol )
    {
        // if no symbol at all is provided, default to using a black halo.
        t->setBackdropColor( osg::Vec4(.3,.3,.3,1) );
        t->setBackdropType( osgText::Text::OUTLINE );
    }

    // this disables the default rendering bin set by osgText::Font. Necessary if we're
    // going to do decluttering at a higher level
    osg::StateSet* stateSet = t->getOrCreateStateSet();

    stateSet->setRenderBinToInherit();

    // add the static "isText=true" uniform; this is a hint for the annotation shaders
    // if they get installed.
    static osg::ref_ptr<osg::Uniform> s_isTextUniform = new osg::Uniform(osg::Uniform::BOOL, UNIFORM_IS_TEXT());
    s_isTextUniform->set( true );
    stateSet->addUniform( s_isTextUniform.get() );

    return t;
}

osg::Geometry*
AnnotationUtils::createImageGeometry(osg::Image*       image,
                                     const osg::Vec2s& pixelOffset,
                                     unsigned          textureUnit )
{
    if ( !image )
        return 0L;

    osg::Texture2D* texture = new osg::Texture2D();
    texture->setFilter(osg::Texture::MIN_FILTER,osg::Texture::LINEAR);
    texture->setFilter(osg::Texture::MAG_FILTER,osg::Texture::LINEAR);
    texture->setResizeNonPowerOfTwoHint(false);
    texture->setImage( image );

    // set up the decoration.
    osg::StateSet* dstate = new osg::StateSet;
    dstate->setMode(GL_CULL_FACE,osg::StateAttribute::OFF);
    dstate->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
    dstate->setMode(GL_BLEND, 1);
    dstate->setTextureAttributeAndModes(0, texture,osg::StateAttribute::ON);   

    // set up the geoset.
    osg::Geometry* geom = new osg::Geometry();
    geom->setStateSet(dstate);

    float x0 = (float)pixelOffset.x() - image->s()/2.0;
    float y0 = (float)pixelOffset.y() - image->t()/2.0;

    osg::Vec3Array* coords = new osg::Vec3Array(4);
    (*coords)[0].set( x0, y0, 0 );
    (*coords)[1].set( x0 + image->s(), y0, 0 );
    (*coords)[2].set( x0 + image->s(), y0 + image->t(), 0 );
    (*coords)[3].set( x0, y0 + image->t(), 0 );
    geom->setVertexArray(coords);

    osg::Vec2Array* tcoords = new osg::Vec2Array(4);
    (*tcoords)[0].set(0, 0);
    (*tcoords)[1].set(1, 0);
    (*tcoords)[2].set(1, 1);
    (*tcoords)[3].set(0, 1);
    geom->setTexCoordArray(textureUnit,tcoords);

    osg::Vec4Array* colors = new osg::Vec4Array(1);
    (*colors)[0].set(1.0f,1.0f,1.0,1.0f);
    geom->setColorArray(colors);
    geom->setColorBinding(osg::Geometry::BIND_OVERALL);

    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS,0,4));

    // add the static "isText=true" uniform; this is a hint for the annotation shaders
    // if they get installed.
    static osg::ref_ptr<osg::Uniform> s_isNotTextUniform = new osg::Uniform(osg::Uniform::BOOL, UNIFORM_IS_TEXT());
    s_isNotTextUniform->set( false );
    dstate->addUniform( s_isNotTextUniform.get() );

    return geom;
}

osg::Uniform*
AnnotationUtils::createFadeUniform()
{
    osg::Uniform* u = new osg::Uniform(osg::Uniform::FLOAT, UNIFORM_FADE());
    u->set( 1.0f );
    return u;
}

osg::Uniform*
AnnotationUtils::createHighlightUniform()
{
    osg::Uniform* u = new osg::Uniform(osg::Uniform::BOOL, UNIFORM_HIGHLIGHT());
    u->set( false );
    return u;
}

osg::Program*
AnnotationUtils::getAnnotationProgram()
{
    static Threading::Mutex           s_mutex;
    static osg::ref_ptr<osg::Program> s_program;

    if ( !s_program.valid() )
    {
        Threading::ScopedMutexLock lock(s_mutex);
        if ( !s_program.valid() )
        {
            std::string frag_source = Stringify() <<
                "uniform float     " << UNIFORM_FADE()      << "; \n"
                "uniform bool      " << UNIFORM_IS_TEXT()   << "; \n"
                "uniform bool      " << UNIFORM_HIGHLIGHT() << "; \n"
                "uniform sampler2D tex0; \n"
                "void main() { \n"
                "    vec4 color; \n"
                "    if (" << UNIFORM_IS_TEXT() << ") { \n"
                "        float alpha = texture2D(tex0,gl_TexCoord[0].st).a; \n"
                "        color = vec4( gl_Color.rgb, gl_Color.a * alpha * " << UNIFORM_FADE() << "); \n"
                "    } \n"
                "    else { \n"
                "        color = gl_Color * texture2D(tex0,gl_TexCoord[0].st) * vec4(1,1,1," << UNIFORM_FADE() << "); \n"
                "    } \n"
                "    if (" << UNIFORM_HIGHLIGHT() << ") { \n"
                "        color = vec4(color.r*1.5, color.g*0.5, color.b*0.25, color.a); \n"
                "    } \n"
                "    gl_FragColor = color; \n"
                "} \n";

            s_program = new osg::Program();
            s_program->setName( PROGRAM_NAME() );
            s_program->addShader( new osg::Shader(osg::Shader::FRAGMENT, frag_source) );
        }
    }
    return s_program.get();
}

//-------------------------------------------------------------------------

// This is identical to osg::AutoTransform::accept, except that we (a) removed
// code that's not used by OrthoNode, and (b) took out the call to
// Transform::accept since we don't want to traverse the child graph from
// this call.
void
AnnotationUtils::OrthoNodeAutoTransform::acceptCullNoTraverse( osg::CullStack* cs )
{
    osg::Viewport::value_type width = _previousWidth;
    osg::Viewport::value_type height = _previousHeight;

    osg::Viewport* viewport = cs->getViewport();
    if (viewport)
    {
        width = viewport->width();
        height = viewport->height();
    }

    osg::Vec3d eyePoint = cs->getEyeLocal(); 
    osg::Vec3d localUp = cs->getUpLocal(); 
    osg::Vec3d position = getPosition();

    const osg::Matrix& projection = *(cs->getProjectionMatrix());

    bool doUpdate = _firstTimeToInitEyePoint;
    if (!_firstTimeToInitEyePoint)
    {
        osg::Vec3d dv = _previousEyePoint-eyePoint;
        if (dv.length2()>getAutoUpdateEyeMovementTolerance()*(eyePoint-getPosition()).length2())
        {
            doUpdate = true;
        }
        osg::Vec3d dupv = _previousLocalUp-localUp;
        // rotating the camera only affects ROTATE_TO_*
        if (_autoRotateMode &&
            dupv.length2()>getAutoUpdateEyeMovementTolerance())
        {
            doUpdate = true;
        }
        else if (width!=_previousWidth || height!=_previousHeight)
        {
            doUpdate = true;
        }
        else if (projection != _previousProjection) 
        {
            doUpdate = true;
        }                
        else if (position != _previousPosition) 
        { 
            doUpdate = true; 
        } 
    }
    _firstTimeToInitEyePoint = false;

    if (doUpdate)
    {            
        if (getAutoScaleToScreen())
        {
            double size = 1.0/cs->pixelSize(getPosition(),0.48f);

            if (_autoScaleTransitionWidthRatio>0.0)
            {
                if (_minimumScale>0.0)
                {
                    double j = _minimumScale;
                    double i = (_maximumScale<DBL_MAX) ? 
                        _minimumScale+(_maximumScale-_minimumScale)*_autoScaleTransitionWidthRatio :
                    _minimumScale*(1.0+_autoScaleTransitionWidthRatio);
                    double c = 1.0/(4.0*(i-j));
                    double b = 1.0 - 2.0*c*i;
                    double a = j + b*b / (4.0*c);
                    double k = -b / (2.0*c);

                    if (size<k) size = _minimumScale;
                    else if (size<i) size = a + b*size + c*(size*size);
                }

                if (_maximumScale<DBL_MAX)
                {
                    double n = _maximumScale;
                    double m = (_minimumScale>0.0) ?
                        _maximumScale+(_minimumScale-_maximumScale)*_autoScaleTransitionWidthRatio :
                    _maximumScale*(1.0-_autoScaleTransitionWidthRatio);
                    double c = 1.0 / (4.0*(m-n));
                    double b = 1.0 - 2.0*c*m;
                    double a = n + b*b/(4.0*c);
                    double p = -b / (2.0*c);

                    if (size>p) size = _maximumScale;
                    else if (size>m) size = a + b*size + c*(size*size);
                }        
            }

            setScale(size);
        }

        if (_autoRotateMode==ROTATE_TO_SCREEN)
        {
            osg::Vec3d translation;
            osg::Quat rotation;
            osg::Vec3d scale;
            osg::Quat so;

            cs->getModelViewMatrix()->decompose( translation, rotation, scale, so );

            setRotation(rotation.inverse());
        }
        // GW: removed other unused auto-rotate modes

        _previousEyePoint = eyePoint;
        _previousLocalUp = localUp;
        _previousWidth = width;
        _previousHeight = height;
        _previousProjection = projection;
        _previousPosition = position;

        _matrixDirty = true;
    }

    // GW: the stock AutoTransform calls Transform::accept here; we do NOT
}

osg::Node* 
AnnotationUtils::createGeodesicSphere( float r, const osg::Vec4& color )
{
    osg::Geometry* geom = new osg::Geometry();

    osg::Vec3Array* v = new osg::Vec3Array();
    v->reserve(6);
    v->push_back( osg::Vec3(0,0,r) ); // top
    v->push_back( osg::Vec3(0,0,-r) ); // bottom
    v->push_back( osg::Vec3(-r,0,0) ); // left
    v->push_back( osg::Vec3(r,0,0) ); // right
    v->push_back( osg::Vec3(0,r,0) ); // back
    v->push_back( osg::Vec3(0,-r,0) ); // front
    geom->setVertexArray(v);

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

    MeshSubdivider ms;
    ms.run( *geom, osg::DegreesToRadians(15.0f), GEOINTERP_GREAT_CIRCLE );

    osg::Vec4Array* c = new osg::Vec4Array(1);
    (*c)[0] = color;
    geom->setColorArray( c );
    geom->setColorBinding( osg::Geometry::BIND_OVERALL );

    osg::Geode* geode = new osg::Geode();
    geode->addDrawable( geom );

    return geode;
}

osg::Node* 
AnnotationUtils::createFullScreenQuad( const osg::Vec4& color )
{
    osg::Geometry* geom = new osg::Geometry();

    osg::Vec3Array* v = new osg::Vec3Array();
    v->reserve(4);
    v->push_back( osg::Vec3(0,0,0) );
    v->push_back( osg::Vec3(1,0,0) );
    v->push_back( osg::Vec3(1,1,0) );
    v->push_back( osg::Vec3(0,1,0) );
    geom->setVertexArray(v);

    osg::DrawElementsUByte* b = new osg::DrawElementsUByte(GL_TRIANGLES);
    b->reserve(6);
    b->push_back(0); b->push_back(1); b->push_back(2);
    b->push_back(2); b->push_back(3); b->push_back(0);
    geom->addPrimitiveSet( b );

    osg::Vec4Array* c = new osg::Vec4Array(1);
    (*c)[0] = color;
    geom->setColorArray( c );
    geom->setColorBinding( osg::Geometry::BIND_OVERALL );

    osg::Geode* geode = new osg::Geode();
    geode->addDrawable( geom );

    osg::StateSet* s = geom->getOrCreateStateSet();
    s->setMode(GL_LIGHTING,0);
    s->setMode(GL_BLEND,1);
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

osg::Drawable*
AnnotationUtils::create2DQuad( const osg::BoundingBox& box, float padding, const osg::Vec4& color )
{
    osg::Geometry* geom = new osg::Geometry();

    osg::Vec3Array* v = new osg::Vec3Array();
    v->reserve(4);
    v->push_back( osg::Vec3(box.xMin()-padding, box.yMin()-padding, 0) );
    v->push_back( osg::Vec3(box.xMax()+padding, box.yMin()-padding, 0) );
    v->push_back( osg::Vec3(box.xMax()+padding, box.yMax()+padding, 0) );
    v->push_back( osg::Vec3(box.xMin()-padding, box.yMax()+padding, 0) );
    geom->setVertexArray(v);

    osg::DrawElementsUByte* b = new osg::DrawElementsUByte(GL_TRIANGLES);
    b->reserve(6);
    b->push_back(0); b->push_back(1); b->push_back(2);
    b->push_back(2); b->push_back(3); b->push_back(0);
    geom->addPrimitiveSet( b );

    osg::Vec4Array* c = new osg::Vec4Array(1);
    (*c)[0] = color;
    geom->setColorArray( c );
    geom->setColorBinding( osg::Geometry::BIND_OVERALL );

    return geom;
}

osg::Drawable*
AnnotationUtils::create2DOutline( const osg::BoundingBox& box, float padding, const osg::Vec4& color )
{
    osg::Geometry* geom = new osg::Geometry();

    osg::Vec3Array* v = new osg::Vec3Array();
    v->reserve(4);
    v->push_back( osg::Vec3(box.xMin()-padding, box.yMin()-padding, 0) );
    v->push_back( osg::Vec3(box.xMax()+padding, box.yMin()-padding, 0) );
    v->push_back( osg::Vec3(box.xMax()+padding, box.yMax()+padding, 0) );
    v->push_back( osg::Vec3(box.xMin()-padding, box.yMax()+padding, 0) );
    geom->setVertexArray(v);

    osg::DrawElementsUByte* b = new osg::DrawElementsUByte(GL_LINE_LOOP);
    b->reserve(4);
    b->push_back(0); b->push_back(1); b->push_back(2); b->push_back(3);
    geom->addPrimitiveSet( b );

    osg::Vec4Array* c = new osg::Vec4Array(1);
    (*c)[0] = color;
    geom->setColorArray( c );
    geom->setColorBinding( osg::Geometry::BIND_OVERALL );

    static osg::ref_ptr<osg::Uniform> s_isNotTextUniform = new osg::Uniform(osg::Uniform::BOOL, UNIFORM_IS_TEXT());
    s_isNotTextUniform->set( true );
    geom->getOrCreateStateSet()->addUniform( s_isNotTextUniform.get() );

    return geom;
}
