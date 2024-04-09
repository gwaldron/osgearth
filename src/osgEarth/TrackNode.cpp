/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2020 Pelican Mapping
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

#include <osgEarth/TrackNode>
#include <osgEarth/AnnotationUtils>
#include <osgEarth/MapNode>
#include <osgEarth/Registry>
#include <osgEarth/ShaderGenerator>
#include <osgEarth/ScreenSpaceLayout>
#include <osgEarth/NodeUtils>
#include <osgEarth/Lighting>
#include <osg/Depth>
#include <osgText/Text>

#define LC "[TrackNode] "

using namespace osgEarth;

//------------------------------------------------------------------------

namespace
{
    const char* iconVS =
        "out vec2 oe_TrackNode_texcoord; \n"
        "void oe_TrackNode_icon_VS(inout vec4 vertex) { \n"
        "    oe_TrackNode_texcoord = gl_MultiTexCoord0.st; \n"
        "} \n";

    const char* iconFS =
        "in vec2 oe_TrackNode_texcoord; \n"
        "uniform sampler2D oe_TrackNode_tex; \n"
        "void oe_TrackNode_icon_FS(inout vec4 color) { \n"
        "    color = texture(oe_TrackNode_tex, oe_TrackNode_texcoord); \n"
        "} \n";
}

//------------------------------------------------------------------------

osg::observer_ptr<osg::StateSet> TrackNode::s_geodeStateSet;
osg::observer_ptr<osg::StateSet> TrackNode::s_imageStateSet;


TrackNode::TrackNode(const GeoPoint& position,
    osg::Image* image,
    const TrackNodeFieldSchema& fieldSchema) :

    GeoPositionNode()
{
    construct();

    if (image)
    {
        IconSymbol* icon = _style.getOrCreate<IconSymbol>();
        icon->setImage(image);
    }

    _fieldSchema = fieldSchema;

    setPosition(position);

    compile();
}

TrackNode::TrackNode(const GeoPoint& position,
    const Style& style,
    const TrackNodeFieldSchema& fieldSchema) :

    GeoPositionNode(),
    _style(style)
{
    construct();

    _fieldSchema = fieldSchema;

    setPosition(position);

    compile();
}

void
TrackNode::construct()
{
    // This class makes its own shaders
    ShaderGenerator::setIgnoreHint(this, true);

    _geode = new osg::Geode();
    getPositionAttitudeTransform()->addChild( _geode );

    // initialize the shared stateset for the shared statesets.
    osg::ref_ptr<osg::StateSet> geodeStateSet;
    if (s_geodeStateSet.lock(geodeStateSet) == false)
    {
        static std::mutex m;
        std::lock_guard<std::mutex> lock(m);
        if (s_geodeStateSet.lock(geodeStateSet) == false)
        {
            s_geodeStateSet = geodeStateSet = new osg::StateSet();

            // draw in the screen-space bin
            ScreenSpaceLayout::activate(geodeStateSet.get());

            // completely disable depth buffer
            geodeStateSet->setAttributeAndModes(
                new osg::Depth(osg::Depth::ALWAYS, 0, 1, false),
                osg::StateAttribute::ON | osg::StateAttribute::PROTECTED);

            // Disable lighting for place nodes by default
            Lighting::set(geodeStateSet.get(), osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED);
        }
    }
    _geode->setStateSet(geodeStateSet.get());

    if (s_imageStateSet.lock(_imageStateSet) == false)
    {
        static std::mutex m;
        std::lock_guard<std::mutex> lock(m);
        if (s_imageStateSet.lock(_imageStateSet) == false)
        {
            s_imageStateSet = _imageStateSet = new osg::StateSet();
            VirtualProgram* vp = VirtualProgram::getOrCreate(_imageStateSet.get());
            vp->setName("TrackNode");
            vp->setFunction("oe_TrackNode_icon_VS", iconVS, VirtualProgram::LOCATION_VERTEX_MODEL);
            vp->setFunction("oe_TrackNode_icon_FS", iconFS, VirtualProgram::LOCATION_FRAGMENT_COLORING);
            _imageStateSet->addUniform(new osg::Uniform("oe_TrackNode_tex", 0));
        }
    }
}

void
TrackNode::compile()
{
    // reset by clearing out any existing nodes:
    _geode->removeChildren(0, _geode->getNumChildren());

    IconSymbol* icon = _style.get<IconSymbol>();
    osg::Image* image = icon ? icon->getImage() : 0L;

    if ( icon && image )
    {
        // apply the image icon.
        osg::Geometry* imageGeom = AnnotationUtils::createImageGeometry( 
            image,                    // image
            osg::Vec2s(0,0),          // offset
            0,                        // tex image unit
            icon->heading()->eval(),
            icon->scale()->eval() );

        if ( imageGeom )
        {
            imageGeom->getOrCreateStateSet()->merge(*_imageStateSet.get());
            _geode->addChild(imageGeom);

            auto layout = ScreenSpaceLayoutData::getOrCreate(imageGeom);
            layout->setPriority(getPriority());
        }

        _iconDrawable = imageGeom;
    }

    if ( !_fieldSchema.empty() )
    {
        // turn the schema defs into text drawables and record a map so we can
        // set the field text later.
        for( TrackNodeFieldSchema::const_iterator i = _fieldSchema.begin(); i != _fieldSchema.end(); ++i )
        {
            const TrackNodeField& field = i->second;
            if ( field._symbol.valid() )
            {
                osg::Vec3 offset(
                    field._symbol->pixelOffset()->x(),
                    field._symbol->pixelOffset()->y(),
                    0.0);

                osg::Drawable* drawable = AnnotationUtils::createTextDrawable( 
                    field._symbol->content()->expr(),   // text
                    field._symbol.get(),                // symbol
                    offset );                           // offset

                if ( drawable )
                {
                    // if the user intends to change the label later, make it dynamic
                    // since osgText updates are not thread-safe
                    if ( field._dynamic )
                        drawable->setDataVariance( osg::Object::DYNAMIC );
                    else
                        drawable->setDataVariance( osg::Object::STATIC );

                    addDrawable( i->first, drawable );
                }
            }
        }
    }

    applyStyle( _style );
}

void
TrackNode::setPriority(float value)
{
    GeoPositionNode::setPriority( value );
    updateLayoutData();
}

void
TrackNode::setIconRotation(const Angle& angle)
{
    _iconAngle = angle;
    auto* layout = ScreenSpaceLayoutData::getOrCreate(_iconDrawable);
    if (layout)
        layout->setRotationDegrees(angle.as(Units::DEGREES));
}

void
TrackNode::updateLayoutData()
{
    osg::ref_ptr<ScreenSpaceLayoutData> data = new ScreenSpaceLayoutData();
    data->setPriority(getPriority());

    // re-apply annotation drawable-level stuff as neccesary.
    for (unsigned i = 0; i<_geode->getNumChildren(); ++i)
    {
        auto* data = ScreenSpaceLayoutData::getOrCreate(_geode->getChild(i));
        data->setPriority(getPriority());
    }
}

void
TrackNode::setFieldValue( const std::string& name, const osgText::String& value )
{
    NamedDrawables::const_iterator i = _namedDrawables.find(name);

    if ( i != _namedDrawables.end() )
    {
        osgText::Text* drawable = dynamic_cast<osgText::Text*>( i->second );
        if ( drawable )
        {
            // only permit updates if the field was declared dynamic, OR
            // this node is not connected yet
            if (drawable->getDataVariance() == osg::Object::DYNAMIC || this->getNumParents() == 0)
            {
                // btw, setText checks for assigning an equal value, so we don't have to
                drawable->setText( value );
            }
            else
            {
                OE_WARN << LC 
                    << "Illegal: attempt to modify a TrackNode field value that is not marked as dynamic"
                    << std::endl;
            }
        }
    }
}

void
TrackNode::addDrawable( const std::string& name, osg::Drawable* drawable )
{
    _namedDrawables[name] = drawable;
    _geode->addChild( drawable );
    updateLayoutData();
}

osg::Drawable*
TrackNode::getDrawable( const std::string& name ) const
{
    NamedDrawables::const_iterator i = _namedDrawables.find(name);
    return i != _namedDrawables.end() ? i->second : 0L;
}
