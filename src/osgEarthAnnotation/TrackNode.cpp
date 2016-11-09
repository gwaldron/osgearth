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

#include <osgEarthAnnotation/TrackNode>
#include <osgEarthAnnotation/AnnotationUtils>
#include <osgEarth/MapNode>
#include <osgEarth/Registry>
#include <osgEarth/ShaderGenerator>
#include <osgEarth/ScreenSpaceLayout>
#include <osg/Depth>
#include <osgText/Text>

#define LC "[TrackNode] "

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Symbology;

//------------------------------------------------------------------------

TrackNode::TrackNode(MapNode*                    mapNode, 
                     const GeoPoint&             position,
                     osg::Image*                 image,
                     const TrackNodeFieldSchema& fieldSchema ) :

GeoPositionNode   ( mapNode, position )
{
    if ( image )
    {
        IconSymbol* icon = _style.getOrCreate<IconSymbol>();
        icon->setImage( image );
    }

    init( fieldSchema );
}

TrackNode::TrackNode(MapNode*                    mapNode, 
                     const GeoPoint&             position,
                     const Style&                style,
                     const TrackNodeFieldSchema& fieldSchema ) :

GeoPositionNode   ( mapNode, position ),
_style      ( style )
{
    init( fieldSchema );
}

void
TrackNode::init( const TrackNodeFieldSchema& schema )
{
    // tracknodes draw in screen space at their geoposition.
    ScreenSpaceLayout::activate( this->getOrCreateStateSet() );

    osgEarth::clearChildren( getPositionAttitudeTransform() );

    _geode = new osg::Geode();

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
            _geode->addDrawable( imageGeom );

            ScreenSpaceLayoutData* layout = new ScreenSpaceLayoutData();
            layout->setPriority(getPriority());
            imageGeom->setUserData(layout);
        }
    }

    if ( !schema.empty() )
    {
        // turn the schema defs into text drawables and record a map so we can
        // set the field text later.
        for( TrackNodeFieldSchema::const_iterator i = schema.begin(); i != schema.end(); ++i )
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

    // ensure depth testing always passes, and disable depth buffer writes.
    osg::StateSet* stateSet = _geode->getOrCreateStateSet();
    stateSet->setAttributeAndModes( new osg::Depth(osg::Depth::ALWAYS, 0, 1, false), 1 );

    applyStyle( _style );

    setLightingIfNotSet( false );

    getPositionAttitudeTransform()->addChild( _geode );
    
    // generate shaders:
    Registry::shaderGenerator().run(
        this,
        "osgEarth.TrackNode",
        Registry::stateSetCache() );
}

void
TrackNode::setPriority(float value)
{
    GeoPositionNode::setPriority( value );
    updateLayoutData();
}

void
TrackNode::updateLayoutData()
{
    osg::ref_ptr<ScreenSpaceLayoutData> data = new ScreenSpaceLayoutData();
    data->setPriority(getPriority());

    // re-apply annotation drawable-level stuff as neccesary.
    for (unsigned i = 0; i<_geode->getNumDrawables(); ++i)
    {
        _geode->getDrawable(i)->setUserData(data.get());
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
    _geode->addDrawable( drawable );
    updateLayoutData();
}

osg::Drawable*
TrackNode::getDrawable( const std::string& name ) const
{
    NamedDrawables::const_iterator i = _namedDrawables.find(name);
    return i != _namedDrawables.end() ? i->second : 0L;
}
