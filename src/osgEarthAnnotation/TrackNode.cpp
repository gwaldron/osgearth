/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2008-2013 Pelican Mapping
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

#include <osgEarthAnnotation/TrackNode>
#include <osgEarthAnnotation/AnnotationUtils>
#include <osgEarth/MapNode>
#include <osgEarth/Registry>
#include <osgEarth/ShaderGenerator>
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

OrthoNode   ( mapNode, position )
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

OrthoNode   ( mapNode, position ),
_style      ( style )
{
    init( fieldSchema );
}

void
TrackNode::init( const TrackNodeFieldSchema& schema )
{
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
            icon->heading()->eval() );

        if ( imageGeom )
        {
            _geode->addDrawable( imageGeom );
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
                osg::Drawable* drawable = AnnotationUtils::createTextDrawable( 
                    field._symbol->content()->expr(),   // text
                    field._symbol.get(),                // symbol
                    osg::Vec3(0,0,0) );                 // offset

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

    getAttachPoint()->addChild( _geode );

    ShaderGenerator gen;
    gen.setProgramName( "osgEarth.TrackNode" );
    gen.run( this, Registry::stateSetCache() );
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
    // attach the annotation data to the drawable:
    if ( _annoData.valid() )
        drawable->setUserData( _annoData.get() );

    _namedDrawables[name] = drawable;
    _geode->addDrawable( drawable );
}

osg::Drawable*
TrackNode::getDrawable( const std::string& name ) const
{
    NamedDrawables::const_iterator i = _namedDrawables.find(name);
    return i != _namedDrawables.end() ? i->second : 0L;
}

void
TrackNode::setAnnotationData( AnnotationData* data )
{
    OrthoNode::setAnnotationData( data );

    // override this method so we can attach the anno data to the drawables.
    const osg::Geode::DrawableList& list = _geode->getDrawableList();
    for( osg::Geode::DrawableList::const_iterator i = list.begin(); i != list.end(); ++i )
    {
        i->get()->setUserData( data );
    }
}
