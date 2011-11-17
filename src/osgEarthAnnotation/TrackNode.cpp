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

#include <osgEarthAnnotation/TrackNode>
#include <osgEarthAnnotation/AnnotationUtils>
#include <osgEarth/Utils>
#include <osg/Depth>
#include <osgText/Text>

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Symbology;

//------------------------------------------------------------------------

TrackNode::TrackNode(MapNode*                    mapNode, 
                     const osg::Vec3d&           position,
                     osg::Image*                 image,
                     const TrackNodeFieldSchema& fieldSchema ) :

OrthoNode   ( mapNode ? mapNode->getMap()->getProfile()->getSRS() : 0L, position ),
_image      ( image )
{
    init( fieldSchema );
}

void
TrackNode::init( const TrackNodeFieldSchema& schema )
{
    osg::Geode* geode = new osg::Geode();

    if ( !schema.empty() )
    {
        // turn the schema defs into text drawables and record a map so we can
        // set the field text later.
        for( TrackNodeFieldSchema::const_iterator i = schema.begin(); i != schema.end(); ++i )
        {
            const TextSymbol* ts = i->second.get();
            if ( ts )
            {
                osg::Drawable* drawable = AnnotationUtils::createTextDrawable( 
                    osgEarth::EMPTY_STRING, ts, osg::Vec3(0,0,0), true );

                if ( drawable )
                {
                    drawable->setDataVariance( osg::Object::DYNAMIC );
                    geode->addDrawable( drawable );
                    _fieldDrawables[i->first] = drawable;
                }
            }
        }
    }
    
    if ( _image.valid() )
    {
        // apply the image icon.
        osg::Geometry* imageGeom = AnnotationUtils::createImageGeometry( 
            _image.get(), osg::Vec2s(0,0), true );

        if ( imageGeom )
        {
            geode->addDrawable( imageGeom );
        }
    }
    
    // ensure depth testing always passes, and disable depth bugger writes.
    osg::StateSet* stateSet = geode->getOrCreateStateSet();
    stateSet->setAttributeAndModes( new osg::Depth(osg::Depth::ALWAYS, 0, 1, false), 1 );

    this->attach( geode );
}

void
TrackNode::setFieldValue( const std::string& name, const std::string& value )
{
    FieldDrawables::const_iterator i = _fieldDrawables.find(name);
    if ( i != _fieldDrawables.end() )
    {
        osgText::Text* drawable = static_cast<osgText::Text*>( i->second );
        if ( drawable )
        {
            drawable->setText( value );
        }
    }
}
