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

#include <osgEarthAnnotation/PlaceNode>
#include <osgEarthAnnotation/LabelNode>
#include <osgEarthFeatures/BuildTextFilter>
#include <osgEarthFeatures/LabelSource>
#include <osgEarth/Utils>
#include <osgText/Text>
#include <osg/Depth>

using namespace osgEarth;
using namespace osgEarth::Annotation;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;


PlaceNode::PlaceNode(MapNode*           mapNode,
                     const osg::Vec3d&  position,
                     osg::Image*        image,
                     const std::string& text,
                     const Style&       style ) :

OrthoNode( mapNode->getMap()->getProfile()->getSRS(), position ),
_image  ( image ),
_text   ( text ),
_style  ( style )
{
    init();
}

void
PlaceNode::init()
{
    osg::Texture2D* texture = new osg::Texture2D();
    texture->setFilter(osg::Texture::MIN_FILTER,osg::Texture::LINEAR);
    texture->setFilter(osg::Texture::MAG_FILTER,osg::Texture::LINEAR);
    texture->setResizeNonPowerOfTwoHint(false);
    texture->setImage( _image.get() );

    // set up the drawstate.
    osg::StateSet* dstate = new osg::StateSet;
    dstate->setMode(GL_CULL_FACE,osg::StateAttribute::OFF);
    dstate->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
    dstate->setMode(GL_BLEND, 1);
    dstate->setTextureAttributeAndModes(0, texture,osg::StateAttribute::ON);   

    // set up the geoset.
    osg::Geometry* geom = new osg::Geometry();
    geom->setStateSet(dstate);

    osg::Vec3Array* coords = new osg::Vec3Array(4);
    (*coords)[0].set( -_image->s()/2.0, 0, 0 );
    (*coords)[1].set( -_image->s()/2.0 + _image->s(), 0, 0 );
    (*coords)[2].set( -_image->s()/2.0 + _image->s(), _image->t()-1, 0 );
    (*coords)[3].set( -_image->s()/2.0, _image->t()-1, 0 );
    geom->setVertexArray(coords);

    osg::Vec2Array* tcoords = new osg::Vec2Array(4);
    (*tcoords)[0].set(0, 0);
    (*tcoords)[1].set(1, 0);
    (*tcoords)[2].set(1, 1);
    (*tcoords)[3].set(0, 1);
    geom->setTexCoordArray(0,tcoords);

    osg::Vec4Array* colors = new osg::Vec4Array(1);
    (*colors)[0].set(1.0f,1.0f,1.0,1.0f);
    geom->setColorArray(colors);
    geom->setColorBinding(osg::Geometry::BIND_OVERALL);

    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS,0,4));

    osg::Drawable* text = LabelUtils::createText(
        osg::Vec3( _image->s()/2.0 + 2, _image->t()/2.0, 0 ),
        _text,
        _style.get<TextSymbol>() );

    osg::Geode* geode = new osg::Geode();
    geode->addDrawable( text );
    geode->addDrawable( geom );
    
    osg::StateSet* stateSet = geode->getOrCreateStateSet();
    stateSet->setAttributeAndModes( new osg::Depth(osg::Depth::ALWAYS, 0, 1, false), 1 );

    this->attach( geode );
}

void
PlaceNode::setIconImage( osg::Image* image )
{
    //todo
}

void
PlaceNode::setText( const std::string& text )
{
    //todo
}

void
PlaceNode::setStyle( const Style& style )
{
    //todo
}
