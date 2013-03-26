/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2012 Pelican Mapping
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
#include <osgEarthSymbology/IconResource>
#include <osgEarth/StringUtils>
#include <osgEarth/ImageUtils>

#include <osg/AutoTransform>
#include <osg/Depth>
#include <osg/Geometry>
#include <osg/TextureRectangle>
#include <osg/Program>

#define LC "[IconResource] "

using namespace osgEarth;
using namespace osgEarth::Symbology;

//---------------------------------------------------------------------------

namespace
{
    osg::Node* buildIconModel(osg::Image* image)
    {
        float width = image->s();
        float height = image->t();

        osg::Geometry* geometry = new osg::Geometry;
        geometry->setUseVertexBufferObjects(true);

        osg::Vec3Array* verts = new osg::Vec3Array(4);
        (*verts)[0] = osg::Vec3(-width/2.0f, -height/2.0, 0.0f);
        (*verts)[1] = osg::Vec3(width/2.0f, -height/2.0, 0.0f);
        (*verts)[2] = osg::Vec3(width/2.0f, height/2.0, 0.0f);
        (*verts)[3] = osg::Vec3(-width/2.0f,height/2.0, 0.0f);
        geometry->setVertexArray( verts );

        bool flip = image->getOrigin()==osg::Image::TOP_LEFT;

        osg::Vec2Array* texcoords = new osg::Vec2Array(4);
        (*texcoords)[0].set(0.0f,flip ? height-1.0f : 0.0f);
        (*texcoords)[1].set(width-1.0f,flip ? height-1.0f : 0.0f);
        (*texcoords)[2].set(width-1.0f,flip ? 0.0 : height-1.0f);
        (*texcoords)[3].set(0.0f,flip ? 0.0 : height-1.0f);
        geometry->setTexCoordArray(0, texcoords);

        osg::Vec4Array* colors = new osg::Vec4Array(1);
        (*colors)[0].set(1,1,1,1);
        geometry->setColorArray( colors );
        geometry->setColorBinding( osg::Geometry::BIND_OVERALL );

        geometry->addPrimitiveSet( new osg::DrawArrays(GL_QUADS, 0, 4));

        osg::StateSet* stateSet = geometry->getOrCreateStateSet();

        osg::TextureRectangle* texture = new osg::TextureRectangle( image );
        stateSet->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON);

        stateSet->setMode( GL_BLEND, 1 );
        stateSet->setRenderBinDetails( 95, "RenderBin" );
        stateSet->setAttributeAndModes( new osg::Depth(osg::Depth::ALWAYS,false), 1 );

        osg::Geode* geode = new osg::Geode;
        geode->addDrawable( geometry );

        osg::AutoTransform* at = new osg::AutoTransform;
        at->setAutoScaleToScreen( true );
        at->setAutoRotateMode( osg::AutoTransform::ROTATE_TO_SCREEN );
        at->addChild( geode );
        return at;
    }
}

//---------------------------------------------------------------------------

IconResource::IconResource( const Config& conf ) :
InstanceResource( conf )
{
    mergeConfig( conf );
}

void
IconResource::mergeConfig( const Config& conf )
{
    //nop
}

Config
IconResource::getConfig() const
{
    Config conf = Resource::getConfig();
    conf.key() = "icon";
    //nop
    return conf;
}

osg::Node*
IconResource::createNodeFromURI( const URI& uri, const osgDB::Options* dbOptions ) const
{
    osg::Node* node = 0L;

    ReadResult r = uri.readImage( dbOptions );
    if ( r.succeeded() )
    {
        if ( r.getImage() )
        {
            node = buildIconModel( r.releaseImage() );
        }
    }

    else // failing that, fall back on the old encoding format..
    {
        StringVector tok;
        StringTokenizer( *uri, tok, "()" );
        if (tok.size() >= 2)
            return createNodeFromURI( URI(tok[1]), dbOptions );
    }

    return node;
}
