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
#if 0
#include <osgEarthFeatures/MarkerFactory>
#include <osgEarth/Utils>
#include <osg/TextureRectangle>
#include <osg/Depth>
#include <osg/AutoTransform>

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

//----------------------------------------------------------------------------

namespace
{
    osg::Node* buildImageModel(osg::Image* image)
    {
        float width = image->s();
        float height = image->t();

        osg::Geometry* geometry = new osg::Geometry;

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

        //PixelAutoTransform* at = new PixelAutoTransform;
        //at->setMinPixelWidth( width );
        osg::AutoTransform* at = new osg::AutoTransform;
        at->setAutoScaleToScreen( true );
        at->setAutoRotateMode( osg::AutoTransform::ROTATE_TO_SCREEN );
        at->addChild( geode );
        return at;
    }
}

//----------------------------------------------------------------------------

MarkerFactory::MarkerFactory( Session* session ) :
_session( session )
{
    //nop
}

osg::Node*
MarkerFactory::getOrCreateNode( const Feature* feature, const MarkerSymbol* symbol, bool useCache )
{
    osg::Node* result =0L;

    if ( symbol )
    {
        if ( symbol->getNode() )
        {
            result = symbol->getNode();
        }
        else if ( symbol->getImage() )
        {
            const std::string& fileName = symbol->getImage()->getFileName();
            if ( !fileName.empty() && _session.valid() && useCache )
            {
                result = _session->getResource<osg::Node>( fileName );
            }

            if ( !result )
            {
                result = buildImageModel( symbol->getImage() );
            }

            if ( result && _session.valid() && useCache && !fileName.empty() )
            {
                _session->putResource(fileName, result);
            }
        }
        else if ( symbol->url().isSet() && !symbol->url()->empty() )
        {            
            StringExpression expr = symbol->url().get();
            std::string val = feature->eval( expr  );//symbol->url()->full();
            URI uri( val, expr.uriContext() );

            OE_DEBUG << "Using model URL " << uri.full() << std::endl;
            if ( _session.valid() && useCache )
            {
                result = _session->getResource<osg::Node>( uri.full() );
            }

            if ( !result )
            {                
                result = createFromURI( uri );
            }

            if ( result && _session.valid() && useCache )
            {
                _session->putResource( uri.full(), result );
            }
        }
    }

    return result;
}

osg::Image*
MarkerFactory::getOrCreateImage( const MarkerSymbol* symbol, bool useCache )
{
    if ( symbol->getImage() )
    {
        return symbol->getImage();
    }
    else if ( symbol->url().isSet() && !symbol->url()->empty() )
    {
        return createImageFromURI( symbol->url()->expr() );
    }
    return 0L;
}

osg::Node*
MarkerFactory::createFromURI( const URI& uri ) const
{
    osg::ref_ptr<osg::Object> obj = uri.readObject();
    if ( obj.valid() )
    {
        if ( dynamic_cast<osg::Image*>( obj.get() ) )
        {
            return buildImageModel( dynamic_cast<osg::Image*>( obj.get() ) );
        }
        else if ( dynamic_cast<osg::Node*>( obj.get() ) )
        {
            return dynamic_cast<osg::Node*>( obj.release() );
        }
    }

    else // failing that, fall back on the old encoding format..
    {
        StringVector tok;
        StringTokenizer( *uri, tok, "()" );
        if (tok.size() >= 2)
            return createFromURI( URI(tok[1]) );
    }

    // fail
    return 0L;
}


osg::Image*
MarkerFactory::createImageFromURI( const URI& uri ) const
{
    StringVector tok;
    StringTokenizer( *uri, tok, "()" );

    if ( tok.size() > 0 )
    {
        URI imageURI( tok[tok.size()-1], uri.context() );
        return imageURI.readImage();
    }

    return 0L;
}

#endif