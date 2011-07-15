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
MarkerFactory::getOrCreateNode( const MarkerSymbol* symbol, bool useCache )
{
    osg::Node* result;

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
            if ( _session.valid() && useCache )
            {
                result = _session->getResource<osg::Node>( *symbol->url() );
            }

            if ( !result )
            {
                result = createFromURI( *symbol->url() );
            }

            if ( result && _session.valid() && useCache )
            {
                _session->putResource( *symbol->url(), result );
            }
        }
		else
			result = NULL;
    }
	else
		result = NULL;

    return result;
}

#if 0
osg::Node*
MarkerFactory::getOrCreateNode( const std::string& markerURI, bool useCache )
{
    osg::Node* result;

    // try to retrieve it from the session cache.
    if ( _session.valid() && useCache )
    {
        result = _session->getResource<osg::Node>( markerURI );
    }

    if ( !result )
    {
        result = createFromURI( markerURI );

        // cache it in the session.
        if ( result && _session.valid() && useCache )
        {
            _session->putResource( markerURI, result );
        }
    }

    return result;
}
#endif

osg::Node*
MarkerFactory::createFromURI( const std::string& uri ) const
{
    StringTokenizer izer( "()" );
    StringVector tok;
    izer.tokenize( uri, tok );

    if (tok.size() > 1)
    {
        if (tok[0].compare("model") == 0)
        {         
            osg::ref_ptr<osg::Node> node;
            HTTPClient::readNodeFile( _session.valid()? _session->resolveURI(tok[1]) : tok[1], node );
            return node.release();
        }
        else if (tok[0].compare("image") == 0)
        {
            osg::ref_ptr<osg::Image> image;
            HTTPClient::readImageFile( _session.valid()? _session->resolveURI(tok[1]) : tok[1], image );
            return buildImageModel( image.get() );
        }    
    }
    else
    {
        osg::ref_ptr<osg::Node> node;
        HTTPClient::readNodeFile( _session.valid()? _session->resolveURI(uri) : uri, node );
        return node.release();
    }

    return 0L;
}
