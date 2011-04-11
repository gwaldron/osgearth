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

#include <osgEarthFeatures/Session>
#include <osgEarth/FileUtils>
#include <osgEarth/HTTPClient>
#include <osgEarth/StringUtils>
#include <osg/AutoTransform>
#include <osg/Texture2D>

#define LC "[Session] "

using namespace osgEarth;
using namespace osgEarth::Features;

Session::Session( const Map* map ) :
_mapFrame( map )
{
    //nop
}

void
Session::setReferenceURI( const std::string& referenceURI )
{
    _referenceURI = referenceURI;
}

std::string
Session::resolveURI( const std::string& inputURI ) const
{
    return osgEarth::getFullPath( _referenceURI, inputURI );
}

void
Session::parseMarker(const std::string& marker, std::string& url, bool &isImage) const
{    
    StringVector tok;
    osgEarth::tokenize( marker, tok, "()", "" );

    if (tok.size() > 1)
    {
        if (tok[0].compare("model") == 0)
        {         
            isImage = false;
            url = resolveURI( tok[1] );
        }
        else if (tok[0].compare("image") == 0)
        {
            url = resolveURI( tok[1] );
            isImage = true;
        }    
    }
    else
    {
        url = resolveURI( marker );
        isImage = false;
    }    
}

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
    (*texcoords)[0].set(0.0f,flip ? 1.0 : 0.0f);
    (*texcoords)[1].set(1.0f,flip ? 1.0 : 0.0f);
    (*texcoords)[2].set(1.0f,flip ? 0.0 : 1.0f);
    (*texcoords)[3].set(0.0f,flip ? 0.0 : 1.0f);
    geometry->setTexCoordArray(0, texcoords);

    geometry->addPrimitiveSet( new osg::DrawArrays(GL_QUADS, 0, 4));

    osg::Texture2D* texture = new osg::Texture2D( image );
    texture->setResizeNonPowerOfTwoHint(false);
    geometry->getOrCreateStateSet()->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON);

    osg::Geode* geode = new osg::Geode;
    geode->addDrawable( geometry );

    osg::AutoTransform* at = new osg::AutoTransform;
    at->setAutoScaleToScreen( true );
    at->setAutoRotateMode( osg::AutoTransform::AutoRotateMode::ROTATE_TO_CAMERA );
    at->addChild( geode );
    return at;
}

osg::Node*
Session::getModel( const std::string& url ) const
{    
    bool isImage;
    std::string absurl;
    parseMarker(url, absurl, isImage);

    // first, check the local repo
    {
        Threading::ScopedReadLock sharedLock( const_cast<Session*>(this)->_modelsMutex );
        ModelMap::const_iterator i = _models.find( absurl );
        if ( i != _models.end() )
            return i->second.get();
    }

    // next, try to load the model from its URL
    osg::ref_ptr<osg::Node> node;
    if (!isImage)
    {
        if ( HTTPClient::readNodeFile( absurl, node ) != HTTPClient::RESULT_OK )
        {
            OE_WARN << LC << "Failed to load model from \"" << url << "\"" << std::endl;
            return 0L;
        }
    }
    else
    {
        osg::ref_ptr< osg::Image > image;
        if (HTTPClient::readImageFile( absurl, image ) != HTTPClient::RESULT_OK )
        {
            OE_WARN << LC << "Failed to load image from \"" << url << "\"" << std::endl;
            return 0L;
        }

        node = buildImageModel( image.get() );
    }

    // add it to the local model cache for next time
    {
        Session* ncthis = const_cast<Session*>(this);
        Threading::ScopedWriteLock exclusiveLock( ncthis->_modelsMutex );
        ncthis->_models[absurl] = node.get();
    }

    return node.release();
}
