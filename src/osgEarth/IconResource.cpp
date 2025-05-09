/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <osgEarth/IconResource>
#include <osgEarth/StringUtils>
#include <osgEarth/ImageUtils>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>

#include <osg/Depth>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/TextureRectangle>
#include <osg/Texture2D>

#define LC "[IconResource] "

using namespace osgEarth;

//---------------------------------------------------------------------------

namespace
{
    osg::Node* buildIconModel(osg::Image* image)
    {
        // because the ShaderGenerator cannot handle texture rectangles yet.
        bool useRect = !Registry::capabilities().supportsGLSL();

        float width = image->s();
        float height = image->t();

        osg::Geometry* geometry = new osg::Geometry;
        geometry->setName("IconResource");
        geometry->setUseVertexBufferObjects(true);

        osg::Vec3Array* verts = new osg::Vec3Array(4);
        (*verts)[0] = osg::Vec3(-width/2.0f, -height/2.0, 0.0f);
        (*verts)[1] = osg::Vec3( width/2.0f, -height/2.0, 0.0f);
        (*verts)[2] = osg::Vec3(-width/2.0f,  height/2.0, 0.0f);
        (*verts)[3] = osg::Vec3( width/2.0f,  height/2.0, 0.0f);
        geometry->setVertexArray( verts );

        bool flip = image->getOrigin()==osg::Image::TOP_LEFT;

        osg::Vec2Array* texcoords = new osg::Vec2Array(4);
        if ( useRect )
        {
            (*texcoords)[0].set(0.0f,      flip ? height-1.0f : 0.0f);
            (*texcoords)[1].set(width-1.0f,flip ? height-1.0f : 0.0f);
            (*texcoords)[2].set(0.0f,      flip ? 0.0         : height-1.0f);
            (*texcoords)[3].set(width-1.0f,flip ? 0.0         : height-1.0f);
        }
        else
        {
            (*texcoords)[0].set(0.0f, flip ? 1.0f : 0.0f);
            (*texcoords)[1].set(1.0f, flip ? 1.0f : 0.0f);
            (*texcoords)[2].set(0.0f, flip ? 0.0  : 1.0f);
            (*texcoords)[3].set(1.0f, flip ? 0.0  : 1.0f);
        }
        geometry->setTexCoordArray(0, texcoords);

        osg::Vec4Array* colors = new osg::Vec4Array(osg::Array::BIND_OVERALL, 1);
        (*colors)[0].set(1,1,1,1);
        geometry->setColorArray( colors );

        geometry->addPrimitiveSet( new osg::DrawArrays(GL_TRIANGLE_STRIP, 0, 4));

        osg::StateSet* stateSet = geometry->getOrCreateStateSet();

        osg::Texture* texture;

        if ( useRect )
        {
            texture = new osg::TextureRectangle( image );
        }
        else
        {
            texture = new osg::Texture2D( image );
        }

        texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::NEAREST);
        texture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
        texture->setWrap  (osg::Texture::WRAP_S,     osg::Texture::CLAMP_TO_EDGE );
        texture->setWrap  (osg::Texture::WRAP_T,     osg::Texture::CLAMP_TO_EDGE );
        if ( Registry::capabilities().supportsNonPowerOfTwoTextures() )
            texture->setResizeNonPowerOfTwoHint( false );

        stateSet->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON);

        stateSet->setMode( GL_BLEND, 1 );
        stateSet->setRenderBinDetails( 95, "DepthSortedBin" );
        stateSet->setAttributeAndModes(
            new osg::Depth(osg::Depth::ALWAYS, 0, 1, false),
            osg::StateAttribute::ON | osg::StateAttribute::PROTECTED);

        osg::Geode* geode = new osg::Geode;
        geode->addDrawable( geometry );

        return geode;
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
    Config conf = InstanceResource::getConfig();
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
        OE_INFO << LC << "Loaded " << uri.base() << "(from " << (r.isFromCache()? "cache" : "source") << ")"
            << std::endl;

        if ( r.getImage() )
        {
            node = buildIconModel( r.releaseImage() );
        }
    }

    else // failing that, fall back on the old encoding format..
    {
        auto tok = StringTokenizer()
            .delim("(")
            .delim(")")
            .standardQuotes()
            .tokenize(*uri);

        if (tok.size() >= 2)
            return createNodeFromURI( URI(tok[1]), dbOptions );
    }

    return node;
}
