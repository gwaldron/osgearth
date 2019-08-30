/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2018 Pelican Mapping
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
#include "DecalLayer"
#include <osgEarth/Map>
#include <osgEarth/Profile>
#include <osgEarth/VirtualProgram>
#include <osg/MatrixTransform>
#include <osg/BlendFunc>
#include <osg/BlendEquation>

using namespace osgEarth;
using namespace osgEarth::Contrib;

#define LC "[DecalLayer] "

REGISTER_OSGEARTH_LAYER(decal, DecalLayer);

namespace
{
    const char* vs =
        "#version " GLSL_VERSION_STR "\n"
        "out vec2 texcoords; \n"
        "void main() { \n"
        "    texcoords = gl_MultiTexCoord0.st; \n"
        "    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex; \n"
        "}\n";

    const char* fs =
        "#version " GLSL_VERSION_STR "\n"
        "in vec2 texcoords; \n"
        "out vec4 output; \n"
        "uniform sampler2D tex; \n"
        "void main() { \n"
        "    output = texture(tex, texcoords); \n"
        "} \n";
}

//........................................................................

Config
DecalLayer::Options::getConfig() const
{
    Config conf = ImageLayer::Options::getConfig();
    return conf;
}

void
DecalLayer::Options::fromConfig(const Config& conf)
{
    //nop
}

//........................................................................

void
DecalLayer::init()
{
    ImageLayer::init();

    setTileSourceExpected(false);

    setUseCreateTexture();

    // Generate Mercator tiles by default.
    setProfile(Profile::create("global-geodetic"));

    if (getName().empty())
        setName("Decal");

    // Create a rasterizer for rendering nodes to images.
    _rasterizer = new TileRasterizer(); 

    // Configure a base stateset for the rasterizer:
    osg::StateSet* rasterizerSS = _rasterizer->getOrCreateStateSet();

    // Program for rendering with the TileRasterizer
    osg::Program* program = new osg::Program();
    program->addShader(new osg::Shader(osg::Shader::VERTEX, vs));
    program->addShader(new osg::Shader(osg::Shader::FRAGMENT, fs));
    rasterizerSS->setAttribute(program);

    // Use normal RGB blending:
    osg::BlendFunc* blendFunc = new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    rasterizerSS->setAttributeAndModes(blendFunc);

    // Use MAX blending for the alpha channel:
    osg::BlendEquation* blendEquation = new osg::BlendEquation(osg::BlendEquation::FUNC_ADD, osg::BlendEquation::RGBA_MAX);
    rasterizerSS->setAttributeAndModes(blendEquation);

    // Create a placeholder image to display before rasterization is complete
    _placeholder = ImageUtils::createEmptyImage();
}

const Status&
DecalLayer::open()
{
    return ImageLayer::open();
}

void
DecalLayer::addedToMap(const Map* map)
{
    ImageLayer::addedToMap(map);
}

void
DecalLayer::removedFromMap(const Map* map)
{
    ImageLayer::removedFromMap(map);
}

osg::Node*
DecalLayer::getNode() const
{
    // adds the Rasterizer to the scene graph so we can rasterize tiles
    return _rasterizer.get();
}

void
DecalLayer::releaseGLObjects(osg::State* state) const
{
    for(TileTable::const_iterator tile = _tiles.begin();
        tile != _tiles.end();
        ++tile)
    {
        tile->second->releaseGLObjects(state);
    }

    ImageLayer::releaseGLObjects(state);
}

void
DecalLayer::resizeGLObjectBuffers(unsigned maxSize)
{
    for(TileTable::const_iterator tile = _tiles.begin();
        tile != _tiles.end();
        ++tile)
    {
        tile->second->resizeGLObjectBuffers(maxSize);
    }

    ImageLayer::resizeGLObjectBuffers(maxSize);
}

void
DecalLayer::updateTexture(const TileKey& key, osg::Texture2D* texture) const
{
    // INTERNAL function -- assume mutex is locked.

    GeoExtent outputExtent;
    osg::ref_ptr<osg::Group> decalGroup = new osg::Group();

    if (_decals.size() > 0)
    {
        // establish a local SRS for rendering our decal'd tile.
        // TODO: consider caching with texture in the texture table?
        outputExtent = key.getExtent();
        const SpatialReference* keySRS = outputExtent.getSRS();
        osg::Vec3d pos = outputExtent.getCentroid();
        osg::ref_ptr<const SpatialReference> srs = keySRS->createTangentPlaneSRS(pos);
        //srs = keySRS->createEquirectangularSRS();
        outputExtent = outputExtent.transform(srs.get());

        for(std::vector<Decal>::const_iterator decal = _decals.begin();
            decal != _decals.end();
            ++decal)
        {
            if (key.getExtent().intersects(decal->_extent))
            {
                GeoExtent e = decal->_extent.transform(outputExtent.getSRS());
                if (e.isValid())
                {
                    osg::Node* mesh = buildMesh(e, decal->_image.get());
                    if (mesh)
                    {
                        decalGroup->addChild(mesh);
                    }
                }
            }
        }
    }

    if (decalGroup->getNumChildren() > 0)
    {
        // Setting a size indicates that this texture is ready:
        if (texture->getTextureWidth() != getTileSize())
        {
            texture->setTextureSize(getTileSize(), getTileSize());
            texture->setImage(NULL);
            texture->dirtyTextureObject();
        }
    }

    // Schedule the rasterization. We must rasterize each tile even if
    // the decal Group is empty, to get an empty texture.
    _rasterizer->push(decalGroup.get(), texture, outputExtent);
}

TextureWindow
DecalLayer::createTexture(const TileKey& key, ProgressCallback* progress) const
{
    if (getStatus().isError())
    {
        return TextureWindow();
    }

    if (key.getLOD() != getMinLevel())
    {
        return TextureWindow();
    }

    Threading::ScopedMutexLock lock(_mutex);

    // allocation the texture for this key:
    osg::ref_ptr<osg::Texture2D> texture;
    {
        TileTable::iterator i = _tiles.find(key);
        if (i != _tiles.end())
        {
            texture = i->second.get();
        }
        else
        {
            // THe placeholder is necessary because otherwise you can see
            // "junk" data on the terrain while the tile is queued for
            // rasterization.
            texture = new osg::Texture2D(_placeholder.get());
            texture->setDataVariance(osg::Object::DYNAMIC);
            // TODO: Don't set the size until we actually render to a texture...?
            // NOTE: Doing this causes the visual update to be weird.
            // NOTE: Maybe don't do this.
            texture->setTextureSize(1, 1);
            texture->setSourceFormat(GL_RGBA);
            texture->setSourceType(GL_UNSIGNED_BYTE);
            texture->setInternalFormat(GL_RGBA8);
            texture->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
            texture->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR );
            texture->setWrap( osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE );
            texture->setWrap( osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE );
            texture->setResizeNonPowerOfTwoHint(false);
            texture->setMaxAnisotropy(1.0f);
            texture->setUnRefImageDataAfterApply(false);


            _tiles[key] = texture;
        }
    }

    updateTexture(key, texture.get());

    return TextureWindow(texture.get(), osg::Matrix::identity());
}

void
DecalLayer::addDecal(const GeoExtent& extent, const osg::Image* image)
{
    Threading::ScopedMutexLock lock(_mutex);

    _decals.resize(_decals.size()+1);
    Decal& decal = _decals.back();
    decal._extent = extent;
    decal._image = image;

    // update any existing tiles that intersect the new decal.
    for(TileTable::const_iterator tile = _tiles.begin();
        tile != _tiles.end();
        ++tile)
    {
        if (extent.intersects(tile->first.getExtent()))
        {
            updateTexture(tile->first, tile->second.get());
        }
    }
}

osg::Node*
DecalLayer::buildMesh(const GeoExtent& extent, const osg::Image* image) const
{
    double w = extent.width(), h = extent.height();

    osg::Geometry* geom = new osg::Geometry();

    osg::Vec3Array* verts = new osg::Vec3Array(4);
    (*verts)[0].set(0, 0, 0);
    (*verts)[1].set(w, 0, 0);
    (*verts)[2].set(w, h, 0);
    (*verts)[3].set(0, h, 0);
    geom->setVertexArray(verts);

    osg::Vec4Array* colors = new osg::Vec4Array(1);
    (*colors)[0].set(1,1,1,1);
    geom->setColorArray(colors, osg::Array::BIND_OVERALL);

    osg::DrawElementsUByte* de = new osg::DrawElementsUByte(GL_TRIANGLES);
    de->reserve(6);
    de->addElement(0);
    de->addElement(1);
    de->addElement(2);    
    de->addElement(2);
    de->addElement(3);
    de->addElement(0);
    geom->addPrimitiveSet(de);

    osg::Vec2Array* texcoords = new osg::Vec2Array(4);
    (*texcoords)[0].set(0,0);
    (*texcoords)[1].set(1,0);
    (*texcoords)[2].set(1,1);
    (*texcoords)[3].set(0,1);
    geom->setTexCoordArray(0, texcoords);

    osg::Texture2D* tex = new osg::Texture2D(const_cast<osg::Image*>(image));
    tex->setWrap( osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE );
    tex->setWrap( osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE );
    tex->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR );
    tex->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
    tex->setResizeNonPowerOfTwoHint(false);
    tex->setUnRefImageDataAfterApply(false);

    osg::StateSet* ss = geom->getOrCreateStateSet();
    ss->setTextureAttribute(0, tex);

    osg::MatrixTransform* mt = new osg::MatrixTransform();
    mt->setMatrix(osg::Matrix::translate(extent.xMin(), extent.yMin(), 0));
    mt->addChild(geom);

    return mt;
}
