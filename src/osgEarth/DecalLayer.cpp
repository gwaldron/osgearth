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
#include <osgDB/WriteFile>

using namespace osgEarth;
using namespace osgEarth::Contrib;

#define LC "[DecalImageLayer] "

REGISTER_OSGEARTH_LAYER(decal, DecalImageLayer);

namespace
{
    // Shader for the rasterizer.
    // Encodes the delta of the alpha channel in X and Y so we can use
    // it to permute normals in the final output.

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
        "    output.x = texture(tex, texcoords + vec2(1.0/255.0, 0)).a; \n"
        "    output.y = texture(tex, texcoords + vec2(0, 1.0/255.0)).a; \n"
        "} \n";
}

//........................................................................

Config
DecalImageLayer::Options::getConfig() const
{
    Config conf = ImageLayer::Options::getConfig();
    return conf;
}

void
DecalImageLayer::Options::fromConfig(const Config& conf)
{
    //nop
}

//........................................................................

//#define USE_TEXTURES

void
DecalImageLayer::init()
{
    ImageLayer::init();

    // This layer does not use a TileSource.
    setTileSourceExpected(false);

    // Set the layer profile.
    setProfile(Profile::create("global-geodetic"));

    if (getName().empty())
        setName("Decal");

#ifdef USE_TEXTURES
    // This layer implements the createTexture() function.
    setUseCreateTexture();

    // Create a rasterizer for rendering nodes to images.
    _rasterizer = new TileRasterizer();

    // Configure a base stateset for the rasterizer:
    osg::StateSet* rasterizerSS = _rasterizer->getOrCreateStateSet();

    // Program for rendering with the TileRasterizer
    osg::Program* program = new osg::Program();
    program->addShader(new osg::Shader(osg::Shader::VERTEX, vs));
    program->addShader(new osg::Shader(osg::Shader::FRAGMENT, fs));
    rasterizerSS->setAttribute(program);

#if 1 // blending is great for textures, bad for elevation deltas!!
    // Use normal RGB blending:
    osg::BlendFunc* blendFunc = new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    rasterizerSS->setAttributeAndModes(blendFunc);

    // Use MAX blending for the alpha channel:
    osg::BlendEquation* blendEquation = new osg::BlendEquation(osg::BlendEquation::FUNC_ADD, osg::BlendEquation::RGBA_MAX);
    rasterizerSS->setAttributeAndModes(blendEquation);
#endif

    // Create a placeholder image to display before rasterization is complete
    _placeholder = ImageUtils::createEmptyImage();
#endif
}

Status
DecalImageLayer::openImplementation()
{
    return ImageLayer::openImplementation();
}

void
DecalImageLayer::addedToMap(const Map* map)
{
    ImageLayer::addedToMap(map);
}

void
DecalImageLayer::removedFromMap(const Map* map)
{
    ImageLayer::removedFromMap(map);
}

osg::Node*
DecalImageLayer::getNode() const
{
    // adds the Rasterizer to the scene graph so we can rasterize tiles
    return _rasterizer.get();
}

void
DecalImageLayer::releaseGLObjects(osg::State* state) const
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
DecalImageLayer::resizeGLObjectBuffers(unsigned maxSize)
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
DecalImageLayer::updateTexture(const TileKey& key, osg::Texture2D* texture) const
{
    // INTERNAL function -- assumes mutex is locked.

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
        // Setting a size indicates that this texture is ready. We have to
        // dirty the underlying texture object to regenerate the texture.
        // This sometimes causes flashing; don't know why
        if (texture->getTextureWidth() != getTileSize())
        {
            texture->setTextureSize(getTileSize(), getTileSize());
            texture->setImage(NULL);
            texture->dirtyTextureObject();
        }
    }

    // Schedule the rasterization. We must rasterize each tile
    // even if the decal Group is empty (to generate an empty texture)
    _rasterizer->push(decalGroup.get(), texture, outputExtent);
}

TextureWindow
DecalImageLayer::createTexture(const TileKey& key, ProgressCallback* progress) const
{
    if (getStatus().isError())
    {
        return TextureWindow();
    }

    if (key.getLOD() < getMinLevel())
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
            // The placeholder is necessary because otherwise you can see
            // "junk" data on the terrain while the tile is queued for
            // rasterization.
            // Possible alternate approach: Have the terrain engine detect the
            // fact that the texture is not ready for render and inject a
            // placeholder in its place? This might also help with the flashing?
            texture = new osg::Texture2D(_placeholder.get());
            texture->setDataVariance(osg::Object::DYNAMIC);

            // TODO: Don't set the size until we actually render to a texture...?
            // NOTE: Doing this causes the visual update to be weird (flashing)
            // NOTE: Maybe don't do this.
            texture->setTextureSize(1, 1);

            // review. See if we can use a tighter format
            texture->setSourceFormat(GL_RGBA);
            texture->setSourceType(GL_UNSIGNED_BYTE);
            texture->setInternalFormat(GL_RGBA8);

            texture->setFilter( osg::Texture::MAG_FILTER, osg::Texture::LINEAR );
            texture->setFilter( osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR ); // review this.
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

GeoImage
DecalImageLayer::createImageImplementation(const TileKey& key, ProgressCallback* progress) const
{
    std::vector<Decal> decals;
    std::vector<GeoExtent> outputExtentsInDecalSRS;
    std::vector<GeoExtent> intersections;

    const GeoExtent& outputExtent = key.getExtent();
    Threading::ScopedMutexLock lock(_mutex);

    // thread-safe collection of intersecting decals
    {
        for(std::vector<Decal>::const_iterator i = _decals.begin();
            i != _decals.end();
            ++i)
        {
            const Decal& decal = *i;
            GeoExtent outputExtentInDecalSRS = outputExtent.transform(decal._extent.getSRS());
            GeoExtent intersectionExtent = decal._extent.intersectionSameSRS(outputExtentInDecalSRS);
            if (intersectionExtent.isValid())
            {
                decals.push_back(decal);
                outputExtentsInDecalSRS.push_back(outputExtentInDecalSRS);
                intersections.push_back(intersectionExtent);
            }
        }
    }

    if (decals.empty())
        return GeoImage::INVALID;

    osg::ref_ptr<osg::Image> output = new osg::Image();
    output->allocateImage(getTileSize(), getTileSize(), 1, GL_RGBA, GL_UNSIGNED_BYTE);
    output->setInternalTextureFormat(GL_RGBA8);
    ::memset(output->data(), 0, output->getTotalSizeInBytes());
    ImageUtils::PixelWriter write(output.get());
    ImageUtils::PixelReader readExisting(output.get());

    osg::Vec4 existingValue;
    osg::Vec4 value;

    for(unsigned i=0; i<decals.size(); ++i)
    {
        const Decal& decal = decals[i];

        const GeoExtent& outputExtentInDecalSRS = outputExtentsInDecalSRS[i];
        const GeoExtent& intersection = intersections[i];

        double writeU0 = ((intersection.xMin()-outputExtentInDecalSRS.xMin())/outputExtentInDecalSRS.width());
        double writeU1 = ((intersection.xMax()-outputExtentInDecalSRS.xMin())/outputExtentInDecalSRS.width());

        double writeV0 = ((intersection.yMin()-outputExtentInDecalSRS.yMin())/outputExtentInDecalSRS.height());
        double writeV1 = ((intersection.yMax()-outputExtentInDecalSRS.yMin())/outputExtentInDecalSRS.height());

        double readU0 = ((intersection.xMin()-decal._extent.xMin())/decal._extent.width());
        double readU1 = ((intersection.xMax()-decal._extent.xMin())/decal._extent.width());

        double readV0 = ((intersection.yMin()-decal._extent.yMin())/decal._extent.height());
        double readV1 = ((intersection.yMax()-decal._extent.yMin())/decal._extent.height());

        ImageUtils::PixelReader read(decal._image.get());

        double w = (writeU1-writeU0) * (double)output->s();
        double h = (writeV1-writeV0) * (double)output->t();

        double ustep = 1.0/w;
        double vstep = 1.0/h;

        if (writeU0 > writeU1 || writeV0 > writeV1 || readU0 > readU1 || readV0 > readV1)
        {
            OE_INFO << std::endl
                <<"writeU0="<<writeU0<<", writeU1="<<writeU1
                <<", writeV0="<<writeV0<<", writeV1="<<writeV1
                <<", readU0="<<readU0<<", readU1="<<readU1
                <<", readV0="<<readV0<<", readV1="<<readV1
                <<", w="<<w<<", h="<<h
                <<std::endl;
        }

        for(double v=0.0; v<=1.0; v+=vstep)
        {
            double readv = readV0+v*(readV1-readV0);
            double writev = writeV0+v*(writeV1-writeV0);

            for(double u=0.0; u<=1.0; u+=ustep)
            {
                double readu = readU0+u*(readU1-readU0);
                double writeu = writeU0+u*(writeU1-writeU0);

                readExisting(existingValue, writeu, writev);
                read(value, readu, readv);

                value.r() = value.r()*value.a() + (existingValue.r()*(1.0-value.a()));
                value.g() = value.g()*value.a() + (existingValue.g()*(1.0-value.a()));
                value.b() = value.b()*value.a() + (existingValue.b()*(1.0-value.a()));
                value.a() = osg::maximum(value.a(), existingValue.a());

                write.f(value, writeu, writev);
            }
        }
    }

    std::string k = key.str();
    osgEarth::replaceIn(k,"/","_");
    osgDB::writeImageFile(*output.get(),"images/"+k+".png");

    return GeoImage(output.get(), outputExtent);
}

void
DecalImageLayer::addDecal(const GeoExtent& extent, const osg::Image* image)
{
    Threading::ScopedMutexLock lock(_mutex);

    _decals.resize(_decals.size()+1);
    Decal& decal = _decals.back();
    decal._extent = extent;
    decal._image = image;

#if USE_TEXTURES
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
#endif
}

// Builds the mesh that the rasterizer will render to the tile texture
osg::Node*
DecalImageLayer::buildMesh(const GeoExtent& extent, const osg::Image* image) const
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

    static const GLubyte indices[6] = {0,1,2,2,3,0};
    geom->addPrimitiveSet(new osg::DrawElementsUByte(GL_TRIANGLES, 6, indices));

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
    tex->setMaxAnisotropy(1.0f);
    tex->setResizeNonPowerOfTwoHint(false);
    tex->setUnRefImageDataAfterApply(false);

    osg::StateSet* ss = geom->getOrCreateStateSet();
    ss->setTextureAttribute(0, tex);

    osg::MatrixTransform* mt = new osg::MatrixTransform();
    mt->setMatrix(osg::Matrix::translate(extent.xMin(), extent.yMin(), 0));
    mt->addChild(geom);

    return mt;
}
