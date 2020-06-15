
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2008-2016 Pelican Mapping
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
#include <osgEarth/Elevation>
#include <osgEarth/Registry>

using namespace osgEarth;


namespace
{
    // octohodreal normal packing
    osg::Vec2 packNormal(const osg::Vec3& v)
    {
        osg::Vec2 p;
        float d = 1.0/(fabs(v.x())+fabs(v.y())+fabs(v.z()));
        p.x() = v.x() * d;
        p.y() = v.y() * d;

        if (v.z() < 0.0)
        {
            p.x() = (1.0 - fabs(p.y())) * (p.x() >= 0.0? 1.0 : -1.0);
            p.y() = (1.0 - fabs(p.x())) * (p.y() >= 0.0? 1.0 : -1.0);
        }

        p.x() = 0.5f*(p.x()+1.0f);
        p.y() = 0.5f*(p.y()+1.0f);

        return p;
    }
}

ElevationTexture::ElevationTexture(const GeoHeightField& in_hf, const NormalMap* normalMap) :
    _extent(in_hf.getExtent())
{
    if (in_hf.valid())
    {
        const osg::HeightField* hf = in_hf.getHeightField();
        osg::Vec4 value;

        osg::Image* heights = new osg::Image();
        heights->allocateImage(hf->getNumColumns(), hf->getNumRows(), 1, GL_RED, GL_FLOAT);
        heights->setInternalTextureFormat(GL_R32F);

        ImageUtils::PixelWriter write(heights);
        // TODO: speed this up since we know the format
        for(unsigned row=0; row<hf->getNumRows(); ++row)
        {
            for(unsigned col=0; col<hf->getNumColumns(); ++col)
            {
                value.r() = hf->getHeight(col, row);
                write(value, col, row);
            }
        }
        setImage(heights);

        setDataVariance(osg::Object::STATIC);
        setInternalFormat(GL_R32F);
        setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
        setFilter(osg::Texture::MIN_FILTER, osg::Texture::NEAREST);
        setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
        setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
        setResizeNonPowerOfTwoHint(false);
        setMaxAnisotropy(1.0f);
        setUnRefImageDataAfterApply(Registry::instance()->unRefImageDataAfterApply().get());

        if (normalMap)
        {
            osg::Image* normals = new osg::Image();
            normals->allocateImage(normalMap->s(), normalMap->t(), 1, GL_RG, GL_UNSIGNED_BYTE);
            normals->setInternalTextureFormat(GL_RG8);
            ImageUtils::PixelWriter writeNormal(normals);

            osg::Vec2 temp;
            for(int t=0; t<normalMap->t(); ++t)
            {
                for(int s=0; s<normalMap->s(); ++s)
                {
                    temp = packNormal(normalMap->getNormal(s, t));
                    value.r() = temp.x(), value.g() = temp.y();
                    // TODO: won't actually be written until we make the format GL_RGB
                    // but we need to rewrite the curvature generator first
                    value.b() = 0.5f*(1.0f+normalMap->getCurvature(s, t));
                    writeNormal(value, s, t);
                }
            }
            _normalTex = new osg::Texture2D(normals);

            _normalTex->setInternalFormat(GL_RG8);
            _normalTex->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
            _normalTex->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR);
            _normalTex->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
            _normalTex->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
            _normalTex->setResizeNonPowerOfTwoHint(false);
            _normalTex->setMaxAnisotropy(1.0f);
            _normalTex->setUnRefImageDataAfterApply(Registry::instance()->unRefImageDataAfterApply().get());
        }

        _read.setTexture(this);
        _read.setSampleAsTexture(false);

        _resolution = Distance(
            getExtent().height() / ((double)(getImage(0)->s()-1)),
            getExtent().getSRS()->getUnits());
    }
}

ElevationTexture::~ElevationTexture()
{
    //nop
}

ElevationSample
ElevationTexture::getElevation(double x, double y) const
{
    double u = (x - getExtent().xMin()) / getExtent().width();
    double v = (y - getExtent().yMin()) / getExtent().height();

    return getElevationUV(u, v);
}

ElevationSample
ElevationTexture::getElevationUV(double u, double v) const
{
    osg::Vec4 value;
    _read(value, u, v);
    return ElevationSample(Distance(value.r(),Units::METERS), _resolution);
}