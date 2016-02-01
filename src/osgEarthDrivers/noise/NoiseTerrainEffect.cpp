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
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/
#include "NoiseTerrainEffect"

#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/ImageUtils>
#include <osgEarthUtil/SimplexNoise>

#include <osgDB/WriteFile>

#define LC "[Noise] "

#define SAMPLER_NAME "oe_noise_tex"
//#define MATRIX_NAME  "oe_nmap_normalTexMatrix"

using namespace osgEarth;
using namespace osgEarth::Noise;

NoiseTerrainEffect::NoiseTerrainEffect(const NoiseOptions& options) :
_options     ( options ),
_texImageUnit( -1 )
{
    _tex = createNoiseTexture();
}

void
NoiseTerrainEffect::onInstall(TerrainEngineNode* engine)
{
    if ( engine )
    {
        engine->getResources()->reserveTextureImageUnit(_texImageUnit, "Noise");
        if ( _texImageUnit >= 0 )
        {        
            osg::StateSet* stateset = engine->getOrCreateStateSet();
            stateset->setTextureAttribute( _texImageUnit, _tex.get() );
            stateset->addUniform( new osg::Uniform(SAMPLER_NAME, _texImageUnit) );
            OE_INFO << LC << "Noise generator installed!\n";
        }
        else
        {
            OE_WARN << LC << "No texture image units available; noise disabled.\n";
        }
    }
}


void
NoiseTerrainEffect::onUninstall(TerrainEngineNode* engine)
{
    if ( engine && _texImageUnit >= 0 )
    {
        osg::StateSet* stateset = engine->getStateSet();
        if ( stateset )
        {
            stateset->removeUniform( SAMPLER_NAME );
            stateset->removeTextureAttribute( _texImageUnit, _tex.get() );
        }

        engine->getResources()->releaseTextureImageUnit( _texImageUnit );
        _texImageUnit = -1;
    }
}


osg::Texture*
NoiseTerrainEffect::createNoiseTexture() const
{
    const int size  = (int)osg::clampBetween(_options.size().get(), 1u, 16384u);
    const int chans = (int)osg::clampBetween(_options.numChannels().get(), 1u, 4u);

    GLenum type = chans > 2 ? GL_RGBA : GL_LUMINANCE;
    
    osg::Image* image = new osg::Image();
    image->allocateImage(size, size, 1, type, GL_UNSIGNED_BYTE);

    // 0 = rocky mountains..
    // 1 = warping...
    const float F[4] = { 4.0f, 16.0f, 4.0f, 8.0f };
    const float P[4] = { 0.8f,  0.6f, 0.8f, 0.9f };
    const float L[4] = { 2.2f,  1.7f, 3.0f, 4.0f };
    
    for(int k=0; k<chans; ++k)
    {
        // Configure the noise function:
        osgEarth::Util::SimplexNoise noise;
        noise.setNormalize( true );
        noise.setRange( 0.0, 1.0 );
        noise.setFrequency( F[k] );
        noise.setPersistence( P[k] );
        noise.setLacunarity( L[k] );
        noise.setOctaves( 8 );

        float nmin = 10.0f;
        float nmax = -10.0f;

        // write repeating noise to the image:
        ImageUtils::PixelReader read ( image );
        ImageUtils::PixelWriter write( image );
        for(int t=0; t<size; ++t)
        {
            double rt = (double)t/size;
            for(int s=0; s<size; ++s)
            {
                double rs = (double)s/(double)size;

                double n = noise.getTiledValue(rs, rt);

                n = osg::clampBetween(n, 0.0, 1.0);

                if ( n < nmin ) nmin = n;
                if ( n > nmax ) nmax = n;
                osg::Vec4f v = read(s, t);
                v[k] = n;
                write(v, s, t);
            }
        }
   
        // histogram stretch to [0..1]
        for(int x=0; x<size*size; ++x)
        {
            int s = x%size, t = x/size;
            osg::Vec4f v = read(s, t);
            v[k] = osg::clampBetween((v[k]-nmin)/(nmax-nmin), 0.0f, 1.0f);
            write(v, s, t);
        }

        //OE_INFO << LC << "Noise: MIN = " << nmin << "; MAX = " << nmax << "\n";
    }

#if 0
    std::string filename("noise.png");
    osgDB::writeImageFile(*image, filename);
    OE_NOTICE << LC << "Wrote noise texture to " << filename << "\n";
#endif

    // make a texture:
    osg::Texture2D* tex = new osg::Texture2D( image );
    tex->setWrap(tex->WRAP_S, tex->REPEAT);
    tex->setWrap(tex->WRAP_T, tex->REPEAT);
    tex->setFilter(tex->MIN_FILTER, tex->LINEAR_MIPMAP_LINEAR);
    tex->setFilter(tex->MAG_FILTER, tex->LINEAR);
    tex->setMaxAnisotropy( 4.0f );
    tex->setUnRefImageDataAfterApply( true );

    return tex;
}
