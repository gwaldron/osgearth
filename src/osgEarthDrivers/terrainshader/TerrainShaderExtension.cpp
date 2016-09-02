/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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
#include "TerrainShaderExtension"

#include <osgEarth/TerrainEffect>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/MapNode>
#include <osgEarth/VirtualProgram>
#include <osgEarth/ShaderLoader>
#include <osgEarth/ImageUtils>

#include <osg/Texture2D>
#include <osg/Texture2DArray>

using namespace osgEarth;
using namespace osgEarth::TerrainShader;

#define LC "[TerrainShaderExtension] "

namespace
{
    class GLSLEffect : public osgEarth::TerrainEffect
    {
    public:
        GLSLEffect(const TerrainShaderOptions& options,
                   const osgDB::Options*       dbOptions ) :
            _options(options), _dbOptions(dbOptions)
        {
            const std::vector<TerrainShaderOptions::Code>& code = _options.code();

            for(unsigned i=0; i<code.size(); ++i)
            {
		std::ostringstream oss;
		oss << i;
                std::string fn = code[i]._uri.isSet() ? code[i]._uri->full() : "$code." + oss.str();
                _package.add( fn, code[i]._source );
            }
        }

        void onInstall(TerrainEngineNode* engine)
        {
            if ( !engine ) return;

            osg::StateSet* stateSet = engine->getSurfaceStateSet();
            
            VirtualProgram* vp = VirtualProgram::getOrCreate(stateSet);
            _package.loadAll( vp, _dbOptions.get() );


            const std::vector<TerrainShaderOptions::Sampler>& samplers = _options.samplers();
            for(int i=0; i<samplers.size(); ++i)
            {
                if ( !samplers[i]._name.empty() )
                {
                    if ( samplers[i]._uris.size() == 1 ) // Texture2D
                    {
                        int unit;    
                        engine->getResources()->reserveTextureImageUnit(unit, "TerrainShader sampler");
                        if ( unit >= 0 )
                        {
                            osg::Image* image = samplers[i]._uris[0].getImage(_dbOptions.get());
                            if ( image )
                            {
                                osg::Texture2D* tex = new osg::Texture2D(image);
                                tex->setFilter(tex->MIN_FILTER, tex->NEAREST_MIPMAP_LINEAR);
                                tex->setFilter(tex->MAG_FILTER, tex->LINEAR);
                                tex->setWrap  (tex->WRAP_S, tex->REPEAT);
                                tex->setWrap  (tex->WRAP_T, tex->REPEAT);
                                tex->setUnRefImageDataAfterApply( true );
                                tex->setMaxAnisotropy( 4.0 );
                                tex->setResizeNonPowerOfTwoHint( false );

                                stateSet->setTextureAttribute(unit, tex);
                                stateSet->addUniform(new osg::Uniform(samplers[i]._name.c_str(), unit));
                            }
                        }
                        else
                        {
                            OE_WARN << LC << "Failed to allocate a texture image unit for this terrain shader sampler!\n";
                        }
                    }

                    else if ( samplers[i]._uris.size() > 1 ) // Texture2DArray
                    {
                        int unit;    
                        engine->getResources()->reserveTextureImageUnit(unit, "TerrainShader sampler array");
                        if ( unit >= 0 )
                        {
                            osg::Texture2DArray* tex = new osg::Texture2DArray();
                            tex->setTextureSize(512, 512, samplers[i]._uris.size());
                            tex->setTextureDepth( samplers[i]._uris.size() );

                            for( int j=0; j<samplers[i]._uris.size(); ++j )
                            {
                                const URI& uri = samplers[i]._uris[j];

                                osg::ref_ptr<osg::Image> image = uri.getImage(_dbOptions.get());
                                if ( image )
                                {
                                    if ( image->s() != 512 || image->t() != 512 )
                                    {
                                        osg::ref_ptr<osg::Image> resizedImage;
                                        ImageUtils::resizeImage(image.get(), 512, 512, resizedImage);
                                        image = resizedImage.get();
                                    }

                                    OE_INFO << LC << "   Added image from \"" << uri.full() << "\"\n";
                                    tex->setImage(i, image);
                                    tex->setFilter(tex->MIN_FILTER, tex->NEAREST_MIPMAP_LINEAR);
                                    tex->setFilter(tex->MAG_FILTER, tex->LINEAR);
                                    tex->setWrap  (tex->WRAP_S, tex->CLAMP_TO_EDGE);
                                    tex->setWrap  (tex->WRAP_T, tex->CLAMP_TO_EDGE);
                                    tex->setUnRefImageDataAfterApply( true );
                                    //tex->setMaxAnisotropy( 4.0 );
                                    tex->setResizeNonPowerOfTwoHint( false );

                                    stateSet->setTextureAttribute(unit, tex);
                                    stateSet->addUniform(new osg::Uniform(samplers[i]._name.c_str(), unit));
                                }
                            }
                        }
                        else
                        {
                            OE_WARN << LC << "Failed to allocate a texture image unit for this terrain shader sampler!\n";
                        }
                    }
                }
            }

            const std::vector<TerrainShaderOptions::Uniform>& uniforms = _options.uniforms();
            for(int i=0; i<uniforms.size(); ++i)
            {
                if ( !uniforms[i]._name.empty() && uniforms[i]._value.isSet() )
                {
                    osg::Uniform* u = new osg::Uniform(uniforms[i]._name.c_str(), (float)uniforms[i]._value.get());
                    stateSet->addUniform( u );
                }
            }
        }

        void onUninstall(TerrainEngineNode* engine)
        {
            if ( engine )
            {
                if ( _options.landCoverGroup().isSet() )
                {
                    //TODO
                    //engine->removeLandCoverGroup( _options.landCoverGroup().get() );
                }
            }
        }

        const TerrainShaderOptions              _options;
        ShaderPackage                           _package;
        osg::ref_ptr<const osgDB::Options>      _dbOptions;
    };
}


REGISTER_OSGEARTH_EXTENSION( osgearth_terrainshader, TerrainShaderExtension )


TerrainShaderExtension::TerrainShaderExtension()
{
    //nop
}

TerrainShaderExtension::TerrainShaderExtension(const TerrainShaderOptions& options) :
TerrainShaderOptions( options )
{
    //nop
}

TerrainShaderExtension::~TerrainShaderExtension()
{
    //nop
}

void
TerrainShaderExtension::setDBOptions(const osgDB::Options* dbOptions)
{
    _dbOptions = dbOptions;
}

bool
TerrainShaderExtension::connect(MapNode* mapNode)
{
    if ( !mapNode )
    {
        OE_WARN << LC << "Illegal: MapNode cannot be null." << std::endl;
        return false;
    }
    _effect = new GLSLEffect( *this, _dbOptions.get() );
    mapNode->getTerrainEngine()->addEffect( _effect.get() );
    
    OE_INFO << LC << "Installed.\n";

    return true;
}

bool
TerrainShaderExtension::disconnect(MapNode* mapNode)
{
    if ( mapNode && _effect.valid() )
    {
        mapNode->getTerrainEngine()->removeEffect( _effect.get() );
        _effect = 0L;
    }

    return true;
}

