/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
* Copyright 2015 Pelican Mapping
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
#include <osgEarth/Lighting>
#include <osg/Program>
#include <osgUtil/CullVisitor>
#include <osgDB/ObjectWrapper>
#include <osgDB/InputStream>
#include <osgDB/OutputStream>

using namespace osgEarth;

#define LC "[Lighting] "

//............................................................................

void
GenerateGL3LightingUniforms::apply(osg::Node& node)
{
    osg::StateSet* stateset = node.getStateSet();
    if (stateset)
    {
        if (_statesets.find(stateset) == _statesets.end())
        {
            const osg::StateSet::RefAttributePair* rap = stateset->getAttributePair(osg::StateAttribute::MATERIAL);
            if (rap)
            {
                osg::Material* material = dynamic_cast<osg::Material*>(rap->first.get());
                if (material)
                {
                    // DYNAMIC variance? Install a callback that will update the material uniforms each frame.
                    if (material->getDataVariance() == material->DYNAMIC)
                    {
                        for (osg::StateSet::ParentList::iterator parent = stateset->getParents().begin();
                             parent != stateset->getParents().end();
                             ++parent)
                        {
                            if (!alreadyInstalled<MaterialGL3UniformGenerator>((*parent)->getCullCallback()))
                            {
                                (*parent)->addCullCallback(new MaterialGL3UniformGenerator());
                            }
                        }
                    }

                    // STATIC variance? hard-code the material uniforms in the stateset.
                    else
                    {
                        MaterialGL3UniformGenerator().generate(stateset, material);
                    }

    #if !defined(OSG_GL_FIXED_FUNCTION_AVAILABLE)
                    // If there's no FFP, we need to replace the Material with a GL3 Material to prevent
                    // error messages on the console.
                    if (dynamic_cast<MaterialGL3*>(material) == 0L)
                    {
                        stateset->setAttributeAndModes(new MaterialGL3(*material), rap->second);
                    }
    #endif
                }


                // mark this stateset as visited.
                _statesets.insert(stateset);
            }
        }
    }
    traverse(node);
}

void
GenerateGL3LightingUniforms::apply(osg::LightSource& lightSource)
{
    if (lightSource.getLight())
    {        
        if (!alreadyInstalled<LightSourceGL3UniformGenerator>(lightSource.getCullCallback()))
        {
            lightSource.addCullCallback(new LightSourceGL3UniformGenerator());
        }

#if !defined(OSG_GL_FIXED_FUNCTION_AVAILABLE)
        // If there's no FFP, we need to replace the Light with a LightGL3 to prevent
        // error messages on the console.
        if (dynamic_cast<MaterialGL3*>(lightSource.getLight()) == 0L)
        {
            lightSource.setLight(new LightGL3(*lightSource.getLight()));
        }
#endif
    }

    apply(static_cast<osg::Node&>(lightSource));
}

//............................................................................

bool
LightSourceGL3UniformGenerator::run(osg::Object* obj, osg::Object* data)
{    
    osg::LightSource* lightSource = dynamic_cast<osg::LightSource*>(obj);
    osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(data);

    if (cv && lightSource && lightSource->getLight())
    {
        osg::Light* light = lightSource->getLight();

        // replace the index with the light number:
        std::string prefix("osg_LightSource[#].");
        prefix.at(16) = (char)('0' + light->getLightNum());

        // Lights are positional state so their location in the scene graph is only important
        // in terms of model transformation, and not in terms of what gets lit.
        // Place these uniforms at the root stateset so they affect the entire graph:
        osg::StateSet* ss = cv->getCurrentRenderStage()->getStateSet();
        if (ss == 0L)
            cv->getCurrentRenderStage()->setStateSet(ss = new osg::StateSet());

        ss->addUniform(new osg::Uniform((prefix + "ambient").c_str(), light->getAmbient()));
        ss->addUniform(new osg::Uniform((prefix + "diffuse").c_str(), light->getDiffuse()));
        ss->addUniform(new osg::Uniform((prefix + "specular").c_str(), light->getSpecular()));

        // add the positional elements:
        const osg::Matrix& mvm = *cv->getModelViewMatrix();
        ss->addUniform(new osg::Uniform((prefix + "position").c_str(), light->getPosition() * mvm));
        osg::Vec3 directionLocal = osg::Matrix::transform3x3(light->getDirection(), mvm);
        ss->addUniform(new osg::Uniform((prefix + "spotDirection").c_str(), directionLocal));

        ss->addUniform(new osg::Uniform((prefix + "spotExponent").c_str(), light->getSpotExponent()));
        ss->addUniform(new osg::Uniform((prefix + "spotCosCutoff").c_str(), cos(light->getSpotExponent())));
        ss->addUniform(new osg::Uniform((prefix + "constantAttenuation").c_str(), light->getConstantAttenuation()));
        ss->addUniform(new osg::Uniform((prefix + "linearAttenuation").c_str(), light->getLinearAttenuation()));
        ss->addUniform(new osg::Uniform((prefix + "quadraticAttenuation").c_str(), light->getQuadraticAttenuation()));

    }
    return traverse(obj, data);
}

//............................................................................

void
MaterialGL3UniformGenerator::generate(osg::StateSet* ss, const osg::Material* m)
{
    ss->addUniform(new osg::Uniform("osg_FrontMaterial.ambient", m->getAmbient(m->FRONT)));
    ss->addUniform(new osg::Uniform("osg_FrontMaterial.diffuse", m->getDiffuse(m->FRONT)));
    ss->addUniform(new osg::Uniform("osg_FrontMaterial.specular", m->getSpecular(m->FRONT)));
    ss->addUniform(new osg::Uniform("osg_FrontMaterial.emission", m->getEmission(m->FRONT)));
    ss->addUniform(new osg::Uniform("osg_FrontMaterial.shininess", m->getShininess(m->FRONT)));

    //TODO: back-face materials
}

bool
MaterialGL3UniformGenerator::run(osg::Object* obj, osg::Object* data)
{
    osg::Node* node = obj->asNode();
    osg::NodeVisitor* nv = data->asNodeVisitor();

    if (node && node->getStateSet() && nv)
    {
        const osg::Material* m = dynamic_cast<const osg::Material*>(node->getStateSet()->getAttribute(osg::StateAttribute::MATERIAL));
        if ( m )
        {
            osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(nv);
    
            osg::StateSet* ss = cv->getCurrentRenderBin()->getStateSet();
            if (!ss) cv->getCurrentRenderBin()->setStateSet(ss = new osg::StateSet());

            generate(ss, m);
        }
    }

    return traverse(node, nv);
}

//............................................................................

void
LightGL3::apply(osg::State& state) const
{
#ifdef OSG_GL_FIXED_FUNCTION_AVAILABLE
    osg::Light::apply(state);
#endif
}

// Serializer for LightGL3.
// The odd namespace name is here because this macro cannot be used more than once
// in the same namespace in the same cpp file.
namespace osgEarth_TEMP1
{
    REGISTER_OBJECT_WRAPPER( 
        LightGL3,
        new osgEarth::LightGL3,
        osgEarth::LightGL3,
        "osg::Object osg::StateAttribute osg::Light osgEarth::LightGL3" ) { }
}

//............................................................................

void
MaterialGL3::apply(osg::State& state) const
{
#ifdef OSG_GL_FIXED_FUNCTION_AVAILABLE
    osg::Light::apply(state);
#endif
}

// Serializer for MaterialGL3.
// The odd namespace name is here because this macro cannot be used more than once
// in the same namespace in the same cpp file.
namespace osgEarth_TEMP2
{
    REGISTER_OBJECT_WRAPPER(
        MaterialGL3,
        new osgEarth::MaterialGL3,
        osgEarth::MaterialGL3,
        "osg::Object osg::StateAttribute osg::Material osgEarth::MaterialGL3") { }
}