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

// prefix to use for uniforms.
#define UPREFIX "osg_"

//............................................................................

GenerateGL3LightingUniforms::GenerateGL3LightingUniforms() :
osg::NodeVisitor(TRAVERSE_ALL_CHILDREN)
{
    setNodeMaskOverride(~0);
}

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
                    osg::Material* mat = material;

    #if !defined(OSG_GL_FIXED_FUNCTION_AVAILABLE)
                    // If there's no FFP, we need to replace the Material with a GL3 Material to prevent
                    // error messages on the console.
                    if (dynamic_cast<MaterialGL3*>(material) == 0L)
                    {
                        mat = new MaterialGL3(*material), rap->second;
                        stateset->setAttributeAndModes(mat);    
                    }
    #endif

                    // Install the MaterialCallback so uniforms are updated.
                    if (!mat->getUpdateCallback())
                    {
                        mat->setUpdateCallback(new MaterialCallback());
                    }

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
        std::string prefix(UPREFIX "LightSource[#].");
        prefix.at(prefix.length()-3) = (char)('0' + light->getLightNum());

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
        directionLocal.normalize();
        ss->addUniform(new osg::Uniform((prefix + "spotDirection").c_str(), directionLocal));

        ss->addUniform(new osg::Uniform((prefix + "spotExponent").c_str(), light->getSpotExponent()));
        ss->addUniform(new osg::Uniform((prefix + "spotCutoff").c_str(), light->getSpotCutoff()));
        ss->addUniform(new osg::Uniform((prefix + "spotCosCutoff").c_str(), cos(light->getSpotCutoff())));
        ss->addUniform(new osg::Uniform((prefix + "constantAttenuation").c_str(), light->getConstantAttenuation()));
        ss->addUniform(new osg::Uniform((prefix + "linearAttenuation").c_str(), light->getLinearAttenuation()));
        ss->addUniform(new osg::Uniform((prefix + "quadraticAttenuation").c_str(), light->getQuadraticAttenuation()));

        LightGL3* lightGL3 = dynamic_cast<LightGL3*>(light);
        bool enabled = lightGL3 ? lightGL3->getEnabled() : true;
        ss->addUniform(new osg::Uniform((prefix + "enabled").c_str(), enabled));
        
        osg::Uniform* fsu = ss->getOrCreateUniform("oe_lighting_framestamp", osg::Uniform::UNSIGNED_INT);
        unsigned fs;
        fsu->get(fs);

        osg::Uniform* numLights = ss->getOrCreateUniform("osg_NumLights", osg::Uniform::INT);

        if (fs != cv->getFrameStamp()->getFrameNumber())
        {
            numLights->set(1);
            fsu->set(cv->getFrameStamp()->getFrameNumber());
        }
        else
        {
            int value;
            numLights->get(value);
            numLights->set(value + 1);
        }
    }
    return traverse(obj, data);
}

//............................................................................
void MaterialCallback::operator() (osg::StateAttribute* attr, osg::NodeVisitor* nv)
{
    osg::Material* material = static_cast<osg::Material*>(attr);
    for (unsigned int i = 0; i < attr->getNumParents(); i++)
    {
        osg::StateSet* stateSet = attr->getParent(i);

        stateSet->getOrCreateUniform(UPREFIX "FrontMaterial.ambient", osg::Uniform::FLOAT_VEC4)->set(material->getAmbient(osg::Material::FRONT));
        stateSet->getOrCreateUniform(UPREFIX "FrontMaterial.diffuse", osg::Uniform::FLOAT_VEC4)->set(material->getDiffuse(osg::Material::FRONT));
        stateSet->getOrCreateUniform(UPREFIX "FrontMaterial.specular", osg::Uniform::FLOAT_VEC4)->set(material->getSpecular(osg::Material::FRONT));
        stateSet->getOrCreateUniform(UPREFIX "FrontMaterial.emission", osg::Uniform::FLOAT_VEC4)->set(material->getEmission(osg::Material::FRONT));
        stateSet->getOrCreateUniform(UPREFIX "FrontMaterial.shininess", osg::Uniform::FLOAT)->set(material->getShininess(osg::Material::FRONT));

        //TODO: back-face materials
    }
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
        "osg::Object osg::StateAttribute osg::Light osgEarth::LightGL3")
    {
        ADD_BOOL_SERIALIZER(Enabled, true);
    }
}

//............................................................................

void
MaterialGL3::apply(osg::State& state) const
{
#ifdef OSG_GL_FIXED_FUNCTION_AVAILABLE
    osg::Material::apply(state);
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