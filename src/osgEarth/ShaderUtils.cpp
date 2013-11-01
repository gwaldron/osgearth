/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
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
#include <osgEarth/ShaderUtils>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/CullingUtils>
#include <list>

using namespace osgEarth;

//------------------------------------------------------------------------

namespace 
{
#if !defined(OSG_GL_FIXED_FUNCTION_AVAILABLE)
    static bool s_NO_FFP = true;
#else
    static bool s_NO_FFP = false;
#endif


    typedef std::list<const osg::StateSet*> StateSetStack;

    static osg::StateAttribute::GLModeValue 
    getModeValue(const StateSetStack& statesetStack, osg::StateAttribute::GLMode mode)
    {
        osg::StateAttribute::GLModeValue base_val = osg::StateAttribute::ON;

        for(StateSetStack::const_iterator itr = statesetStack.begin();
            itr != statesetStack.end();
            ++itr)
        {
            osg::StateAttribute::GLModeValue val = (*itr)->getMode(mode);

            if ( (val & osg::StateAttribute::INHERIT) == 0 )
            {

                if ((val & osg::StateAttribute::PROTECTED)!=0 ||
                    (base_val & osg::StateAttribute::OVERRIDE)==0)
                {
                    base_val = val;
                }
            }
        }
        return base_val;
    }
    
    static const osg::Light*
    getLightByID(const StateSetStack& statesetStack, int id)
    {
        const osg::Light* base_light = NULL;
        osg::StateAttribute::GLModeValue base_val = osg::StateAttribute::ON;
        
        for(StateSetStack::const_iterator itr = statesetStack.begin();
            itr != statesetStack.end();
            ++itr)
        {
            
            osg::StateAttribute::GLModeValue val = (*itr)->getMode(GL_LIGHT0+id);

            //if ( (val & osg::StateAttribute::INHERIT) == 0 )
            {
            //    if ((val & osg::StateAttribute::PROTECTED)!=0 ||
            //        (base_val & osg::StateAttribute::OVERRIDE)==0)
                {
                    base_val = val;
                    const osg::StateAttribute* lightAtt = (*itr)->getAttribute(osg::StateAttribute::LIGHT, id);
                    if(lightAtt){
                        const osg::Light* asLight = dynamic_cast<const osg::Light*>(lightAtt);
                        if(val){
                            base_light = asLight;
                        }
                    }
                }
            }
            
        }
        return base_light;
    }
    
    static const osg::Material*
    getFrontMaterial(const StateSetStack& statesetStack)
    {
        const osg::Material* base_material = NULL;
        osg::StateAttribute::GLModeValue base_val = osg::StateAttribute::ON;
        
        for(StateSetStack::const_iterator itr = statesetStack.begin();
            itr != statesetStack.end();
            ++itr)
        {
            
            osg::StateAttribute::GLModeValue val = (*itr)->getMode(GL_DIFFUSE);//?
            
            //if ( (val & osg::StateAttribute::INHERIT) == 0 )
            {
            //    if ((val & osg::StateAttribute::PROTECTED)!=0 ||
             //       (base_val & osg::StateAttribute::OVERRIDE)==0)
                {
                    base_val = val;
                    const osg::StateAttribute* materialAtt = (*itr)->getAttribute(osg::StateAttribute::MATERIAL);
                    if(materialAtt){
                        const osg::Material* asMaterial = dynamic_cast<const osg::Material*>(materialAtt);
                        if(val){
                            base_material = asMaterial;
                        }
                    }
                }
            }
            
        }
        return base_material;
    }
}

//------------------------------------------------------------------------

namespace State_Utils
{
    // Code borrowed from osg::State.cpp
    bool replace(std::string& str, const std::string& original_phrase, const std::string& new_phrase)
    {
        bool replacedStr = false;
        std::string::size_type pos = 0;
        while((pos=str.find(original_phrase, pos))!=std::string::npos)
        {
            std::string::size_type endOfPhrasePos = pos+original_phrase.size();
            if (endOfPhrasePos<str.size())
            {
                char c = str[endOfPhrasePos];
                if ((c>='0' && c<='9') ||
                    (c>='a' && c<='z') ||
                    (c>='A' && c<='Z') ||
                    (c==']'))
                {
                    pos = endOfPhrasePos;
                    continue;
                }
            }

            replacedStr = true;
            str.replace(pos, original_phrase.size(), new_phrase);
        }
        return replacedStr;
    }

    void replaceAndInsertDeclaration(std::string& source, std::string::size_type declPos, const std::string& originalStr, const std::string& newStr, const std::string& declarationPrefix, const std::string& declarationSuffix ="")
    {
        if (replace(source, originalStr, newStr))
        {
            source.insert(declPos, declarationPrefix + newStr + declarationSuffix + std::string(";\n"));
        }
    }
}

void
ShaderPreProcessor::run(osg::Shader* shader)
{
    // only runs for non-FFP (GLES, GL3+, etc.)

    if ( s_NO_FFP && shader )
    {
        std::string source = shader->getShaderSource();

        // find the first legal insertion point for replacement declarations. GLSL requires that nothing
        // precede a "#verson" compiler directive, so we must insert new declarations after it.
        std::string::size_type declPos = source.rfind( "#version " );
        if ( declPos != std::string::npos )
        {
            // found the string, now find the next linefeed and set the insertion point after it.
            declPos = source.find( '\n', declPos );
            declPos = declPos != std::string::npos ? declPos+1 : source.length();
        }
        else
        {
            declPos = 0;
        }

        int maxLights = Registry::capabilities().getMaxLights();

        for( int i=0; i<maxLights; ++i )
        {
            State_Utils::replaceAndInsertDeclaration(
                source, declPos,
                Stringify() << "gl_LightSource[" << i << "]",
                Stringify() << "osg_LightSource" << i,
                Stringify() 
                    << osg_LightSourceParameters::glslDefinition() << "\n"
                    << "uniform osg_LightSourceParameters " );

            State_Utils::replaceAndInsertDeclaration(
                source, declPos,
                Stringify() << "gl_FrontLightProduct[" << i << "]", 
                Stringify() << "osg_FrontLightProduct" << i,
                Stringify()
                    << osg_LightProducts::glslDefinition() << "\n"
                    << "uniform osg_LightProducts " );
        }

        shader->setShaderSource( source );
    }
}

//------------------------------------------------------------------------

osg_LightProducts::osg_LightProducts(int id)
{
    std::stringstream uniNameStream;
    uniNameStream << "osg_FrontLightProduct" << id; //[" << id << "]";
    std::string uniName = uniNameStream.str();

    ambient = new osg::Uniform(osg::Uniform::FLOAT_VEC4, uniName+".ambient"); // vec4
    diffuse = new osg::Uniform(osg::Uniform::FLOAT_VEC4, uniName+".diffuse"); // vec4
    specular = new osg::Uniform(osg::Uniform::FLOAT_VEC4, uniName+".specular"); // vec4
}

std::string 
osg_LightProducts::glslDefinition()
{
    //Note: it's important that there be NO linefeeds in here, since that would
    // break the shader merging code.
    return
        "struct osg_LightProducts {"
        " vec4 ambient;"
        " vec4 diffuse;"
        " vec4 specular;"
        " };";
}

//------------------------------------------------------------------------

osg_LightSourceParameters::osg_LightSourceParameters(int id)
    : _frontLightProduct(id)
{
    std::stringstream uniNameStream;
    uniNameStream << "osg_LightSource" << id; // [" << id << "]";
    std::string uniName = uniNameStream.str();
    
    ambient = new osg::Uniform(osg::Uniform::FLOAT_VEC4, uniName+".ambient"); // vec4
    diffuse = new osg::Uniform(osg::Uniform::FLOAT_VEC4, uniName+".diffuse"); // vec4
    specular = new osg::Uniform(osg::Uniform::FLOAT_VEC4, uniName+".specular"); // vec4
    position = new osg::Uniform(osg::Uniform::FLOAT_VEC4, uniName+".position"); // vec4
    halfVector = new osg::Uniform(osg::Uniform::FLOAT_VEC4, uniName+".halfVector"); // vec4
    spotDirection = new osg::Uniform(osg::Uniform::FLOAT_VEC3, uniName+".spotDirection"); // vec3
    spotExponent = new osg::Uniform(osg::Uniform::FLOAT, uniName+".spotExponent"); // float
    spotCutoff = new osg::Uniform(osg::Uniform::FLOAT, uniName+".spotCutoff"); // float
    spotCosCutoff = new osg::Uniform(osg::Uniform::FLOAT, uniName+".spotCosCutoff"); // float
    constantAttenuation = new osg::Uniform(osg::Uniform::FLOAT, uniName+".constantAttenuation"); // float
    linearAttenuation = new osg::Uniform(osg::Uniform::FLOAT, uniName+".linearAttenuation"); // float
    quadraticAttenuation = new osg::Uniform(osg::Uniform::FLOAT, uniName+".quadraticAttenuation"); // float
}

void osg_LightSourceParameters::setUniformsFromOsgLight(const osg::Light* light, osg::Matrix viewMatrix, const osg::Material* frontMat)
{
    if(light){
        ambient->set(light->getAmbient());
        diffuse->set(light->getDiffuse());
        specular->set(light->getSpecular());
        
        osg::Vec4 eyeLightPos = light->getPosition()*viewMatrix;
        position->set(eyeLightPos);
       
        // compute half vec
        osg::Vec4 normPos = eyeLightPos;
        normPos.normalize();
        osg::Vec4 halfVec4 = normPos + osg::Vec4(0,0,1,0);
        halfVec4.normalize();
        halfVector->set(halfVec4);
        
        spotDirection->set(light->getDirection()*viewMatrix);
        spotExponent->set(light->getSpotExponent());
        spotCutoff->set(light->getSpotCutoff());
        //need to compute cosCutOff
        //spotCosCutoff->set(light->get)
        constantAttenuation->set(light->getConstantAttenuation());
        linearAttenuation->set(light->getLinearAttenuation());
        quadraticAttenuation->set(light->getQuadraticAttenuation());
        
        //front product
        if(frontMat){
             osg::Vec4 frontAmbient = frontMat->getAmbient(osg::Material::FRONT);
             osg::Vec4 frontDiffuse = frontMat->getDiffuse(osg::Material::FRONT);
             osg::Vec4 frontSpecular = frontMat->getSpecular(osg::Material::FRONT);
            _frontLightProduct.ambient->set(osg::Vec4(light->getAmbient().x() * frontAmbient.x(),
                                                      light->getAmbient().y() * frontAmbient.y(),
                                                      light->getAmbient().z() * frontAmbient.z(),
                                                      light->getAmbient().w() * frontAmbient.w()));
            
            _frontLightProduct.diffuse->set(osg::Vec4(light->getDiffuse().x() * frontDiffuse.x(),
                                                      light->getDiffuse().y() * frontDiffuse.y(),
                                                      light->getDiffuse().z() * frontDiffuse.z(),
                                                      light->getDiffuse().w() * frontDiffuse.w()));
            
            _frontLightProduct.specular->set(osg::Vec4(light->getSpecular().x() * frontSpecular.x(),
                                                      light->getSpecular().y() * frontSpecular.y(),
                                                      light->getSpecular().z() * frontSpecular.z(),
                                                      light->getSpecular().w() * frontSpecular.w()));
        }
    }
}

void osg_LightSourceParameters::applyState(osg::StateSet* stateset)
{
    stateset->addUniform(ambient.get());
    stateset->addUniform(diffuse.get());
    stateset->addUniform(specular.get());
    stateset->addUniform(position.get());
    stateset->addUniform(halfVector.get());
    stateset->addUniform(spotDirection.get());
    stateset->addUniform(spotExponent.get());
    stateset->addUniform(spotCutoff.get());
    stateset->addUniform(spotCosCutoff.get());
    stateset->addUniform(constantAttenuation.get());
    stateset->addUniform(linearAttenuation.get());
    stateset->addUniform(quadraticAttenuation.get());
    
    //apply front light product
    stateset->addUniform(_frontLightProduct.ambient.get());
    stateset->addUniform(_frontLightProduct.diffuse.get());
    stateset->addUniform(_frontLightProduct.specular.get());
}

std::string
osg_LightSourceParameters::glslDefinition()
{
    //Note: it's important that there be NO linefeeds in here, since that would
    // break the shader merging code.
    return
        "struct osg_LightSourceParameters {"
        " vec4 ambient;"
        " vec4 diffuse;"
        " vec4 specular;"
        " vec4 position;"
        " vec4 halfVector;"
        " vec3 spotDirection;"
        " float spotExponent;"
        " float spotCutoff;"
        " float spotCosCutoff;"
        " float constantAttenuation;"
        " float linearAttenuation;"
        " float quadraticAttenuation;"
        " };";
}

//------------------------------------------------------------------------


#undef LC
#define LC "[UpdateLightingUniformHelper] "

UpdateLightingUniformsHelper::UpdateLightingUniformsHelper( bool useUpdateTrav ) :
_lightingEnabled( true ),
_dirty          ( true ),
_applied        ( false ),
_useUpdateTrav  ( useUpdateTrav )
{
    _maxLights = Registry::instance()->getCapabilities().getMaxLights();

    _lightEnabled = new bool[ _maxLights ];
    if ( _maxLights > 0 ){
        _lightEnabled[0] = true;
        //allocate light
        _osgLightSourceParameters.push_back(osg_LightSourceParameters(0));
    }
    for(int i=1; i<_maxLights; ++i ){
        _lightEnabled[i] = true;
        _osgLightSourceParameters.push_back(osg_LightSourceParameters(i));
    }

    _lightingEnabledUniform = new osg::Uniform( osg::Uniform::BOOL, "oe_mode_GL_LIGHTING" );
    _lightEnabledUniform    = new osg::Uniform( osg::Uniform::BOOL, "oe_mode_GL_LIGHT", _maxLights );

    if ( !_useUpdateTrav )
    {
        // setting the data variance the DYNAMIC makes it safe to change the uniform values
        // during the CULL traversal.
        _lightingEnabledUniform->setDataVariance( osg::Object::DYNAMIC );
        _lightEnabledUniform->setDataVariance( osg::Object::DYNAMIC );
    }
}

UpdateLightingUniformsHelper::~UpdateLightingUniformsHelper()
{
    delete [] _lightEnabled;
}

void
UpdateLightingUniformsHelper::cullTraverse( osg::Node* node, osg::NodeVisitor* nv )
{
    osgUtil::CullVisitor* cv = Culling::asCullVisitor(nv);
    if ( cv )
    {
        StateSetStack stateSetStack;

        if ( node->getStateSet() )
            stateSetStack.push_front( node->getStateSet() );

        osgUtil::StateGraph* sg = cv->getCurrentStateGraph();
        while( sg )
        {
            const osg::StateSet* stateset = sg->getStateSet();
            if (stateset)
            {
                stateSetStack.push_front(stateset);
            }                
            sg = sg->_parent;
        }

        // Update the overall lighting-enabled value:
        bool lightingEnabled =
            ( getModeValue(stateSetStack, GL_LIGHTING) & osg::StateAttribute::ON ) != 0;

        if ( lightingEnabled != _lightingEnabled || !_applied )
        {
            _lightingEnabled = lightingEnabled;
            if ( _useUpdateTrav )
                _dirty = true;
            else
                _lightingEnabledUniform->set( _lightingEnabled );
        }

        osg::View* view = cv->getCurrentCamera()->getView();
        if ( view )
        {
            osg::Light* light = view->getLight();
            if ( light )
            {
                const osg::Material* material = getFrontMaterial(stateSetStack);
                _osgLightSourceParameters[0].setUniformsFromOsgLight(light, cv->getCurrentCamera()->getViewMatrix(), material);
            }
        }

        else
        {
            // Update the list of enabled lights:
            for( int i=0; i < _maxLights; ++i )
            {
                bool enabled =
                    ( getModeValue( stateSetStack, GL_LIGHT0 + i ) & osg::StateAttribute::ON ) != 0;
                
                const osg::Light* light = getLightByID(stateSetStack, i);
                const osg::Material* material = getFrontMaterial(stateSetStack);

                if ( light )
                {
                    OE_NOTICE << "Found Light " << i << std::endl;
                }

                if ( _lightEnabled[i] != enabled || !_applied )
                {
                    _lightEnabled[i] = enabled;
                    if ( _useUpdateTrav ){
                        _dirty = true;
                    }else{
                        _lightEnabledUniform->setElement( i, _lightEnabled[i] );
                    }
                }
                
                //update light position info regardsless of if applied for now
                if(light){
                    OE_NOTICE << "Setting light source params." << std::endl;
                    _osgLightSourceParameters[i].setUniformsFromOsgLight(light, cv->getCurrentCamera()->getViewMatrix(), material);
                }
            }	
        }

        // apply if necessary:
        if ( !_applied && !_useUpdateTrav )
        {
            OpenThreads::ScopedLock<OpenThreads::Mutex> lock( _stateSetMutex );
            if (!_applied)
            {
                node->getOrCreateStateSet()->addUniform( _lightingEnabledUniform.get() );
                node->getStateSet()->addUniform( _lightEnabledUniform.get() );
                for( int i=0; i < _maxLights; ++i )
                {
                    _osgLightSourceParameters[i].applyState(node->getStateSet());
                }
                _applied = true;
            }
        }		
    }        
}

void
UpdateLightingUniformsHelper::updateTraverse( osg::Node* node )
{
    if ( _dirty )
    {
        _lightingEnabledUniform->set( _lightingEnabled );

        for( int i=0; i < _maxLights; ++i )
            _lightEnabledUniform->setElement( i, _lightEnabled[i] );

        _dirty = false;

        if ( !_applied )
        {
            osg::StateSet* stateSet = node->getOrCreateStateSet();
            stateSet->addUniform( _lightingEnabledUniform.get() );
            stateSet->addUniform( _lightEnabledUniform.get() );
            for( int i=0; i < _maxLights; ++i )
            {
                _osgLightSourceParameters[i].applyState(stateSet);
            }
        }
    }
}

void
UpdateLightingUniformsHelper::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
    cullTraverse( node, nv );
    traverse(node, nv);
}

//------------------------------------------------------------------------

ArrayUniform::ArrayUniform( const std::string& name, osg::Uniform::Type type, osg::StateSet* stateSet, unsigned size )
{
    attach( name, type, stateSet, size );
}

void
ArrayUniform::attach( const std::string& name, osg::Uniform::Type type, osg::StateSet* stateSet, unsigned size )
{
    _uniform    = stateSet->getUniform( name );
    _uniformAlt = stateSet->getUniform( name + "[0]" );

    if ( !isValid() )
    {
        _uniform    = new osg::Uniform( type, name, size );
        _uniformAlt = new osg::Uniform( type, name + "[0]", size );
        stateSet->addUniform( _uniform.get() );
        stateSet->addUniform( _uniformAlt.get() );
    }

    _stateSet = stateSet;
}

void 
ArrayUniform::setElement( unsigned index, int value )
{
    if ( isValid() )
    {
        ensureCapacity( index+1 );
        _uniform->setElement( index, value );
        _uniformAlt->setElement( index, value );
    }
}

void 
ArrayUniform::setElement( unsigned index, unsigned value )
{
    if ( isValid() )
    {
        ensureCapacity( index+1 );
        _uniform->setElement( index, value );
        _uniformAlt->setElement( index, value );
    }
}

void 
ArrayUniform::setElement( unsigned index, bool value )
{
    if ( isValid() )
    {
        ensureCapacity( index+1 );
        _uniform->setElement( index, value );
        _uniformAlt->setElement( index, value );
    }
}

void 
ArrayUniform::setElement( unsigned index, float value )
{
    if ( isValid() )
    {
        ensureCapacity( index+1 );
        _uniform->setElement( index, value );
        _uniformAlt->setElement( index, value );
    }
}

void
ArrayUniform::setElement( unsigned index, const osg::Matrix& value )
{
    if ( isValid() )
    {
        ensureCapacity( index+1 );
        _uniform->setElement( index, value );
        _uniformAlt->setElement( index, value );
    }
}

void
ArrayUniform::setElement( unsigned index, const osg::Vec3& value )
{
    if ( isValid() )
    {
        ensureCapacity( index+1 );
        _uniform->setElement( index, value );
        _uniformAlt->setElement( index, value );
    }
}

bool 
ArrayUniform::getElement( unsigned index, int& out_value ) const
{
    return isValid() ? _uniform->getElement( index, out_value ) : false;
}

bool 
ArrayUniform::getElement( unsigned index, unsigned& out_value ) const
{
    return isValid() ? _uniform->getElement( index, out_value ) : false;
}

bool 
ArrayUniform::getElement( unsigned index, bool& out_value ) const
{
    return isValid() ? _uniform->getElement( index, out_value ) : false;
}

bool 
ArrayUniform::getElement( unsigned index, float& out_value ) const
{
    return isValid() ? _uniform->getElement( index, out_value ) : false;
}

bool 
ArrayUniform::getElement( unsigned index, osg::Matrix& out_value ) const
{
    return isValid() ? _uniform->getElement( index, out_value ) : false;
}

bool 
ArrayUniform::getElement( unsigned index, osg::Vec3& out_value ) const
{
    return isValid() ? _uniform->getElement( index, out_value ) : false;
}


void
ArrayUniform::ensureCapacity( unsigned newSize )
{
    if ( isValid() && _uniform->getNumElements() < newSize )
    {
        osg::ref_ptr<osg::StateSet> stateSet_safe = _stateSet.get();
        if ( stateSet_safe.valid() )
        {
            osg::ref_ptr<osg::Uniform> _oldUniform    = _uniform.get();
            osg::ref_ptr<osg::Uniform> _oldUniformAlt = _oldUniform.get();

            stateSet_safe->removeUniform( _uniform->getName() );
            stateSet_safe->removeUniform( _uniformAlt->getName() );

            _uniform    = new osg::Uniform( _uniform->getType(), _uniform->getName(), newSize );
            _uniformAlt = new osg::Uniform( _uniform->getType(), _uniform->getName() + "[0]", newSize );

            switch( _oldUniform->getInternalArrayType(_oldUniform->getType()) )
            {
            case GL_FLOAT:
              {
                for( unsigned i = 0; i < _oldUniform->getNumElements(); ++i )
                {
                  float value;
                  _oldUniform->getElement(i, value);
                  setElement( i, value );
                }
              }
              break;

            case GL_INT:
              {
                for( unsigned i = 0; i < _oldUniform->getNumElements(); ++i )
                {
                  int value;
                  _oldUniform->getElement(i, value);
                  setElement( i, value );
                }
              }
              break;

            case GL_UNSIGNED_INT:
              {
                for( unsigned i = 0; i < _oldUniform->getNumElements(); ++i )
                {
                  unsigned value;
                  _oldUniform->getElement(i, value);
                  setElement( i, value );
                }
              }
              break;
            }

            stateSet_safe->addUniform( _uniform.get() );
            stateSet_safe->addUniform( _uniformAlt.get() );

            stateSet_safe.release(); // don't want to unref delete
        }
    }
}

void
ArrayUniform::detach()
{
    if ( isValid() )
    {
        osg::ref_ptr<osg::StateSet> stateSet_safe = _stateSet.get();
        if ( stateSet_safe.valid() )
        {
            stateSet_safe->removeUniform( _uniform->getName() );
            stateSet_safe->removeUniform( _uniformAlt->getName() );

            _uniform = 0L;
            _uniformAlt = 0L;
            _stateSet = 0L;

            stateSet_safe.release(); // don't want to unref delete
        }
    }
}
