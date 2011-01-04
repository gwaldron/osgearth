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
#include <osgEarth/ShaderUtils>
#include <osgEarth/Registry>
#include <list>

using namespace osgEarth;

//------------------------------------------------------------------------

namespace 
{
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
            if ((val & ~osg::StateAttribute::INHERIT)!=0)
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
}

//------------------------------------------------------------------------

#undef LC
#define LC "[UpdateLightingUniformHelper] "

UpdateLightingUniformsHelper::UpdateLightingUniformsHelper( bool useUpdateTrav ) :
_lightingEnabled( true ),
_dirty( true ),
_applied( false ),
_useUpdateTrav( useUpdateTrav )
{
    _maxLights = Registry::instance()->getCapabilities().getMaxLights();

    _lightEnabled = new bool[ _maxLights ];
    if ( _maxLights > 0 )
        _lightEnabled[0] = 1;
    for(int i=1; i<_maxLights; ++i )
        _lightEnabled[i] = 0;

    _lightingEnabledUniform = new osg::Uniform( osg::Uniform::BOOL, "osgearth_LightingEnabled" );
    _lightEnabledUniform    = new osg::Uniform( osg::Uniform::INT,  "osgearth_LightEnabled", _maxLights );

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
    osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>( nv );
    if ( cv )
    {
        StateSetStack stateSetStack;

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

        // Update the list of enabled lights:
        for( int i=0; i < _maxLights; ++i )
        {
            bool enabled =
                ( getModeValue( stateSetStack, GL_LIGHT0 + i ) & osg::StateAttribute::ON ) != 0;

            if ( _lightEnabled[i] != enabled || !_applied )
            {
                _lightEnabled[i] = enabled;
                if ( _useUpdateTrav )
                    _dirty = true;
                else
                    _lightEnabledUniform->setElement( i, _lightEnabled[i] );
            }
        }	

        // apply if necessary:
        if ( !_applied && !_useUpdateTrav )
        {
            node->getOrCreateStateSet()->addUniform( _lightingEnabledUniform.get() );
            node->getStateSet()->addUniform( _lightEnabledUniform.get() );
            _applied = true;
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
        }
    }
}

//------------------------------------------------------------------------

ArrayUniform::ArrayUniform( osg::Uniform::Type type, const std::string& name, int size )
{
    _uniform = new osg::Uniform( type, name, size );
    _uniformAlt = new osg::Uniform( type, name + "[0]", size );
}

ArrayUniform::ArrayUniform( osg::StateSet* stateSet, const std::string& name )
{
    _uniform = stateSet->getUniform( name );
    _uniformAlt = stateSet->getUniform( name + "[0]" );
}

void 
ArrayUniform::setElement( int index, int value )
{
    _uniform->setElement( index, value );
    _uniformAlt->setElement( index, value );
}

void 
ArrayUniform::setElement( int index, bool value )
{
    _uniform->setElement( index, value );
    _uniformAlt->setElement( index, value );
}

void 
ArrayUniform::setElement( int index, float value )
{
    _uniform->setElement( index, value );
    _uniformAlt->setElement( index, value );
}

void 
ArrayUniform::addTo( osg::StateSet* stateSet )
{
    if ( stateSet )
    {
        stateSet->addUniform( _uniform.get() );
        stateSet->addUniform( _uniformAlt.get() );
    }
}

void 
ArrayUniform::removeFrom( osg::StateSet* stateSet )
{
    if ( stateSet )
    {
        stateSet->removeUniform( _uniform->getName() );
        stateSet->removeUniform( _uniformAlt->getName() );
    }
}
