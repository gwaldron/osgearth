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

UpdateLightingUniformsCallback::UpdateLightingUniformsCallback() :
_applied( false ),
_dirty( true ),
_lightingEnabled( true )
{
    _maxLights = Registry::instance()->getCapabilities().getMaxLights();

    _lightEnabled = new bool[ _maxLights ];
    if ( _maxLights > 0 )
        _lightEnabled[0] = 1;
    for(int i=1; i<_maxLights; ++i )
        _lightEnabled[i] = 0;

    _lightingEnabledUniform = new osg::Uniform( osg::Uniform::BOOL, "osgearth_lighting_enabled" );
    _lightEnabledUniform    = new osg::Uniform( osg::Uniform::INT,  "osgearth_light_enabled", _maxLights );
    //_numLightsUniform       = new osg::Uniform( osg::Uniform::INT,  "osgearth_num_lights" );
}

UpdateLightingUniformsCallback::~UpdateLightingUniformsCallback()
{
    delete [] _lightEnabled;
}

void
UpdateLightingUniformsCallback::operator()( osg::Node* node, osg::NodeVisitor* nv )
{
    if ( nv->getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR )
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
    else
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

            if ( lightingEnabled != _lightingEnabled )
            {
                _lightingEnabled = lightingEnabled;
                _dirty = true;
            }

            // Update the list of enabled lights:
            for( int i=0; i < _maxLights; ++i )
            {
                bool enabled =
                    ( getModeValue( stateSetStack, GL_LIGHT0 + i ) & osg::StateAttribute::ON ) != 0;

                if ( _lightEnabled[i] != enabled )
                {
                    _lightEnabled[i] = enabled;
                    _dirty = true;
                }
            }			
        }        
    }

    traverse( node, nv );
}
