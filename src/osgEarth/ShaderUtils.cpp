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
#include <osgUtil/CullVisitor>
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
            OpenThreads::ScopedLock<OpenThreads::Mutex> lock( _stateSetMutex );
            if (!_applied)
            {
                node->getOrCreateStateSet()->addUniform( _lightingEnabledUniform.get() );
                node->getStateSet()->addUniform( _lightEnabledUniform.get() );
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
        }
    }
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
