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

#include <osgEarth/TextureCompositor>
#include <osgEarth/TextureCompositorTexArray>
#include <osgEarth/TextureCompositorTex3D>
#include <osgEarth/TextureCompositorAtlas>
#include <osgEarth/TextureCompositorMulti>
#include <osgEarth/TextureCompositorSoftware>
#include <osgEarth/Capabilities>
#include <osgEarth/ImageUtils>
#include <osgEarth/Registry>
#include <osg/Texture2DArray>
#include <osg/Texture2D>
#include <osg/Texture3D>
#include <vector>

using namespace osgEarth;
using namespace OpenThreads;

#define LC "[TextureCompositor] "

//---------------------------------------------------------------------------

TextureLayout::TextureLayout() :
_firstTextureSlot( 0 )
{
    //nop
}

int
TextureLayout::getSlot( UID layerUID ) const
{
    TextureSlotVector::const_iterator i = std::find( _slots.begin(), _slots.end(), layerUID );
    return i != _slots.end() ? (int)(i-_slots.begin()) : -1;
}

int 
TextureLayout::getOrder( UID layerUID ) const
{
    int slot = getSlot( layerUID );
    RenderOrderVector::const_iterator i = std::find( _order.begin(), _order.end(), slot );
    return i != _order.end() ? (int)(i-_order.begin()) : -1;
}

int
TextureLayout::getMaxUsedSlot() const
{
    for( int i = _slots.size()-1; i >= 0; --i )
        if ( i >= 0 )
            return i;
    return -1;
}

void
TextureLayout::applyMapModelChange( const MapModelChange& change )
{
    if ( change.getAction() == MapModelChange::ADD_IMAGE_LAYER )
    {
        bool found = false;
        for( TextureSlotVector::iterator i = _slots.begin(); i != _slots.end() && !found; ++i )
        {
            if ( *i < 0 ) // negative UID means the slot is empty.
            {
                *i = change.getImageLayer()->getUID();
                
                if ( change.getFirstIndex() >= (int)_order.size() )
                    _order.resize( change.getFirstIndex() + 1, -1 );

                _order[change.getFirstIndex()] = (int)(i - _slots.begin());
                found = true;
                break;
            }
        }

        if ( !found )
        {
            _slots.push_back( change.getImageLayer()->getUID() );
            _order.push_back( _slots.size() - 1 );
        }
    }

    else if ( change.getAction() == MapModelChange::REMOVE_IMAGE_LAYER )
    {
        for( TextureSlotVector::iterator i = _slots.begin(); i != _slots.end(); ++i )
        {
            if ( *i == change.getLayer()->getUID() )
            {
                *i = -1;
                for( RenderOrderVector::iterator j = _order.begin(); j != _order.end(); ++j )
                {
                    if ( *j == (int)(i - _slots.begin()) )
                    {
                        j = _order.erase( j );
                        break;
                    }
                }
                break;
            }
        }
    }

    else if ( change.getAction() == MapModelChange::MOVE_IMAGE_LAYER )
    {
        int fromIndex = getOrder( change.getLayer()->getUID() );
        int toIndex = change.getSecondIndex();

        if ( fromIndex != toIndex )
        {
            int slot = _order[fromIndex];
            _order.erase( _order.begin() + fromIndex );
            _order.insert( _order.begin() + toIndex, slot );
        }
    }

    //OE_INFO << LC << "Layout Slots: " << std::endl;
    //for( int i=0; i<_slots.size(); ++i )
    //    OE_INFO << LC << "  Slot " << i << ": uid=" << _slots[i] << ", order=" << getOrder(_slots[i]) << std::endl;
    //OE_INFO << LC << "Layout Order: " << std::endl;
    //for( int i=0; i<_order.size(); ++i )
    //    OE_INFO << LC << "  Ordr " << i << ": slot=" << _order[i] << std::endl;
}

//---------------------------------------------------------------------------

TextureCompositor::TextureCompositor( const TerrainOptions::CompositingTechnique& tech ) :
osg::Referenced( true ),
_tech( tech ),
_forceTech( false )
{
    // for debugging:
    if ( _tech == TerrainOptions::COMPOSITING_AUTO && ::getenv( "OSGEARTH_COMPOSITOR_TECH" ) )
    {
        TerrainOptions::CompositingTechnique oldTech = _tech;
        std::string t( ::getenv( "OSGEARTH_COMPOSITOR_TECH" ) );
        if      ( t == "TEXTURE_ARRAY" )    _tech = TerrainOptions::COMPOSITING_TEXTURE_ARRAY;
        //else if ( t == "TEXTURE_3D" )       _tech = TECH_TEXTURE_3D;
        //else if ( t == "TEXTURE_ATLAS" )    _tech = TerrainOptions::COMPOSITING_TEXTURE_ATLAS;
        else if ( t == "MULTITEXTURE_GPU" ) _tech = TerrainOptions::COMPOSITING_MULTITEXTURE_GPU;
        //else if ( t == "MULTITEXTURE_FFP" ) _tech = TerrainOptions::COMPOSITING_MULTITEXTURE_FFP;
        else if ( t == "MULTIPASS" )        _tech = TerrainOptions::COMPOSITING_MULTIPASS;
        //else if ( t == "SOFTWARE" )         _tech = TECH_SOFTWARE;
        if ( oldTech != _tech )
            _forceTech = true;
    }

    init();
}

void
TextureCompositor::applyMapModelChange( const MapModelChange& change )
{
    Threading::ScopedWriteLock exclusiveLock( _layoutMutex );
    _layout.applyMapModelChange( change );
}

#if 0
osg::StateSet*
TextureCompositor::createStateSet( const GeoImageVector& stack, const GeoExtent& tileExtent ) const
{
    return _impl ? _impl->createStateSet( stack, tileExtent ) : 0L;
}
#endif

bool
TextureCompositor::supportsLayerUpdate() const
{
    return _impl.valid() ? _impl->supportsLayerUpdate() : false;
}

GeoImage
TextureCompositor::prepareImage( const GeoImage& image, const GeoExtent& tileExtent ) const
{
    return _impl.valid() ? _impl->prepareImage( image, tileExtent ) : GeoImage::INVALID;
}

void
TextureCompositor::applyLayerUpdate(osg::StateSet* stateSet,
                                    UID layerUID,
                                    const GeoImage& preparedImage,
                                    const GeoExtent& tileExtent ) const
{
    if ( _impl.valid() )
    {
        Threading::ScopedReadLock sharedLock( const_cast<TextureCompositor*>(this)->_layoutMutex );
        _impl->applyLayerUpdate( stateSet, layerUID, preparedImage, tileExtent, _layout );
    }
}

void
TextureCompositor::applyLayerRemoval(osg::StateSet* stateSet,
                                     UID layerUID ) const
{
    if ( _impl.valid() )
        _impl->applyLayerRemoval( stateSet, layerUID );
}

bool
TextureCompositor::requiresUnitTextureSpace() const
{
    return _impl.valid() ? _impl->requiresUnitTextureSpace() : false;
}

bool
TextureCompositor::usesShaderComposition() const
{
    return _impl.valid() ? _impl->usesShaderComposition() : false;
}

void
TextureCompositor::updateMasterStateSet( osg::StateSet* stateSet ) const
{
    if ( _impl.valid() )
    {
        Threading::ScopedReadLock sharedLock( const_cast<TextureCompositor*>(this)->_layoutMutex );
        _impl->updateMasterStateSet( stateSet, _layout );
    }
}

void
TextureCompositor::assignTexCoordArray(osg::Geometry* geom,
                                       UID layerUID,
                                       osg::Vec2Array* texCoords ) const
{
    if ( geom && texCoords )
    {
        if ( _tech == TerrainOptions::COMPOSITING_MULTIPASS )
        {
            geom->setTexCoordArray( 0, texCoords );
        }
        else
        {
            int slot;
            {            
                Threading::ScopedReadLock sharedLock( const_cast<TextureCompositor*>(this)->_layoutMutex );
                slot = _layout.getSlot( layerUID );
            }
            if ( slot >= 0 )
                geom->setTexCoordArray( slot, texCoords );
        }
    }
}

int
TextureCompositor::getRenderOrder( UID layerUID ) const
{
    Threading::ScopedReadLock sharedLock( const_cast<TextureCompositor*>(this)->_layoutMutex );
    return _layout.getOrder( layerUID );
}

void
TextureCompositor::init()
{        
    if ( _impl.valid() ) // double-check pattern
    {
        return; // already initialized
    }

    bool isAuto = _tech == TerrainOptions::COMPOSITING_AUTO;

    const Capabilities& caps = Registry::instance()->getCapabilities();

#if OSG_VERSION_GREATER_OR_EQUAL( 2, 9, 8 )

    if (_tech == TerrainOptions::COMPOSITING_TEXTURE_ARRAY || 
        (isAuto && caps.supportsGLSL(1.30f) && caps.supportsTextureArrays()) )
    {
        _tech = TerrainOptions::COMPOSITING_TEXTURE_ARRAY;
        _impl = new TextureCompositorTexArray();
        OE_INFO << LC << "Compositing technique = TEXTURE ARRAY" << std::endl;
    }

    else

#endif // OSG_VERSION_GREATER_OR_EQUAL( 2, 9, 8 )

    if (_tech == TerrainOptions::COMPOSITING_MULTITEXTURE_GPU ||
        (isAuto && caps.supportsGLSL(1.20f) && caps.supportsMultiTexture()) ) 
    {
        _tech = TerrainOptions::COMPOSITING_MULTITEXTURE_GPU;
        _impl = new TextureCompositorMultiTexture( true );
        OE_INFO << LC << "Compositing technique = MULTITEXTURE/GPU" << std::endl;
    }

    // NOTE: uncomment this to "else if" when Software mode is ready
    else if ( _tech == TerrainOptions::COMPOSITING_MULTITEXTURE_FFP || (isAuto && caps.supportsMultiTexture()) )
    {
        _tech = TerrainOptions::COMPOSITING_MULTITEXTURE_FFP;
        _impl = new TextureCompositorMultiTexture( false );
        OE_INFO << LC << "Compositing technique = MULTITEXTURE/FFP" << std::endl;
    }

    // Fallback of last resort. The Compositor is actually a NO-OP for multipass mode.
    else
    {
        _tech = TerrainOptions::COMPOSITING_MULTIPASS;
        _impl = 0L;
        OE_INFO << LC << "Compositing technique = MULTIPASS" << std::endl;
    }
}
