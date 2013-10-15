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

#include <osgEarth/TextureCompositor>
#include <osgEarth/TextureCompositorTexArray>
#include <osgEarth/TextureCompositorMulti>
#include <osgEarth/Capabilities>
#include <osgEarth/ImageUtils>
#include <osgEarth/Registry>
#include <osgEarth/Map>
#include <osgEarth/MapModelChange>
#include <osg/Texture2DArray>
#include <osg/Texture2D>
#include <osg/Texture3D>
#include <vector>

using namespace osgEarth;
using namespace OpenThreads;

#define LC "[TextureCompositor] "

//---------------------------------------------------------------------------

TextureLayout::TextureLayout()
{
    //nop
}

int
TextureLayout::getSlot( UID layerUID, unsigned which, unsigned maxSlotsToSearch ) const
{
    for( unsigned slot = 0; slot < _slots.size() && slot < maxSlotsToSearch; ++slot )
    {
        if ( _slots[slot] == layerUID )
        {
            if ( which == 0 )
                return slot;
            else
                --which;
        }
    }
    return -1;
}

int 
TextureLayout::getOrder( UID layerUID ) const
{
    int slot = getSlot( layerUID, 0 );
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
TextureLayout::assignPrimarySlot( ImageLayer* layer, int orderIndex )
{
    int slot = -1;

    bool found = false;
    for( TextureSlotVector::iterator i = _slots.begin(); i != _slots.end() && !found; ++i )
    {
        slot = (int)(i - _slots.begin());

        // negative UID means the slot is empty.
        bool slotAvailable = (*i < 0) && (_reservedSlots.find(slot) == _reservedSlots.end());
        if ( slotAvailable )
        {  
            *i = layer->getUID();
            found = true;
            break;
        }
    }

    if ( !found )
    {
        // put the UID in the next available slot (that's not reserved).
        while( _reservedSlots.find(_slots.size()) != _reservedSlots.end() )
            _slots.push_back( -1 );

        slot = _slots.size();
        _slots.push_back( layer->getUID() );     
    }

    // record the render order of this slot:
    if ( orderIndex >= (int)_order.size() )
    {
        _order.resize( orderIndex + 1, -1 );
        _order[orderIndex] = slot;
    }
    else
    {
        if (_order[orderIndex] == -1)
            _order[orderIndex] = slot;
        else
            _order.insert(_order.begin() + orderIndex, slot);
    }



    OE_DEBUG << LC << "Allocated SLOT " << slot << "; primary slot for layer \"" << layer->getName() << "\"" << std::endl;
}

void
TextureLayout::assignSecondarySlot( ImageLayer* layer )
{
    int slot = -1;
    bool found = false;

    for( TextureSlotVector::iterator i = _slots.begin(); i != _slots.end() && !found; ++i )
    {
        slot = (int)(i - _slots.begin());

        // negative UID means the slot is empty.
        bool slotAvailable = (*i < 0) && (_reservedSlots.find(slot) == _reservedSlots.end());
        if ( slotAvailable )
        {
            // record this UID in the new slot:
            *i = layer->getUID();
            found = true;
            break;
        }
    }

    if ( !found )
    {
        // put the UID in the next available slot (that's not reserved).
        while( _reservedSlots.find(_slots.size()) != _reservedSlots.end() )
            _slots.push_back( -1 );

        slot = _slots.size();
        _slots.push_back( layer->getUID() );
    }

    OE_INFO << LC << "Allocated SLOT " << slot << "; secondary slot for layer \"" << layer->getName() << "\"" << std::endl;
}

void
TextureLayout::applyMapModelChange(const MapModelChange& change, 
                                   bool reserveSeconarySlotIfNecessary,
                                   bool disableLODBlending )
{
    if ( change.getAction() == MapModelChange::ADD_IMAGE_LAYER )
    {
        assignPrimarySlot( change.getImageLayer(), change.getFirstIndex() );

        // did the layer specify LOD blending (and is it supported?)
        bool blendingOn = 
          !disableLODBlending &&
          change.getImageLayer()->getImageLayerOptions().lodBlending() == true;
    
        _lodBlending[ change.getImageLayer()->getUID() ] = blendingOn;

        if ( blendingOn && reserveSeconarySlotIfNecessary )
        {
            assignSecondarySlot( change.getImageLayer() );
        }
    }

    else if ( change.getAction() == MapModelChange::REMOVE_IMAGE_LAYER )
    {
        for( int which = 0; which <= 1; ++which )
        {
            int slot = getSlot( change.getLayer()->getUID(), which );
            if ( slot < 0 )
                break;

            _slots[slot] = -1;

            if ( which == 0 ) // primary slot; remove from render order:
            {
                for( RenderOrderVector::iterator j = _order.begin(); j != _order.end(); )
                {
                    if ( *j == slot )
                        j = _order.erase( j );
                    else
                        ++j;
                }
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

void
TextureLayout::setReservedSlots( const std::set<int>& reservedSlots )
{
    _reservedSlots = reservedSlots;
}

bool
TextureLayout::isSlotAvailable( int i ) const
{
    if ( (i < (int)_slots.size() && _slots[i] < 0) || i >= (int)_slots.size() )
    {
        if ( _reservedSlots.find(i) == _reservedSlots.end() )
        {
            return true;
        }
    }
    return false;
}

bool
TextureLayout::containsSecondarySlots( unsigned maxSlotsToSearch ) const
{
    for( int slot = 0; slot < (int)_slots.size() && slot < (int)maxSlotsToSearch; ++slot )
    {
        UID uid = _slots[slot];
        if ( getSlot(uid, 0) != slot )
            return true;
    }
    return false;
}

bool
TextureLayout::isBlendingEnabled( UID layerUID ) const
{
    std::map<UID,bool>::const_iterator i = _lodBlending.find(layerUID);
    return i != _lodBlending.end() ? i->second : false;
}

//---------------------------------------------------------------------------

TextureCompositor::TextureCompositor(const TerrainOptions& options) :
osg::Referenced( true ),
_tech( options.compositingTechnique().value() ),
_options( options ),
_forceTech( false )
{
    // for debugging:
    if ( _tech == TerrainOptions::COMPOSITING_AUTO && ::getenv( "OSGEARTH_COMPOSITOR_TECH" ) )
    {
        TerrainOptions::CompositingTechnique oldTech = _tech;
        std::string t( ::getenv( "OSGEARTH_COMPOSITOR_TECH" ) );
        if      ( t == "TEXTURE_ARRAY" )    _tech = TerrainOptions::COMPOSITING_TEXTURE_ARRAY;
        else if ( t == "MULTITEXTURE_GPU" ) _tech = TerrainOptions::COMPOSITING_MULTITEXTURE_GPU;
        else if ( t == "MULTIPASS" )        _tech = TerrainOptions::COMPOSITING_MULTIPASS;
        if ( oldTech != _tech )
            _forceTech = true;
    }

    init();
}

bool
TextureCompositor::reserveTextureImageUnit( int& out_unit )
{
    //todo: move this into the impls!!

    out_unit = -1;

    //TODO: this only supports GPU texturing....
    unsigned maxUnits = osgEarth::Registry::instance()->getCapabilities().getMaxGPUTextureUnits();

    if ( _tech == TerrainOptions::COMPOSITING_MULTITEXTURE_GPU )
    {
        Threading::ScopedWriteLock exclusiveLock( _layoutMutex );

        for( unsigned i=0; i<maxUnits; ++i )
        {
            if ( _layout.isSlotAvailable(i) )
            {
                out_unit = i;
                _reservedUnits.insert( i );
                _layout.setReservedSlots( _reservedUnits ); // in multitexture, slots == units
                return true;
            }
        }

        // all taken, return false.
        return false;
    }

    else if ( _tech == TerrainOptions::COMPOSITING_TEXTURE_ARRAY )
    {
        // texture array reserved slots 0 and 1 (for primary and blending)
        for( unsigned i=2; i<maxUnits; ++i ) // 0 and 1 always reserved.
        {
            if ( _reservedUnits.find( i ) == _reservedUnits.end() )
            {
                out_unit = i;
                _reservedUnits.insert( i );
                return true;
            }
        }

        // all taken, return false.
        return false;
    }

    else // multipass or USER .. just simple reservations.
    {
        // search for an unused unit.
        for( unsigned i=0; i<maxUnits; ++i )
        {
            if ( _reservedUnits.find( i ) == _reservedUnits.end() )
            {
                out_unit = i;
                _reservedUnits.insert( i );
                OE_INFO << LC << "Reserved image unit " << i << std::endl;
                return true;
            }
        }

        // all taken, return false.
        return false;
    }
}

void
TextureCompositor::releaseTextureImageUnit( int unit )
{
    _reservedUnits.erase( unit );
    OE_INFO << LC << "Released image unit " << unit << std::endl;

    if ( _tech == TerrainOptions::COMPOSITING_MULTITEXTURE_GPU )
    {
        Threading::ScopedWriteLock exclusiveLock( _layoutMutex );
        _layout.setReservedSlots( _reservedUnits );
    }
}

void
TextureCompositor::applyMapModelChange( const MapModelChange& change )
{
    // verify it's actually an image layer
    ImageLayer* layer = change.getImageLayer();
    if ( !layer )
        return;

    Threading::ScopedWriteLock exclusiveLock( _layoutMutex );

    // LOD blending does not work with mercator fast path texture mapping.
    bool disableLODBlending =
      layer->getProfile() &&
      layer->getProfile()->getSRS()->isSphericalMercator() &&
      _options.enableMercatorFastPath() == true;

    // Let the use know why they aren't getting LOD blending!
    if ( disableLODBlending && layer->getImageLayerOptions().lodBlending() == true )
    {
        OE_WARN << LC << "LOD blending disabled for layer \"" << layer->getName()
            << "\" because it uses Mercator fast-path rendering" << std::endl;
    }

    _layout.applyMapModelChange(
        change, 
        _impl.valid() ? _impl->blendingRequiresSecondarySlot() : false,
        disableLODBlending );
}

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

GeoImage
TextureCompositor::prepareSecondaryImage( const GeoImage& image, const GeoExtent& tileExtent ) const
{
    return _impl.valid() ? _impl->prepareSecondaryImage( image, tileExtent ) : GeoImage::INVALID;
}

void
TextureCompositor::applyLayerUpdate(osg::StateSet* stateSet,
                                    UID layerUID,
                                    const GeoImage& preparedImage,
                                    const TileKey& tileKey,
                                    osg::StateSet* parentStateSet) const
{
    if ( _impl.valid() )
    {
        Threading::ScopedReadLock sharedLock( const_cast<TextureCompositor*>(this)->_layoutMutex );
        _impl->applyLayerUpdate( stateSet, layerUID, preparedImage, tileKey,
                                 _layout,  parentStateSet );
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
    return _impl.valid() ? _impl->usesShaderComposition() : true;
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

osg::Shader*
TextureCompositor::createSamplerFunction(UID                layerUID, 
                                         const std::string& functionName,
                                         osg::Shader::Type  type ) const
{
    osg::Shader* result = 0L;
    if ( _impl.valid() )
    {
        Threading::ScopedReadLock sharedLock( const_cast<TextureCompositor*>(this)->_layoutMutex );
        result = _impl->createSamplerFunction( layerUID, functionName, type, _layout );
    }

    if ( !result )
    {
        std::string fname = !functionName.empty() ? functionName : "defaultSamplerFunction";
        std::stringstream buf;
        buf << "vec4 " << functionName << "() { \n return vec4(0,0,0,0); \n } \n";
        std::string str;
        str = buf.str();
        result = new osg::Shader( type, str );
    }

    return result;
}

void
TextureCompositor::setTechnique( TextureCompositorTechnique* tech )
{
    _tech = TerrainOptions::COMPOSITING_USER;
    _impl = tech;
    //OE_INFO << LC << "Custom texture compositing technique installed" << std::endl;
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

    // MULTITEXTURE_GPU is the current default.
    
    if (_tech == TerrainOptions::COMPOSITING_MULTITEXTURE_GPU ||
        ( isAuto && TextureCompositorMultiTexture::isSupported(true) ) )
    {
        _tech = TerrainOptions::COMPOSITING_MULTITEXTURE_GPU;
        _impl = new TextureCompositorMultiTexture( true, _options );
        //OE_INFO << LC << "Compositing technique = MULTITEXTURE/GPU" << std::endl;
    }

#if OSG_VERSION_GREATER_OR_EQUAL( 2, 9, 8 )

    else
    if (_tech == TerrainOptions::COMPOSITING_TEXTURE_ARRAY || 
        ( isAuto && TextureCompositorTexArray::isSupported() ) )
    {
        _tech = TerrainOptions::COMPOSITING_TEXTURE_ARRAY;
        _impl = new TextureCompositorTexArray( _options );
        //OE_INFO << LC << "Compositing technique = TEXTURE ARRAY" << std::endl;
    }

#endif // OSG_VERSION_GREATER_OR_EQUAL( 2, 9, 8 )

    else
    if ( _tech == TerrainOptions::COMPOSITING_MULTITEXTURE_FFP || 
        (isAuto && TextureCompositorMultiTexture::isSupported(false) ) )
    {
        _tech = TerrainOptions::COMPOSITING_MULTITEXTURE_FFP;
        _impl = new TextureCompositorMultiTexture( false, _options );
        //OE_INFO << LC << "Compositing technique = MULTITEXTURE/FFP" << std::endl;
    }

    // Fallback of last resort. The implementation is actually a NO-OP for multipass mode.
    else
    {
        _tech = TerrainOptions::COMPOSITING_MULTIPASS;
        _impl = 0L;
        //OE_INFO << LC << "Compositing technique = MULTIPASS" << std::endl;
    }
}
