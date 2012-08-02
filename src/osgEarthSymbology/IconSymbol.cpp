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
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */
#include <osgEarthSymbology/IconSymbol>
#include <osgEarthSymbology/IconResource>
#include <osgEarth/ThreadingUtils>
#include <osgEarth/Registry>
#include <osgEarth/ImageUtils>
#include <osgDB/Options>

using namespace osgEarth;
using namespace osgEarth::Symbology;

IconSymbol::IconSymbol( const Config& conf ) :
InstanceSymbol( conf ),
_alignment    ( ALIGN_CENTER_BOTTOM ),
_heading      ( NumericExpression(0.0) )
{
    mergeConfig( conf );
}

Config 
IconSymbol::getConfig() const
{
    Config conf = InstanceSymbol::getConfig();
    conf.key() = "icon";

    conf.addIfSet( "alignment", "left_top",      _alignment, ALIGN_LEFT_TOP );
    conf.addIfSet( "alignment", "left_center",   _alignment, ALIGN_LEFT_CENTER );
    conf.addIfSet( "alignment", "left_bottom",   _alignment, ALIGN_LEFT_BOTTOM );
    conf.addIfSet( "alignment", "center_top",    _alignment, ALIGN_CENTER_TOP );
    conf.addIfSet( "alignment", "center_center", _alignment, ALIGN_CENTER_CENTER );
    conf.addIfSet( "alignment", "center_bottom", _alignment, ALIGN_CENTER_BOTTOM );
    conf.addIfSet( "alignment", "right_top",     _alignment, ALIGN_RIGHT_TOP );
    conf.addIfSet( "alignment", "right_center",  _alignment, ALIGN_RIGHT_CENTER );
    conf.addIfSet( "alignment", "right_bottom",  _alignment, ALIGN_RIGHT_BOTTOM );

    conf.addObjIfSet( "heading", _heading );

    conf.addNonSerializable( "IconSymbol::image", _image.get() );
    return conf;
}

void 
IconSymbol::mergeConfig( const Config& conf )
{
    conf.getIfSet( "alignment", "left_top",      _alignment, ALIGN_LEFT_TOP );
    conf.getIfSet( "alignment", "left_center",   _alignment, ALIGN_LEFT_CENTER );
    conf.getIfSet( "alignment", "left_bottom",   _alignment, ALIGN_LEFT_BOTTOM );
    conf.getIfSet( "alignment", "center_top",    _alignment, ALIGN_CENTER_TOP );
    conf.getIfSet( "alignment", "center_center", _alignment, ALIGN_CENTER_CENTER );
    conf.getIfSet( "alignment", "center_bottom", _alignment, ALIGN_CENTER_BOTTOM );
    conf.getIfSet( "alignment", "right_top",     _alignment, ALIGN_RIGHT_TOP );
    conf.getIfSet( "alignment", "right_center",  _alignment, ALIGN_RIGHT_CENTER );
    conf.getIfSet( "alignment", "right_bottom",  _alignment, ALIGN_RIGHT_BOTTOM );

    conf.getObjIfSet( "heading", _heading );

    _image = conf.getNonSerializable<osg::Image>( "IconSymbol::image" );
}

namespace
{
    static Threading::Mutex s_getImageMutex;
}

osg::Image*
IconSymbol::getImage( unsigned maxSize ) const
{
    if ( !_image.valid() && _url.isSet() )
    {
        Threading::ScopedMutexLock lock(s_getImageMutex);
        if ( !_image.valid() )
        {
            osg::ref_ptr<osgDB::Options> dbOptions = Registry::instance()->cloneOrCreateOptions();
            dbOptions->setObjectCacheHint( osgDB::Options::CACHE_IMAGES );
            _image = URI(_url->eval(), _url->uriContext()).getImage( dbOptions.get() );
            if ( _image.valid() && (maxSize < (unsigned int)_image->s() || maxSize < (unsigned int)_image->t()) )
            {
                unsigned new_s, new_t;

                if ( _image->s() >= _image->t() ) {
                    new_s = maxSize;
                    float ratio = (float)new_s/(float)_image->s();
                    new_t = (unsigned)((float)_image->t() * ratio);
                }
                else {
                    new_t = maxSize;
                    float ratio = (float)new_t/(float)_image->t();
                    new_s = (unsigned)((float)_image->s() * ratio);
                }
                    
                osg::ref_ptr<osg::Image> result;
                ImageUtils::resizeImage( _image.get(), new_s, new_t, result );
                _image = result.get();
            }
        }
    }
    return _image.get();
}

InstanceResource*
IconSymbol::createResource() const 
{
    return new IconResource();
}
