/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2019 Pelican Mapping
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
#include <osgEarthSymbology/Style>
#include <osgEarth/ThreadingUtils>
#include <osgEarth/Registry>
#include <osgEarth/ImageUtils>
#include <osgDB/Options>

using namespace osgEarth;
using namespace osgEarth::Symbology;

OSGEARTH_REGISTER_SIMPLE_SYMBOL(icon, IconSymbol);

IconSymbol::IconSymbol(const IconSymbol& rhs,const osg::CopyOp& copyop):
InstanceSymbol(rhs, copyop),
_alignment(rhs._alignment),
_heading(rhs._heading),
_declutter(rhs._declutter),
_image(rhs._image),
_occlusionCull(rhs._occlusionCull),
_occlusionCullAltitude(rhs._occlusionCullAltitude)
{
}

IconSymbol::IconSymbol( const Config& conf ) :
InstanceSymbol        ( conf ),
_alignment            ( ALIGN_CENTER_BOTTOM ),
_heading              ( NumericExpression(0.0) ),
_declutter            ( true ),
_occlusionCull        ( false ),
_occlusionCullAltitude( 200000 )
{
    mergeConfig( conf );
}

Config 
IconSymbol::getConfig() const
{
    Config conf = InstanceSymbol::getConfig();
    conf.key() = "icon";

    conf.set( "alignment", "left_top",      _alignment, ALIGN_LEFT_TOP );
    conf.set( "alignment", "left_center",   _alignment, ALIGN_LEFT_CENTER );
    conf.set( "alignment", "left_bottom",   _alignment, ALIGN_LEFT_BOTTOM );
    conf.set( "alignment", "center_top",    _alignment, ALIGN_CENTER_TOP );
    conf.set( "alignment", "center_center", _alignment, ALIGN_CENTER_CENTER );
    conf.set( "alignment", "center_bottom", _alignment, ALIGN_CENTER_BOTTOM );
    conf.set( "alignment", "right_top",     _alignment, ALIGN_RIGHT_TOP );
    conf.set( "alignment", "right_center",  _alignment, ALIGN_RIGHT_CENTER );
    conf.set( "alignment", "right_bottom",  _alignment, ALIGN_RIGHT_BOTTOM );

    conf.set( "heading",   _heading );
    conf.set( "declutter", _declutter );	                  
	conf.set( "icon-occlusion-cull", _occlusionCull );
    conf.set( "icon-occlusion-cull-altitude", _occlusionCullAltitude );

    conf.setNonSerializable( "IconSymbol::image", _image.get() );
    return conf;
}

void 
IconSymbol::mergeConfig( const Config& conf )
{
    conf.get( "alignment", "left_top",      _alignment, ALIGN_LEFT_TOP );
    conf.get( "alignment", "left_center",   _alignment, ALIGN_LEFT_CENTER );
    conf.get( "alignment", "left_bottom",   _alignment, ALIGN_LEFT_BOTTOM );
    conf.get( "alignment", "center_top",    _alignment, ALIGN_CENTER_TOP );
    conf.get( "alignment", "center_center", _alignment, ALIGN_CENTER_CENTER );
    conf.get( "alignment", "center_bottom", _alignment, ALIGN_CENTER_BOTTOM );
    conf.get( "alignment", "right_top",     _alignment, ALIGN_RIGHT_TOP );
    conf.get( "alignment", "right_center",  _alignment, ALIGN_RIGHT_CENTER );
    conf.get( "alignment", "right_bottom",  _alignment, ALIGN_RIGHT_BOTTOM );

    conf.get( "heading",   _heading );
    conf.get( "declutter", _declutter );
	conf.get( "icon-occlusion-cull", _occlusionCull );
    conf.get( "icon-occlusion-cull-altitude", _occlusionCullAltitude );

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
            _image = _url->evalURI().getImage( dbOptions.get() );
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


void
IconSymbol::parseSLD(const Config& c, Style& style)
{
    IconSymbol defaults;

    if ( match(c.key(), "icon") ) {
        style.getOrCreate<IconSymbol>()->url() = c.value();
        style.getOrCreate<IconSymbol>()->url()->setURIContext( c.referrer() );
    }
    else if ( match(c.key(),"icon-library") ) {
        style.getOrCreate<IconSymbol>()->library() = StringExpression(c.value());
    }
    else if ( match(c.key(), "icon-placement") ) {
        if      ( match(c.value(), "vertex") )   
            style.getOrCreate<IconSymbol>()->placement() = ModelSymbol::PLACEMENT_VERTEX;
        else if ( match(c.value(), "interval") ) 
            style.getOrCreate<IconSymbol>()->placement() = ModelSymbol::PLACEMENT_INTERVAL;
        else if ( match(c.value(), "random") )   
            style.getOrCreate<IconSymbol>()->placement() = ModelSymbol::PLACEMENT_RANDOM;
        else if ( match(c.value(), "centroid") ) 
            style.getOrCreate<IconSymbol>()->placement() = ModelSymbol::PLACEMENT_CENTROID;
    }
    else if ( match(c.key(), "icon-density") ) {
        style.getOrCreate<IconSymbol>()->density() = as<float>(c.value(), *defaults.density() );
    }
    else if ( match(c.key(), "icon-random-seed") ) {
        style.getOrCreate<IconSymbol>()->randomSeed() = as<unsigned>(c.value(), *defaults.randomSeed());
    }
    else if ( match(c.key(), "icon-scale") ) {
        style.getOrCreate<IconSymbol>()->scale() = NumericExpression(c.value());
    }
    else if ( match(c.key(), "icon-align") ) {
        if      ( match(c.value(), "left-top") )      
            style.getOrCreate<IconSymbol>()->alignment() = IconSymbol::ALIGN_LEFT_TOP;
        else if ( match(c.value(), "left-center") )   
            style.getOrCreate<IconSymbol>()->alignment() = IconSymbol::ALIGN_LEFT_CENTER;
        else if ( match(c.value(), "left-bottom") )   
            style.getOrCreate<IconSymbol>()->alignment() = IconSymbol::ALIGN_LEFT_BOTTOM;
        else if ( match(c.value(), "center-top")  )   
            style.getOrCreate<IconSymbol>()->alignment() = IconSymbol::ALIGN_CENTER_TOP;
        else if ( match(c.value(), "center-center") ) 
            style.getOrCreate<IconSymbol>()->alignment() = IconSymbol::ALIGN_CENTER_CENTER;
        else if ( match(c.value(), "center-bottom") ) 
            style.getOrCreate<IconSymbol>()->alignment() = IconSymbol::ALIGN_CENTER_BOTTOM;
        else if ( match(c.value(), "right-top") )     
            style.getOrCreate<IconSymbol>()->alignment() = IconSymbol::ALIGN_RIGHT_TOP;
        else if ( match(c.value(), "right-center") )  
            style.getOrCreate<IconSymbol>()->alignment() = IconSymbol::ALIGN_RIGHT_CENTER;
        else if ( match(c.value(), "right-bottom") )  
            style.getOrCreate<IconSymbol>()->alignment() = IconSymbol::ALIGN_RIGHT_BOTTOM;
    }
    else if ( match(c.key(), "icon-heading") ) {
        style.getOrCreate<IconSymbol>()->heading() = NumericExpression(c.value());
    }
    else if ( match(c.key(), "icon-declutter") ) {
        style.getOrCreate<IconSymbol>()->declutter() = as<bool>(c.value(), *defaults.declutter());
    }
    else if ( match(c.key(), "icon-occlusion-cull") ) {
        style.getOrCreate<IconSymbol>()->occlusionCull() = as<bool>(c.value(), *defaults.occlusionCull());
    }
    else if ( match(c.key(), "icon-occlusion-cull-altitude") ) {
        style.getOrCreate<IconSymbol>()->occlusionCullAltitude() = as<float>(c.value(), *defaults.occlusionCullAltitude());
    }
    else if ( match(c.key(), "icon-script") ) {
        style.getOrCreate<IconSymbol>()->script() = StringExpression(c.value());
    }
}
