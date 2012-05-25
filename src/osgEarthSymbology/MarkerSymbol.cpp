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
#include <osgEarthSymbology/MarkerSymbol>
#include <osgEarth/ThreadingUtils>
#include <osgEarth/Registry>
#include <osgEarth/ImageUtils>
#include <osgDB/Options>

using namespace osgEarth;
using namespace osgEarth::Symbology;

MarkerSymbol::MarkerSymbol( const Config& conf ) :
Symbol     ( conf ),
_placement ( PLACEMENT_CENTROID ),
_density   ( 25.0f ),
_randomSeed( 0 ),
_alignment( ALIGN_CENTER_BOTTOM )
{
    mergeConfig( conf );
}

Config 
MarkerSymbol::getConfig() const
{
    Config conf = Symbol::getConfig();
    conf.key() = "marker";
    conf.addObjIfSet( "url", _url );
    conf.addObjIfSet( "library", _libraryName );
    conf.addObjIfSet( "scale", _scale );
    conf.addIfSet( "orientation", _orientation);
    conf.addIfSet( "placement", "vertex",   _placement, PLACEMENT_VERTEX );
    conf.addIfSet( "placement", "interval", _placement, PLACEMENT_INTERVAL );
    conf.addIfSet( "placement", "random",   _placement, PLACEMENT_RANDOM );
    conf.addIfSet( "density", _density );
    conf.addIfSet( "random_seed", _randomSeed );
    conf.addIfSet( "is_model", _isModelHint );

	conf.addIfSet( "alignment", "left_top",                _alignment, ALIGN_LEFT_TOP );
	conf.addIfSet( "alignment", "left_center",             _alignment, ALIGN_LEFT_CENTER );
	conf.addIfSet( "alignment", "left_bottom",             _alignment, ALIGN_LEFT_BOTTOM );
	conf.addIfSet( "alignment", "center_top",              _alignment, ALIGN_CENTER_TOP );
	conf.addIfSet( "alignment", "center_center",           _alignment, ALIGN_CENTER_CENTER );
	conf.addIfSet( "alignment", "center_bottom",           _alignment, ALIGN_CENTER_BOTTOM );
	conf.addIfSet( "alignment", "right_top",               _alignment, ALIGN_RIGHT_TOP );
	conf.addIfSet( "alignment", "right_center",            _alignment, ALIGN_RIGHT_CENTER );
	conf.addIfSet( "alignment", "right_bottom",            _alignment, ALIGN_RIGHT_BOTTOM );

	conf.addNonSerializable( "MarkerSymbol::image", _image.get() );
    conf.addNonSerializable( "MarkerSymbol::node", _node.get() );
    return conf;
}

void 
MarkerSymbol::mergeConfig( const Config& conf )
{
    conf.getObjIfSet( "url", _url );
    conf.getObjIfSet( "library", _libraryName );
    conf.getObjIfSet( "scale", _scale );    
    conf.getIfSet( "placement", "vertex",   _placement, PLACEMENT_VERTEX );
    conf.getIfSet( "placement", "interval", _placement, PLACEMENT_INTERVAL );
    conf.getIfSet( "placement", "random",   _placement, PLACEMENT_RANDOM );
    conf.getIfSet( "density", _density );
    conf.getIfSet( "random_seed", _randomSeed );
    conf.getIfSet( "orientation", _orientation);
    conf.getIfSet( "is_model", _isModelHint );

	conf.getIfSet( "alignment", "left_top",                _alignment, ALIGN_LEFT_TOP );
	conf.getIfSet( "alignment", "left_center",             _alignment, ALIGN_LEFT_CENTER );
	conf.getIfSet( "alignment", "left_bottom",             _alignment, ALIGN_LEFT_BOTTOM );
	conf.getIfSet( "alignment", "center_top",              _alignment, ALIGN_CENTER_TOP );
	conf.getIfSet( "alignment", "center_center",           _alignment, ALIGN_CENTER_CENTER );
	conf.getIfSet( "alignment", "center_bottom",           _alignment, ALIGN_CENTER_BOTTOM );
	conf.getIfSet( "alignment", "right_top",               _alignment, ALIGN_RIGHT_TOP );
	conf.getIfSet( "alignment", "right_center",            _alignment, ALIGN_RIGHT_CENTER );
	conf.getIfSet( "alignment", "right_bottom",            _alignment, ALIGN_RIGHT_BOTTOM );

    _image = conf.getNonSerializable<osg::Image>( "MarkerSymbol::image" );
    _node = conf.getNonSerializable<osg::Node>( "MarkerSymbol::node" );
}

osg::Image*
MarkerSymbol::getImage( unsigned maxSize ) const
{
    static Threading::Mutex s_mutex;
    if ( !_image.valid() && _url.isSet() )
    {
        Threading::ScopedMutexLock lock(s_mutex);
        if ( !_image.valid() )
        {
            osg::ref_ptr<osgDB::Options> dbOptions = Registry::instance()->cloneOrCreateOptions();
            dbOptions->setObjectCacheHint( osgDB::Options::CACHE_IMAGES );
            _image = URI(_url->eval(), _url->uriContext()).getImage( dbOptions.get() );
            if ( _image.valid() && (maxSize < (unsigned)_image->s() || maxSize < (unsigned)_image->t()) )
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
