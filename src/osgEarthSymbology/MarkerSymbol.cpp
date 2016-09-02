/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
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
#include <osgEarthSymbology/IconSymbol>
#include <osgEarthSymbology/ModelSymbol>
#include <osgEarthSymbology/Style>
#include <osgEarth/ThreadingUtils>
#include <osgEarth/Registry>
#include <osgEarth/ImageUtils>

#include <osgDB/Options>
#include <osgDB/FileNameUtils>
#include <osgDB/Registry>

using namespace osgEarth;
using namespace osgEarth::Symbology;

OSGEARTH_REGISTER_SIMPLE_SYMBOL(marker, MarkerSymbol);

MarkerSymbol::MarkerSymbol(const MarkerSymbol& rhs,const osg::CopyOp& copyop):
Symbol(rhs, copyop),
_url(rhs._url),
_library(rhs._library),
_scale(rhs._scale),
_placement(rhs._placement),
_orientation(rhs._orientation),
_density(rhs._density),
_randomSeed(rhs._randomSeed),
_isModelHint(rhs._isModelHint),
_alignment(rhs._alignment),
_node(rhs._node),
_image(rhs._image)
{
}

MarkerSymbol::MarkerSymbol( const Config& conf ) :
Symbol     ( conf ),
_placement ( PLACEMENT_CENTROID ),
_density   ( 25.0f ),
_randomSeed( 0 ),
_alignment ( ALIGN_CENTER_BOTTOM )
{
    mergeConfig( conf );
}

Config 
MarkerSymbol::getConfig() const
{
    Config conf = Symbol::getConfig();
    conf.key() = "marker";
    conf.addObjIfSet( "url", _url );
    conf.addObjIfSet( "library", _library );
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
    conf.getObjIfSet( "library", _library );
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


InstanceSymbol*
MarkerSymbol::convertToInstanceSymbol() const
{
    InstanceSymbol* result = 0L;

    bool isModel = true;

    if ( this->isModel().isSet() )
    {
        isModel = *this->isModel();
    }
    else if ( this->getModel() )
    {
        isModel = true;
    }
    else if ( this->url().isSet() )
    {
        const std::string& str = this->url()->expr();
        std::string ext = osgDB::getLowerCaseFileExtension(str);
        if ( !ext.empty() )
        {
            osg::ref_ptr<osgDB::ReaderWriter> rw = osgDB::Registry::instance()->getReaderWriterForExtension(ext);
            if ( rw.valid() )
            {
                unsigned features = (unsigned)rw->supportedFeatures();
                if ( (features & osgDB::ReaderWriter::FEATURE_READ_IMAGE) != 0 )
                {
                    isModel = false;
                }
            }

#if 0 // original method-- but getMimeTypeExtensionMap didn't exist until post-3.0
            const osgDB::Registry::MimeTypeExtensionMap& exmap = osgDB::Registry::instance()->getMimeTypeExtensionMap();
            for( osgDB::Registry::MimeTypeExtensionMap::const_iterator i = exmap.begin(); i != exmap.end(); ++i )
            {
                if ( i->second == ext )
                {
                    if ( i->first.compare(0, 6, "image/") == 0 )
                        isModel = false;
                    break;
                }
            }
#endif

        }
    }

    if ( isModel )
    {
        ModelSymbol* model = new ModelSymbol();

        if ( this->orientation().isSet() )
            model->heading() = NumericExpression(this->orientation()->x());

        if ( model->getModel() )
            model->setModel( this->getModel() );

        result = model;
    }
    else // icon image
    {
        IconSymbol* icon = new IconSymbol();

        if ( this->alignment().isSet() )
            icon->alignment() = (IconSymbol::Alignment)this->alignment().get();

        if ( this->getImage() )
            icon->setImage( this->getImage() );

        result = icon;
    }

    if ( this->url().isSet() )
        result->url() = this->url().get();
    if ( this->library().isSet() )
        result->library() = this->library().get();
    if ( this->placement().isSet() )
        result->placement() = (InstanceSymbol::Placement)this->placement().get();
    if ( this->density().isSet() )
        result->density() = this->density().get();
    if ( this->scale().isSet() )
        result->scale() = this->scale().get();
    if ( this->randomSeed().isSet() )
        result->randomSeed() = this->randomSeed().get();

    return result;
}


void
MarkerSymbol::parseSLD(const Config& c, Style& style)
{
    if ( match(c.key(), "marker") ) {
        style.getOrCreate<MarkerSymbol>()->url() = c.value();
        style.getOrCreate<MarkerSymbol>()->url()->setURIContext( c.referrer() );
    }
    else if ( match(c.key(),"marker-library") ) {
        style.getOrCreate<MarkerSymbol>()->library() = StringExpression(c.value());
    }
    else if ( match(c.key(), "marker-placement") ) {
        if      ( match(c.value(), "vertex") )   
            style.getOrCreate<MarkerSymbol>()->placement() = MarkerSymbol::PLACEMENT_VERTEX;
        else if ( match(c.value(), "interval") ) 
            style.getOrCreate<MarkerSymbol>()->placement() = MarkerSymbol::PLACEMENT_INTERVAL;
        else if ( match(c.value(), "random") )   
            style.getOrCreate<MarkerSymbol>()->placement() = MarkerSymbol::PLACEMENT_RANDOM;
        else if ( match(c.value(), "centroid") ) 
            style.getOrCreate<MarkerSymbol>()->placement() = MarkerSymbol::PLACEMENT_CENTROID;
    }
    else if ( match(c.key(), "marker-density") ) {
        style.getOrCreate<MarkerSymbol>()->density() = as<float>(c.value(), 1.0f);
    }
    else if ( match(c.key(), "marker-random-seed") ) {
        style.getOrCreate<MarkerSymbol>()->randomSeed() = as<unsigned>(c.value(), 0);
    }
    else if ( match(c.key(), "marker-scale") ) {
        style.getOrCreate<MarkerSymbol>()->scale() = NumericExpression(c.value());
    }
    else if ( match(c.key(), "marker-icon-align") ) {
        if      ( match(c.value(), "left-top") ) 
            style.getOrCreate<MarkerSymbol>()->alignment() = MarkerSymbol::ALIGN_LEFT_TOP;
        else if ( match(c.value(), "left-center") ) 
            style.getOrCreate<MarkerSymbol>()->alignment() = MarkerSymbol::ALIGN_LEFT_CENTER;
        else if ( match(c.value(), "left-bottom") ) 
            style.getOrCreate<MarkerSymbol>()->alignment() = MarkerSymbol::ALIGN_LEFT_BOTTOM;
        else if ( match(c.value(), "center-top")  ) 
            style.getOrCreate<MarkerSymbol>()->alignment() = MarkerSymbol::ALIGN_CENTER_TOP;
        else if ( match(c.value(), "center-center") ) 
            style.getOrCreate<MarkerSymbol>()->alignment() = MarkerSymbol::ALIGN_CENTER_CENTER;
        else if ( match(c.value(), "center-bottom") ) 
            style.getOrCreate<MarkerSymbol>()->alignment() = MarkerSymbol::ALIGN_CENTER_BOTTOM;
        else if ( match(c.value(), "right-top") ) 
            style.getOrCreate<MarkerSymbol>()->alignment() = MarkerSymbol::ALIGN_RIGHT_TOP;
        else if ( match(c.value(), "right-center") ) 
            style.getOrCreate<MarkerSymbol>()->alignment() = MarkerSymbol::ALIGN_RIGHT_CENTER;
        else if ( match(c.value(), "right-bottom") ) 
            style.getOrCreate<MarkerSymbol>()->alignment() = MarkerSymbol::ALIGN_RIGHT_BOTTOM;
    }
}

