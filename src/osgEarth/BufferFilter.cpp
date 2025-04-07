/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <osgEarth/BufferFilter>
#include <osgEarth/FilterContext>
#include <osgEarth/Notify>

#define LC "[BufferFilter] "

using namespace osgEarth;

bool
BufferFilter::isSupported()
{
#ifdef OSGEARTH_HAVE_GEOS
    return true;
#else
    return false;
#endif
}

#define ASSERT_SUPPORT() \
    if ( !BufferFilter::isSupported() ) { \
        OE_NOTICE << "BufferFilter NOT SUPPORTED - please compile osgEarth with GEOS" << std::endl; }

OSGEARTH_REGISTER_SIMPLE_FEATUREFILTER(buffer, BufferFilter );

BufferFilter::BufferFilter() :
_distance   ( 1.0 ),
_numQuadSegs( 0 ),
_capStyle   ( Stroke::LINECAP_SQUARE )
{
    //NOP
}

BufferFilter::BufferFilter( const BufferFilter& rhs ) :
_distance   ( rhs._distance ),
_numQuadSegs( rhs._numQuadSegs ),
_capStyle   ( rhs._capStyle )
{
    //NOP
}

BufferFilter::BufferFilter( const Config& conf ) :
_distance   ( 1.0 ),
_numQuadSegs( 0 ),
_capStyle   ( Stroke::LINECAP_SQUARE )
{
    if (conf.key() == "buffer")
    {
        conf.get( "distance", _distance );
    }
}

Config BufferFilter::getConfig() const
{
    Config config( "buffer" );
    config.set( "distance", _distance);
    return config;
}

FilterContext
BufferFilter::push( FeatureList& input, FilterContext& context )
{
    if ( !isSupported() )
    {
        OE_WARN << "BufferFilter support not enabled - please compile osgEarth with GEOS" << std::endl;
        return context;
    }

    FeatureList output;
    output.reserve(input.size());

    //OE_NOTICE << "Buffer: input = " << input.size() << " features" << std::endl;
    for(auto& feature : input)
    {
        if ( !feature.valid() || !feature->getGeometry() )
            continue;

        osg::ref_ptr<Geometry> geom;

        BufferParameters params;
        
        params._capStyle =
                _capStyle == Stroke::LINECAP_ROUND  ? BufferParameters::CAP_ROUND :
                _capStyle == Stroke::LINECAP_SQUARE ? BufferParameters::CAP_SQUARE :
                _capStyle == Stroke::LINECAP_FLAT   ? BufferParameters::CAP_FLAT :
                                                      BufferParameters::CAP_SQUARE;

        params._cornerSegs = _numQuadSegs;

        if (feature->getGeometry()->buffer(_distance.value(), geom, params))
        {
            feature->setGeometry(geom.get());
            output.emplace_back(feature);
        }
        else
        {
            OE_DEBUG << LC << "feature " << feature->getFID() << " yielded no geometry" << std::endl;
        }
    }

    output.swap(input);

    return context;
}
