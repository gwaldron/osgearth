/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include <osgEarth/BillboardSymbol>
#include <osgEarth/BillboardResource>
#include <osgEarth/Style>
#include <osgEarth/URI>

using namespace osgEarth;

OSGEARTH_REGISTER_SIMPLE_SYMBOL(billboard, BillboardSymbol);


BillboardSymbol::BillboardSymbol(const BillboardSymbol& rhs,const osg::CopyOp& copyop):
InstanceSymbol(rhs, copyop)
{
    width().init(15.0f);
    height().init(10.0f);
    sizeVariation().init(0.0f);
    selectionWeight().init(1);
}

BillboardSymbol::BillboardSymbol( const Config& conf ) :
InstanceSymbol( conf )
{
    width().init(15.0f);
    height().init(10.0f);
    sizeVariation().init(0.0f);
    selectionWeight().init(1);

    mergeConfig( conf );
}

Config 
BillboardSymbol::getConfig() const
{
    Config conf = InstanceSymbol::getConfig();
    conf.key() = "billboard";
    conf.set( "width", _width );
    conf.set( "height", _height );
    conf.set( "size_variation", _sizeVariation );
    conf.set( "selection_weight", _selectionWeight );
    conf.set( "top_url", _topURL );
    return conf;
}

void 
BillboardSymbol::mergeConfig( const Config& conf )
{
    conf.get( "width", _width );
    conf.get( "height", _height );
    conf.get( "size_variation", _sizeVariation );
    conf.get( "selection_weight", _selectionWeight );
    conf.get( "top_url", _topURL );
}

InstanceResource*
BillboardSymbol::createResource() const
{
    return new BillboardResource();
}

void
BillboardSymbol::parseSLD(const Config& c, Style& style)
{
    if (match(c.key(), "library")) {
        if (!c.value().empty())
            style.getOrCreate<SkinSymbol>()->library() = Strings::unquote(c.value());
    }
    else
    if ( match(c.key(), "billboard-image") ) {
        style.getOrCreate<BillboardSymbol>()->url() = StringExpression(c.value(), c.referrer());
    }
    else if (match(c.key(), "billboard-top-image")) {
        style.getOrCreate<BillboardSymbol>()->topURL() = StringExpression(c.value(), c.referrer());
    }
    else if ( match(c.key(), "billboard-width") ) {
        style.getOrCreate<BillboardSymbol>()->width() = as<float>(c.value(), 10.0f);
    }
    else if ( match(c.key(), "billboard-height") ) {
        style.getOrCreate<BillboardSymbol>()->height() = as<float>(c.value(), 10.0f);
    }
    else if ( match(c.key(), "billboard-size-variation") ) {
        style.getOrCreate<BillboardSymbol>()->sizeVariation() = as<float>(c.value(), 0.0f);
    }
    else if ( match(c.key(), "billboard-selection-weight") ) {
        style.getOrCreate<BillboardSymbol>()->selectionWeight() = as<unsigned>(c.value(), 1u);
    }
}

