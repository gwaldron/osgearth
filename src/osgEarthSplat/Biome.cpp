/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/
#include "Biome"

using namespace osgEarth;
using namespace osgEarth::Splat;

//..........................................................

void AssetUsage::Options::fromConfig(const Config& conf)
{
    _initConfig = conf;

    width().setDefault(5.0f);
    height().setDefault(10.0f);
    sizeVariation().setDefault(0.0f);
    selectionWeight().setDefault(1.0f);
    fill().setDefault(1.0f);

    conf.get("url", sideBillboardURI());
    conf.get("top_url", topBillboardURI());
    conf.get("model", modelURI());
    conf.get("width", width());
    conf.get("height", height());
    conf.get("size_variation", sizeVariation());
    conf.get("selection_weight", selectionWeight());
    conf.get("weight", selectionWeight());
    conf.get("fill", fill());
}

Config AssetUsage::Options::getConfig() const
{
    //Config conf("billboard");
    Config conf("asset");
    conf.set("url", sideBillboardURI());
    conf.set("top_url", topBillboardURI());
    conf.set("model", modelURI());
    conf.set("width", width());
    conf.set("height", height());
    conf.set("size_variation", sizeVariation());
    conf.set("selection_weight", selectionWeight());
    conf.set("fill", fill());
    return conf;
}

//..........................................................

void LandCoverGroup::Options::fromConfig(const Config& conf)
{
    fill().setDefault(1.0f);
    sizeVariation().setDefault(0.0f);

    conf.get("classes", landCoverClasses());
    conf.get("fill", fill());
    conf.get("size_variation", sizeVariation());
    for (ConfigSet::const_iterator i = conf.children().begin(); i != conf.children().end(); ++i)
    {
        if (i->key() == "asset" || i->key() == "billboard")
            assets().push_back(AssetUsage(*i));
    }
}

Config LandCoverGroup::Options::getConfig() const
{
    Config conf("biome");
    conf.set("classes", landCoverClasses());
    conf.set("fill", fill());
    conf.set("size_variation", sizeVariation());
    for (int i = 0; i < assets().size(); ++i)
    {
        conf.add("asset", assets()[i].getConfig());
    }
    return conf;
}

void
LandCoverGroup::init()
{
    _classNames = Strings::StringTokenizer()
        .whitespaceDelims()
        .standardQuotes()
        .tokenize(options().landCoverClasses().get());
}

//..........................................................

void BiomeZone::Options::fromConfig(const Config& conf)
{
    conf.get("name", name());
    const Config* bconf = conf.child_ptr("boundaries");
    if ( bconf ) {
        for(ConfigSet::const_iterator i = bconf->children().begin(); i != bconf->children().end(); ++i) {
            boundaries().push_back(osg::BoundingBox(
                i->value("xmin", -FLT_MAX), i->value("ymin", -FLT_MAX), i->value("zmin", -FLT_MAX),
                i->value("xmax",  FLT_MAX), i->value("ymax",  FLT_MAX), i->value("zmax",  FLT_MAX)));
        }
    }

    const Config& gcConf = conf.child("groundcover");
    fill().setDefault(1.0f);
    spacing().setDefault(Distance(20.0, Units::METERS));

    gcConf.get("spacing", spacing());
    gcConf.get("fill", fill());
    gcConf.get("max_distance", maxDistance());
    const ConfigSet biomes = gcConf.child("biomes").children("biome");
    for (ConfigSet::const_iterator i = biomes.begin(); i != biomes.end(); ++i)
    {
        if (i->key() == "biome")
            landCoverGroups().push_back(LandCoverGroup(*i));
    }
}

Config BiomeZone::Options::getConfig() const
{
    // old skool:
    Config conf("zone");
    conf.set("name", name());
    if ( boundaries().size() > 0 ) {
        Config regions("boundaries");
        for(int i=0; i<boundaries().size(); ++i) {
            Config region("boundary");
            if ( boundaries()[i].xMin() > -FLT_MAX ) region.set("xmin", boundaries()[i].xMin());
            if ( boundaries()[i].yMin() > -FLT_MAX ) region.set("ymin", boundaries()[i].yMin());
            if ( boundaries()[i].zMin() > -FLT_MAX ) region.set("zmin", boundaries()[i].zMin());
            if ( boundaries()[i].xMax() <  FLT_MAX ) region.set("xmax", boundaries()[i].xMax());
            if ( boundaries()[i].yMax() <  FLT_MAX ) region.set("ymax", boundaries()[i].yMax());
            if ( boundaries()[i].zMax() <  FLT_MAX ) region.set("zmax", boundaries()[i].zMax());
            regions.add(region);
        }
        conf.set(regions);
    }

    Config gcConf("groundcover");
    gcConf.set("name", name());
    gcConf.set("spacing", spacing());
    gcConf.set("fill", fill());
    gcConf.set("max_distance", maxDistance());
    Config biomesConf("biomes");
    for (int i = 0; i < landCoverGroups().size(); ++i)
        biomesConf.add("biome", landCoverGroups()[i].getConfig());
    if (!biomesConf.empty())
        gcConf.add(biomesConf);
    conf.add(gcConf);

    return conf;
}

void
BiomeZone::init()
{
    for(int i=0; i<options().boundaries().size(); ++i)
    {
        const osg::BoundingBox& box = _options.boundaries()[i];
        _boundaries.push_back( Boundary() );
        Boundary& b = _boundaries.back();

        b.extent = GeoExtent(
            SpatialReference::get("wgs84"),
            osg::clampBetween(static_cast<float>(box.xMin()), -180.0f, 180.0f),
            osg::clampBetween(static_cast<float>(box.yMin()),  -90.0f,  90.0f),
            osg::clampBetween(static_cast<float>(box.xMax()), -180.0f, 180.0f),
            osg::clampBetween(static_cast<float>(box.yMax()),  -90.0f,  90.0f));

        b.extent.createPolytope( b.tope );
        b.zmin2 = box.zMin() > -FLT_MAX ? box.zMin()*box.zMin() : box.zMin();
        b.zmax2 = box.zMax() <  FLT_MAX ? box.zMax()*box.zMax() : box.zMax();

        // this only needs to be very approximate.
        double meanRadius = b.extent.getSRS()->isGeographic() ?
            b.extent.getSRS()->getEllipsoid().getRadiusEquator() : 0.0;
        b.meanRadius2 = meanRadius*meanRadius;
    }
}

const LandCoverGroup* 
BiomeZone::getLandCoverGroup(const LandCoverClass* lc) const
{
    for(unsigned i=0; i<getLandCoverGroups().size(); ++i)
    {
        const LandCoverGroup& group = getLandCoverGroups()[i];
        if (std::find(group.getLandCoverClassNames().begin(), group.getLandCoverClassNames().end(), lc->getName()) != group.getLandCoverClassNames().end())
            return &group;
    }
    return NULL;
}

bool
BiomeZone::contains(const osg::Vec3& point) const
{
    for(Boundaries::const_iterator b = _boundaries.begin();
        b != _boundaries.end(); 
        ++b)
    {
        if ( b->tope.empty() )
        {
            return true;
        }

        else if ( b->tope.contains(point) )
        {
            double hat2 = point.length2() - b->meanRadius2; // assumes round earth
            if ( hat2 >= b->zmin2 && hat2 <= b->zmax2 )
            {
                return true;
            }
        }
    }

    return false;
}



bool
BiomeZone::contains(const GeoPoint& point) const
{
    for(Boundaries::const_iterator b = _boundaries.begin();
        b != _boundaries.end(); 
        ++b)
    {
        if (b->extent.contains(point.x(), point.y()))
        {
            return true;
        }
    }

    return false;
}