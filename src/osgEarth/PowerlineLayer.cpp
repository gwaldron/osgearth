/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2018 Pelican Mapping
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

#include <osgEarth/PowerlineLayer>
#include <osgEarth/AltitudeFilter>
#include <osgEarth/JoinPointsLinesFilter>

using namespace osgEarth;

#define LC "[PowerlineLayer]"

#define OE_TEST OE_NULL

REGISTER_OSGEARTH_LAYER(PowerlineModel, PowerlineLayer);

PowerlineLayer::Options::Options()
    : FeatureModelLayer::Options()
{
    fromConfig(_conf);
}

PowerlineLayer::Options::Options(const ConfigOptions& options)
    : FeatureModelLayer::Options(options)
{
    fromConfig(_conf);
}

void PowerlineLayer::Options::fromConfig(const Config& conf)
{
    LayerReference<FeatureSource>::get(conf, "line_features", _lineSourceLayer, _lineSource);
    FeatureDisplayLayout layout = _layout.get();
    layout.cropFeatures() = true;
    _layout = layout;
}

Config
PowerlineLayer::Options::getConfig() const
{
    Config conf = FeatureModelLayer::Options::getConfig();
    LayerReference<FeatureSource>::set(conf, "line_features", _lineSourceLayer, _lineSource);
    return conf;
}

void PowerlineLayer::Options::mergeConfig(const Config& conf)
{
    FeatureModelLayer::Options::mergeConfig(conf);
    fromConfig(conf);
}

class PowerlineFeatureNodeFactory : public GeomFeatureNodeFactory
{
public:
    PowerlineFeatureNodeFactory(const PowerlineLayer::Options& options)
        : GeomFeatureNodeFactory(options),
          _lineSourceLayer(options.lineSourceLayer().get()),
          _lineSource(options.lineSource().get())
        {}
    bool createOrUpdateNode(FeatureCursor* cursor, const Style& style,
                            const FilterContext& context,
                            osg::ref_ptr<osg::Node>& node);
private:
    std::string _lineSourceLayer;
    FeatureSource::Options _lineSource;
};

FeatureNodeFactory*
PowerlineLayer::createFeatureNodeFactoryImplementation() const
{
    return new PowerlineFeatureNodeFactory(options());
}

bool PowerlineFeatureNodeFactory::createOrUpdateNode(FeatureCursor* cursor, const Style& style,
                                                     const FilterContext& context,
                                                     osg::ref_ptr<osg::Node>& node)
{
    FilterContext sharedCX = context;
    FeatureList workingSet; 
    cursor->fill(workingSet);
    osgEarth::Util::JoinPointsLinesFilter pointsLinesFilter;
    pointsLinesFilter.lineSourceLayer() = "lines";
    pointsLinesFilter.lineSource() = _lineSource;
    FilterContext localCX = pointsLinesFilter.push(workingSet, sharedCX);
    // Render towers and lines (cables) seperately
    // Could write another filter for this?
    FeatureList pointSet;
    for (auto& feature : workingSet)
    {
        Geometry* geom = feature->getGeometry();
        if (geom->getType() == Geometry::TYPE_POINTSET)
        {
            pointSet.push_back(feature);
        }
    }
    osg::ref_ptr<FeatureListCursor> listCursor = new FeatureListCursor(pointSet);
    return GeomFeatureNodeFactory::createOrUpdateNode(listCursor.get(), style, localCX, node);
}
