/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
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

#ifndef OSGEARTHFEATURES_RESAMPLE_FILTER_H
#define OSGEARTHFEATURES_RESAMPLE_FILTER_H 1

#include <osgEarthFeatures/Common>
#include <osgEarthFeatures/Feature>
#include <osgEarthFeatures/Filter>
#include <osg/Geode>

namespace osgEarth { namespace Features
{
    using namespace osgEarth;

    class ResampleFilterOptions : public ConfigOptions
    {
    public:
        ResampleFilterOptions(const ConfigOptions& co =ConfigOptions()) : ConfigOptions(co) {
            _minLen.init(0.0);
            _maxLen.init(DBL_MAX);
            _mode.init  (RESAMPLE_LINEAR);
            fromConfig(_conf);
        }

        enum ResampleMode
        {
            RESAMPLE_LINEAR,
            RESAMPLE_GREATCIRCLE,
            RESAMPLE_RHUMB
        };

        optional<double>& minLength() { return _minLen; }
        const optional<double>& minLength() const { return _minLen; }

        optional<double>& maxLength() { return _maxLen; }
        const optional<double>& maxLength() const { return _maxLen; }

        optional<ResampleMode>& resampleMode() { return _mode;}
        const optional<ResampleMode>& resampleMode() const { return _mode; }

        void fromConfig(const Config& conf) {
            conf.get("min_length", _minLen);
            conf.get("max_length", _maxLen);
            conf.get("mode", "linear",       _mode, RESAMPLE_LINEAR);
            conf.get("mode", "great_circle", _mode, RESAMPLE_GREATCIRCLE);
            conf.get("mode", "rhumb_line",   _mode, RESAMPLE_RHUMB);
        }

        Config getConfig() const {
            Config conf = ConfigOptions::getConfig();
            conf.key() = "resample";
            conf.set("min_length", _minLen);
            conf.set("max_length", _maxLen);
            conf.set("mode", "linear",       _mode, RESAMPLE_LINEAR);
            conf.set("mode", "great_circle", _mode, RESAMPLE_GREATCIRCLE);
            conf.set("mode", "rhumb_line",   _mode, RESAMPLE_RHUMB);
            return conf;
        }

    protected:
        optional<double> _minLen, _maxLen, _perturbThresh;
        optional<ResampleMode> _mode;
    };

    /**
     * This filter will resample line segments so they are between
     * the min and max specified length.
     */
    class OSGEARTHFEATURES_EXPORT ResampleFilter : public FeatureFilter,
                                                   public ResampleFilterOptions
    {
    public:
        // Call this determine whether this filter is available.
        static bool isSupported();    

    public:
        ResampleFilter();
        ResampleFilter( double minLength, double maxLength );
        ResampleFilter( const Config& conf );

        virtual ~ResampleFilter() { }

    public:
        virtual FilterContext push( FeatureList& input, FilterContext& context );

    protected:
        bool push( Feature* input, FilterContext& context );
    };

} } // namespace osgEarth::Features

#endif // OSGEARTHFEATURES_RESAMPLE_FILTER_H
