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
#ifndef OSGEARTHUTIL_GEODETIC_GRATICULE_H
#define OSGEARTHUTIL_GEODETIC_GRATICULE_H

#include <osgEarthUtil/Common>
#include <osgEarthUtil/LatLongFormatter>
#include <osgEarthUtil/GeodeticLabelingEngine>
#include <osgEarth/VisibleLayer>
#include <osgEarth/MapNode>
#include <osgEarthSymbology/Style>
#include <osgEarthAnnotation/LabelNode>
#include <osg/ClipPlane>

namespace osgEarth { namespace Util
{
    using namespace osgEarth;
    using namespace osgEarth::Annotation;
    using namespace osgEarth::Symbology;


    class GeodeticGraticuleOptions : public VisibleLayerOptions
    {
    public:
        //! Grid line color
        optional<Color>& color() { return _color; }
        const optional<Color>& color() const { return _color; }

        //! Style for grid labels
        optional<Style>& gridLabelStyle() { return _gridLabelStyle; }
        const optional<Style>& gridLabelStyle() const { return _gridLabelStyle; }

        //! Style for edge labels
        optional<Style>& edgeLabelStyle() { return _edgeLabelStyle; }
        const optional<Style>& edgeLabelStyle() const { return _edgeLabelStyle; }

        //! Grid line width in pixels
        optional<float>& lineWidth() { return _lineWidth; }
        const optional<float>& lineWidth() const { return _lineWidth; }

        //! A target number of grid lines to view on screen at once.
        optional<int>& gridLines() { return _gridLines; }
        const optional<int>& gridLines() const { return _gridLines; }

        /** Resolutions for the graticule separated by spaces
         *  Resolutions are in degrees listed from lowest to highest resolution
         *  For example:  10 5 2.5 1.25 */
        optional<std::string>& resolutions() { return _resolutions; }
        const optional<std::string>& resolutions() const { return _resolutions; }

        optional<bool>& gridLinesVisible() { return _gridLinesVisible; }
        const optional<bool>& gridLinesVisible() const { return _gridLinesVisible; }

        optional<bool>& gridLabelsVisible() { return _gridLabelsVisible; }
        const optional<bool>& gridLabelsVisible() const { return _gridLabelsVisible; }

        optional<bool>& edgeLabelsVisible() { return _edgeLabelsVisible; }
        const optional<bool>& edgeLabelsVisible() const { return _edgeLabelsVisible; }

    public:
        GeodeticGraticuleOptions(const ConfigOptions& opt =ConfigOptions()) : VisibleLayerOptions( opt )
        {
            _lineWidth.init(2.0f);
            _color.init(Color(Color::Yellow, 0.5f));
            _gridLines.init(10);
            _gridLinesVisible.init(true);
            _gridLabelsVisible.init(true);
            _edgeLabelsVisible.init(true);
            _resolutions.init("10 5 2.5 1.0 0.5 0.25 0.125 0.0625 0.3125");
            Style labelStyle;
            TextSymbol* t = labelStyle.getOrCreate<TextSymbol>();
            t->fill()->color().set(1,1,1,1);
            _gridLabelStyle.init(labelStyle);
            _edgeLabelStyle.init(labelStyle);
            fromConfig( _conf );
        }

    public:
        virtual Config getConfig() const {
            Config conf = VisibleLayerOptions::getConfig();
            conf.set("line_width", _lineWidth);
            conf.set("color", _color);
            conf.set("grid_lines", _gridLines);
            conf.set("resolutions", _resolutions);
            conf.set("grid_lines_visible", _gridLinesVisible);
            conf.set("grid_labels_visible", _gridLabelsVisible);
            conf.set("edge_labels_visible", _edgeLabelsVisible);
            conf.set("grid_label_style", _gridLabelStyle);
            conf.set("edge_label_style", _edgeLabelStyle);
            return conf;
        }

    protected:
        virtual void mergeConfig( const Config& conf ) {
            VisibleLayerOptions::mergeConfig( conf );
            fromConfig( conf );
        }

        void fromConfig( const Config& conf ) {
            conf.get("line_width", _lineWidth);
            conf.get("color", _color);
            conf.get("grid_lines", _gridLines);
            conf.get("resolutions", _resolutions);
            conf.get("grid_lines_visible", _gridLinesVisible);
            conf.get("grid_labels_visible", _gridLabelsVisible);
            conf.get("edge_labels_visible", _edgeLabelsVisible);
            conf.get("grid_label_style", _gridLabelStyle);
            conf.get("edge_label_style", _edgeLabelStyle);
        }

        optional<float>       _lineWidth;
        optional<Color>       _color;
        optional<int>         _gridLines;
        optional<std::string> _resolutions;
        optional<bool>        _gridLinesVisible;
        optional<bool>        _gridLabelsVisible;
        optional<bool>        _edgeLabelsVisible;
        optional<Style>       _gridLabelStyle;
        optional<Style>       _edgeLabelStyle;
    };


    /**
     * Graticule that shows lat/long lines and automatically places labels.
     */
    class OSGEARTHUTIL_EXPORT GeodeticGraticule : public VisibleLayer
    {
    public:
        META_Layer(osgEarthUtil, GeodeticGraticule, GeodeticGraticuleOptions, geodetic_graticule);

        //! Construct a graticule with default settings.
        GeodeticGraticule();

        //! Construct a graticule with custom settings.
        GeodeticGraticule(const GeodeticGraticuleOptions& options);

        //! Rebuild the graticule after changing options.
        void dirty();

        //! Get whether to show the grid lines
        bool getGridLinesVisible() const;

        //! Set whether to show the grid lines
        void setGridLinesVisible(bool gridLinesVisible);

        //! Get whether to show the grid labels
        bool getGridLabelsVisible() const;

        //! Set whether to show the grid labels
        void setGridLabelsVisible(bool gridLabelsVisible);

        //! Get whether to show the edge labels
        bool getEdgeLabelsVisible() const;

        //! Set whether to show the edge labels
        void setEdgeLabelsVisible(bool edgeLabelsVisible);

        //! Set the styling to use for grid labels
        void setGridLabelStyle(const Style& style);
        const Style& getGridLabelStyle() const { return options().gridLabelStyle().get(); }

        //! Set ths styling to use for edge labels
        void setEdgeLabelStyle(const Style& style);
        const Style& getEdgeLabelStyle() const { return options().edgeLabelStyle().get(); }

    public: // Layer

        virtual void addedToMap(const Map* map);

        virtual void removedFromMap(const Map* map);

        virtual osg::Node* getNode() const;

        virtual void init();

    public: // VisibleLayer

        virtual void setVisible(bool value);

    public: // osg::Object

        virtual void resizeGLObjectBuffers(unsigned maxSize);

        virtual void releaseGLObjects(osg::State* state) const;

    protected:

        /** dtor */
        virtual ~GeodeticGraticule() { }

    private:

        void rebuild();

        void updateGridLineVisibility();

        UID _uid;
        osg::ref_ptr<const Profile> _profile;
        osg::ref_ptr<osg::Group> _root;
        osg::ref_ptr<const SpatialReference> _mapSRS;
        osg::ref_ptr<osg::NodeCallback > _callback;
        osg::ref_ptr<LatLongFormatter> _formatter;
        float _defaultResolution;
        osg::Vec2f _centerOffset;
        std::vector< double > _resolutions;

        struct CameraData
        {
            osg::ref_ptr<osg::StateSet> _stateset;
            osg::ref_ptr<osg::Uniform> _resolutionUniform;
            osg::ref_ptr<osg::StateSet> _labelStateset;
            std::vector< osg::ref_ptr<LabelNode> > _labelPool;
            float _resolution;
            osg::Matrixd _lastViewMatrix;
            GeoExtent _viewExtent;
            double _lon;
            double _lat;
            double _metersPerPixel;
            void releaseGLObjects(osg::State*) const;
            ~CameraData();
        };
        typedef std::map<osg::Camera*, CameraData> CameraDataMap;
        mutable CameraDataMap _cameraDataMap;
        mutable Threading::Mutex _cameraDataMapMutex;

        CameraData& getCameraData(osg::Camera*) const;

        void initLabelPool(CameraData&);

        std::string getText(const GeoPoint& location, bool lat);

        GeodeticLabelingEngine* _labelingEngine;

        GeoExtent getViewExtent(osgUtil::CullVisitor*) const;


    public:

        void updateLabels();
        void cull(osgUtil::CullVisitor*);
        osg::StateSet* getStateSet(osgUtil::CullVisitor* cv);

    private:
        osg::observer_ptr<MapNode> _mapNode;
        void setMapNode(MapNode*);

        struct MyGroup : public osg::Group {
            MyGroup(GeodeticGraticule*);
            GeodeticGraticule* _graticule;
            void traverse(osg::NodeVisitor& nv); 
        };
        friend struct MyGroup;
    };
} } // namespace osgEarth::Util

#endif // OSGEARTHUTIL_GEODETIC_GRATICULE_H
