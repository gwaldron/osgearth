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
#ifndef OSGEARTH_ANNOTATION_ANNOTATION_LAYER
#define OSGEARTH_ANNOTATION_ANNOTATION_LAYER

#include <osgEarth/VisibleLayer>
#include <osgEarthAnnotation/Common>

namespace osgEarth { namespace Annotation
{
    using namespace osgEarth;

    class AnnotationNode;

    /**
     * Configuration options for AnnotationLayer
     */
    class AnnotationLayerOptions : public VisibleLayerOptions
    {
    public:
        AnnotationLayerOptions(const ConfigOptions& conf =ConfigOptions()) : VisibleLayerOptions(conf) {
            fromConfig(_conf);
        }

    public:
        virtual Config getConfig() const {
            Config conf = VisibleLayerOptions::getConfig();
            return conf;
        }

    protected:

        virtual void mergeConfig(const Config& conf) {
            VisibleLayerOptions::mergeConfig(conf);
            fromConfig(conf);
        }

        void fromConfig(const Config& conf) {
            //nop
        }
    };

    /**
     * Layer that holds annotation nodes.
     *
     * Use with the options structure is intended for loading from an earth file.
     * To use from the API, you can just call getGroup() and add Annotations there.
     */
    class OSGEARTHANNO_EXPORT AnnotationLayer : public VisibleLayer
    {
    public:
        META_Layer(osgEarthAnnotation, AnnotationLayer, AnnotationLayerOptions, annotations);

        //! Construct a default layer
        AnnotationLayer();

        //! Construct a layer with custom options
        AnnotationLayer(const AnnotationLayerOptions& options);

        //! Adds an annotation to the layer
        void addChild(AnnotationNode*);

        //! Gets the group to which you can add annotations
        osg::Group* getGroup() const;

    public: // Layer
        
        virtual osg::Node* getNode() const;

        virtual void init();

    protected:

        /** dtor */
        virtual ~AnnotationLayer() { }        

    private:

        osg::ref_ptr<osg::Group> _root;

        void deserialize();
    };  
} } // namespace osgEarth::Annotation

#endif // OSGEARTH_ANNOTATION_ANNOTATION_LAYER
