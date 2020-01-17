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
#ifndef OSGEARTH_LAND_COVER_H
#define OSGEARTH_LAND_COVER_H 1

#include <osgEarth/Config>
#include <osgEarth/Layer>
#include <osgEarth/ImageLayer>
#include <vector>

namespace osgEarth
{
    /**
     * A single classification definition of land cover and the coded
     * value that osgEarth uses to represent the class in a coverage raster.
     *
     * For example, "forest"=11 or "water"=230.
     *
     * A collection of these makes up a land cover dictionary.
     * Note: use set/getName to set the classification text that maps to the value.
     */
    class OSGEARTH_EXPORT LandCoverClass : public osg::Object
    {
    public:
        META_Object(osgEarth, LandCoverClass);

        //! Construct an empty land cover class.
        LandCoverClass();

        //! Construct a land cover class with the given name and numerical value
        LandCoverClass(const std::string& name, int value);

        //! Construct a land cover class from a serialized Config.
        LandCoverClass(const Config& conf);

        //! Copy constructor
        LandCoverClass(const LandCoverClass& rhs, const osg::CopyOp& op);

        //! Code value that represents this class in a coverage raster
        void setValue(int value) { _value = value; }
        int getValue() const { return _value; }

    public:
        void fromConfig(const Config& conf);
        Config getConfig() const;

    private:
        int _value;
    };
    typedef std::vector< osg::ref_ptr<LandCoverClass> > LandCoverClassVector;


    /**
     * Configures a LandCoverDictionary layer.
     */
    class OSGEARTH_EXPORT LandCoverDictionaryOptions : public LayerOptions
    {
    public:
        //! Construct new dictionary options
        LandCoverDictionaryOptions(const ConfigOptions& co = ConfigOptions()) :
            LayerOptions(co) { fromConfig(_conf); }
        
        //! Collection of classes defined in this dictionary.
        LandCoverClassVector& classes() { return _landCoverClasses; }
        const LandCoverClassVector& classes() const { return _landCoverClasses; }

        //! Load the options from an XML file
        bool loadFromXML(const URI& uri);
        
    public:
        virtual Config getConfig() const;
        
    protected:
        virtual void mergeConfig(const Config& conf) {
            LayerOptions::mergeConfig(conf);
            fromConfig(conf);
        }
        void fromConfig(const Config& conf);

        
        LandCoverClassVector _landCoverClasses;
    };


    /**
     * Complete set of available land cover classes.
     * Add this to a Map so that land cover facilities can find and use it.
     * Adding more than one LandCoverDictionary to a Map will have undefined results!
     */
    class OSGEARTH_EXPORT LandCoverDictionary : public Layer
    {
    public:
        META_Layer(osgEarth, LandCoverDictionary, LandCoverDictionaryOptions, land_cover_dictionary);

        //! Constructs an empty dictionary */
        LandCoverDictionary();

        //! Constructs a dictionary from serializable options. */
        LandCoverDictionary(const LandCoverDictionaryOptions&);

        //! Add a class.
        void addClass(const std::string& name, int value =INT_MAX);

        //! Load from an XML file.
        bool loadFromXML(const URI& uri);
        
    public:
        LandCoverClassVector& getClasses() { return options().classes(); }
        const LandCoverClassVector& getClasses() const { return options().classes(); }

        //! Gets a land cover class by its name.
        const LandCoverClass* getClassByName(const std::string& name) const;

        //! Gets the land cover class corresponding to the integer value.
        const LandCoverClass* getClassByValue(int value) const;
    };


    /**
     * Maps an integral value from a land cover coverage raster to one of the 
     * land cover classes in the dictionary.
     * For example, 42 -> "tundra".
     */
    class OSGEARTH_EXPORT LandCoverValueMapping : public osg::Object
    {
    public:
        META_Object(osgEarth, LandCoverValueMapping);

        //! Construct a blank mapping
        LandCoverValueMapping();

        //! Construct with values
        LandCoverValueMapping(int value, const std::string& className);

        //! Deserialize a mapping
        LandCoverValueMapping(const Config& conf);

        //! Copy a mapping
        LandCoverValueMapping(const LandCoverValueMapping& rhs, const osg::CopyOp& op);

    public:
        //! Value in the coverage raster to map to a land over class
        void setValue(int value) { _value = value; }
        int getValue() const { return _value.get(); }

        //! Name of the land cover class we are mapping to
        void setLandCoverClassName(const std::string& name) { _lcClassName = name; }
        const std::string& getLandCoverClassName() const { return _lcClassName.get(); }

    public:
        void fromConfig(const Config&);
        Config getConfig() const;

    private:
        optional<int> _value;
        optional<std::string> _lcClassName;
    };
    typedef std::vector< osg::ref_ptr<LandCoverValueMapping> > LandCoverValueMappingVector;
    
    /**
     * Serializable configuration for a LandCoverCoverageLayer.
     */
    class OSGEARTH_EXPORT LandCoverCoverageLayerOptions : public ImageLayerOptions
    {
    public:
        //! Construct the options.
        LandCoverCoverageLayerOptions(const ConfigOptions& co = ConfigOptions());

        //! Collection of value mappings for the coverage layer.
        LandCoverValueMappingVector& mappings() { return _valueMappings; }
        const LandCoverValueMappingVector& mappings() const { return _valueMappings; }

        //! Warping factor for this layer (default is 0.0)
        optional<float>& warp() { return _warp; }
        const optional<float>& warp() const { return _warp; }

        //! Load the data from an XML file
        bool loadMappingsFromXML(const URI& uri);

        //! Add a mapping from a value to a class (convenience)
        void map(int value, const std::string& className);

    public:
        virtual Config getConfig() const;

    protected:
        void fromConfig(const Config& conf);

        virtual void mergeConfig(const Config& conf) {
            ImageLayerOptions::mergeConfig(conf);
            fromConfig(conf);
        }

    private:
        LandCoverValueMappingVector _valueMappings;
        optional<float> _warp;
    };

    /**
     * Component coverage layer of a LandCoverLayer.
     * This layer only lives inside a LandCoverLayer; it makes no sense to add
     * it to a map directly.
     */
    class OSGEARTH_EXPORT LandCoverCoverageLayer : public ImageLayer
    {
    public:
        META_Layer(osgEarth, LandCoverCoverageLayer, LandCoverCoverageLayerOptions, coverage);

        /** Construct an empty land cover coverage layer. */
        LandCoverCoverageLayer();

        /** Deserialize a land cover coverage layer. */
        LandCoverCoverageLayer(const LandCoverCoverageLayerOptions& options);

        //! Code mappings.
        LandCoverValueMappingVector& getMappings() { return options().mappings(); }
        const LandCoverValueMappingVector& getMappings() const { return options().mappings(); }

        //! Land cover dictionary to use.
        void setDictionary(LandCoverDictionary* value) { _lcDictionary = value; }
        LandCoverDictionary* getDictionary() const { return _lcDictionary.get(); }

        //! Load mappings from XML
        bool loadMappingsFromXML(const URI& uri);

    public: // Layer

    private:
        osg::ref_ptr<LandCoverDictionary> _lcDictionary;
    };

} // namespace osgEarth

#endif // OSGEARTH_LAND_COVER_H
