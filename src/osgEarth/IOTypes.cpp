/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2008-2013 Pelican Mapping
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
#include <osgEarth/IOTypes>
#include <osgEarth/URI>
#include <osgEarth/XmlUtils>
#include <osgDB/FileNameUtils>
#include <osgDB/ReaderWriter>
#include <osgDB/Registry>

using namespace osgEarth;

//------------------------------------------------------------------------

const std::string IOMetadata::CONTENT_TYPE = "Content-type";

//------------------------------------------------------------------------

StringObject::StringObject() :
osg::Object()
{
    //nop
}

StringObject::~StringObject()
{
}

const std::string& StringObject::getString() const
{
    return _str;
}

void StringObject::setString( const std::string& value )
{
    _str = value;
}

//------------------------------------------------------------------------

URIReadCallback::URIReadCallback()
{
    //nop
}

URIReadCallback::~URIReadCallback()
{
}

//------------------------------------------------------------------------

/**
 * Registerd "StringObject" with OSG's serialization framework. Basically
 * that means that StringObject instances can be read/written to an .osgb
 * file. We use this for caching string data (XML, JSON files for example).
 */
REGISTER_OBJECT_WRAPPER(StringObject,
                        new osgEarth::StringObject,
                        osgEarth::StringObject,
                        "osgEarth::StringObject")
{
    ADD_STRING_SERIALIZER( String, "" );  // _str
}

//------------------------------------------------------------------------

/**
 * This is an OSG reader/writer template. We're using this to register 
 * readers for "XML" and "JSON" format -- these are just text files, but we
 * have to do this to enable OSG to read them from inside an archive (like
 * a ZIP file). For example, this lets OSG read the tilemap.xml file stores
 * with a TMS repository in a ZIP.
 */

#define STRING_READER_WRITER_SHIM(SUFFIX, EXTENSION, DEF) \
struct osgEarthStringReaderWriter##SUFFIX : public osgDB::ReaderWriter \
{ \
    osgEarthStringReaderWriter##SUFFIX () { \
        supportsExtension( EXTENSION, DEF ); \
    } \
    osgDB::ReaderWriter::ReadResult readObject(const std::string& uri, const osgDB::Options* dbOptions) const { \
        std::string ext = osgDB::getLowerCaseFileExtension( uri ); \
        if ( !acceptsExtension(ext) ) return osgDB::ReaderWriter::ReadResult::FILE_NOT_HANDLED; \
        osgEarth::ReadResult r = URI( uri ).readString( dbOptions ); \
        if ( r.succeeded() ) return r.release<StringObject>(); \
        return osgDB::ReaderWriter::ReadResult::ERROR_IN_READING_FILE; \
    } \
    osgDB::ReaderWriter::ReadResult readObject(std::istream& in, const osgDB::Options* dbOptions ) const { \
        URIContext uriContext( dbOptions ); \
        return new StringObject( Stringify() << in.rdbuf() ); \
    } \
    osgDB::ReaderWriter::WriteResult writeObject( const osg::Object& obj, const std::string& location, const osgDB::Options* dbOptions ) const { \
        std::string ext = osgDB::getLowerCaseFileExtension(location); \
        if ( !acceptsExtension(ext) ) return osgDB::ReaderWriter::WriteResult::FILE_NOT_HANDLED; \
        const StringObject* so = dynamic_cast<const StringObject*>(&obj); \
        if ( !so ) return osgDB::ReaderWriter::WriteResult::FILE_NOT_HANDLED; \
        std::ofstream out(location.c_str()); \
        if ( out.is_open() ) { out << so->getString(); out.close(); return osgDB::ReaderWriter::WriteResult::FILE_SAVED; } \
        return osgDB::ReaderWriter::WriteResult::ERROR_IN_WRITING_FILE; \
    } \
}

STRING_READER_WRITER_SHIM( XML, "xml", "osgEarth XML shim" );
REGISTER_OSGPLUGIN( xml, osgEarthStringReaderWriterXML );

STRING_READER_WRITER_SHIM( JSON, "json", "osgEarth JSON shim" );
REGISTER_OSGPLUGIN( json, osgEarthStringReaderWriterJSON );
