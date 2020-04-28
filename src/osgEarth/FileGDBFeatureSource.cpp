/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2008-2014 Pelican Mapping
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
#ifdef OSGEARTH_HAVE_FILEGDB

#include <osgEarth/FileGDBFeatureSource>
#include "FileGDBAPI.h"
#include <locale>
#include <codecvt>

using namespace osgEarth;

#define LC "[FileGDB] "

//........................................................................

Config
FileGDBFeatureSource::Options::getConfig() const
{
    Config conf = FeatureSource::Options::getConfig();
    conf.set("url", url());
    conf.set("table", table());
    return conf;
}

void
FileGDBFeatureSource::Options::fromConfig(const Config& conf)
{
    conf.get("url", url());
    conf.get("table", table());
}

//........................................................................

REGISTER_OSGEARTH_LAYER(filegdbfeatures, FileGDBFeatureSource);

OE_LAYER_PROPERTY_IMPL(FileGDBFeatureSource, URI, URL, url);
OE_LAYER_PROPERTY_IMPL(FileGDBFeatureSource, std::string, Table, table);

namespace
{
    std::string WtoS(const std::wstring& in) {
        return std::string(in.begin(), in.end());
    }
    std::wstring StoW(const std::string& in) {
        return std::wstring(in.begin(), in.end());
    }
}

void
FileGDBFeatureSource::init()
{
    FeatureSource::init();
    _db = NULL;
    _table = NULL;
}

Status
FileGDBFeatureSource::openImplementation()
{
    Status parent = FeatureSource::openImplementation();
    if (parent.isError())
        return parent;

    if (!options().table().isSet())
        return setStatus(Status::ConfigurationError, "Missing required table name");

    close();

    _db = new FileGDBAPI::Geodatabase();
    FileGDBAPI::Geodatabase& db = *(FileGDBAPI::Geodatabase*)_db;

    std::wstring e;

    // Open the database:
    std::wstring wideurl( StoW(options().url()->full()) );
    fgdbError hr = FileGDBAPI::OpenGeodatabase(wideurl, db);
    if (hr != S_OK)
    {
        std::wstring e;
        FileGDBAPI::ErrorInfo::GetErrorDescription(hr, e);
        return setStatus(Status::ResourceUnavailable, WtoS(e));
    }
    OE_INFO << LC << "Opened geodatabase " << options().url()->base() << " OK" << std::endl;
    
    // Query the datasets: (no)
    std::wstring parentPath = L"\\";
    std::wstring datasetType = L"Feature Class";
    std::vector<std::wstring> datasets;
    if ((hr = db.GetChildDatasets(parentPath, datasetType, datasets)) != S_OK)
    {
        FileGDBAPI::ErrorInfo::GetErrorDescription(hr, e);
        return setStatus(Status::ResourceUnavailable, WtoS(e));
    }
    OE_INFO << LC << "Found feature classes:" << std::endl;
    for(std::vector<std::wstring>::const_iterator i = datasets.begin();
        i != datasets.end(); 
        ++i)
    {
        OE_INFO << LC << "    " << WtoS(*i) << std::endl;
    }

    // Open the table:
    std::wstring wTableName = L"\\" + StoW(options().table().get());

    _table = new FileGDBAPI::Table();
    FileGDBAPI::Table& table = *(FileGDBAPI::Table*)_table;

    if ((hr = db.OpenTable(wTableName, table)) != S_OK)
    {
        delete _table;
        _table = NULL;
        FileGDBAPI::ErrorInfo::GetErrorDescription(hr, e);
        return setStatus(Status::ResourceUnavailable, WtoS(e));
    }
    OE_INFO << LC << "Opened table " << options().table().get() << " OK" << std::endl;

    // Determine the schema and the feature profile
    std::vector<FileGDBAPI::FieldDef> fieldDefs;
    table.GetFields(fieldDefs);
    for(int i=0; i<fieldDefs.size(); ++i)
    {
        std::wstring wname;
        fieldDefs[i].GetName(wname);
        if (wname == L"Shape")
        {
            FileGDBAPI::GeometryDef geomDef;
            fieldDefs[i].GetGeometryDef(geomDef);
            FileGDBAPI::SpatialReference srs;
            geomDef.GetSpatialReference(srs);
        }
        else
        {
            std::string name = WtoS(wname);
            FileGDBAPI::FieldType type;
            fieldDefs[i].GetType(type);
            switch(type)
            {
            case FileGDBAPI::fieldTypeSingle:
            case FileGDBAPI::fieldTypeDouble:
                _schema[name] = ATTRTYPE_DOUBLE; break;
            case FileGDBAPI::fieldTypeInteger:
                _schema[name] = ATTRTYPE_INT; break;
            case FileGDBAPI::fieldTypeString:
                _schema[name] = ATTRTYPE_STRING; break;
            default:
                _schema[name] = ATTRTYPE_UNSPECIFIED;
            }
        }
    }


    return Status::NoError;
}

void
FileGDBFeatureSource::close()
{
    if (_db)
    {
        FileGDBAPI::Geodatabase& db = *(FileGDBAPI::Geodatabase*)_db;
        if (_table)
        {
            FileGDBAPI::Table& table = *(FileGDBAPI::Table*)_table;
            db.CloseTable(table);
            delete _table;
            _table = NULL;
        }
        FileGDBAPI::CloseGeodatabase(db);
        delete _db;
    }
    _db = NULL;
}

FeatureCursor*
FileGDBFeatureSource::createFeatureCursorImplementation(const Query& query, ProgressCallback* progress)
{
    if (!_db || !_table || getStatus().isError())
        return NULL;

    FileGDBAPI::Geodatabase& db = *(FileGDBAPI::Geodatabase*)_db;
    FileGDBAPI::Table& table = *(FileGDBAPI::Table*)_table;

    FileGDBAPI::Envelope envelope;
    FileGDBAPI::EnumRows enumRows;
    fgdbError hr;
    
    if ((hr = table.Search(L"*", L"", envelope, true, enumRows)) != S_OK)
    {
        std::wstring e;
        FileGDBAPI::ErrorInfo::GetErrorDescription(hr, e);
        OE_WARN << LC << "Error: " << WtoS(e) << std::endl;
        return NULL;
    }

    FileGDBAPI::Row row;
    FileGDBAPI::MultiPatchShapeBuffer shapeBuf;

    while(enumRows.Next(row) == S_OK)
    {
        std::vector<FileGDBAPI::FieldDef> fieldDefs;
        row.GetFields(fieldDefs);

        row.GetGeometry(shapeBuf);
        
        int* ids;
        shapeBuf.GetIDs(ids);

        int* parts;
        shapeBuf.GetParts(parts);

        float* normals;
        shapeBuf.GetNormals(normals);

        int* partDescriptors;
        shapeBuf.GetPartDescriptors(partDescriptors);

        //osgEarth::Geometry* 
    }

    return NULL;
}

#endif // OSGEARTH_HAVE_FILEGDB
