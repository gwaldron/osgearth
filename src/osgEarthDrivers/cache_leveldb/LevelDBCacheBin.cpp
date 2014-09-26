/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
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
#include "LevelDBCacheBin"
#include <osgEarth/Cache>
#include <osgEarth/Registry>
#include <osgEarth/Random>
#include <osgDB/Registry>
#include <leveldb/write_batch.h>
#include <string>

using namespace osgEarth;
using namespace osgEarth::Threading;
using namespace osgEarth::Drivers::LevelDBCache;

//------------------------------------------------------------------------

namespace
{
    void encodeMeta(const Config& meta, std::string& out)
    {
        out = Stringify() << meta.toJSON(false);
    }

    void decodeMeta(const std::string& in, Config& meta)
    {
        std::istringstream inmeta(in);
        inmeta >> std::noskipws;
        std::stringstream buf;
        buf << inmeta.rdbuf();
        std::string bufStr;
        bufStr = buf.str();
        meta.fromJSON( bufStr );
    }

    void blend(std::string& data, unsigned seed)
    {
        osgEarth::Random prng(seed, osgEarth::Random::METHOD_FAST);
        unsigned paddedSize = data.size();
        if ( data.size() % 4 > 0 ) paddedSize += 4 - (data.size() % 4);
        char* buf = new char[paddedSize];
        memcpy(buf, data.c_str(), data.size());
        unsigned* ptr = (unsigned*)buf;
        for(unsigned i=0; i<paddedSize/4; ++i, ++ptr)
            (*ptr) ^= prng.next(INT_MAX);
        data = std::string(buf, data.size());
        delete buf;
    }

    void unblend(std::string& data, unsigned seed)
    {
        blend(data, seed);
    }
}

//------------------------------------------------------------------------

#undef  LC
#define LC "[LevelDBCacheBin] "

#undef  OE_TEST
#define OE_TEST OE_NOTICE

#define TIME_FIELD "leveldb.time"


LevelDBCacheBin::LevelDBCacheBin(const std::string& binID,
                                 leveldb::DB*       db,
                                 Tracker*           tracker) :
osgEarth::CacheBin( binID ),
_db               ( db ),
_tracker          ( tracker ),
_debug            ( false )
{
    // reader to parse data:
    _rw = osgDB::Registry::instance()->getReaderWriterForExtension( "osgb" );
    _rwOptions = osgEarth::Registry::instance()->cloneOrCreateOptions();    
    
    if ( ::getenv("OSGEARTH_CACHE_DEBUG") )
        _debug = true;
}

LevelDBCacheBin::~LevelDBCacheBin()
{
    // nop
}

bool
LevelDBCacheBin::binValidForReading(bool silent)
{
    bool ok = _db != 0L;
    if ( !ok && !silent )
    {
        OE_WARN << LC << "Failed to locate cache bin (" << getID() << ")" << std::endl;
    }
    return ok;
}

bool
LevelDBCacheBin::binValidForWriting(bool silent)
{
    bool ok = _db != 0L;
    if ( !ok && !silent )
    {
        OE_WARN << LC << "Failed to locate cache bin (" << getID() << ")" << std::endl;
    }
    return ok;
}

#define SEP std::string("!")

std::string
LevelDBCacheBin::binPhrase()
{
    return SEP + getID() + SEP;
}

std::string
LevelDBCacheBin::binKey()
{
    return "b" + SEP + getID();
}

std::string
LevelDBCacheBin::dataKey(const std::string& key)
{
    return "d" + SEP + binDataKeyTuple(key);
}

std::string
LevelDBCacheBin::binDataKeyTuple(const std::string& key)
{
    return getID() + SEP + key;
}

std::string
LevelDBCacheBin::dataKeyFromTuple(const std::string& tuple)
{
    return "d" + SEP + tuple;
}

std::string
LevelDBCacheBin::dataBegin()
{
    return "d" + SEP + getID() + SEP;
}

std::string
LevelDBCacheBin::dataEnd()
{
    return "d" + SEP + getID() + SEP + "\xff";
}

std::string
LevelDBCacheBin::metaKey(const std::string& key)
{
    return "m" + SEP + binDataKeyTuple(key);
}

std::string
LevelDBCacheBin::metaKeyFromTuple(const std::string& tuple)
{
    return "m" + SEP + tuple;
}

std::string
LevelDBCacheBin::metaBegin()
{
    return "m" + SEP + getID() + SEP;
}

std::string
LevelDBCacheBin::metaEnd()
{
    return "m" + SEP + getID() + SEP + "\xff";
}

std::string
LevelDBCacheBin::timeKey(const DateTime& t, const std::string& key)
{
    return "t" + SEP + t.asCompactISO8601() + SEP + getID() + SEP + key;
}

std::string
LevelDBCacheBin::timeBegin()
{
    return "t" + SEP + getID() + SEP;
}

std::string
LevelDBCacheBin::timeEnd()
{
    return "t" + SEP + getID() + SEP + "\xff";
}

std::string
LevelDBCacheBin::timeBeginGlobal()
{
    return "t" + SEP;
}

std::string
LevelDBCacheBin::timeEndGlobal()
{
    return "t" + SEP + "\xff";
}

ReadResult
LevelDBCacheBin::readImage(const std::string& key)
{
    return read(key, ImageReader(_rw.get(), _rwOptions.get()));    
}

ReadResult
LevelDBCacheBin::readObject(const std::string& key)
{
    //OE_INFO << LC << "Read attempt: " << key << " from " << getID() << std::endl;
    return read(key, ObjectReader(_rw.get(), _rwOptions.get()));
}

ReadResult
LevelDBCacheBin::readNode(const std::string& key)
{
    return read(key, NodeReader(_rw.get(), _rwOptions.get()));
}

ReadResult
LevelDBCacheBin::read(const std::string& key, const Reader& reader)
{
    if ( !binValidForReading() ) 
        return ReadResult(ReadResult::RESULT_NOT_FOUND);

    ++_tracker->reads;

    Config metadata;
    leveldb::Status status;
    leveldb::ReadOptions ro;

    // first read the metadata record.
    std::string metavalue;
    status = _db->Get( ro, metaKey(key), &metavalue );
    TimeStamp lastModified;
    if ( status.ok() )
    {        
        decodeMeta(metavalue, metadata);
        DateTime t( metadata.value(TIME_FIELD));
        lastModified = t.asTimeStamp();
    }
        
    // next read the data record.
    std::string datakey = dataKey(key);
    std::string datavalue;
    status = _db->Get( ro, datakey, &datavalue );
    if ( !status.ok() )
    {
        // main record not found for some reason.
        return ReadResult(ReadResult::RESULT_NOT_FOUND);
    }

    // blend the data string
    if ( _tracker->seed().isSet() )
        unblend(datavalue, _tracker->seed().value());

    // finally, decode the OSGB stream into an object.
    std::istringstream datastream(datavalue);
    osgDB::ReaderWriter::ReadResult r = reader.read(datastream);
    if ( !r.success() )
    {
        OE_WARN << LC << "Read failure - bad key?" << std::endl;
        return ReadResult(ReadResult::RESULT_READER_ERROR);
    }
        
    if ( _debug )
    {
        OE_NOTICE << LC << "Read (" << key << ") from bin " << getID() << std::endl;
    }

    // if there's a size limit, we need to 'touch' the record.
    if ( _tracker->hasSizeLimit() )
    {
        // Room for optimization here since we already have the 
        // meta/time records around.
        touch( key );
    }

    ++_tracker->hits;
    ReadResult rr(r.getObject(), metadata);
    rr.setLastModifiedTime(lastModified);    
    return rr;
}

ReadResult
LevelDBCacheBin::readString(const std::string& key)
{
    ReadResult r = readObject(key);
    if ( r.succeeded() )
    {
        if ( r.get<StringObject>() )
            return r;
        else
            return ReadResult();
    }
    else
    {
        return r;
    }
}

bool
LevelDBCacheBin::write(const std::string& key, const osg::Object* object, const Config& meta)
{
    if ( !binValidForWriting() || !object ) 
        return false;
        
    osgDB::ReaderWriter::WriteResult r;
    bool objWriteOK = false;

    std::string       data;
    std::stringstream datastream;

    if ( dynamic_cast<const osg::Image*>(object) )
    {
        r = _rw->writeImage( *static_cast<const osg::Image*>(object), datastream, _rwOptions.get() );
        objWriteOK = r.success();
    }
    else if ( dynamic_cast<const osg::Node*>(object) )
    {
        r = _rw->writeNode( *static_cast<const osg::Node*>(object), datastream, _rwOptions.get() );
        objWriteOK = r.success();
    }
    else
    {
        r = _rw->writeObject( *object, datastream );
        objWriteOK = r.success();
    }

    if (objWriteOK)
    {
        DateTime now;
        leveldb::WriteBatch batch;

        // write the data:
        data = datastream.str();
        if ( _tracker->seed().isSet() )
            blend(data, _tracker->seed().value());
        batch.Put( dataKey(key), data );

        // write the timestamp index:
        batch.Put( timeKey(now, key), binDataKeyTuple(key) );

        // write the metadata:
        Config metadata(meta);
        metadata.set( TIME_FIELD, now.asCompactISO8601() );
        encodeMeta( metadata, data );
        batch.Put( metaKey(key), data );

        objWriteOK = _db->Write( leveldb::WriteOptions(), &batch ).ok();

        if ( objWriteOK )
        {
            ++_tracker->writes;
            postWrite();
            
            if ( _debug )
            {
                OE_NOTICE << LC << "Wrote (" << dataKey(key) << ") to bin " << getID() << std::endl;
            }
        }
    }

        
    if ( !objWriteOK )
    {
        OE_WARN << LC << "FAILED to write \"" << key << "\" to bin " << getID()
            << "; msg = \"" << r.message() << "\"" << std::endl;
    }

    return objWriteOK;
}

void
LevelDBCacheBin::postWrite()
{
    if ( _tracker->hasSizeLimit() )
    {
        if ( _tracker->isOverLimit() )
        {
            if ( _tracker->isTimeToPurge() )
            {
                this->purgeOldest(_tracker->numToPurge());

                if (_debug)
                {
                    off_t size = _tracker->calcSize();
                    OE_NOTICE 
                        << LC << "Cache size = " << (size/1048576) << " MB; " 
                        << "Hit ratio = " << (float)_tracker->hits/(float)_tracker->reads << std::endl;
                }
            }
        }
        else
        {
            if ( _tracker->isTimeToCheckSize() )
            {
                off_t size = _tracker->calcSize();
                if ( _debug )
                {
                    OE_NOTICE 
                        << LC << "Cache size = " << (size/1048576) << " MB; " 
                        << "Hit ratio = " << (float)_tracker->hits/(float)_tracker->reads << std::endl;
                }
            }
        }
    }
}

CacheBin::RecordStatus
LevelDBCacheBin::getRecordStatus(const std::string& key)
{
    if ( !binValidForReading() ) 
        return STATUS_NOT_FOUND;

    leveldb::Status status;
    leveldb::ReadOptions ro;

    // read the metadata record.
    std::string metavalue;
    status = _db->Get( ro, metaKey(key), &metavalue );
    if ( status.ok() )
    {        
        return STATUS_OK;
    }
    else
    {
        return STATUS_NOT_FOUND;
    }
}

bool
LevelDBCacheBin::remove(const std::string& key)
{
    if ( !binValidForReading() )
        return false;

    // first read in the time from the metadata record.
    std::string metavalue;
    if ( _db->Get(leveldb::ReadOptions(), metaKey(key), &metavalue).ok() == false )
        return false;

    Config metadata;
    decodeMeta(metavalue, metadata);
    DateTime t(metadata.value(TIME_FIELD));

    leveldb::WriteBatch batch;
    batch.Delete( dataKey(key) );
    batch.Delete( metaKey(key) );
    batch.Delete( timeKey(t, key) );
        
    leveldb::Status status = _db->Write(leveldb::WriteOptions(), &batch);
    if ( !status.ok() )
    {
        OE_WARN << LC << "Failed to remove (" << key << ") from bin " << getID() << std::endl;
        return false;
    }
    else if ( _debug )
    {
        OE_NOTICE << LC << "Removed (" << key << ") from bin " << getID() << std::endl;
    }

    return true;
}

bool
LevelDBCacheBin::touch(const std::string& key)
{    
    if ( !binValidForWriting() )
        return false;

    // first read in the time from the metadata record.
    std::string metavalue;
    if ( _db->Get(leveldb::ReadOptions(), metaKey(key), &metavalue).ok() == false )
        return false;

    Config metadata;
    decodeMeta(metavalue, metadata);
    DateTime oldtime(metadata.value(TIME_FIELD));
        
    leveldb::WriteBatch batch;

    // In a transaction, update the metadata record with the current time.
    std::string newtime = DateTime().asCompactISO8601();
    metadata.set(TIME_FIELD, newtime);
    encodeMeta(metadata, metavalue);
    batch.Put(metaKey(key), metavalue);

    // ...remove the old time index record:
    batch.Delete( timeKey(oldtime, key) );

    // ...and write a new time index record.
    batch.Put( timeKey(newtime, key), binDataKeyTuple(key) );

    leveldb::Status status = _db->Write(leveldb::WriteOptions(), &batch);
    if ( !status.ok() )
    {
        OE_WARN << LC << "Failed to touch (" << key << ") in bin " << getID() << std::endl;
    }
    else if ( _debug )
    {
        OE_NOTICE << LC << "Touched (" << key << ") in bin " << getID() << std::endl;
    }
    return status.ok();
}

bool
LevelDBCacheBin::clear()
{
    if ( !binValidForWriting() )
        return false;
    
    leveldb::WriteOptions wo;
    std::string binphrase = binPhrase();
    leveldb::WriteBatch batch;
    leveldb::Iterator* i = _db->NewIterator(leveldb::ReadOptions());
    for(i->SeekToFirst(); i->Valid(); i->Next())
    {
        std::string key = i->key().ToString();
        if ( key.find(binphrase) != std::string::npos )
        {
            _db->Delete( wo, i->key() );
        }
    }
    delete i;

    if ( _debug )
    {
        OE_NOTICE << LC << "Cleared bin " << getID() << std::endl;
    }

    return true;
}

bool
LevelDBCacheBin::compact()
{
    if ( !binValidForWriting() )
        return false;

    // This could take a while.
    _db->CompactRange(0L, 0L);

    return false;
}

unsigned
LevelDBCacheBin::getStorageSize()
{
    if ( !binValidForReading() )
        return false;

    //Note: doesn't work..
    leveldb::Range ranges[3];
    uint64_t       sizes[3];

    ranges[0] = leveldb::Range(dataBegin(), dataEnd());
    ranges[1] = leveldb::Range(metaBegin(), metaEnd());
    ranges[2] = leveldb::Range(timeBegin(), timeEnd());
    sizes[0] = sizes[1] = sizes[2] = 0;

    _db->GetApproximateSizes( ranges, 3, sizes );
    return sizes[0] + sizes[1] + sizes[2];
}

Config
LevelDBCacheBin::readMetadata()
{
    if ( !binValidForReading() )
        return Config();

    ScopedMutexLock exclusiveLock( _rwMutex );

    std::string binvalue;
    leveldb::Status status = _db->Get(leveldb::ReadOptions(), binKey(), &binvalue);
    if ( !status.ok() )
        return Config();

    Config binMetadata;
    decodeMeta(binvalue, binMetadata);
    return binMetadata;
}

bool
LevelDBCacheBin::writeMetadata(const Config& conf)
{
    if ( !binValidForWriting() )
        return false;

    ScopedMutexLock exclusiveLock( _rwMutex );

    // inject the cache version
    Config mutableConf(conf);
    mutableConf.set("leveldb.cache_version", LEVELDB_CACHE_VERSION);

    std::string value;
    encodeMeta(mutableConf, value);

    if ( _db->Put(leveldb::WriteOptions(), binKey(), value).ok() == false )
    {
        OE_WARN << LC << "Failed to write metadata record for bin (" << getID() << ")" << std::endl;
        return false;
    }

    return true;
}

bool
LevelDBCacheBin::purgeOldest(unsigned maxnum)
{
    if ( !binValidForWriting() )
        return false;

    leveldb::Iterator* it = _db->NewIterator(leveldb::ReadOptions());

    unsigned count = 0;
    std::string limit = timeEndGlobal();

    // note: this will delete records NOT OF THIS BIN as well!
    for(it->Seek(timeBeginGlobal());
        count < maxnum && it->Valid() && it->key().ToString() < limit;
        it->Next(), ++count )
    {
        if ( !it->status().ok() )
            break;

        std::string tuple = it->value().ToString();

        // doing this in a WriteBatch did not work. The size of the
        // database would never go down.
        leveldb::WriteOptions wo;
        _db->Delete( wo, dataKeyFromTuple(tuple) );
        _db->Delete( wo, metaKeyFromTuple(tuple) );
        _db->Delete( wo, it->key() );
    }

    delete it;

    if ( _debug )
    {
        OE_NOTICE << LC << "Purged " << count << " record(s) for "
            << (_tracker->calcSize()/1048576) << " MB" << std::endl;
    }

    return true;
}
