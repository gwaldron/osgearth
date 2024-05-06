
find_package(RocksDB)

if(RocksDB_FOUND)

    # set(LIB_NAME osgdb_osgearth_cache_rocksdb)

    set(CMAKE_CXX_STANDARD 17)

    # set(TARGET_H_PUBLIC    
        # RocksDBCacheOptions)
        
    # set(TARGET_H
        # ${TARGET_H_PUBLIC}
        # RocksDBCache
        # RocksDBCacheBin
        # Tracker
    # )
    # set(TARGET_SRC 
        # RocksDBCache.cpp
        # RocksDBCacheBin.cpp
        # RocksDBCacheDriver.cpp
    # )
    
    #add_library_as_osgdb_plugin(osgdb_osgearth_cache_rocksdb)
    
    add_osgearth_plugin(
        TARGET 
            osgdb_osgearth_cache_rocksdb
        PUBLIC_HEADERS
            RocksDBCacheOptions
        HEADERS
            RocksDBCache
            RocksDBCacheBin
            Tracker
        SOURCES 
            RocksDBCache.cpp
            RocksDBCacheBin.cpp
            RocksDBCacheDriver.cpp )
            
    
    target_link_libraries(osgdb_osgearth_cache_rocksdb
        PRIVATE RocksDB::rocksdb RocksDB::rocksdb-shared)    
        
endif()
