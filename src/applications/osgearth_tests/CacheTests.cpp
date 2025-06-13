/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/

#include <osgEarth/catch.hpp>
#include <osgEarth/GeoData>
#include <osgEarth/Registry>
#include <osgEarth/MemCache>
#include <osgEarth/Containers>  // For osgEarth::LRUCache

using namespace osgEarth;

TEST_CASE("Cache")
{
    // Get the cache
    osg::ref_ptr<Cache> cache = new MemCache();
    REQUIRE(cache.valid());

    // Open a bin:
    osg::ref_ptr<CacheBin> bin = cache->addBin("test_bin");
    REQUIRE(bin.valid());

    SECTION("String")
    {
        std::string key("string_key");
        std::string value("What is the sound of one hand clapping?");
        osg::ref_ptr<StringObject> s = new StringObject(value);

        // Write a string to the cache
        REQUIRE(bin->write(key, s.get(), 0L));

        // Read the string from the cache
        ReadResult r = bin->readString(key, 0L);
        REQUIRE(r.succeeded());
        REQUIRE(r.getString() == value);

        // Remove the string
        REQUIRE(bin->remove(key));

        // Verify it was removed
        ReadResult r2 = bin->readString(key, 0L);
        REQUIRE(r2.failed());
    }

    SECTION("Image")
    {
        std::string key("image_key");
        osg::ref_ptr<osg::Image> image = ImageUtils::createOnePixelImage(osg::Vec4(1, 0, 0, 1));

        // Write an image to the cache
        REQUIRE(bin->write(key, image.get(), 0L));

        // Read it back
        ReadResult r = bin->readImage(key, 0L);
        REQUIRE(r.succeeded());
        REQUIRE(ImageUtils::areEquivalent(r.getImage(), image.get()));

        // Remove it
        REQUIRE(bin->remove(key));

        // Confirm removal
        ReadResult r2 = bin->readImage(key, 0L);
        REQUIRE(r2.failed());
    }
}

TEST_CASE("LRUCache")
{
    SECTION("LRUCache_BasicEviction")
    {
        // LRUCache with capacity 3
        osgEarth::LRUCache<int, std::string> cache(3u);

        cache.insert(1, "one");
        cache.insert(2, "two");
        cache.insert(3, "three");

        // Verify retrieval
        REQUIRE(cache.get(1) == "one");
        REQUIRE(cache.get(2) == "two");
        REQUIRE(cache.get(3) == "three");

        // Insert a fourth, expecting key 1 to be evicted
        cache.insert(4, "four");

        REQUIRE_FALSE(cache.touch(1));
        REQUIRE(cache.touch(2));
        REQUIRE(cache.touch(3));
        REQUIRE(cache.touch(4));
        REQUIRE(cache.get(2) == "two");
        REQUIRE(cache.get(3) == "three");
        REQUIRE(cache.get(4) == "four");
    }

    SECTION("LRUCache_UsageRefresh")
    {
        osgEarth::LRUCache<int, std::string> cache(3u);

        cache.insert(1, "one");
        cache.insert(2, "two");
        cache.insert(3, "three");

        // Access keys 1 and 2, marking them as recently used
        REQUIRE(cache.get(1) == "one");
        REQUIRE(cache.get(2) == "two");

        // Insert new item expecting the least recently used (key 3) to be evicted
        cache.insert(4, "four");

        REQUIRE_FALSE(cache.touch(3));
        REQUIRE(cache.touch(1));
        REQUIRE(cache.touch(2));
        REQUIRE(cache.touch(4));
        REQUIRE(cache.get(1) == "one");
        REQUIRE(cache.get(2) == "two");
        REQUIRE(cache.get(4) == "four");
    }

    SECTION("LRUCache_get_or_insert")
    {
        osgEarth::LRUCache<int, std::string> cache(2u);

        // Insert a value using get_or_insert for a missing key
        auto v1 = cache.get_or_insert(1, [](std::optional<std::string>& out) { out = std::string("one"); });
        REQUIRE(v1.has_value());
        REQUIRE(v1.value() == "one");
        REQUIRE(cache.get(1).has_value());
        REQUIRE(cache.get(1).value() == "one");

        // get_or_insert for an existing key should not call the functor, should return the cached value
        auto v2 = cache.get_or_insert(1, [](std::optional<std::string>& out) { out = std::string("should_not_be_used"); });
        REQUIRE(v2.has_value());
        REQUIRE(v2.value() == "one");

        // Insert another value
        auto v3 = cache.get_or_insert(2, [](std::optional<std::string>& out) { out = std::string("two"); });
        REQUIRE(v3.has_value());
        REQUIRE(v3.value() == "two");
        REQUIRE(cache.get(2).has_value());
        REQUIRE(cache.get(2).value() == "two");

        // Insert a third value, which should evict the least recently used (key 1)
        auto v4 = cache.get_or_insert(3, [](std::optional<std::string>& out) { out = std::string("three"); });
        REQUIRE(v4.has_value());
        REQUIRE(v4.value() == "three");
        REQUIRE(cache.get(3).has_value());
        REQUIRE(cache.get(3).value() == "three");
        REQUIRE_FALSE(cache.touch(1)); // key 1 should be evicted

        // get_or_insert for an evicted key should call the functor again
        auto v5 = cache.get_or_insert(1, [](std::optional<std::string>& out) { out = std::string("one-again"); });
        REQUIRE(v5.has_value());
        REQUIRE(v5.value() == "one-again");
        REQUIRE(cache.get(1).has_value());
        REQUIRE(cache.get(1).value() == "one-again");

        // Test that if the functor does not set the value, nothing is inserted
        auto v6 = cache.get_or_insert(4, [](std::optional<std::string>&) { /* do not set */ });
        REQUIRE_FALSE(v6.has_value());
        REQUIRE_FALSE(cache.touch(4));
    }

}