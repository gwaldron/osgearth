/* osgEarth
* Copyright 2025 Pelican Mapping
* MIT License
*/

#include <osgEarth/catch.hpp>
#include <osgEarth/Expression>
#include <osgEarth/ScriptEngine>
#include <osgEarth/Math>

using namespace osgEarth;
using namespace osgEarth::Util;

TEST_CASE("Expression<>") {

    osg::ref_ptr<ScriptEngine> engine = ScriptEngineFactory::instance()->create("javascript");

    SECTION("Expression<float>")
    {
        Expression<float> expr("2 + 3 * 4");
        auto result = expr.eval(engine.get());
        REQUIRE(equivalent(result, 14.0f));
    }

    SECTION("Expression<float> JS")
    {
        Expression<float> expr(R"(
            function sum(a, b) {
                return a + b;
            }
            sum(1, 2);
        )");
        auto result = expr.eval(engine.get());
        REQUIRE(result == 3);
    }

    SECTION("Expression<double>")
    {
        Expression<double> expr("2 + 3 * 4");
        auto result = expr.eval(engine.get());
        REQUIRE(equivalent(result, 14.0));
    }

    SECTION("Expression<int>")
    {
        Expression<int> expr("2 + 3 * 4");
        auto result = expr.eval(engine.get());
        REQUIRE(result == 14);
    }

    SECTION("Expression<unsigned>")
    {
        Expression<unsigned> expr("2 + 3 * 4");
        auto result = expr.eval(engine.get());
        REQUIRE(result == 14u);
    }

    SECTION("Expression<String>")
    {
        Expression<String> expr("'Hello, world'");
        auto result = expr.eval(engine.get());
        REQUIRE(result == "Hello, world");
    }

    SECTION("Expression<String> Literal")
    {
        Expression<String> expr(String{ "Hello, world" });
        auto result = expr.eval(engine.get());
        REQUIRE(result == "Hello, world");
    }

    SECTION("Expression<String> JS")
    {
        Expression<String> expr("'Hello' + ', ' + 'world'");
        auto result = expr.eval(engine.get());
        REQUIRE(result == "Hello, world");
    }

    SECTION("Expression<Distance>")
    {
        Expression<Distance> expr("10km");
        Distance d = expr.eval(engine.get());
        REQUIRE(d == Distance(10, Units::KILOMETERS));
        REQUIRE(expr.literal() == Distance(10, Units::KILOMETERS));
    }

    SECTION("Expression<Distance> JS")
    {
        Expression<Distance> expr(R"(
            function sum(a, b) {
                return a + b;
            }
            sum(1, 2) + 'km';
        )");
        auto result = expr.eval(engine.get());
        REQUIRE(result == Distance(3, Units::KILOMETERS));
    }

    SECTION("Expression<Angle>")
    {
        Expression<Angle> expr("45");
        Angle a = expr.eval(engine.get());
        REQUIRE(a == Angle(45, Units::DEGREES));
    }

    SECTION("Expression<Angle> JS")
    {
        Expression<Angle> expr(R"(
            function sum(a, b) {
                return a + b;
            }
            sum(1, 2) + 'deg';
        )");
        auto result = expr.eval(engine.get());
        REQUIRE(result == Angle(3, Units::DEGREES));
    }
}
