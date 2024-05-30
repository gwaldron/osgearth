# IOS build support
if(APPLE AND NOT ANDROID)
    OPTION(OSGEARTH_BUILD_PLATFORM_IPHONE "Enable Apple iOS build" OFF)

    IF(OSGEARTH_BUILD_PLATFORM_IPHONE)

        SET(OSGEARTH_USE_GLES TRUE)

        # set sdk and min versions
        SET (IPHONE_SDKVER "10.2" CACHE STRING "IOS SDK-Version")
        SET (IPHONE_VERSION_MIN "7.0" CACHE STRING "IOS minimum os version, use 7.0 or greater to get 64bit support")

        # get full path to sdk from requested versions
        SET (IPHONE_DEVROOT "/Applications/Xcode.app/Contents/Developer/Platforms/iPhoneOS.platform/Developer")
        SET (IPHONE_SDKROOT "${IPHONE_DEVROOT}/SDKs/iPhoneOS${IPHONE_SDKVER}.sdk")

        # optionally enable bitcode for the build
        SET (IPHONE_ENABLE_BITCODE "NO" CACHE STRING "IOS Enable Bitcode")

        # seamless toggle between device and simulator
        SET(CMAKE_XCODE_EFFECTIVE_PLATFORMS "-iphoneos;-iphonesimulator")

        # set deployment target to min version
        SET(CMAKE_XCODE_ATTRIBUTE_IPHONEOS_DEPLOYMENT_TARGET "${IPHONE_VERSION_MIN}" CACHE STRING "Deployment target for iOS" FORCE)

        # Set standard architectures
        SET(CMAKE_OSX_ARCHITECTURES "$(ARCHS_STANDARD)")

        # set the sdk path as our sysroot
        SET(CMAKE_OSX_SYSROOT "${IPHONE_SDKROOT}" CACHE STRING "System root for iOS" FORCE)

    ENDIF ()
    
endif()
