find_package(OpenCV 3.0 QUIET COMPONENTS ${opencv-modules})
if(OpenCV_FOUND)
    otr_module(wiiv STATIC)
    target_link_libraries(opentrack-wiiv ${OpenCV_LIBS})
    target_include_directories(opentrack-wiiv SYSTEM PUBLIC ${OpenCV_INCLUDE_DIRS})
endif()
