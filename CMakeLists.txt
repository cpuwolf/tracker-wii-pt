find_package(OpenCV 3.0 QUIET COMPONENTS ${opencv-modules})
add_subdirectory("wiiyourself")
add_subdirectory("wiiv")

if(OpenCV_FOUND)
    opentrack_boilerplate(opentrack-tracker-wii-pt)
    target_link_libraries(opentrack-tracker-wii-pt opentrack-tracker-wii-pt-wiiyourself opentrack-wiiv ${OpenCV_LIBS})
    target_include_directories(opentrack-tracker-wii-pt SYSTEM PUBLIC ${OpenCV_INCLUDE_DIRS})
endif()

