find_package(OpenCV 3.0 QUIET COMPONENTS ${opencv-modules})
add_subdirectory("wiiyourself")

if(OpenCV_FOUND)
    otr_module(tracker-wii-pt)
    target_link_libraries(opentrack-tracker-wii-pt opentrack-cv opentrack-tracker-wii-pt-wiiyourself ${OpenCV_LIBS})
    target_include_directories(opentrack-tracker-wii-pt SYSTEM PUBLIC ${OpenCV_INCLUDE_DIRS})
endif()

