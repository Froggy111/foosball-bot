target_sources(${CMAKE_PROJECT_NAME}
PRIVATE
    main.cpp
)

target_include_directories(${CMAKE_PROJECT_NAME}
PRIVATE
    include
)

add_subdirectory(init)
