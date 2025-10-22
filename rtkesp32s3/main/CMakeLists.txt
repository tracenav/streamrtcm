idf_component_register(SRCS "main_minimal.c"
                    INCLUDE_DIRS ".")

# Link the PPL library
target_link_libraries(${COMPONENT_LIB} "${CMAKE_CURRENT_SOURCE_DIR}/libpointperfect.a")