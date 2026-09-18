idf_component_register(
    SRCS "desk_controller.cpp"
    INCLUDE_DIRS "include"
    REQUIRES desk_protocol
)

target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)
