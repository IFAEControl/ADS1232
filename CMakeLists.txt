set(COMPONENT_REQUIRES driver esp_timer)
file(GLOB COMPONENT_SRCS *.cpp)
set(COMPONENT_ADD_INCLUDEDIRS ".")
register_component()
