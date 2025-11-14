set(COMPONENT_REQUIRES esp_driver_gpio esp_timer)
file(GLOB COMPONENT_SRCS *.cpp)
set(COMPONENT_ADD_INCLUDEDIRS ".")
register_component()
