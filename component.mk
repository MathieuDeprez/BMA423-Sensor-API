#
# Component Makefile
#

# Set simple includes as default
#ifndef LV_CONF_INCLUDE_SIMPLE
#CFLAGS += -DLV_CONF_INCLUDE_SIMPLE
#endif

COMPONENT_SRCDIRS := BMA423-Sensor-API/
COMPONENT_ADD_INCLUDEDIRS := $(COMPONENT_SRCDIRS) .
