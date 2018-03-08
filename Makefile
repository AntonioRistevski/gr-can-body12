#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := canopener

GIT_VERSION := $(shell git describe --abbrev=7 --dirty --always --tags)
CFLAGS += -DGRVERSION=\"$(GIT_VERSION)\"

include $(IDF_PATH)/make/project.mk

