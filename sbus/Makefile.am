AUTOMAKE_OPTIONS=foreign subdir-objects
CURRENTPATH=$(shell /bin/pwd)
AM_CPPFLAGS=--std=c++11 -I$(CURRENTPATH)/src
noinst_LIBRARIES = libsbus.a
libsbus_a_SOURCES = src/sbus.cpp src/function.cpp

bin_PROGRAMS=sbus
sbus_SOURCES=src/main.cpp
sbus_LDADD=libsbus.a