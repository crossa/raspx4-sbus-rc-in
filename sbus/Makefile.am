AUTOMAKE_OPTIONS=foreign subdir-objects
CURRENTPATH=$(shell /bin/pwd)
AM_CPPFLAGS=--std=c11 -I$(CURRENTPATH)/src
noinst_LIBRARIES = libsbus.a libfunc.a
libsbus_a_SOURCES = sbus.cpp
libfunc_a_SOURCES = function.cpp

bin_PROGRAMS=sbus
god_SOURCES=src/main.cpp
god_LDADD=libsbus.a libfunc.a