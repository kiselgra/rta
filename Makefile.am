
# the order listed here is the order the directories get built
#
SUBDIRS = librta rta bbvh sphs
SUBDIRS += brute-force 
SUBDIRS += sbvh 
if opencl
SUBDIRS += bbvh-ocl 
SUBDIRS += sbvh-ocl 
endif
if use_librctest
SUBDIRS += wrapper-for-librctest 
endif
if cuda
SUBDIRS += bbvh-cuda
endif

SUBDIRS += doc

.PHONY: doc
doc:
	cd doc; make doc

ACLOCAL_AMFLAGS = -I m4

EXTRA_DIST = configure-for-secondary-gcc built-plugins 
EXTRA_DIST += dragon-param dragon-param-rttest

pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = rta.pc

if cuda
cudadir = $(pkgdatadir)
cuda_DATA = nvcc-wrapper
endif
