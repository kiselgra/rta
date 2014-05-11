lib_LTLIBRARIES = librta.la

librta_la_SOURCES = aabb.h  \
					basic_types.h  \
					cuda.h cuda.cpp cuda-vec.h cuda-kernels.cu \
					image.h  \
					intersect.h  \
					material.h material.cpp \
					plugins.h plugins.cpp\
					raytrav.h  \
					template-magic.h  \
					tri.h \
					ocl.h ocl.cpp \
					rta-config.h \
					librta.h \
					wall-timer.h

library_includedir = $(includedir)/$(PACKAGE)-$(VERSION)/librta
library_include_HEADERS = aabb.h  basic_types.h  cuda.h  cuda-vec.h  cuda-kernels.h  image.h  intersect.h  material.h  plugins.h  raytrav.h  template-magic.h  tri.h ocl.h rta-config.h librta.h wall-timer.h

# setting c preprocessor flags for main project
librta_la_CPPFLAGS = -D PACKAGE=\"$(PACKAGE)\" -D VERSION=\"$(VERSION)\" -D PKG_LIB_DIR=\"$(pkglibdir)\"
librta_la_CXXFLAGS = -Wreturn-type

.cu.lo:
	$(LIBTOOL) --tag=CXX --mode=compile $(top_srcdir)/nvcc-wrapper $(DEFAULT_INCLUDES) $(librta_la_CXXFLAGS) $(CXXFLAGS) $(librta_la_CPPFLAGS) $(CPPFLAGS) -c -o $@ $< -v
