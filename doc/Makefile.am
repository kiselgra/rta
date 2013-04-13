
.PHONY: doc
doc:
	doxygen doxyfile

EXTRA_DIST = html
install-data-local:
	$(MKDIR_P) $(docdir)
	tar cf html.tar html
	tar xf html.tar -C $(docdir)
	rm html.tar
