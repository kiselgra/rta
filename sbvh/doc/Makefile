BASE=sbvh
TEX=$(BASE).tex
PDF=$(BASE).pdf

$(PDF): $(TEX) Makefile
	latex $(BASE)
	bibtex $(BASE)
	latex $(BASE) > /dev/null < /dev/null
	latex $(BASE) > /dev/null < /dev/null
	dvipdf $(BASE)

show: $(PDF)
	evince $(PDF)

clean:
	rm -f a1.aux  a1.dvi  a1.log  a1.pdf  a1.bbl  a1.blg
