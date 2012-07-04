BASE=messungen
TEX=$(BASE).tex
PDF=$(BASE).pdf

$(PDF): $(TEX) Makefile
	pdflatex $(BASE)
# 	bibtex $(BASE)
	pdflatex $(BASE) > /dev/null < /dev/null
	pdflatex $(BASE) > /dev/null < /dev/null

show: $(PDF)
	evince $(PDF)

clean:
	rm -f a1.aux  a1.dvi  a1.log  a1.pdf  a1.bbl  a1.blg
