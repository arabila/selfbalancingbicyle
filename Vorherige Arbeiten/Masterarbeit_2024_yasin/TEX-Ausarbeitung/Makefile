


abschlussarbeit.pdf: abschlussarbeit.tex
	pdflatex $<
	bibtex ${<:.tex=}
	pdflatex $<
	pdflatex $<



clean:
	-rm *.log *.ps *.dvi *.blg *.aux *.bcf *.lot *.lof *.lol *.bbl *.out *.run.xml *.toc 

cleaner: clean
	-rm *.pdf

