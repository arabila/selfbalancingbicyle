FN=seminar_vorlage

all:
	pdflatex $(FN).tex
	bibtex $(FN)
	pdflatex $(FN).tex
	bibtex $(FN)
	pdflatex $(FN).tex

clean:
	rm -f $(FN).{pdf,toc,blg,log,aux}
