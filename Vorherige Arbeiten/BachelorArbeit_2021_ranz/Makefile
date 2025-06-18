TARGETS= bicycle
all:	$(TARGETS)

bicycle:	bicycle.o
	gcc -Wall -o bicycle bicycle.c -lgpiod -lm

clean:
	-rm -f *.o bicycle
