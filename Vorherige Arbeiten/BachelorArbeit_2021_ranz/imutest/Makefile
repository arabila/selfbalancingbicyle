TARGETS= imutest
all:	$(TARGETS)

imutest:	imutest.o
	gcc -Wall -o imutest imutest.c -lgpiod -lm

clean:
	-rm -f *.o imutest
