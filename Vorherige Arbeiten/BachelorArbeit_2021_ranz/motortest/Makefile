TARGETS= motortest
all:	$(TARGETS)

motortest:	motortest.o
	gcc -Wall -o motortest motortest.c -lgpiod -lm

clean:
	-rm -f *.o motortest
