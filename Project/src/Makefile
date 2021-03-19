# Final Project Makefile

# declare variable
CC=gcc
SOURCES=main.c
OBJECTS=$(SOURCES:.c=.o)
BINDIR=../bin/
EXE=project


all: $(SOURCES) $(EXE)
$(EXE): $(OBJECTS)
	$(CC) $(OBJECTS) -lm -o $(BINDIR)$@

clean:
	rm -rf *.o $(BINDIR)$(EXE)
