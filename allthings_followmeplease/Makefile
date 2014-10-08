#Makefile OpenUAS Folllow Me Please (FMP)
#if pkg-config --cflags does not return useful info
#check if PKG_CONFIG_PATH is set OK

FMP_HOME = $(PWD)
FMP_SOURCE = $(PWD)/src
FMP_BIN = $(PWD)/bin
INCLUDES = `pkg-config --cflags --libs gtk+-2.0` -I$(FMP_SOURCE) -lglibivy -lgps

CPP = g++ -g -Wno-deprecated
CC = gcc -g
CFLAGS = -c $(INCLUDES)

all : $(FMP_SOURCE)/follow_me_please.c
	$(CC)  $(FMP_SOURCE)/follow_me_please.c -o $(FMP_BIN)/follow_me_please $(INCLUDES)

clean :
	cd $(FMP_HOME) rm -f *.o
	cd $(FMP_SOURCE); rm -f *.o;
	cd $(FMP_BIN); rm -f follow_me_please;
