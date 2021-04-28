###############################################################################
# Makefile for project 
###############################################################################

## General options
PROJECT = 
MCU = PIC16F876A
COMPILER = "MPASM (Proteus)"
TARGET = Debug
SHELL = C:\WINDOWS\system32\cmd.exe

## Tools general options
ASMFLAGS=/q- /p16f876a
LDFLAGS=/x /p16f876a

## Processing Tools
ASM=MPASMWIN.exe
LD=mplink.exe
LIB=mplib.exe
#  D:/_dev_/trip-computer/proteus/PIC16F876A/Debug/Debug.cof: TOOL=LD
#     D:/_dev_/trip-computer/proteus/PIC16F876A/Debug/BK.o: TOOL=ASM
#        D:/_dev_/trip-computer/proteus/PIC16F876A/BK.asm

# Build tree:
all:	Debug

Debug:	Debug.cof

Debug.cof:	BK.o 
	$(LD) $(LDFLAGS) /o "./Debug.cof" 16f876a_g.lkr "BK.o"

BK.o:	../BK.asm 
	$(ASM) /e"./BK.err" /l"./BK.lst" /o"BK.o" $(ASMFLAGS)  "..\BK.asm"

# tidy - delete all temporary files which are not involved in the target generation
tidy:
	rm -rf ./BK.err
	rm -rf ./BK.hex
	rm -rf ./BK.lst

# cleanup - delete all generated files
clean:	tidy
	rm -rf Debug.cof
	rm -rf BK.o
