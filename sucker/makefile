# EXE MAKEFILE
#------------------------------
ROOT = ../../..
EXE_TARGET = flocking_chasing_sucker

LIBS = ../../Epfl/*.o  # the only way to have a wroking program ?? :-(
INCS =
OPTS = -Wall 


# don't modify what is below ...
#------------------------------

ASM_SRCS 	= $(wildcard *.s)
C_SRCS 	= $(wildcard *.c)
SRCS 		= $(C_SRCS) $(ASM_SRCS)
HDRS 		= $(wildcard *.h) $(wildcard *.inc)
OBJS 		= $(addsuffix .o,$(basename $(SRCS)))

CC	  		= pic30-elf-gcc
LD	  		= pic30-elf-ld
AS	  		= pic30-elf-as
BIN2HEX  	= pic30-elf-bin2hex
AR			= pic30-elf-ar
RANLIB		= pic30-elf-ranlib

CC_FLAGS 	= -mcpu=30f6014a -I/usr/pic30-elf/include
AS_FLAGS 	= -I/usr/share/pic30-support/inc -p=30f6014a
LD_FLAGS 	= -L/usr/pic30-elf/lib --heap=512 --defsym=__ICD2RAM=1 --script=/usr/share/pic30-support/gld/p30f6014a.gld -mpic30_elf32


all : $(EXE_TARGET).hex

$(EXE_TARGET).hex : $(EXE_TARGET).cof
	$(BIN2HEX) $(EXE_TARGET).cof
	if [ ! -d $(ROOT)/bin ]; then mkdir $(ROOT)/bin; fi
	cp $@ $(ROOT)/bin/
	make clean

$(EXE_TARGET).cof : $(OBJS) 
	$(LD) -o $@ -L$(ROOT)/lib $(LD_FLAGS) --start-group -lpic30-elf -lm-elf -lc-elf $? $(LIBS) --end-group

%.o : %.c %.h
	$(CC) $(CC_FLAGS) $(OPTS) $(INCS) -I$(ROOT)/include/ -c $< -o $(<:.c=.o)

%.o : %.c
	$(CC) $(CC_FLAGS) $(OPTS) $(INCS) -I$(ROOT)/include/ -c $< -o $(<:.c=.o)

%.o : %.s
	$(AS) -o $(<:.s=.o) $(AS_FLAGS) $(INCS) -I$(ROOT)/include/ $<


.PHONY : clean
clean : 
	rm -f *~ *.o *.a *.bak *.hex *.elf *.cof $(LIB_TARGET) core gmon.out  




