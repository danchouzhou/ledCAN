include ./makefile.conf
NAME=Servo

STARTUP_DEFS=-D__STARTUP_CLEAR_BSS -D__START=main

# Need following option for LTO as LTO will treat retarget functions as
# unused without following option
CFLAGS+=-fno-builtin
# Add definition for ARM DSP Library
CFLAGS+=-D ARM_MATH_CM0

LDSCRIPTS=-L. -L$(BASE)Library/Device/Nuvoton/M0A21/Source/GCC -T gcc_arm.ld

LFLAGS=$(USE_NANO) $(USE_NOHOST) $(LDSCRIPTS) $(GC) $(MAP)

#$(NAME)-$(CORE).axf: main.c $(NAME).c $(STARTUP)
#	$(CC) $^ $(CFLAGS) $(LFLAGS) -o $@
IPATH= . \
       ../M0A21BSP/Library/CMSIS/Include \
       ../M0A21BSP/Library/Device/Nuvoton/M0A21/Include \
       ../M0A21BSP/Library/StdDriver/inc \
       ../NuLibrary \
       User
BSP=$(wildcard \
       ../M0A21BSP/Library/CMSIS/DSP_Lib/Source/*/*.c \
       ../M0A21BSP/Library/Device/Nuvoton/M0A21/Source/*.c \
       ../M0A21BSP/Library/Device/Nuvoton/M0A21/Source/GCC/*.c \
       ../M0A21BSP/Library/StdDriver/src/*.c \
)
LIB=$(wildcard \
       ../NuLibrary/*.c \
)
USR=$(wildcard \
       *.c \
       User/*.c \
)
OBJ_BSP = $(patsubst %.c,./Objects/%.o,$(notdir $(BSP)))
OBJ_LIB = $(patsubst %.c,./Objects/%.o,$(notdir $(LIB)))
OBJ_USR = $(patsubst %.c,./Objects/%.o,$(notdir $(USR)))

$(NAME): $(OBJ_USR) $(OBJ_LIB) $(OBJ_BSP)
	mkdir -p ./Objects
	cd ./Objects && $(CC) *.o $(CFLAGS) $(LFLAGS) $(STARTUP) -o $@.axf $(addprefix -I ../,$(IPATH)) 
	$(CP) -O binary ./Objects/$@.axf ./Objects/$@.bin

$(OBJ_USR): $(USR)
	mkdir -p ./Objects
	cd ./Objects && $(CC) $(addprefix ../,$^) $(CFLAGS) -c $(addprefix -I ../,$(IPATH))

$(OBJ_LIB): $(LIB)
	mkdir -p ./Objects
	cd ./Objects && $(CC) $(addprefix ../,$^) $(CFLAGS) -c $(addprefix -I ../,$(IPATH))

$(OBJ_BSP): $(BSP)
	mkdir -p ./Objects
	cd ./Objects && $(CC) $(addprefix ../,$^) $(CFLAGS) -c $(addprefix -I ../,$(IPATH))

.PHONY: all clean

all: $(NAME)

clean:
	rm -f ./Objects/*