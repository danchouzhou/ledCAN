include ./makefile.conf
NAME=Servo

STARTUP_DEFS=-D__STARTUP_CLEAR_BSS -D__START=main

# Need following option for LTO as LTO will treat retarget functions as
# unused without following option
CFLAGS+=-fno-builtin

LDSCRIPTS=-L. -L$(BASE)Library/Device/Nuvoton/M0A21/Source/GCC -T gcc_arm.ld

LFLAGS=$(USE_NANO) $(USE_NOHOST) $(LDSCRIPTS) $(GC) $(MAP)

#$(NAME)-$(CORE).axf: main.c $(NAME).c $(STARTUP)
#	$(CC) $^ $(CFLAGS) $(LFLAGS) -o $@
IPATH= . \
       ../M0A21BSP/Library/CMSIS/Include \
       ../M0A21BSP/Library/Device/Nuvoton/M0A21/Include \
       ../M0A21BSP/Library/StdDriver/inc \
       User
SRC=$(wildcard \
       *.c \
       ../M0A21BSP/Library/Device/Nuvoton/M0A21/Source/*.c \
       ../M0A21BSP/Library/Device/Nuvoton/M0A21/Source/GCC/*.c \
       ../M0A21BSP/Library/StdDriver/src/*.c \
       User/*.c \
)
OBJS = $(patsubst %.c,./Objects/%.o,$(notdir $(SRC)))

$(NAME): $(OBJS)
	mkdir -p ./Objects
	cd ./Objects && $(CC) *.o $(CFLAGS) $(LFLAGS) $(STARTUP) -o $@.axf $(addprefix -I ../,$(IPATH)) 
	$(CP) -O binary ./Objects/$@.axf ./Objects/$@.bin

$(OBJS): $(SRC)
	mkdir -p ./Objects
	cd ./Objects && $(CC) $(addprefix ../,$^) $(CFLAGS) -c $(addprefix -I ../,$(IPATH))

.PHONY: all clean

all: $(NAME)

clean:
	rm -f ./Objects/*