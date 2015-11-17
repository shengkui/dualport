TARGET=dualport

CFLAGS=-Wall -O #-DDEBUG
LDLIBS=-pthread

all: $(TARGET)

%: %.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

clean:
	$(RM) $(TARGET)
