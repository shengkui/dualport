TARGET=dualport

CFLAGS=-Wall -O
LDLIBS=-pthread

all: $(TARGET)

%: %.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

clean:
	$(RM) $(TARGET)
