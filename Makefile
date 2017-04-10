
CFLAGS= -O2 -Wall

LDFLAGS = -lm

all: iplc-sim.c
	$(CC) $(CFLAGS) iplc-sim.c -o iplc-sim $(LDFLAGS)

clean:
	rm iplc-sim
