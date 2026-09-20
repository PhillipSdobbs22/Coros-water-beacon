CC      ?= cc
CFLAGS  ?= -std=c11 -O2 -Wall -Wextra -Wshadow -Wconversion -Wsign-conversion
LDLIBS  := -lm

OBJS := bb_beacon.o sim_world.o sim_main.o

beacon_sim: $(OBJS)
	$(CC) $(CFLAGS) -o $@ $(OBJS) $(LDLIBS)

bb_beacon.o:  bb_beacon.c bb_beacon.h
sim_world.o:  sim_world.c sim_world.h
sim_main.o:   sim_main.c bb_beacon.h sim_world.h

run: beacon_sim
	./beacon_sim

clean:
	rm -f $(OBJS) beacon_sim

.PHONY: run clean

# --- visual demo data -------------------------------------------------------
export_data: tools/export_data.o bb_beacon.o sim_world.o
	$(CC) $(CFLAGS) -o $@ tools/export_data.o bb_beacon.o sim_world.o $(LDLIBS)

tools/export_data.o: tools/export_data.c bb_beacon.h sim_world.h

docs/demo/beacon-data.json: export_data
	mkdir -p docs/demo
	./export_data > $@

data: docs/demo/beacon-data.json
.PHONY: data
