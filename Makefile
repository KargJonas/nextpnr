# make image                     the container everything runs in (yosys, nextpnr + uarch)
# make himbaechel-image          the same through nextpnr-himbaechel (himbaechel/README.md)
# make build/hb/<name>/bitstream_mcu.bin   a design under designs/ through it
# make build/<name>/bitstream_mcu.bin   any design under designs/<name>/<name>.{v,pins}
# make flash BIN=<bitstream_mcu.bin>    program the Shrike and capture 2 s on the LA

.PHONY: image flash himbaechel-image
.SECONDEXPANSION:

image:
	podman build -t nextpnr-slg47910 -f Containerfile .

# The Project Inkwell database the himbaechel chipdb is built from.
FORGEFPGA ?= ../Project-Inkwell/forgefpga

himbaechel-image:
	podman build -t localhost/nextpnr-slg47910-hb --build-context forgefpga=$(FORGEFPGA) -f himbaechel/Containerfile himbaechel

build/hb/%/bitstream_mcu.bin: designs/%/$$*.v designs/%/$$*.pins build/chipdb.json
	himbaechel/flow.sh designs/$*/$*.v $* designs/$*/$*.pins build/hb/$*

build/chipdb.json: device/graph/minplacer.rgraph_def tools/gen_chipdb.py tools/routing_graph.py
	python3 tools/gen_chipdb.py -o $@

build/%/bitstream_mcu.bin: designs/%/$$*.v designs/%/$$*.pins build/chipdb.json
	./flow.sh designs/$*/$*.v $* designs/$*/$*.pins build/$*

flash:
	python3 hw/hw.py ab $(BIN)
