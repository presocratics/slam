all: slam sensor-emu multitap
slam:
	$(MAKE) -C src/slam $(MFLAGS)
sensor-emu:
	$(MAKE) -C src/sensor_emu $(MFLAGS)
multitap:
	$(MAKE) -C src/multitap $(MFLAGS)
clean:
	$(MAKE) clean -C src/slam
	$(MAKE) clean -C src/sensor_emu
	$(MAKE) clean -C src/multitap
