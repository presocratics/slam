all: slam sensor-emu multitap img2body
slam:
	$(MAKE) -C src/slam $(MFLAGS)
sensor-emu:
	$(MAKE) -C src/sensor_emu $(MFLAGS)
multitap:
	$(MAKE) -C src/multitap $(MFLAGS)
img2body:
	$(MAKE) -C src/img2body $(MFLAGS)
clean:
	$(MAKE) clean -C src/slam
	$(MAKE) clean -C src/sensor_emu
	$(MAKE) clean -C src/multitap
	$(MAKE) clean -C src/img2body
