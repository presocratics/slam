all: slam sensor-emu multitap img2body
lpf:
	$(MAKE) -C src/lpf $(MFLAGS)
slam:
	$(MAKE) -C src/slam $(MFLAGS)
sensor-emu:
	$(MAKE) -C src/sensor_emu $(MFLAGS)
multitap:
	$(MAKE) -C src/multitap $(MFLAGS)
img2body:
	$(MAKE) -C src/img2body $(MFLAGS)
euler2qbw:
	$(MAKE) -C src/euler2qbw $(MFLAGS)
clean:
	$(MAKE) clean -C src/slam
	$(MAKE) clean -C src/lpf
	$(MAKE) clean -C src/sensor_emu
	$(MAKE) clean -C src/multitap
	$(MAKE) clean -C src/img2body
	$(MAKE) clean -C src/euler2qbw
