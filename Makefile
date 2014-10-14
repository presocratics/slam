all: slam sensor-emu sensor-emu-stamp-feature multitap img2body lpf euler2qbw rmbias downsample getdt ARC rotate mean
ARC:
	$(MAKE) -C src/ARC $(MFLAGS)
lpf:
	$(MAKE) -C src/lpf $(MFLAGS)
slam:
	$(MAKE) -C src/slam $(MFLAGS)
sensor-emu:
	$(MAKE) -C src/sensor_emu $(MFLAGS)
sensor-emu-stamp-feature:
	$(MAKE) -C src/sensor_emu_stamp_feature $(MFLAGS)
multitap:
	$(MAKE) -C src/multitap $(MFLAGS)
img2body:
	$(MAKE) -C src/img2body $(MFLAGS)
euler2qbw:
	$(MAKE) -C src/euler2qbw $(MFLAGS)
rmbias:
	$(MAKE) -C src/rmbias $(MFLAGS)
downsample:
	$(MAKE) -C src/downsample $(MFLAGS)
getdt:
	$(MAKE) -C src/getdt $(MFLAGS)
rotate:
	$(MAKE) -C src/rotate $(MFLAGS)
mean:
	$(MAKE) -C src/mean $(MFLAGS)

clean:
	$(MAKE) clean -C src/slam
	$(MAKE) clean -C src/lpf
	$(MAKE) clean -C src/sensor_emu
	$(MAKE) clean -C src/multitap
	$(MAKE) clean -C src/img2body
	$(MAKE) clean -C src/euler2qbw
	$(MAKE) clean -C src/ARC
	$(MAKE) clean -C src/rotate
	$(MAKE) clean -C src/mean
