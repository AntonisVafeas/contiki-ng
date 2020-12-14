
all:
	$(MAKE) -C sphere-wearable3/screen-interfacing
	$(MAKE) -C sphere-wearable3/ble-adv-test
	$(MAKE) BOARD=spg-2/cc2650 -C generic/ble-adv-example
	$(MAKE) BOARD=spg-2/cc2650 -C generic/ble-scanner-example

clean:
	$(MAKE) -C sphere-wearable3/screen-interfacing clean
	$(MAKE) -C sphere-wearable3/ble-adv-test clean
	$(MAKE) -C generic/ble-adv-example clean
	$(MAKE) -C generic/ble-scanner-example clean

distclean:
	$(MAKE) -C sphere-wearable3/screen-interfacing distclean
	$(MAKE) -C sphere-wearable3/ble-adv-test distclean
	$(MAKE) -C generic/ble-adv-example distclean
	$(MAKE) -C generic/ble-scanner-example distclean
