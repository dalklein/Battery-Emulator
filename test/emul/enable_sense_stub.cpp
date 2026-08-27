/* Host stub for enable_sense.
 *
 * The real module (Software/src/devboard/utils/enable_sense.cpp) needs Arduino GPIO and
 * WiFi, neither of which the host emulation provides. The inverter module references
 * enable_sense_drive_datalayer at setup() time to opt the 12 V line into driving
 * readiness, so the host build needs the symbol to link.
 *
 * Stubbed the same way MQTT is (emul/mqtt_stub.cpp): the tests exercise the Modbus
 * register logic, not pin sensing. The enable->readiness path is hardware-verified on the
 * board instead -- see "THE EMULATOR ANSWERS ON REAL HARDWARE" in the sniffer README.
 */
bool enable_sense_drive_datalayer = false;
