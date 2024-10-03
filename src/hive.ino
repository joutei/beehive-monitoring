#include "hive.h"


void setup() {
  initSensors();

  initAndSetupOS();

  // Start job
  do_read_critical(&readCriticalJob);
  do_update(&updatejob);
  do_send(&sendjob);

  // The packet indicating that the node has been turned on/reset has already been sent (do_sent(...)), now we set it back to 0
  isReset = 0;
} 

void loop() {
    os_runloop_once();
}
