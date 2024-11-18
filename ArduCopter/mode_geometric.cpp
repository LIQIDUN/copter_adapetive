#include "Copter.h"
#include <GCS_MAVLink/GCS.h>
/*
 * Init and run calls for stabilize flight mode
 */

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void ModeGeometric::run()
{
    update_simple_mode();
    gcs().send_text(MAV_SEVERITY_CRITICAL,"mode hang fly");  //地面站消息发送
}
