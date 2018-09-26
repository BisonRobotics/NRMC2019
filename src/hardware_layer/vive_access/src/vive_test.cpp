#include <openvr/openvr.h>
#include <vive_access/lighthouse_tracking.h>
#include <boost/thread/thread.hpp>

using boost::this_thread::sleep_for;
using boost::chrono::seconds;

int main( int argc, char** argv )
{
  // If false we'll parse tracking data continously, if true we parse when an openvr event fires
  bool bAcquireTrackingDataByWaitingForVREvents = false;

  // Create a new LighthouseTracking instance and parse as needed
  LighthouseTracking *lighthouseTracking = new LighthouseTracking();
  if (lighthouseTracking) {
    printf("Press 'q' to quit. Starting capture of tracking data...\n");
    sleep_for(seconds(1));

    while (lighthouseTracking->RunProcedure(bAcquireTrackingDataByWaitingForVREvents)) {

      sleep_for(seconds(10));
    }

    delete lighthouseTracking;
  }
  return EXIT_SUCCESS;
}