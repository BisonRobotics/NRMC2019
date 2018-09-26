#ifndef VIVE_ACCESS_LIGHTHOUSETRACKING_H
#define VIVE_ACCESS_LIGHTHOUSETRACKING_H

// OpenVR
#include <openvr/openvr.h>
#include <openvr/shared/Matrices.h>

class LighthouseTracking {
  private:

    // Basic stuff
    vr::IVRSystem *m_pHMD = NULL;
    vr::TrackedDevicePose_t m_rTrackedDevicePose[vr::k_unMaxTrackedDeviceCount];
    Matrix4 m_rmat4DevicePose[vr::k_unMaxTrackedDeviceCount];

    // Position and rotation of pose
    vr::HmdVector3_t    GetPosition(vr::HmdMatrix34_t matrix);
    vr::HmdQuaternion_t GetRotation(vr::HmdMatrix34_t matrix);

    // If false the program will parse tracking data continously and not wait for openvr events
    bool bWaitForEventsBeforeParsing = false;

  public:
    ~LighthouseTracking();
    LighthouseTracking();

    // Main loop that listens for openvr events and calls process and parse routines, if false the service has quit
    bool RunProcedure(bool bWaitForEvents);

    // Process a VR event and print some general info of what happens
    bool ProcessVREvent(const vr::VREvent_t & event);

    // Parse a tracking frame and print its position / rotation
    void ParseTrackingFrame();

};

#endif //VIVE_ACCESS_LIGHTHOUSETRACKING_H
