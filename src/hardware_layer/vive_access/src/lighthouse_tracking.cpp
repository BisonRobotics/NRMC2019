//
// Created by nmach on 9/19/18.
//

//
// HTC Vive Lighthouse Tracking Example
// By Peter Thor 2016
//
// Shows how to extract basic tracking data
//
#include <dlfcn.h>

#include <vive_access/lighthouse_tracking.h>

// Destructor
LighthouseTracking::~LighthouseTracking() {
  if (m_pHMD != NULL)
  {
    vr::VR_Shutdown();
    m_pHMD = NULL;
  }
}

// Constructor
LighthouseTracking::LighthouseTracking() {
  vr::EVRInitError eError = vr::VRInitError_None;
  m_pHMD = vr::VR_Init(&eError, vr::VRApplication_Background);

  if (eError != vr::VRInitError_None)
  {
    m_pHMD = NULL;
    std::cout << "Unable to init VR runtime: %s" << vr::VR_GetVRInitErrorAsEnglishDescription(eError) << std::endl;
    exit(EXIT_FAILURE);
  }
}

/*
* Loop-listen for events then parses them (e.g. prints the to user)
* Returns true if success or false if openvr has quit
*/
bool LighthouseTracking::RunProcedure(bool bWaitForEvents) {

  // Either A) wait for events, such as hand controller button press, before parsing...
  if (bWaitForEvents) {
    // Process VREvent
    vr::VREvent_t event;
    while (m_pHMD->PollNextEvent(&event, sizeof(event)))
    {
      // Process event
      if (!ProcessVREvent(event)) {
        std::cout << "(OpenVR) service quit" << std::endl;
        //return false;
      }

      ParseTrackingFrame();
    }
  }
  else {
    // ... or B) continous parsint of tracking data irrespective of events
    std::cout << std::endl << "Parsing next frame...";

    ParseTrackingFrame();
  }

  return true;
}

//-----------------------------------------------------------------------------
// Purpose: Processes a single VR event
//-----------------------------------------------------------------------------

bool LighthouseTracking::ProcessVREvent(const vr::VREvent_t & event)
{
  switch (event.eventType)
  {
    case vr::VREvent_TrackedDeviceActivated:
    {
      //SetupRenderModelForTrackedDevice(event.trackedDeviceIndex);
      std::cout <<  "(OpenVR) Device : " << event.trackedDeviceIndex << " attached" << std::endl;
    }
      break;

    case vr::VREvent_TrackedDeviceDeactivated:
    {
      std::cout <<  "(OpenVR) Device : " << event.trackedDeviceIndex << " detached" << std::endl;
    }
      break;

    case vr::VREvent_TrackedDeviceUpdated:
    {
      std::cout <<  "(OpenVR) Device : " << event.trackedDeviceIndex << " updated" << std::endl;
    }
      break;

    case (vr::VREvent_DashboardActivated) :
    {
      std::cout <<  "(OpenVR) Dashboard activated" << std::endl;

    }
      break;

    case (vr::VREvent_DashboardDeactivated) :
    {
      std::cout <<  "(OpenVR) Dashboard deactivated" << std::endl;
    }
      break;

    case (vr::VREvent_ChaperoneDataHasChanged) :
    {
      std::cout <<  "(OpenVR) Chaperone data has changed" << std::endl;
    }
      break;

    case (vr::VREvent_ChaperoneSettingsHaveChanged) :
    {
      std::cout <<  "(OpenVR) Chaperone settings have changed" << std::endl;
    }
      break;

    case (vr::VREvent_ChaperoneUniverseHasChanged) :
    {
      std::cout <<  "(OpenVR) Chaperone universe has changed" << std::endl;
    }
      break;

    case (vr::VREvent_ApplicationTransitionStarted) :
    {
      std::cout <<  "(OpenCV) Application Transition: Transition has started" << std::endl;
    }
      break;

    case (vr::VREvent_ApplicationTransitionNewAppStarted) :
    {
      std::cout << "(OpenVR) Application transition: New app has started << std::endl" << std::endl;
    }
      break;

    case (vr::VREvent_Quit) :
    {
      std::cout << "(OpenVR) Received SteamVR Quit (" <<  vr::VREvent_Quit <<")" << std::endl;
      return false;
    }
      break;

    case (vr::VREvent_ProcessQuit) :
    {
      std::cout << "(OpenVR) SteamVR Quit Process (" << vr::VREvent_ProcessQuit << ")" << std::endl;
      return false;
    }
      break;

    case (vr::VREvent_QuitAborted_UserPrompt) :
    {
      std::cout << "(OpenVR) SteamVR Quit Aborted UserPrompt (" << vr::VREvent_QuitAborted_UserPrompt << ")" << std::endl;
      return false;
    }
      break;

    case (vr::VREvent_QuitAcknowledged) :
    {
      std::cout << "(OpenVR) SteamVR Quit Acknowledged (" << vr::VREvent_QuitAcknowledged << ")" << std::endl;
      return false;
    }
      break;

    case (vr::VREvent_TrackedDeviceRoleChanged) :
    {
      std::cout << "(OpenVR) TrackedDeviceRoleChanged: %" << event.trackedDeviceIndex;
      break;
    }

    case (vr::VREvent_TrackedDeviceUserInteractionStarted) :
    {
      char buf[1024];
      std::cout << "(OpenVR) TrackedDeviceUserInteractionStarted: " << event.trackedDeviceIndex << std::endl;
      break;
    }

    default:
      char buf[1024];
      std::cout << "(OpenVR) Event: " << event.eventType << std::endl;
      break;
  }

  return true;
}


// Get the quaternion representing the rotation
vr::HmdQuaternion_t LighthouseTracking::GetRotation(vr::HmdMatrix34_t matrix) {
  vr::HmdQuaternion_t q;

  q.w = sqrt(fmax(0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2;
  q.x = sqrt(fmax(0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2;
  q.y = sqrt(fmax(0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2;
  q.z = sqrt(fmax(0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2;
  q.x = copysign(q.x, matrix.m[2][1] - matrix.m[1][2]);
  q.y = copysign(q.y, matrix.m[0][2] - matrix.m[2][0]);
  q.z = copysign(q.z, matrix.m[1][0] - matrix.m[0][1]);
  return q;
}

// Get the vector representing the position
vr::HmdVector3_t LighthouseTracking::GetPosition(vr::HmdMatrix34_t matrix) {
  vr::HmdVector3_t vector;

  vector.v[0] = matrix.m[0][3];
  vector.v[1] = matrix.m[1][3];
  vector.v[2] = matrix.m[2][3];

  return vector;
}

/*
* Parse a Frame with data from the tracking system
*
* Handy reference:
* https://github.com/TomorrowTodayLabs/NewtonVR/blob/master/Assets/SteamVR/Scripts/SteamVR_Utils.cs
*
* Also:
* Open VR Convention (same as OpenGL)
* right-handed system
* +y is up
* +x is to the right
* -z is going away from you
* http://www.3dgep.com/understanding-the-view-matrix/
*
*/
void LighthouseTracking::ParseTrackingFrame() {

  // Process SteamVR device states
  for (vr::TrackedDeviceIndex_t unDevice = 0; unDevice < vr::k_unMaxTrackedDeviceCount; unDevice++)
  {
    // if not connected just skip the rest of the routine
    if (!m_pHMD->IsTrackedDeviceConnected(unDevice))
      continue;

    vr::TrackedDevicePose_t trackedDevicePose;
    vr::TrackedDevicePose_t *devicePose = &trackedDevicePose;

    vr::VRControllerState_t controllerState;
    vr::VRControllerState_t *ontrollerState_ptr = &controllerState;

    vr::HmdVector3_t position;
    vr::HmdQuaternion_t quaternion;

    /*if (vr::VRSystem()->IsInputFocusCapturedByAnotherProcess()) {
      std::cout << "Input Focus by Another Process" << std::endl;
    }*/ // TODO figure out how to fix?

    bool bPoseValid = trackedDevicePose.bPoseIsValid;
    vr::HmdVector3_t vVel;
    vr::HmdVector3_t vAngVel;
    vr::ETrackingResult eTrackingResult;

    // Get what type of device it is and work with its data
    vr::ETrackedDeviceClass trackedDeviceClass = vr::VRSystem()->GetTrackedDeviceClass(unDevice);
    switch (trackedDeviceClass) {
      case vr::ETrackedDeviceClass::TrackedDeviceClass_GenericTracker:
        // Simliar to the HMD case block above, please adapt as you like
        // to get away with code duplication and general confusion

        vr::VRSystem()->GetControllerStateWithPose(vr::TrackingUniverseStanding, unDevice, &controllerState, sizeof(controllerState), &trackedDevicePose);

        position = GetPosition(devicePose->mDeviceToAbsoluteTracking);
        quaternion = GetRotation(devicePose->mDeviceToAbsoluteTracking);

        vVel = trackedDevicePose.vVelocity;
        vAngVel = trackedDevicePose.vAngularVelocity;
        eTrackingResult = trackedDevicePose.eTrackingResult;
        bPoseValid = trackedDevicePose.bPoseIsValid;


        printf("\nTracker position\nx: %.2f y: %.2f z: %.2f\n", position.v[0], position.v[1], position.v[2]);
        printf("qw: %.2f qx: %.2f qy: %.2f qz: %.2f\n", quaternion.w, quaternion.x, quaternion.y, quaternion.z);

        switch (eTrackingResult) {
          case vr::ETrackingResult::TrackingResult_Uninitialized:
            printf("Invalid tracking result\n");
            break;
          case vr::ETrackingResult::TrackingResult_Calibrating_InProgress:
            printf("Calibrating in progress\n");
            break;
          case vr::ETrackingResult::TrackingResult_Calibrating_OutOfRange:
            printf("Calibrating Out of range\n");
            break;
          case vr::ETrackingResult::TrackingResult_Running_OK:
            printf("Running OK\n");
            break;
          case vr::ETrackingResult::TrackingResult_Running_OutOfRange:
            printf("WARNING: Running Out of Range\n");

            break;
          default:
            printf("Default\n");
            break;
        }

        if (bPoseValid)
          printf("Valid pose\n");
        else
          printf("Invalid pose\n");

        break;
    }
  }
}