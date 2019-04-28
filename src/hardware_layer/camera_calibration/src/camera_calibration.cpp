#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <cstdio>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include <camera/ocam_camera.h>

using namespace cv;
using namespace std;

tracker::Camera* initializeOCam(tracker::CameraInfo info)
{

  tracker::Camera* camera = nullptr;
  while (camera == nullptr)
  {
    try
    {
      // brightness (int)    : min=0 max=255 step=1 default=64 value=50
      // exposure_auto (menu)   : min=0 max=3 default=1 value=1
      // exposure_absolute (int)    : min=1 max=1000 step=1 default=55 value=90
      camera = new tracker::OCamCamera(info, 50, 64, 80);
    }
    catch (std::runtime_error &e)
    {
      printf("%s", e.what());
      usleep(1000000);
    }
  }
  return camera;
}

class Settings
{
public:
  Settings() : goodInput(false)
  {}

  enum Pattern
  {
    NOT_EXISTING, CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID
  };
  enum InputType
  {
    INVALID, CAMERA, VIDEO_FILE, IMAGE_LIST
  };

  void write(FileStorage &fs) const                        //Write serialization for this class
  {
    fs << "{"
       << "BoardSize_Width" << boardSize.width
       << "BoardSize_Height" << boardSize.height
       << "Square_Size" << squareSize
       << "Calibrate_Pattern" << patternToUse
       << "Calibrate_NrOfFrameToUse" << nrFrames
       << "Calibrate_FixAspectRatio" << aspectRatio
       << "Calibrate_AssumeZeroTangentialDistortion" << calibZeroTangentDist
       << "Calibrate_FixPrincipalPointAtTheCenter" << calibFixPrincipalPoint

       << "Write_DetectedFeaturePoints" << writePoints
       << "Write_extrinsicParameters" << writeExtrinsics
       << "Write_gridPoints" << writeGrid
       << "Write_outputFileName" << outputFileName

       << "Show_UndistortedImage" << showUndistorsed

       << "Input_FlipAroundHorizontalAxis" << flipVertical
       << "Input_Delay" << delay
       << "Input" << input
       << "}";
  }

  void read(const FileNode &node)                          //Read serialization for this class
  {
    node["BoardSize_Width"] >> boardSize.width;
    node["BoardSize_Height"] >> boardSize.height;
    node["Calibrate_Pattern"] >> patternToUse;
    node["Square_Size"] >> squareSize;
    node["Calibrate_NrOfFrameToUse"] >> nrFrames;
    node["Calibrate_FixAspectRatio"] >> aspectRatio;
    node["Write_DetectedFeaturePoints"] >> writePoints;
    node["Write_extrinsicParameters"] >> writeExtrinsics;
    node["Write_gridPoints"] >> writeGrid;
    node["Write_outputFileName"] >> outputFileName;
    node["Calibrate_AssumeZeroTangentialDistortion"] >> calibZeroTangentDist;
    node["Calibrate_FixPrincipalPointAtTheCenter"] >> calibFixPrincipalPoint;
    node["Calibrate_UseFisheyeModel"] >> useFisheye;
    node["Input_FlipAroundHorizontalAxis"] >> flipVertical;
    node["Show_UndistortedImage"] >> showUndistorsed;
    node["Input"] >> input;
    node["Input_Delay"] >> delay;
    node["Fix_K1"] >> fixK1;
    node["Fix_K2"] >> fixK2;
    node["Fix_K3"] >> fixK3;
    node["Fix_K4"] >> fixK4;
    node["Fix_K5"] >> fixK5;

    validate();
  }

  void validate()
  {
    goodInput = true;
    if (boardSize.width <= 0 || boardSize.height <= 0)
    {
      cerr << "Invalid Board size: " << boardSize.width << " " << boardSize.height << endl;
      goodInput = false;
    }
    if (squareSize <= 10e-6)
    {
      cerr << "Invalid square size " << squareSize << endl;
      goodInput = false;
    }
    if (nrFrames <= 0)
    {
      cerr << "Invalid number of frames " << nrFrames << endl;
      goodInput = false;
    }

    if (input.empty())      // Check for valid input
      inputType = INVALID;
    else
    {
      tracker::CameraInfo info(input);
      camera = initializeOCam(info);
    }
    if (inputType == INVALID)
    {
      cerr << " Input does not exist: " << input;
      goodInput = false;
    }

    flag = 0;
    if (calibFixPrincipalPoint) flag |= CALIB_FIX_PRINCIPAL_POINT;
    if (calibZeroTangentDist) flag |= CALIB_ZERO_TANGENT_DIST;
    if (aspectRatio) flag |= CALIB_FIX_ASPECT_RATIO;
    if (fixK1) flag |= CALIB_FIX_K1;
    if (fixK2) flag |= CALIB_FIX_K2;
    if (fixK3) flag |= CALIB_FIX_K3;
    if (fixK4) flag |= CALIB_FIX_K4;
    if (fixK5) flag |= CALIB_FIX_K5;

    if (useFisheye)
    {
      // the fisheye model has its own enum, so overwrite the flags
      flag = fisheye::CALIB_FIX_SKEW | fisheye::CALIB_RECOMPUTE_EXTRINSIC;
      if (fixK1) flag |= fisheye::CALIB_FIX_K1;
      if (fixK2) flag |= fisheye::CALIB_FIX_K2;
      if (fixK3) flag |= fisheye::CALIB_FIX_K3;
      if (fixK4) flag |= fisheye::CALIB_FIX_K4;
      if (calibFixPrincipalPoint) flag |= fisheye::CALIB_FIX_PRINCIPAL_POINT;
    }

    calibrationPattern = NOT_EXISTING;
    if (!patternToUse.compare("CHESSBOARD")) calibrationPattern = CHESSBOARD;
    if (!patternToUse.compare("CIRCLES_GRID")) calibrationPattern = CIRCLES_GRID;
    if (!patternToUse.compare("ASYMMETRIC_CIRCLES_GRID")) calibrationPattern = ASYMMETRIC_CIRCLES_GRID;
    if (calibrationPattern == NOT_EXISTING)
    {
      cerr << " Camera calibration mode does not exist: " << patternToUse << endl;
      goodInput = false;
    }
    atImageList = 0;

  }

  Mat nextImage()
  {
    Mat result(camera->getHeight(), camera->getWidth(), CV_8UC1);
    camera->getFrame(result.data);
    cv::rotate(result, result, cv::ROTATE_180);
    return result;
  }

  static bool readStringList(const string &filename, vector<string> &l)
  {
    l.clear();
    FileStorage fs(filename, FileStorage::READ);
    if (!fs.isOpened())
      return false;
    FileNode n = fs.getFirstTopLevelNode();
    if (n.type() != FileNode::SEQ)
      return false;
    FileNodeIterator it = n.begin(), it_end = n.end();
    for (; it != it_end; ++it)
      l.push_back((string) *it);
    return true;
  }

  static bool isListOfImages(const string &filename)
  {
    string s(filename);
    // Look for file extension
    if (s.find(".xml") == string::npos && s.find(".yaml") == string::npos && s.find(".yml") == string::npos)
      return false;
    else
      return true;
  }

public:
  Size boardSize;              // The size of the board -> Number of items by width and height
  Pattern calibrationPattern;  // One of the Chessboard, circles, or asymmetric circle pattern
  float squareSize;            // The size of a square in your defined unit (point, millimeter,etc).
  int nrFrames;                // The number of frames to use from the input for calibration
  float aspectRatio;           // The aspect ratio
  int delay;                   // In case of a video input
  bool writePoints;            // Write detected feature points
  bool writeExtrinsics;        // Write extrinsic parameters
  bool writeGrid;              // Write refined 3D target grid points
  bool calibZeroTangentDist;   // Assume zero tangential distortion
  bool calibFixPrincipalPoint; // Fix the principal point at the center
  bool flipVertical;           // Flip the captured images around the horizontal axis
  string outputFileName;       // The name of the file where to write
  bool showUndistorsed;        // Show undistorted images after calibration
  string input;                // The input ->
  bool useFisheye;             // use fisheye camera model for calibration
  bool fixK1;                  // fix K1 distortion coefficient
  bool fixK2;                  // fix K2 distortion coefficient
  bool fixK3;                  // fix K3 distortion coefficient
  bool fixK4;                  // fix K4 distortion coefficient
  bool fixK5;                  // fix K5 distortion coefficient

  int cameraID;
  vector<string> imageList;
  size_t atImageList;
  //VideoCapture inputCapture;
  tracker::Camera *camera;
  InputType inputType;
  bool goodInput;
  int flag;

private:
  string patternToUse;


};

static inline void read(const FileNode &node, Settings &x, const Settings &default_value = Settings())
{
  if (node.empty())
    x = default_value;
  else
    x.read(node);
}

enum
{
  DETECTION = 0, CAPTURING = 1, CALIBRATED = 2
};

bool runCalibrationAndSave(Settings &s, Size imageSize, Mat &cameraMatrix, Mat &distCoeffs,
                           vector<vector<Point2f> > imagePoints, float grid_width, bool release_object);

int main(int argc, char *argv[])
{
  const String keys
      = "{help h usage ? |           | print this message            }"
        "{@settings      |default.xml| input setting file            }"
        "{d              |           | actual distance between top-left and top-right corners of "
        "the calibration grid }"
        "{winSize        | 11        | Half of search window for cornerSubPix }";
  CommandLineParser parser(argc, argv, keys);
  parser.about("This is a camera calibration sample.\n"
               "Usage: camera_calibration [configuration_file -- default ./default.xml]\n"
               "Near the sample file you'll find the configuration file, which has detailed help of "
               "how to edit it. It may be any OpenCV supported file format XML/YAML.");
  if (!parser.check())
  {
    parser.printErrors();
    return 0;
  }

  if (parser.has("help"))
  {
    parser.printMessage();
    return 0;
  }

  //! [file_read]
  Settings s;
  const string inputSettingsFile = parser.get<string>(0);
  FileStorage fs(inputSettingsFile, FileStorage::READ); // Read the settings
  if (!fs.isOpened())
  {
    cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << endl;
    parser.printMessage();
    return -1;
  }
  fs["Settings"] >> s;
  fs.release();                                         // close Settings file
  //! [file_read]

  //FileStorage fout("settings.yml", FileStorage::WRITE); // write config as YAML
  //fout << "Settings" << s;

  if (!s.goodInput)
  {
    cout << "Invalid input detected. Application stopping. " << endl;
    return -1;
  }

  int winSize = parser.get<int>("winSize");

  float grid_width = s.squareSize * (s.boardSize.width - 1);
  bool release_object = false;
  if (parser.has("d"))
  {
    grid_width = parser.get<float>("d");
    release_object = true;
  }

  vector<vector<Point2f> > imagePoints;
  Mat cameraMatrix, distCoeffs;
  Size imageSize;
  int mode = s.inputType == Settings::IMAGE_LIST ? CAPTURING : DETECTION;
  clock_t prevTimestamp = 0;
  const Scalar RED(0, 0, 255), GREEN(0, 255, 0);
  const char ESC_KEY = 27;

  /*
   * Custom
   */
  s.camera->start();

  //! [get_input]
  Mat gray, view, backup_view;
  bool hold = false, found = false, calibrated = false, undistorted = false;
  int count = 0;
  vector<Point2f> pointBuf;
  printf("\nStarting calibration...\n");
  for (;;)
  {

    // Grab new image or hold onto current image
    if (!hold)
    {
      pointBuf.clear();
      try
      {
        gray = s.nextImage();
      }
      catch (std::runtime_error &e)
      {
        printf("[ERROR] %s\n", e.what());
        usleep(100000);
        continue;
      }
      if (undistorted)
      {
        Mat temp = gray.clone();
        undistort(temp, gray, cameraMatrix, distCoeffs);
      }
      cvtColor(gray, view, COLOR_GRAY2BGR);
    }
    else
    {
      Mat tmp = s.nextImage();
    }

    // Show current image
    cv::imshow("Calibration Tool", view);

    // Handle input
    int key = cv::waitKey(1);
    // Quit
    if (key == 'q')
    {
      break;
    }
    // Hold or unhold
    else if (key == 'h')
    {
      hold = !hold;
      if (hold)
      {
        hold = true;
        int chessBoardFlags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE;
        found = findChessboardCorners(gray, s.boardSize, pointBuf, chessBoardFlags);
        if (found)
        {
          // Improve detection
          cornerSubPix(gray, pointBuf, Size(winSize, winSize),
                       Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.0001));

          // Draw the corners.
          drawChessboardCorners(view, s.boardSize, Mat(pointBuf), found);
        }
      }
    }
    // Capture
    else if (key == 'y')
    {
      if (hold && found)
      {
        hold = false;
        if (!undistorted)
        {
          printf("Capture %i\n", count++);
          imagePoints.push_back(pointBuf);
        }
        else
        {
          printf("Can't capture while undistorted is on\n");
        }
      }
      else
      {
        printf("Must be holding a detected image to capture\n");
      }
    }
    // Calibrate and save
    else if (key == 'k')
    {
      printf("Calibrating...\n");
      calibrated = runCalibrationAndSave(s, view.size(),  cameraMatrix, distCoeffs, imagePoints, grid_width, release_object);
      if (calibrated)
      {
        printf("Successfully calibrated\n");
        printf("Camera matrix:\n");
        std::cout << cameraMatrix << std::endl;
        printf("fx = %f, fy = %f, cx = %f, cy = %f\n",
            cameraMatrix.at<double>(0,0), cameraMatrix.at<double>(1,1),
            cameraMatrix.at<double>(0,2), cameraMatrix.at<double>(1,2));
      }
      else
      {
        printf("Calibration failed\n");
      }
    }
    // Undistort
    else if (key == 'u')
    {

      if (calibrated)
      {
        undistorted = !undistorted;
        if (undistorted)
        {
          printf("Undistort enabled\n");
          Mat temp = view.clone();
          undistort(temp, view, cameraMatrix, distCoeffs);
        }
        else
        {
          printf("Undistort disabled\n");
        }
      }
      else
      {
        printf("Calibrate first\n");
      }
    }
  }
  s.camera->stop();
  printf("\nDone...\n\n");
  if (calibrated)
  {
    printf("Camera matrix:\n");
    std::cout << cameraMatrix << std::endl;
    printf("fx = %f, fy = %f, cx = %f, cy = %f\n",
           cameraMatrix.at<double>(0,0), cameraMatrix.at<double>(1,1),
           cameraMatrix.at<double>(0,2), cameraMatrix.at<double>(1,2));
  }

  return 0;
}

//! [compute_errors]
static double computeReprojectionErrors(const vector<vector<Point3f> > &objectPoints,
                                        const vector<vector<Point2f> > &imagePoints,
                                        const vector<Mat> &rvecs, const vector<Mat> &tvecs,
                                        const Mat &cameraMatrix, const Mat &distCoeffs,
                                        vector<float> &perViewErrors, bool fisheye)
{
  vector<Point2f> imagePoints2;
  size_t totalPoints = 0;
  double totalErr = 0, err;
  perViewErrors.resize(objectPoints.size());

  for (size_t i = 0; i < objectPoints.size(); ++i)
  {
    if (fisheye)
    {
      fisheye::projectPoints(objectPoints[i], imagePoints2, rvecs[i], tvecs[i], cameraMatrix,
                             distCoeffs);
    }
    else
    {
      projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
    }
    err = norm(imagePoints[i], imagePoints2, NORM_L2);

    size_t n = objectPoints[i].size();
    perViewErrors[i] = (float) std::sqrt(err * err / n);
    totalErr += err * err;
    totalPoints += n;
  }

  return std::sqrt(totalErr / totalPoints);
}

//! [compute_errors]
//! [board_corners]
static void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f> &corners,
                                     Settings::Pattern patternType /*= Settings::CHESSBOARD*/)
{
  corners.clear();

  switch (patternType)
  {
    case Settings::CHESSBOARD:
    case Settings::CIRCLES_GRID:
      for (int i = 0; i < boardSize.height; ++i)
        for (int j = 0; j < boardSize.width; ++j)
          corners.push_back(Point3f(j * squareSize, i * squareSize, 0));
      break;

    case Settings::ASYMMETRIC_CIRCLES_GRID:
      for (int i = 0; i < boardSize.height; i++)
        for (int j = 0; j < boardSize.width; j++)
          corners.push_back(Point3f((2 * j + i % 2) * squareSize, i * squareSize, 0));
      break;
    default:
      break;
  }
}

//! [board_corners]
static bool runCalibration(Settings &s, Size &imageSize, Mat &cameraMatrix, Mat &distCoeffs,
                           vector<vector<Point2f> > imagePoints, vector<Mat> &rvecs, vector<Mat> &tvecs,
                           vector<float> &reprojErrs, double &totalAvgErr, vector<Point3f> &newObjPoints,
                           float grid_width, bool release_object)
{
  //! [fixed_aspect]
  cameraMatrix = Mat::eye(3, 3, CV_64F);
  if (s.flag & CALIB_FIX_ASPECT_RATIO)
    cameraMatrix.at<double>(0, 0) = s.aspectRatio;
  //! [fixed_aspect]
  if (s.useFisheye)
  {
    distCoeffs = Mat::zeros(4, 1, CV_64F);
  }
  else
  {
    distCoeffs = Mat::zeros(8, 1, CV_64F);
  }

  vector<vector<Point3f> > objectPoints(1);
  calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0], s.calibrationPattern);
  objectPoints[0][s.boardSize.width - 1].x = objectPoints[0][0].x + grid_width;
  newObjPoints = objectPoints[0];

  objectPoints.resize(imagePoints.size(), objectPoints[0]);

  //Find intrinsic and extrinsic camera parameters
  double rms;

  if (s.useFisheye)
  {
    Mat _rvecs, _tvecs;
    rms = fisheye::calibrate(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, _rvecs,
                             _tvecs, s.flag);

    rvecs.reserve(_rvecs.rows);
    tvecs.reserve(_tvecs.rows);
    for (int i = 0; i < int(objectPoints.size()); i++)
    {
      rvecs.push_back(_rvecs.row(i));
      tvecs.push_back(_tvecs.row(i));
    }
  }
  else
  {
    int iFixedPoint = -1;
    if (release_object)
      iFixedPoint = s.boardSize.width - 1;
    rms = calibrateCameraRO(objectPoints, imagePoints, imageSize, iFixedPoint,
                            cameraMatrix, distCoeffs, rvecs, tvecs, newObjPoints,
                            s.flag | CALIB_USE_LU);
  }

  if (release_object)
  {
    cout << "New board corners: " << endl;
    cout << newObjPoints[0] << endl;
    cout << newObjPoints[s.boardSize.width - 1] << endl;
    cout << newObjPoints[s.boardSize.width * (s.boardSize.height - 1)] << endl;
    cout << newObjPoints.back() << endl;
  }

  cout << "Re-projection error reported by calibrateCamera: " << rms << endl;

  bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

  objectPoints.clear();
  objectPoints.resize(imagePoints.size(), newObjPoints);
  totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs, cameraMatrix,
                                          distCoeffs, reprojErrs, s.useFisheye);

  return ok;
}

// Print camera parameters to the output file
static void saveCameraParams(Settings &s, Size &imageSize, Mat &cameraMatrix, Mat &distCoeffs,
                             const vector<Mat> &rvecs, const vector<Mat> &tvecs,
                             const vector<float> &reprojErrs, const vector<vector<Point2f> > &imagePoints,
                             double totalAvgErr, const vector<Point3f> &newObjPoints)
{
  FileStorage fs(s.outputFileName, FileStorage::WRITE);

  time_t tm;
  time(&tm);
  struct tm *t2 = localtime(&tm);
  char buf[1024];
  strftime(buf, sizeof(buf), "%c", t2);

  fs << "calibration_time" << buf;

  if (!rvecs.empty() || !reprojErrs.empty())
    fs << "nr_of_frames" << (int) std::max(rvecs.size(), reprojErrs.size());
  fs << "image_width" << imageSize.width;
  fs << "image_height" << imageSize.height;
  fs << "board_width" << s.boardSize.width;
  fs << "board_height" << s.boardSize.height;
  fs << "square_size" << s.squareSize;

  if (s.flag & CALIB_FIX_ASPECT_RATIO)
    fs << "fix_aspect_ratio" << s.aspectRatio;

  if (s.flag)
  {
    std::stringstream flagsStringStream;
    if (s.useFisheye)
    {
      flagsStringStream << "flags:"
                        << (s.flag & fisheye::CALIB_FIX_SKEW ? " +fix_skew" : "")
                        << (s.flag & fisheye::CALIB_FIX_K1 ? " +fix_k1" : "")
                        << (s.flag & fisheye::CALIB_FIX_K2 ? " +fix_k2" : "")
                        << (s.flag & fisheye::CALIB_FIX_K3 ? " +fix_k3" : "")
                        << (s.flag & fisheye::CALIB_FIX_K4 ? " +fix_k4" : "")
                        << (s.flag & fisheye::CALIB_RECOMPUTE_EXTRINSIC ? " +recompute_extrinsic" : "");
    }
    else
    {
      flagsStringStream << "flags:"
                        << (s.flag & CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess" : "")
                        << (s.flag & CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "")
                        << (s.flag & CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point" : "")
                        << (s.flag & CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist" : "")
                        << (s.flag & CALIB_FIX_K1 ? " +fix_k1" : "")
                        << (s.flag & CALIB_FIX_K2 ? " +fix_k2" : "")
                        << (s.flag & CALIB_FIX_K3 ? " +fix_k3" : "")
                        << (s.flag & CALIB_FIX_K4 ? " +fix_k4" : "")
                        << (s.flag & CALIB_FIX_K5 ? " +fix_k5" : "");
    }
    fs.writeComment(flagsStringStream.str());
  }

  fs << "flags" << s.flag;

  fs << "fisheye_model" << s.useFisheye;

  fs << "camera_matrix" << cameraMatrix;
  fs << "distortion_coefficients" << distCoeffs;

  fs << "avg_reprojection_error" << totalAvgErr;
  if (s.writeExtrinsics && !reprojErrs.empty())
    fs << "per_view_reprojection_errors" << Mat(reprojErrs);

  if (s.writeExtrinsics && !rvecs.empty() && !tvecs.empty())
  {
    CV_Assert(rvecs[0].type() == tvecs[0].type());
    Mat bigmat((int) rvecs.size(), 6, CV_MAKETYPE(rvecs[0].type(), 1));
    bool needReshapeR = rvecs[0].depth() != 1 ? true : false;
    bool needReshapeT = tvecs[0].depth() != 1 ? true : false;

    for (size_t i = 0; i < rvecs.size(); i++)
    {
      Mat r = bigmat(Range(int(i), int(i + 1)), Range(0, 3));
      Mat t = bigmat(Range(int(i), int(i + 1)), Range(3, 6));

      if (needReshapeR)
        rvecs[i].reshape(1, 1).copyTo(r);
      else
      {
        //*.t() is MatExpr (not Mat) so we can use assignment operator
        CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
        r = rvecs[i].t();
      }

      if (needReshapeT)
        tvecs[i].reshape(1, 1).copyTo(t);
      else
      {
        CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
        t = tvecs[i].t();
      }
    }
    fs.writeComment("a set of 6-tuples (rotation vector + translation vector) for each view");
    fs << "extrinsic_parameters" << bigmat;
  }

  if (s.writePoints && !imagePoints.empty())
  {
    Mat imagePtMat((int) imagePoints.size(), (int) imagePoints[0].size(), CV_32FC2);
    for (size_t i = 0; i < imagePoints.size(); i++)
    {
      Mat r = imagePtMat.row(int(i)).reshape(2, imagePtMat.cols);
      Mat imgpti(imagePoints[i]);
      imgpti.copyTo(r);
    }
    fs << "image_points" << imagePtMat;
  }

  if (s.writeGrid && !newObjPoints.empty())
  {
    fs << "grid_points" << newObjPoints;
  }
}

//! [run_and_save]
bool runCalibrationAndSave(Settings &s, Size imageSize, Mat &cameraMatrix, Mat &distCoeffs,
                           vector<vector<Point2f> > imagePoints, float grid_width, bool release_object)
{
  vector<Mat> rvecs, tvecs;
  vector<float> reprojErrs;
  double totalAvgErr = 0;
  vector<Point3f> newObjPoints;

  bool ok = runCalibration(s, imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs, reprojErrs,
                           totalAvgErr, newObjPoints, grid_width, release_object);
  cout << (ok ? "Calibration succeeded" : "Calibration failed")
       << ". avg re projection error = " << totalAvgErr << endl;

  if (ok)
    saveCameraParams(s, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, reprojErrs, imagePoints,
                     totalAvgErr, newObjPoints);
  return ok;
}
//! [run_and_save]
