#include <cassert>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <string.h>
#include <time.h>
#include <thread>

// ros default
#include "rclcpp/rclcpp.hpp"
#include "raptor_interface/msg/pose.hpp"

// vicon
#include "DataStreamClient.h"
#include "vicon/vicon_helper.h"
#include "PX4-Matrix/matrix/math.hpp"
#ifdef WIN32
#include <conio.h>   // For _kbhit()
#include <windows.h> // For Sleep()
#include <cstdio> // For getchar()
#else
#include <unistd.h> // For sleep()
#endif              // WIN32


// parameters TODO
const int LOOP_RATE_HZ = 100;
rclcpp::Rate loop_rate(LOOP_RATE_HZ);
// constexpr static float x_offset = 0.5;
// constexpr static float y_offset = 0.5;
constexpr static float x_offset = 0.;
constexpr static float y_offset = 0.;




class ViconPublisher : public rclcpp::Node
{
public:
  ViconPublisher(const std::string &node_name, const std::string &topic_name)
  : Node(node_name)
  {
    pose_publisher_ = this->create_publisher<raptor_interface::msg::Pose>(topic_name, 10);
  }

  void publish(raptor_interface::msg::Pose &message) 
  {
    pose_publisher_->publish(message);
    rclcpp::spin_some(this->get_node_base_interface());
  }

private:
  rclcpp::Publisher<raptor_interface::msg::Pose>::SharedPtr pose_publisher_;
};




int main(int argc, char *argv[])
{
  // POI
  // TODO check argument
  if (argc < 2) {
    std::cout << "No command line argument given. Required: Object name.\n";
    return 1;
  }

  std::string vicon_identifier = std::string("srl_") + argv[1];
  std::string node_name = std::string("vicon_publisher_") + argv[1];
  std::string topic_name = argv[1] + std::string("_pose_nwu");
  
  // initialize ros
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ViconPublisher>(node_name, topic_name);

  raptor_interface::msg::Pose message = raptor_interface::msg::Pose();

  RCLCPP_INFO(node->get_logger(), 
      "Initializing %s for '%s' using the topic '%s'.",
      node_name.c_str(), vicon_identifier.c_str(), topic_name.c_str());
  

  ///////////////////////////////////////////////////// Vicon Datastream Options
  std::vector<std::string> Hosts;
  Hosts.push_back("10.10.10.5"); // vicon address LEO C 6
  if (Hosts.empty())
  {
    Hosts.push_back("localhost:801");
  }

  std::vector<std::string> HapticOnList(0);
  unsigned int ClientBufferSize = 0;
  std::string AxisMapping = "ZUp";
  std::vector<std::string> FilteredSubjects;
  std::vector<std::string> LocalAdapters;

  // bool bQuiet = false;
  // std::ostream &OutputStream(bQuiet ? NullStream : std::cout);
  std::ostream &OutputStream(std::cout);

  bool First = true;
  std::string HostName;
  for (const auto &rHost : Hosts)
  {
    if (!First)
    {
      HostName += ";";
    }
    HostName += rHost;
    First = false;
  }

  // Make a new client
  ViconDataStreamSDK::CPP::Client DirectClient;

  bool bOptimizeWireless = false;
  if (bOptimizeWireless)
  {
    const Output_ConfigureWireless ConfigureWirelessResult =
        DirectClient.ConfigureWireless();

    if (ConfigureWirelessResult.Result != Result::Success)
    {
      std::cout << "Wireless Config: " << ConfigureWirelessResult.Error
                << std::endl;
    }
  }

  // POI
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
      "Connecting to '%s'...",
      HostName.c_str());

  while (!DirectClient.IsConnected().Connected)
  {
    // Direct connection
    const Output_Connect ConnectResult = DirectClient.Connect(HostName);
    const bool ok = (ConnectResult.Result == Result::Success);

    if (!ok)
    {
      std::cout << "Warning - connect failed... ";
      switch (ConnectResult.Result)
      {
      case Result::ClientAlreadyConnected:
        std::cout << "Client Already Connected" << std::endl;
        break;
      case Result::InvalidHostName:
        std::cout << "Invalid Host Name" << std::endl;
        break;
      case Result::ClientConnectionFailed:
        std::cout << "Client Connection Failed" << std::endl;
        break;
      default:
        std::cout << "Unrecognized Error: " << ConnectResult.Result
                  << std::endl;
        break;
      }
      return 1;
    }

#ifdef WIN32
    Sleep(1000);
#else
    sleep(1);
#endif
    // }

    // Enable some different data types
    DirectClient.EnableSegmentData();

    DirectClient.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ServerPush);
    // }

    // Set the global up axis
    DirectClient.SetAxisMapping(Direction::Forward, Direction::Left,
                                Direction::Up); // Z-up

    if (AxisMapping == "YUp")
    {
      DirectClient.SetAxisMapping(Direction::Forward, Direction::Up,
                                  Direction::Right); // Y-up
    }
    else if (AxisMapping == "XUp")
    {
      DirectClient.SetAxisMapping(Direction::Up, Direction::Forward,
                                  Direction::Left); // Z-up
    }

    // Output_GetAxisMapping _Output_GetAxisMapping = DirectClient.GetAxisMapping();
    // std::cout << "Axis Mapping: X-" << Adapt(_Output_GetAxisMapping.XAxis)
    //           << " Y-" << Adapt(_Output_GetAxisMapping.YAxis) << " Z-"
    //           << Adapt(_Output_GetAxisMapping.ZAxis) << std::endl;

    // Discover the version number
    // Output_GetVersion _Output_GetVersion = DirectClient.GetVersion();
    // std::cout << "Version: " << _Output_GetVersion.Major << "."
    //           << _Output_GetVersion.Minor << "." << _Output_GetVersion.Point
    //           << "." << _Output_GetVersion.Revision << std::endl;

    if (ClientBufferSize > 0)
    {
      DirectClient.SetBufferSize(ClientBufferSize);
      std::cout << "Setting client buffer size to " << ClientBufferSize
                << std::endl;
    }

    bool bSubjectFilterApplied = false;

    ViconDataStreamSDK::CPP::Client &MyClient(DirectClient);




    ////////////////////////////////////////////////////////////// main loop POI
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Vicon interface is ready.");

    // Loop until a key is pressed
#ifdef WIN32
    while (!Hit())
#else
    while (true)
#endif
    {
      // Get a frame
      // OutputStream << "Waiting for new frame...";
      while (MyClient.GetFrame().Result != Result::Success)
      {
        // Sleep a little so that we don't lumber the CPU with a busy poll
        // #ifdef WIN32
        //         Sleep(200);
        // #else
        //         sleep(1);
        // #endif

        // OutputStream << "No frame received from Vicon.";
        // OutputStream << std::endl;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "No frame received from Vicon.");
        if (!rclcpp::ok()) {
          rclcpp::shutdown();
          exit(0);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      // We have to call this after the call to get frame, otherwise we don't
      // have any subject info to map the name to ids
      if (!bSubjectFilterApplied)
      {
        for (const auto &rSubject : FilteredSubjects)
        {
          Output_AddToSubjectFilter SubjectFilterResult =
              MyClient.AddToSubjectFilter(rSubject);
          bSubjectFilterApplied = bSubjectFilterApplied ||
                                  SubjectFilterResult.Result == Result::Success;
        }
      }

      // ///////////////////////////////////////////////////////////
      // Set frame number
      // ///////////////////////////////////////////////////////////

      // Get the frame number POI
      // Output_GetFrameNumber _Output_GetFrameNumber = MyClient.GetFrameNumber();
      // Output_GetFrameRate Rate = MyClient.GetFrameRate();
      // OutputStream << "Frame rate: " << Rate.FrameRateHz << std::endl;

      // Show frame rates
      for (unsigned int FramerateIndex = 0;
           FramerateIndex < MyClient.GetFrameRateCount().Count;
           ++FramerateIndex)
      {
        std::string FramerateName =
            MyClient.GetFrameRateName(FramerateIndex).Name;
        // double FramerateValue = MyClient.GetFrameRateValue(FramerateName).Value;

        // OutputStream << FramerateName << ": " << FramerateValue << "Hz"
        //              << std::endl;
      }
      // OutputStream << std::endl;

      // Get the timecode
      // Output_GetTimecode _Output_GetTimecode = MyClient.GetTimecode();

      ///////////////////////////////////////////////////////////
      // Set latency
      ///////////////////////////////////////////////////////////

      // Get the latency
      // float latency = MyClient.GetLatencyTotal().Total;

      // for (unsigned int LatencySampleIndex = 0;
      //      LatencySampleIndex < MyClient.GetLatencySampleCount().Count;
      //      ++LatencySampleIndex) {
      //   std::string SampleName =
      //       MyClient.GetLatencySampleName(LatencySampleIndex).Name;
      //   double SampleValue =
      //   MyClient.GetLatencySampleValue(SampleName).Value;

      //   OutputStream << "  " << SampleName << " " << SampleValue << "s"
      //                << std::endl;
      // }

      // OutputStream << std::endl;

      // Output_GetHardwareFrameNumber _Output_GetHardwareFrameNumber =
      //     MyClient.GetHardwareFrameNumber();
      // OutputStream << "Hardware Frame Number: "
      //              << _Output_GetHardwareFrameNumber.HardwareFrameNumber
      //              << std::endl;

      ///////////////////////////////////////////////////////////
      // find requested subject POI
      ///////////////////////////////////////////////////////////

      // Count the number of subjects
      unsigned int SubjectCount = MyClient.GetSubjectCount().SubjectCount;
      // OutputStream << "Subjects (" << SubjectCount << "):" << std::endl;

      bool subjectFound = false;
      unsigned int SubjectIndex = 0;
      for (unsigned int i = 0; i < SubjectCount; ++i)
      {
        std::string SubjectName = MyClient.GetSubjectName(i).SubjectName;
        // OutputStream << SubjectName << std::endl;

        if (SubjectName.compare(vicon_identifier) == 0)
        {
          SubjectIndex = i;
          subjectFound = true;
          break;
        }
      }
      // exit if subject wasn't found
      if (!subjectFound) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
            "Subject '%s' not found.", vicon_identifier.c_str());
        return 1;
      }

      // OutputStream << "  Subject #" << SubjectIndex << std::endl;
      // Get the subject name
      std::string SubjectName =
          MyClient.GetSubjectName(SubjectIndex).SubjectName;
      // OutputStream << "    Name: " << SubjectName << std::endl;

      ///////////////////////////////////////////////////////////
      // segments
      ///////////////////////////////////////////////////////////

      // Get the root segment
      // std::string RootSegment =
      //     MyClient.GetSubjectRootSegmentName(SubjectName).SegmentName;
      // OutputStream << "    Root Segment: " << RootSegment << std::endl;

      // Count the number of segments
      unsigned int SegmentCount =
          MyClient.GetSegmentCount(SubjectName).SegmentCount;
      // OutputStream << "    Segments (" << SegmentCount << "):" <<
      // std::endl;

      // loop over all segments 
      // TODO there should only be one segment for the quad anyway?
      for (unsigned int SegmentIndex = 0; SegmentIndex < SegmentCount;
            ++SegmentIndex)
      {
        // OutputStream << "      Segment #" << SegmentIndex << std::endl;

        // Get the segment name
        std::string SegmentName =
            MyClient.GetSegmentName(SubjectName, SegmentIndex).SegmentName;
        // OutputStream << "        Name: " << SegmentName << std::endl;

        // Get the global segment translation
        Output_GetSegmentGlobalTranslation
            _Output_GetSegmentGlobalTranslation =
                MyClient.GetSegmentGlobalTranslation(SubjectName,
                                                      SegmentName);

        ///////////////////////////////////////////////////////////
        // global translation POI
        ///////////////////////////////////////////////////////////

        message.x_m =
            _Output_GetSegmentGlobalTranslation.Translation[0] / 1000.0;
        message.y_m =
            _Output_GetSegmentGlobalTranslation.Translation[1] /1000.0;
        message.z_m =
            _Output_GetSegmentGlobalTranslation.Translation[2] / 1000.0;


        ///////////////////////////////////////////////////////////
        // global rotation POI
        ///////////////////////////////////////////////////////////

        // Get the global segment rotation in quaternion co-ordinates
        Output_GetSegmentGlobalRotationQuaternion
            _Output_GetSegmentGlobalRotationQuaternion =
                MyClient.GetSegmentGlobalRotationQuaternion(SubjectName,
                                                            SegmentName);

        // Set global rotation quaternion POI
        matrix::Quatf quat_orientation(
            _Output_GetSegmentGlobalRotationQuaternion.Rotation[1],
            _Output_GetSegmentGlobalRotationQuaternion.Rotation[2],
            _Output_GetSegmentGlobalRotationQuaternion.Rotation[3],
            _Output_GetSegmentGlobalRotationQuaternion.Rotation[0]);

        matrix::Eulerf euler_orientation(quat_orientation);

        // Roll transformation
        if (euler_orientation(0) > 0)
          message.roll_deg = M_PI - euler_orientation(0);

        else if (euler_orientation(0) < 0)
          message.roll_deg = -M_PI - euler_orientation(0);

        // Pitch transformation
        message.pitch_deg = -euler_orientation(1);

        if (euler_orientation(2) > 0)
          message.yaw_deg = M_PI - euler_orientation(2);

        else if (euler_orientation(2) < 0)
          message.yaw_deg = -M_PI - euler_orientation(2);

        // Invert Sign to match px4 convention
        message.yaw_deg = -message.yaw_deg;

        // transform to deg
        message.roll_deg = message.roll_deg * 180.0 / M_PI;
        message.pitch_deg = message.pitch_deg * 180.0 / M_PI;
        message.yaw_deg = message.yaw_deg * 180.0 / M_PI;

        //   // Get the global segment rotation in EulerXYZ co-ordinates
        //   Output_GetSegmentGlobalRotationEulerXYZ
        //       _Output_GetSegmentGlobalRotationEulerXYZ =
        //           MyClient.GetSegmentGlobalRotationEulerXYZ(SubjectName,
        //                                                     SegmentName);
      }

      // // Get the quality of the subject (object) if supported
      // Output_GetObjectQuality _Output_GetObjectQuality =
      //     MyClient.GetObjectQuality(SubjectName);
      // if (_Output_GetObjectQuality.Result == Result::Success) {
      //   double Quality = _Output_GetObjectQuality.Quality;
      //   OutputStream << "    Quality: " << Quality << std::endl;
      // }




      /////////////////////////////////////////////////////////// POI
      // check if vicon data is valid
      if (abs(message.x_m) < 0.0001 &&
            abs(message.y_m) < 0.0001 &&
            abs(message.z_m) < 0.0001) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Vicon data is invalid.");
        continue;
      }
      
      // transform
      message.y_m = -message.y_m;
      message.z_m = message.z_m;
      message.pitch_deg = -message.pitch_deg;
      message.yaw_deg = -message.yaw_deg;

      node->publish(message);
      // rclcpp::spin_some(node);


      // dev printers
      // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
      //     "Vicon pose: [%f, %f, %f | %f, %f, %f]",
      //     message.x_m,
      //     message.y_m,
      //     message.z_m,
      //     message.roll_deg,
      //     message.pitch_deg,
      //     message.yaw_deg);


      // control loop rate
      loop_rate.sleep();

    } // main loop end
  } // connection loop end

  rclcpp::shutdown();
  return 0;
}
