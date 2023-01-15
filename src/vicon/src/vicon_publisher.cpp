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
#include "vicon_helper.h"
#ifdef WIN32
#include <conio.h>   // For _kbhit()
#include <windows.h> // For Sleep()
#include <cstdio> // For getchar()
#else
#include <unistd.h> // For sleep()
#endif              // WIN32


// parameters TODO
const int PUBLISHER_INTERVAL_MS = 20;
// constexpr static float x_offset = 0.5;
// constexpr static float y_offset = 0.5;
constexpr static float x_offset = 0.;
constexpr static float y_offset = 0.;

class ViconPublisher : public rclcpp::Node
{
public:
  ViconPublisher(const std::string &topic_name)
  : Node("vicon_publisher")
  {
    publisher_ =
      this->create_publisher<raptor_interface::msg::Pose>(topic_name, 10);
    
    // timer_ = this->create_wall_timer(
    //   std::chrono::milliseconds(PUBLISHER_INTERVAL_MS),
    //   std::bind(&ViconPublisher::publisherCallback, this));
  }

  void publish(raptor_interface::msg::Pose &message) 
  {
    publisher_->publish(message);
    rclcpp::spin_some(this->get_node_base_interface());
  }

private:
  rclcpp::Publisher<raptor_interface::msg::Pose>::SharedPtr publisher_;
  // rclcpp::TimerBase::SharedPtr timer_;
};



int main(int argc, char *argv[])
{
  // POI
  // TODO check argument
  if (argc < 2) {
    std::cout << "No command line argument given. Required: Vicon Identifier.\n";
    return 1;
  }
  
  // initialize ros
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ViconPublisher>("quad_vicon_pose");

  raptor_interface::msg::Pose message = raptor_interface::msg::Pose();
  std::string vicon_identifier = argv[1];

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
      "Initializing publisher for '%s' using the topic 'quad_vicon_pose'.",
      vicon_identifier.c_str());
  

  /* VICON Datastream Settings */
  std::vector<std::string> Hosts;
  // HARD CODE Vicon Address from LEO C6
  Hosts.push_back("10.10.10.5");
  if (Hosts.empty())
  {
    Hosts.push_back("localhost:801");
  }

  std::vector<std::string> HapticOnList(0);
  unsigned int ClientBufferSize = 0;
  std::string AxisMapping = "ZUp";
  std::vector<std::string> FilteredSubjects;
  std::vector<std::string> LocalAdapters;

  bool bQuiet = false;
  std::ostream &OutputStream(bQuiet ? NullStream : std::cout);

  // create hostname for connection
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

  // bOptimizeWireless settings
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
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Connection failed...");
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
    
    // std::cout << ".";

#ifdef WIN32
    Sleep(1000);
#else
    sleep(1);
#endif
    // }

    // std::cout << std::endl;
    // std::cout << std::endl;

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
                                  Direction::Left); // Y-up
    }

    Output_GetAxisMapping _Output_GetAxisMapping =
        DirectClient.GetAxisMapping();
    // std::cout << "Axis Mapping: X-" << Adapt(_Output_GetAxisMapping.XAxis) <<
    // " Y-" << Adapt(_Output_GetAxisMapping.YAxis) << " Z-"
    //          << Adapt(_Output_GetAxisMapping.ZAxis) << std::endl;

    // Discover the version number
    Output_GetVersion _Output_GetVersion = DirectClient.GetVersion();
    // std::cout << "Version: " << _Output_GetVersion.Major << "." <<
    // _Output_GetVersion.Minor << "." << _Output_GetVersion.Point << "."
    //           << _Output_GetVersion.Revision << std::endl;

    if (ClientBufferSize > 0)
    {
      DirectClient.SetBufferSize(ClientBufferSize);
      std::cout << "Setting client buffer size to " << ClientBufferSize
                << std::endl;
    }

    bool bSubjectFilterApplied = false;

    ViconDataStreamSDK::CPP::Client &MyClient(DirectClient);

    // POI
    // inform interface ready
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Vicon interface server is ready.");

    size_t Counter = 0;
    const std::chrono::high_resolution_clock::time_point StartTime =
        std::chrono::high_resolution_clock::now();

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
#ifdef WIN32
        Sleep(200);
#else
        sleep(1);
#endif

        // OutputStream << ".";
      }
      // OutputStream << std::endl;

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

      const std::chrono::high_resolution_clock::time_point Now =
          std::chrono::high_resolution_clock::now();

      // Get the frame number
      Output_GetFrameNumber _Output_GetFrameNumber = MyClient.GetFrameNumber();
      // OutputStream << "Frame Number: " << _Output_GetFrameNumber.FrameNumber
      // << std::endl;

      ////////////////////////////////////////////////////////
      // for (int i = 0; i < N; i++) {
      //   mocap_msg.at(i).header.timestamp =
      //   _Output_GetFrameNumber.FrameNumber;
      // }
      // for (int i = 0; i < argc - 1; i++)
      // {
      //   msg[i].header.timestamp = _Output_GetFrameNumber.FrameNumber;
      // }
      ////////////////////////////////////////////////////////
      Output_GetFrameRate Rate = MyClient.GetFrameRate();
      // OutputStream << "Frame rate: " << Rate.FrameRateHz << std::endl;

      // Show frame rates
      for (unsigned int FramerateIndex = 0;
           FramerateIndex < MyClient.GetFrameRateCount().Count;
           ++FramerateIndex)
      {
        std::string FramerateName =
            MyClient.GetFrameRateName(FramerateIndex).Name;
        double FramerateValue = MyClient.GetFrameRateValue(FramerateName).Value;

        // OutputStream << FramerateName << ": " << FramerateValue << "Hz" <<
        // std::endl;
      }
      // OutputStream << std::endl;

      // Get the timecode
      Output_GetTimecode _Output_GetTimecode = MyClient.GetTimecode();

      // OutputStream << "Timecode: " << _Output_GetTimecode.Hours << "h " <<
      // _Output_GetTimecode.Minutes << "m " << _Output_GetTimecode.Seconds <<
      // "s
      // "
      //   << _Output_GetTimecode.Frames << "f " << _Output_GetTimecode.SubFrame
      //   << "sf " << Adapt(_Output_GetTimecode.FieldFlag) << " "
      //   << Adapt(_Output_GetTimecode.Standard) << " " <<
      //   _Output_GetTimecode.SubFramesPerFrame << " " <<
      //   _Output_GetTimecode.UserBits
      //   << std::endl
      //   << std::endl;

      // Get the latency
      // OutputStream << "Latency: " << MyClient.GetLatencyTotal().Total << "s"
      // << std::endl;

      ///////////////////////////////////////////////////////////////
      // for (int i = 0; i < N; i++) {
      //   mocap_msg.at(i).latency = MyClient.GetLatencyTotal().Total;
      // }
      // for (int i = 0; i < argc - 1; i++)
      // {
      //   msg[i].latency = MyClient.GetLatencyTotal().Total;
      // }

      for (unsigned int LatencySampleIndex = 0;
           LatencySampleIndex < MyClient.GetLatencySampleCount().Count;
           ++LatencySampleIndex)
      {
        std::string SampleName =
            MyClient.GetLatencySampleName(LatencySampleIndex).Name;
        double SampleValue = MyClient.GetLatencySampleValue(SampleName).Value;

        // OutputStream << "  " << SampleName << " " << SampleValue << "s" <<
        // std::endl;
      }
      // OutputStream << std::endl;

      Output_GetHardwareFrameNumber _Output_GetHardwareFrameNumber =
          MyClient.GetHardwareFrameNumber();
      // OutputStream << "Hardware Frame Number: " <<
      // _Output_GetHardwareFrameNumber.HardwareFrameNumber << std::endl;

      // Count the number of subjects
      unsigned int SubjectCount = MyClient.GetSubjectCount().SubjectCount;
      // OutputStream << "Subjects (" << SubjectCount << "):" << std::endl;

      // POI
      // find requested subject
      bool subjectFound = false;
      unsigned int SubjectIndex = 0;
      for (unsigned int i = 0; i < SubjectCount; ++i)
      {
        std::string SubjectName =
            MyClient.GetSubjectName(SubjectIndex).SubjectName;

        if (SubjectName.compare(vicon_identifier) == 0)
        {
          SubjectIndex = i;
          subjectFound = true;
          break;
        }
      }

      // POI
      // exit if subject wasn't found
      if (!subjectFound) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
            "Subject '%s' not found.",
            vicon_identifier.c_str());
        // std::cout << "Subject '" << vicon_identifier << "' not found.\n";
        return 1;
      }


      // loop over all subject
      // for (unsigned int SubjectIndex = 0; SubjectIndex < SubjectCount;
      //      ++SubjectIndex)
      // { ////////////////////////////////////////////for
      /// loop starts here
      // OutputStream << "  Subject #" << SubjectIndex << std::endl;

      // Get the subject name
      std::string SubjectName =
          MyClient.GetSubjectName(SubjectIndex).SubjectName;
      // OutputStream << "    Name: " << SubjectName << std::endl;

      ////////////////////////////////////////////
      // std::cout << "header: " << SubjectName << std::endl;
      // std::cout << "header: " << parameters::objects.at(0) << std::endl;
      // for (int i = 0; i < N; i++) {
      //   if (SubjectName.compare(parameters::objects.at(i)) == 0) {
      //     std::cout << "header: " << i << SubjectName << std::endl;
      //     mocap_msg.at(i).header.id = SubjectName;
      //   }
      // }
      // for (int i = 0; i < argc - 1; i++)
      // {
      //   if (SubjectName.compare(objects.at(i)) == 0)
      //   {
      //     msg[i].header.description = SubjectName;
      //   }
      // }

      ////////////////////////////////////////////

      // Get the root segment
      std::string RootSegment =
          MyClient.GetSubjectRootSegmentName(SubjectName).SegmentName;
      // OutputStream << "    Root Segment: " << RootSegment << std::endl;

      // Count the number of segments
      unsigned int SegmentCount =
          MyClient.GetSegmentCount(SubjectName).SegmentCount;
      // OutputStream << "    Segments (" << SegmentCount << "):" <<
      // std::endl;

      // loop over all segments
      for (unsigned int SegmentIndex = 0; SegmentIndex < SegmentCount;
            ++SegmentIndex)
      {
        // OutputStream << "      Segment #" << SegmentIndex << std::endl;

        // Get the segment name
        std::string SegmentName =
            MyClient.GetSegmentName(SubjectName, SegmentIndex).SegmentName;
        // OutputStream << "        Name: " << SegmentName << std::endl;

        // Get the segment parent
        std::string SegmentParentName =
            MyClient.GetSegmentParentName(SubjectName, SegmentName)
                .SegmentName;
        // OutputStream << "        Parent: " << SegmentParentName <<
        // std::endl;

        // Get the segment's children
        unsigned int ChildCount =
            MyClient.GetSegmentChildCount(SubjectName, SegmentName)
                .SegmentCount;
        // OutputStream << "     Children (" << ChildCount << "):" <<
        // std::endl;
        for (unsigned int ChildIndex = 0; ChildIndex < ChildCount;
              ++ChildIndex)
        {
          std::string ChildName =
              MyClient
                  .GetSegmentChildName(SubjectName, SegmentName, ChildIndex)
                  .SegmentName;
          // OutputStream << "       " << ChildName << std::endl;
        }

        // Get the static segment scale
        Output_GetSegmentStaticScale _Output_GetSegmentStaticScale =
            MyClient.GetSegmentStaticScale(SubjectName, SegmentName);
        if (_Output_GetSegmentStaticScale.Result == Result::Success)
        {
          // OutputStream << "        Static Scale: (" <<
          // _Output_GetSegmentStaticScale.Scale[0] << ", " <<
          // _Output_GetSegmentStaticScale.Scale[1]
          //            << ", " << _Output_GetSegmentStaticScale.Scale[2] <<
          //            ")" << std::endl;
        }

        // Get the static segment translation
        Output_GetSegmentStaticTranslation
            _Output_GetSegmentStaticTranslation =
                MyClient.GetSegmentStaticTranslation(SubjectName,
                                                      SegmentName);
        // OutputStream << "        Static Translation: (" <<
        // _Output_GetSegmentStaticTranslation.Translation[0] << ", "
        //            << _Output_GetSegmentStaticTranslation.Translation[1] <<
        //            ", " <<
        //            _Output_GetSegmentStaticTranslation.Translation[2] <<
        //            ")"
        //          << std::endl;

        // Get the static segment rotation in helical co-ordinates
        /* Output_GetSegmentStaticRotationHelical
            _Output_GetSegmentStaticRotationHelical =
                MyClient.GetSegmentStaticRotationHelical(SubjectName,
                                                          SegmentName); */
        // OutputStream << "        Static Rotation Helical: (" <<
        // _Output_GetSegmentStaticRotationHelical.Rotation[0] << ", "
        //            << _Output_GetSegmentStaticRotationHelical.Rotation[1]
        //            << ", " <<
        //            _Output_GetSegmentStaticRotationHelical.Rotation[2] <<
        //            ")"
        //          << std::endl;

        // Get the static segment rotation as a matrix
        /* Output_GetSegmentStaticRotationMatrix
            _Output_GetSegmentStaticRotationMatrix =
                MyClient.GetSegmentStaticRotationMatrix(SubjectName,
                                                        SegmentName); */
        // OutputStream << "        Static Rotation Matrix: (" <<
        // _Output_GetSegmentStaticRotationMatrix.Rotation[0] << ", "
        //            << _Output_GetSegmentStaticRotationMatrix.Rotation[1] <<
        //            ", " <<
        //            _Output_GetSegmentStaticRotationMatrix.Rotation[2] << ",
        //            "
        //          << _Output_GetSegmentStaticRotationMatrix.Rotation[3] <<
        //          ", " << _Output_GetSegmentStaticRotationMatrix.Rotation[4]
        //          << ", "
        //        << _Output_GetSegmentStaticRotationMatrix.Rotation[5] << ",
        //        " << _Output_GetSegmentStaticRotationMatrix.Rotation[6] <<
        //        ", "
        //      << _Output_GetSegmentStaticRotationMatrix.Rotation[7] << ", "
        //      << _Output_GetSegmentStaticRotationMatrix.Rotation[8] << ")"
        //    << std::endl;

        // Get the static segment rotation in quaternion co-ordinates
        /* Output_GetSegmentStaticRotationQuaternion
            _Output_GetSegmentStaticRotationQuaternion =
                MyClient.GetSegmentStaticRotationQuaternion(SubjectName,
                                                            SegmentName); */
        // OutputStream << "        Static Rotation Quaternion: (" <<
        // _Output_GetSegmentStaticRotationQuaternion.Rotation[0] << ", "
        //             <<
        //             _Output_GetSegmentStaticRotationQuaternion.Rotation[1]
        //             << ", " <<
        //             _Output_GetSegmentStaticRotationQuaternion.Rotation[2]
        //           << ", " <<
        //           _Output_GetSegmentStaticRotationQuaternion.Rotation[3] <<
        //           ")" << std::endl;

        // Get the static segment rotation in EulerXYZ co-ordinates
        /* Output_GetSegmentStaticRotationEulerXYZ
            _Output_GetSegmentStaticRotationEulerXYZ =
                MyClient.GetSegmentStaticRotationEulerXYZ(SubjectName,
                                                          SegmentName); */
        // OutputStream << "        Static Rotation EulerXYZ: (" <<
        // _Output_GetSegmentStaticRotationEulerXYZ.Rotation[0] << ", "
        //           << _Output_GetSegmentStaticRotationEulerXYZ.Rotation[1]
        //           << ", " <<
        //           _Output_GetSegmentStaticRotationEulerXYZ.Rotation[2] <<
        //           ")"
        //         << std::endl;

        // Get the global segment translation
        Output_GetSegmentGlobalTranslation
            _Output_GetSegmentGlobalTranslation =
                MyClient.GetSegmentGlobalTranslation(SubjectName,
                                                      SegmentName);
        // OutputStream << "        Global Translation: (" <<
        // _Output_GetSegmentGlobalTranslation.Translation[0] << ", "
        //         << _Output_GetSegmentGlobalTranslation.Translation[1] << ",
        //         " << _Output_GetSegmentGlobalTranslation.Translation[2] <<
        //         ") "
        //       << Adapt(_Output_GetSegmentGlobalTranslation.Occluded) <<
        //       std::endl;

        // Capture position in FastDDS message
        ////////////////////////////////////////////
        // for (int i = 0; i < N; i++) {
        //   if (SubjectName.compare(parameters::objects.at(i)) == 0) {
        //     std::cout << "setting position" << std::endl;
        //     mocap_msg.at(i).pose.position.x =
        //         _Output_GetSegmentGlobalTranslation.Translation[0] /
        //         1000.0;
        //     mocap_msg.at(i).pose.position.y =
        //         _Output_GetSegmentGlobalTranslation.Translation[1] /
        //         1000.0;
        //     mocap_msg.at(i).pose.position.z =
        //         _Output_GetSegmentGlobalTranslation.Translation[2] /
        //         1000.0;
        //   }
        // }

        // POI
        message.x =
            (_Output_GetSegmentGlobalTranslation.Translation[0] /
              1000.0) -
            x_offset;
        message.y =
            ((_Output_GetSegmentGlobalTranslation.Translation[1] /
              1000.0) +
              y_offset) *
            (-1.0);
        message.z =
            _Output_GetSegmentGlobalTranslation.Translation[2] / 1000.0;

        // check data, skip if bad
        if (message.z < 0.00001) {
          continue;
        }

        ////////////////////////////////////////////

        // Get the global segment rotation in helical co-ordinates
        /* Output_GetSegmentGlobalRotationHelical
            _Output_GetSegmentGlobalRotationHelical =
                MyClient.GetSegmentGlobalRotationHelical(SubjectName,
                                                          SegmentName); */
        // OutputStream << "        Global Rotation Helical: (" <<
        // _Output_GetSegmentGlobalRotationHelical.Rotation[0] << ", "
        //          << _Output_GetSegmentGlobalRotationHelical.Rotation[1] <<
        //          ", " <<
        //          _Output_GetSegmentGlobalRotationHelical.Rotation[2] << ")
        //          "
        //        << Adapt(_Output_GetSegmentGlobalRotationHelical.Occluded)
        //        << std::endl;

        // Get the global segment rotation as a matrix
        /* Output_GetSegmentGlobalRotationMatrix
            _Output_GetSegmentGlobalRotationMatrix =
                MyClient.GetSegmentGlobalRotationMatrix(SubjectName,
                                                        SegmentName); */
        // OutputStream << "        Global Rotation Matrix: (" <<
        // _Output_GetSegmentGlobalRotationMatrix.Rotation[0] << ", "
        //            << _Output_GetSegmentGlobalRotationMatrix.Rotation[1] <<
        //            ", " <<
        //            _Output_GetSegmentGlobalRotationMatrix.Rotation[2] << ",
        //            "
        //          << _Output_GetSegmentGlobalRotationMatrix.Rotation[3] <<
        //          ", " << _Output_GetSegmentGlobalRotationMatrix.Rotation[4]
        //          << ", "
        //        << _Output_GetSegmentGlobalRotationMatrix.Rotation[5] << ",
        //        " << _Output_GetSegmentGlobalRotationMatrix.Rotation[6] <<
        //        ", "
        //      << _Output_GetSegmentGlobalRotationMatrix.Rotation[7] << ", "
        //      << _Output_GetSegmentGlobalRotationMatrix.Rotation[8] << ") "
        //    << Adapt(_Output_GetSegmentGlobalRotationMatrix.Occluded) <<
        //    std::endl;

        // Get the global segment rotation in quaternion co-ordinates
        /* Output_GetSegmentGlobalRotationQuaternion
            _Output_GetSegmentGlobalRotationQuaternion =
                MyClient.GetSegmentGlobalRotationQuaternion(SubjectName,
                                                            SegmentName); */
        // OutputStream << "        Global Rotation Quaternion: (" <<
        // _Output_GetSegmentGlobalRotationQuaternion.Rotation[0] << ", "
        //            <<
        //            _Output_GetSegmentGlobalRotationQuaternion.Rotation[1]
        //            << ", " <<
        //            _Output_GetSegmentGlobalRotationQuaternion.Rotation[2]
        //          << ", " <<
        //          _Output_GetSegmentGlobalRotationQuaternion.Rotation[3] <<
        //          ") "
        //        <<
        //        Adapt(_Output_GetSegmentGlobalRotationQuaternion.Occluded)
        //        << std::endl;

        // Capture position in FastDDS message
        ////////////////////////////////////////////
        // for (int i = 0; i < N; i++) {
        //   if (SubjectName.compare(parameters::objects.at(i)) == 0) {
        //     std::cout << "setting rotation" << std::endl;
        //     mocap_msg.at(i).pose.orientation_quat.x =
        //         _Output_GetSegmentGlobalRotationQuaternion.Rotation[0];
        //     mocap_msg.at(i).pose.orientation_quat.y =
        //         _Output_GetSegmentGlobalRotationQuaternion.Rotation[1];
        //     mocap_msg.at(i).pose.orientation_quat.z =
        //         _Output_GetSegmentGlobalRotationQuaternion.Rotation[2];
        //     mocap_msg.at(i).pose.orientation_quat.w =
        //         _Output_GetSegmentGlobalRotationQuaternion.Rotation[3];
        //   }
        // }

        ////////////////////////////////////////////

        // Get the global segment rotation in EulerXYZ co-ordinates
        /* Output_GetSegmentGlobalRotationEulerXYZ
            _Output_GetSegmentGlobalRotationEulerXYZ =
                MyClient.GetSegmentGlobalRotationEulerXYZ(SubjectName,
                                                          SegmentName); */
        // OutputStream << "        Global Rotation EulerXYZ: (" <<
        // _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[0] << ", "
        //            << _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[1]
        //            << ", " <<
        //            _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[2] <<
        //            ")
        //            "
        //          <<
        //          Adapt(_Output_GetSegmentGlobalRotationEulerXYZ.Occluded)
        //          << std::endl;
        // publish euler Rotation to DDS in degrees
        /* for (int i = 0; i < argc - 1; i++)
        {
          if (SubjectName.compare(objects.at(i)) == 0)
          {
            msg[i].orientation.roll =
                _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[0] *
                (180.0 / M_PI);
            msg[i].orientation.pitch =
                _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[1] *
                (180.0 / M_PI);
            msg[i].orientation.yaw =
                _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[2] *
                (180.0 / M_PI);
          }
        } */

        // Get the local segment translation
        Output_GetSegmentLocalTranslation _Output_GetSegmentLocalTranslation =
            MyClient.GetSegmentLocalTranslation(SubjectName, SegmentName);
        // OutputStream << "        Local Translation: (" <<
        // _Output_GetSegmentLocalTranslation.Translation[0] << ", "
        //          << _Output_GetSegmentLocalTranslation.Translation[1] << ",
        //          " << _Output_GetSegmentLocalTranslation.Translation[2] <<
        //          ") "
        //        << Adapt(_Output_GetSegmentLocalTranslation.Occluded) <<
        //        std::endl;

        // Get the local segment rotation in helical co-ordinates
        /* Output_GetSegmentLocalRotationHelical
            _Output_GetSegmentLocalRotationHelical =
                MyClient.GetSegmentLocalRotationHelical(SubjectName,
                                                        SegmentName); */
        // OutputStream << "        Local Rotation Helical: (" <<
        // _Output_GetSegmentLocalRotationHelical.Rotation[0] << ", "
        //              << _Output_GetSegmentLocalRotationHelical.Rotation[1]
        //              << ", " <<
        //              _Output_GetSegmentLocalRotationHelical.Rotation[2] <<
        //              ") "
        //              <<
        //              Adapt(_Output_GetSegmentLocalRotationHelical.Occluded)
        //              << std::endl;

        // Get the local segment rotation as a matrix
        /* Output_GetSegmentLocalRotationMatrix
            _Output_GetSegmentLocalRotationMatrix =
                MyClient.GetSegmentLocalRotationMatrix(SubjectName,
                                                        SegmentName); */
        // OutputStream << "        Local Rotation Matrix: (" <<
        // _Output_GetSegmentLocalRotationMatrix.Rotation[0] << ", "
        //              << _Output_GetSegmentLocalRotationMatrix.Rotation[1]
        //              << ", " <<
        //              _Output_GetSegmentLocalRotationMatrix.Rotation[2] <<
        //              ", "
        //              << _Output_GetSegmentLocalRotationMatrix.Rotation[3]
        //              << ", " <<
        //              _Output_GetSegmentLocalRotationMatrix.Rotation[4] <<
        //              ", "
        //              << _Output_GetSegmentLocalRotationMatrix.Rotation[5]
        //              << ", " <<
        //              _Output_GetSegmentLocalRotationMatrix.Rotation[6] <<
        //              ", "
        //              << _Output_GetSegmentLocalRotationMatrix.Rotation[7]
        //              << ", " <<
        //              _Output_GetSegmentLocalRotationMatrix.Rotation[8] <<
        //              ") "
        //              <<
        //              Adapt(_Output_GetSegmentLocalRotationMatrix.Occluded)
        //              << std::endl;

        // Get the local segment rotation in quaternion co-ordinates
        /* Output_GetSegmentLocalRotationQuaternion
            _Output_GetSegmentLocalRotationQuaternion =
                MyClient.GetSegmentLocalRotationQuaternion(SubjectName,
                                                            SegmentName); */
        // OutputStream << "        Local Rotation Quaternion: (" <<
        // _Output_GetSegmentLocalRotationQuaternion.Rotation[0] << ", "
        //              <<
        //              _Output_GetSegmentLocalRotationQuaternion.Rotation[1]
        //              << ", " <<
        //              _Output_GetSegmentLocalRotationQuaternion.Rotation[2]
        //              << ", " <<
        //              _Output_GetSegmentLocalRotationQuaternion.Rotation[3]
        //              << ") "
        //              <<
        //              Adapt(_Output_GetSegmentLocalRotationQuaternion.Occluded)
        //              << std::endl;

        // Get the local segment rotation in EulerXYZ co-ordinates
        /* Output_GetSegmentLocalRotationEulerXYZ
            _Output_GetSegmentLocalRotationEulerXYZ =
                MyClient.GetSegmentLocalRotationEulerXYZ(SubjectName,
                                                          SegmentName); */
        // OutputStream << "        Local Rotation EulerXYZ: (" <<
        // _Output_GetSegmentLocalRotationEulerXYZ.Rotation[0] << ", "
        //              << _Output_GetSegmentLocalRotationEulerXYZ.Rotation[1]
        //              << ", " <<
        //              _Output_GetSegmentLocalRotationEulerXYZ.Rotation[2] <<
        //              ")
        //              "
        //              <<
        //              Adapt(_Output_GetSegmentLocalRotationEulerXYZ.Occluded)
        //              << std::endl;
      }

      // Get the quality of the subject (object) if supported
      Output_GetObjectQuality _Output_GetObjectQuality =
          MyClient.GetObjectQuality(SubjectName);
      if (_Output_GetObjectQuality.Result == Result::Success)
      {
        double Quality = _Output_GetObjectQuality.Quality;
        // OutputStream << "    Quality: " << Quality << std::endl;
      }

      // POI
      // publish
      node->publish(message);
      // control rate
      std::this_thread::sleep_for(std::chrono::milliseconds(PUBLISHER_INTERVAL_MS));
    }

    ++Counter;
  }

  rclcpp::shutdown();
  return 0;
}