
#include "u3Streamer.h"
#include <iostream>

u3Streamer::u3Streamer() {
  hDevice_ = openUSBConnection(320095409);

  // Opening first found U3 over USB
  if (hDevice_ == NULL) {
    std::cout << "Cannot find U3 device, please check connections" << std::endl;
    return;
  }

  // Getting calibration information from U3
  if (getCalibrationInfo(hDevice_, &cali_info_) < 0) {
    std::cout << "closing USB connection" << std::endl;
    closeUSBConnection(hDevice_);
  }

  if (ConfigIO(hDevice_, &dac1_enabled_) != 0) {
    std::cout << "closing USB connection" << std::endl;
    closeUSBConnection(hDevice_);
  }

  // Stopping any previous streams
  StreamStop(hDevice_);

  if (StreamConfig(hDevice_) != 0) {
    std::cout << "closing USB connection" << std::endl;
    closeUSBConnection(hDevice_);
  }

  if (StreamStart(hDevice_) != 0) {
    std::cout << "closing USB connection" << std::endl;
    closeUSBConnection(hDevice_);
  }

  // StreamData(hDevice_, &cali_info_, dac1_enabled_);
  // StreamStop(hDevice_);

  std::cout << "U3 initialized" << std::endl;
  init_flag_ = true;
}

u3Streamer::~u3Streamer() { StreamStop(hDevice_); }

bool u3Streamer::isInit() { return init_flag_; }

int u3Streamer::ConfigIO(HANDLE hDevice, int *isDAC1Enabled) {
  uint8 sendBuff[12], recBuff[12];
  uint16 checksumTotal;
  int sendChars, recChars;

  sendBuff[1] = (uint8)(0xF8); // Command byte
  sendBuff[2] = (uint8)(0x03); // Number of data words
  sendBuff[3] = (uint8)(0x0B); // Extended command number

  sendBuff[6] =
      13; // Writemask : Setting writemask for timerCounterConfig (bit 0),
          //             FIOAnalog (bit 2) and EIOAnalog (bit 3)

  sendBuff[7] = 0; // Reserved
  sendBuff[8] =
      64; // TimerCounterConfig: Disabling all timers and counters,
          //                     set TimerCounterPinOffset to 4 (bits 4-7)
  sendBuff[9] = 0; // DAC1Enable

  sendBuff[10] = 255; // FIOAnalog : setting all FIOs as analog inputs
  sendBuff[11] = 255; // EIOAnalog : setting all EIOs as analog inputs
  extendedChecksum(sendBuff, 12);

  // Sending command to U3
  if ((sendChars = LJUSB_Write(hDevice, sendBuff, 12)) < 12) {
    if (sendChars == 0)
      printf("ConfigIO error : write failed\n");
    else
      printf("ConfigIO error : did not write all of the buffer\n");
    return -1;
  }

  // Reading response from U3
  if ((recChars = LJUSB_Read(hDevice, recBuff, 12)) < 12) {
    if (recChars == 0)
      printf("ConfigIO error : read failed\n");
    else
      printf("ConfigIO error : did not read all of the buffer\n");
    return -1;
  }

  checksumTotal = extendedChecksum16(recBuff, 12);
  if ((uint8)((checksumTotal / 256) & 0xFF) != recBuff[5]) {
    printf("ConfigIO error : read buffer has bad checksum16(MSB)\n");
    return -1;
  }

  if ((uint8)(checksumTotal & 0xFF) != recBuff[4]) {
    printf("ConfigIO error : read buffer has bad checksum16(LBS)\n");
    return -1;
  }

  if (extendedChecksum8(recBuff) != recBuff[0]) {
    printf("ConfigIO error : read buffer has bad checksum8\n");
    return -1;
  }

  if (recBuff[1] != (uint8)(0xF8) || recBuff[2] != (uint8)(0x03) ||
      recBuff[3] != (uint8)(0x0B)) {
    printf("ConfigIO error : read buffer has wrong command bytes\n");
    return -1;
  }

  if (recBuff[6] != 0) {
    printf("ConfigIO error : read buffer received errorcode %d\n", recBuff[6]);
    return -1;
  }

  if (recBuff[8] != 64) {
    printf("ConfigIO error : TimerCounterConfig did not get set correctly\n");
    return -1;
  }

  if (recBuff[10] != 255 && recBuff[10] != (uint8)(0x0F)) {
    printf("ConfigIO error : FIOAnalog did not set get correctly\n");
    return -1;
  }

  if (recBuff[11] != 255) {
    printf("ConfigIO error : EIOAnalog did not set get correctly (%d)\n",
           recBuff[11]);
    return -1;
  }

  *isDAC1Enabled = (int)recBuff[9];

  return 0;
}

int u3Streamer::StreamConfig(HANDLE hDevice) {
  uint8 sendBuff[64], recBuff[8];
  uint16 checksumTotal, scanInterval;
  int sendBuffSize, sendChars, recChars, i;

  sendBuffSize = 12 + num_channels_ * 2;

  sendBuff[1] = (uint8)(0xF8);       // Command byte
  sendBuff[2] = 3 + num_channels_;   // Number of data words = num_channels_ + 3
  sendBuff[3] = (uint8)(0x11);       // Extended command number
  sendBuff[6] = num_channels_;       // num_channels_
  sendBuff[7] = samples_per_packet_; // samples_per_packet_
  sendBuff[8] = 0;                   // Reserved
  sendBuff[9] = 1;                   // ScanConfig:
                                     //  Bit 7: Reserved
                                     //  Bit 6: Reserved
                   //  Bit 3: Internal stream clock frequency = b0: 4 MHz
                   //  Bit 2: Divide Clock by 256 = b0
                   //  Bits 0-1: Resolution = b01: 11.9-bit effective

  scanInterval = 4000;
  sendBuff[10] = (uint8)(scanInterval & (0x00FF)); // Scan interval (low byte)
  sendBuff[11] = (uint8)(scanInterval / 256);      // Scan interval (high byte)

  for (i = 0; i < num_channels_; i++) {
    sendBuff[12 + i * 2] = i;  // PChannel = i
    sendBuff[13 + i * 2] = 31; // NChannel = 31: Single Ended
  }

  extendedChecksum(sendBuff, sendBuffSize);

  // Sending command to U3
  sendChars = LJUSB_Write(hDevice, sendBuff, sendBuffSize);
  if (sendChars < sendBuffSize) {
    if (sendChars == 0)
      printf("Error : write failed (StreamConfig).\n");
    else
      printf("Error : did not write all of the buffer (StreamConfig).\n");
    return -1;
  }

  for (i = 0; i < 8; i++)
    recBuff[i] = 0;

  // Reading response from U3
  recChars = LJUSB_Read(hDevice, recBuff, 8);
  if (recChars < 8) {
    if (recChars == 0)
      printf("Error : read failed (StreamConfig).\n");
    else
      printf("Error : did not read all of the buffer, %d (StreamConfig).\n",
             recChars);

    for (i = 0; i < 8; i++)
      printf("%d ", recBuff[i]);

    return -1;
  }

  checksumTotal = extendedChecksum16(recBuff, 8);
  if ((uint8)((checksumTotal / 256) & 0xFF) != recBuff[5]) {
    printf("Error : read buffer has bad checksum16(MSB) (StreamConfig).\n");
    return -1;
  }

  if ((uint8)(checksumTotal & 0xFF) != recBuff[4]) {
    printf("Error : read buffer has bad checksum16(LBS) (StreamConfig).\n");
    return -1;
  }

  if (extendedChecksum8(recBuff) != recBuff[0]) {
    printf("Error : read buffer has bad checksum8 (StreamConfig).\n");
    return -1;
  }

  if (recBuff[1] != (uint8)(0xF8) || recBuff[2] != (uint8)(0x01) ||
      recBuff[3] != (uint8)(0x11) || recBuff[7] != (uint8)(0x00)) {
    printf("Error : read buffer has wrong command bytes (StreamConfig).\n");
    return -1;
  }

  if (recBuff[6] != 0) {
    printf("Errorcode # %d from StreamConfig read.\n",
           (unsigned int)recBuff[6]);
    return -1;
  }

  return 0;
}

int u3Streamer::StreamStart(HANDLE hDevice) {
  uint8 sendBuff[2], recBuff[4];
  int sendChars, recChars;

  sendBuff[0] = (uint8)(0xA8); // CheckSum8
  sendBuff[1] = (uint8)(0xA8); // command byte

  // Sending command to U3
  sendChars = LJUSB_Write(hDevice, sendBuff, 2);
  if (sendChars < 2) {
    if (sendChars == 0)
      printf("Error : write failed.\n");
    else
      printf("Error : did not write all of the buffer.\n");
    return -1;
  }

  // Reading response from U3
  recChars = LJUSB_Read(hDevice, recBuff, 4);
  if (recChars < 4) {
    if (recChars == 0)
      printf("Error : read failed.\n");
    else
      printf("Error : did not read all of the buffer.\n");
    return -1;
  }

  if (normalChecksum8(recBuff, 4) != recBuff[0]) {
    printf("Error : read buffer has bad checksum8 (StreamStart).\n");
    return -1;
  }

  if (recBuff[1] != (uint8)(0xA9) || recBuff[3] != (uint8)(0x00)) {
    printf("Error : read buffer has wrong command bytes \n");
    return -1;
  }

  if (recBuff[2] != 0) {
    printf("Errorcode # %d from StreamStart read.\n", (unsigned int)recBuff[2]);
    return -1;
  }

  return 0;
}

int u3Streamer::getStreamData(std::array<double, NumChannels> &voltages) {
//   std::cout << "u3Streamer::getStreamData" << std::endl;

  uint16 voltageBytes, checksumTotal;
  long startTime, endTime;
  double hardwareVersion;
  int recBuffSize, recChars, backLog, autoRecoveryOn;
  int packetCounter, currChannel, scanNumber;
  int totalPackets;       // The total number of StreamData responses read
  int numDisplay;         // Number of times to display streaming information
  int numReadsPerDisplay; // Number of packets to read before displaying
                          // streaming information
  int readSizeMultiplier; // Multiplier for the StreamData receive buffer size
  int responseSize;       // The number of bytes in a StreamData response
                          //(differs with samples_per_packet_)

  numReadsPerDisplay = 1;
  readSizeMultiplier = 1;
  responseSize = 14 + samples_per_packet_ * 2;

  /* Each StreamData response contains (samples_per_packet_ / num_channels_) *
   * readSizeMultiplier samples for each channel. Total number of scans =
   * (samples_per_packet_ / num_channels_) * readSizeMultiplier *
   * numReadsPerDisplay * numDisplay
   */
  uint8 recBuff[responseSize * readSizeMultiplier];

  // packetCounter = 0;
  currChannel = 0;
  totalPackets = 0;
  recChars = 0;
  autoRecoveryOn = 0;
  recBuffSize = 14 + samples_per_packet_ * 2;
  hardwareVersion = cali_info_.hardwareVersion;

  /* For USB StreamData, use Endpoint 3 for reads.  You can read the
   * multiple StreamData responses of 64 bytes only if
   * samples_per_packet_ is 25 to help improve streaming performance.  In
   * this example this multiple is adjusted by the readSizeMultiplier
   * variable.
   */
  for (int j = 0; j < numReadsPerDisplay; j++) {
    // Reading stream response from U3
    recChars =
        LJUSB_Stream(hDevice_, recBuff, responseSize * readSizeMultiplier);

    if (recChars < responseSize * readSizeMultiplier) {
      if (recChars == 0)
        printf("Error : read failed (getStreamData).\n");
      else
        printf("Error : did not read all of the buffer, expected %d bytes but "
               "received %d(StreamData).\n",
               responseSize * readSizeMultiplier, recChars);
      return -1;
    }

    // Checking for errors and getting data out of each StreamData
    // response
    totalPackets++;

    checksumTotal = extendedChecksum16(recBuff, recBuffSize);
    if ((uint8)((checksumTotal / 256) & 0xFF) != recBuff[5]) {
      printf("Error : read buffer has bad checksum16(MSB) (getStreamData).\n");
      return -1;
    }

    if ((uint8)(checksumTotal & 0xFF) != recBuff[4]) {
      printf("Error : read buffer has bad checksum16(LBS) (getStreamData).\n");
      return -1;
    }

    checksumTotal = extendedChecksum8(recBuff);
    if (checksumTotal != recBuff[0]) {
      printf("Error : read buffer has bad checksum8 (getStreamData).\n");
      return -1;
    }

    if (recBuff[1] != (uint8)(0xF9) || recBuff[2] != 4 + samples_per_packet_ ||
        recBuff[3] != (uint8)(0xC0)) {
      printf("Error : read buffer has wrong command bytes (getStreamData).\n");
      return -1;
    }

    if (recBuff[11] == 59) {
      if (!autoRecoveryOn) {
        printf("\nU3a data buffer overflow detected in packet %d.\nNow using "
               "auto-recovery and reading buffered samples.\n",
               totalPackets);
        autoRecoveryOn = 1;
      }
    } else if (recBuff[11] == 60) {
      printf("Auto-recovery report in packet %d: %d scans were "
             "dropped.\nAuto-recovery is now off.\n",
             totalPackets, recBuff[6] + recBuff[7] * 256);
      autoRecoveryOn = 0;
    } else if (recBuff[11] != 0) {
      printf("Errorcode # %d from StreamData read.\n",
             (unsigned int)recBuff[11]);
      return -1;
    }

    if (packet_counter_ != (int)recBuff[10]) {
      printf("PacketCounter (%d) does not match with with current packet count "
             "(%d)(getStreamData).\n",
             recBuff[10], packet_counter_);
      return -1;
    }

    backLog = (int)recBuff[12 + samples_per_packet_ * 2];

    for (int k = 12; k < (12 + samples_per_packet_ * 2); k += 2) {
      voltageBytes = (uint16)recBuff[k] + (uint16)recBuff[k + 1] * 256;

      if (hardwareVersion >= 1.30)
        getAinVoltCalibrated_hw130(&cali_info_, currChannel, 31, voltageBytes,
                                   &(voltages[currChannel]));
      else
        getAinVoltCalibrated(&cali_info_, dac1_enabled_, 31, voltageBytes,
                             &(voltages[currChannel]));

      currChannel++;
      if (currChannel >= num_channels_) {
        currChannel = 0;
      }
    }

    if (packet_counter_ >= 255)
      packet_counter_ = 0;
    else
      packet_counter_++;
  }
  return 0;
}

void u3Streamer::stream() { StreamData(hDevice_, &cali_info_, dac1_enabled_); }

// Reads the StreamData low-level function response in a loop.  All voltages
// from the stream are stored in the voltages 2D array.
int u3Streamer::StreamData(HANDLE hDevice, u3CalibrationInfo *caliInfo,
                           int isDAC1Enabled) {
  uint16 voltageBytes, checksumTotal;
  long startTime, endTime;
  double hardwareVersion;
  int recBuffSize, recChars, backLog, autoRecoveryOn;
  int packetCounter, currChannel, scanNumber;
  int i, j, k, m;
  int totalPackets;       // The total number of StreamData responses read
  int numDisplay;         // Number of times to display streaming information
  int numReadsPerDisplay; // Number of packets to read before displaying
                          // streaming information
  int readSizeMultiplier; // Multiplier for the StreamData receive buffer size
  int responseSize;       // The number of bytes in a StreamData response
                    //(differs with SamplesPerPacket)

  numDisplay = 1;
  numReadsPerDisplay = 10;
  readSizeMultiplier = 1;
  responseSize = 14 + samples_per_packet_ * 2;

  /* Each StreamData response contains (SamplesPerPacket / NumChannels) *
   * readSizeMultiplier samples for each channel. Total number of scans =
   * (SamplesPerPacket / NumChannels) * readSizeMultiplier * numReadsPerDisplay
   * * numDisplay
   */
  // std::cout << "voltage first dim : " <<
  // (samples_per_packet_/NumChannels)*readSizeMultiplier*numReadsPerDisplay*numDisplay
  // <<std::endl;
  double voltages[(samples_per_packet_ / NumChannels) * readSizeMultiplier *
                  numReadsPerDisplay * numDisplay][NumChannels];
  uint8 recBuff[responseSize * readSizeMultiplier];

  // packet_counter_ = 0;
  currChannel = 0;
  scanNumber = 0;
  totalPackets = 0;
  recChars = 0;
  autoRecoveryOn = 0;
  recBuffSize = 14 + samples_per_packet_ * 2;
  hardwareVersion = caliInfo->hardwareVersion;

  printf("Reading Samples...\n");

  startTime = getTickCount();

  for (i = 0; i < numDisplay; i++) {
    for (j = 0; j < numReadsPerDisplay; j++) {
      /* For USB StreamData, use Endpoint 3 for reads.  You can read the
       * multiple StreamData responses of 64 bytes only if
       * SamplesPerPacket is 25 to help improve streaming performance.  In
       * this example this multiple is adjusted by the readSizeMultiplier
       * variable.
       */

      // Reading stream response from U3
      recChars =
          LJUSB_Stream(hDevice, recBuff, responseSize * readSizeMultiplier);

      // std::cout << std::hex << (int)recBuff[11]<<std::endl;

      if (recChars < responseSize * readSizeMultiplier) {
        if (recChars == 0)
          printf("Error : read failed (StreamData).\n");
        else
          printf("Error : did not read all of the buffer, expected %d bytes "
                 "but received %d(StreamData).\n",
                 responseSize * readSizeMultiplier, recChars);
        return -1;
      }

      // Checking for errors and getting data out of each StreamData
      // response
      for (m = 0; m < readSizeMultiplier; m++) {
        totalPackets++;

        checksumTotal =
            extendedChecksum16(recBuff + m * recBuffSize, recBuffSize);
        if ((uint8)((checksumTotal / 256) & 0xFF) !=
            recBuff[m * recBuffSize + 5]) {
          printf("Error : read buffer has bad checksum16(MSB) (StreamData).\n");
          return -1;
        }

        if ((uint8)(checksumTotal & 0xFF) != recBuff[m * recBuffSize + 4]) {
          printf("Error : read buffer has bad checksum16(LBS) (StreamData).\n");
          return -1;
        }

        checksumTotal = extendedChecksum8(recBuff + m * recBuffSize);
        if (checksumTotal != recBuff[m * recBuffSize]) {
          printf("Error : read buffer has bad checksum8 (StreamData).\n");
          return -1;
        }

        if (recBuff[m * recBuffSize + 1] != (uint8)(0xF9) ||
            recBuff[m * recBuffSize + 2] != 4 + samples_per_packet_ ||
            recBuff[m * recBuffSize + 3] != (uint8)(0xC0)) {
          printf("Error : read buffer has wrong command bytes (StreamData).\n");
          return -1;
        }

        if (recBuff[m * recBuffSize + 11] == 59) {
          if (!autoRecoveryOn) {
            printf("\nU3b data buffer overflow detected in packet %d.\nNow "
                   "using auto-recovery and reading buffered samples.\n",
                   totalPackets);
            autoRecoveryOn = 1;
          }
        } else if (recBuff[m * recBuffSize + 11] == 60) {
          printf("Auto-recovery report in packet %d: %d scans were "
                 "dropped.\nAuto-recovery is now off.\n",
                 totalPackets,
                 recBuff[m * recBuffSize + 6] +
                     recBuff[m * recBuffSize + 7] * 256);
          autoRecoveryOn = 0;
        } else if (recBuff[m * recBuffSize + 11] != 0) {
          printf("Errorcode # %d from StreamData read.\n",
                 (unsigned int)recBuff[11]);
          return -1;
        }

        if (packet_counter_ != (int)recBuff[m * recBuffSize + 10]) {
          printf("PacketCounter (%d) does not match with with current packet "
                 "count (%d)(StreamData).\n",
                 recBuff[m * recBuffSize + 10], packet_counter_);
          return -1;
        }

        backLog = (int)recBuff[m * 48 + 12 + samples_per_packet_ * 2];

        for (k = 12; k < (12 + samples_per_packet_ * 2); k += 2) {
          voltageBytes = (uint16)recBuff[m * recBuffSize + k] +
                         (uint16)recBuff[m * recBuffSize + k + 1] * 256;

          if (hardwareVersion >= 1.30)
            getAinVoltCalibrated_hw130(caliInfo, currChannel, 31, voltageBytes,
                                       &(voltages[scanNumber][currChannel]));
          else
            getAinVoltCalibrated(caliInfo, isDAC1Enabled, 31, voltageBytes,
                                 &(voltages[scanNumber][currChannel]));

          currChannel++;
          if (currChannel >= NumChannels) {
            currChannel = 0;
            scanNumber++;
          }
        }

        if (packet_counter_ >= 255)
          packet_counter_ = 0;
        else
          packet_counter_++;
      }
    }

    printf("\nNumber of scans: %d\n", scanNumber);
    printf("Total packets read: %d\n", totalPackets);
    printf("Current PacketCounter: %d\n",
           ((packet_counter_ == 0) ? 255 : packet_counter_ - 1));
    printf("Current BackLog: %d\n", backLog);
    // std::cout<<"scan number is : " <<scanNumber<<std::endl;
    for (k = 0; k < NumChannels; k++)
      printf("  AI%d: %.4f V\n", k, voltages[scanNumber - 1][k]);
  }

  endTime = getTickCount();
  printf("\nRate of samples: %.0lf samples per second\n",
         ((scanNumber * NumChannels) / ((endTime - startTime) / 1000.0)));
  printf("Rate of scans: %.0lf scans per second\n\n",
         (scanNumber / ((endTime - startTime) / 1000.0)));

  return 0;
}

int u3Streamer::StreamStop(HANDLE hDevice) {
  uint8 sendBuff[2], recBuff[4];
  int sendChars, recChars;

  sendBuff[0] = (uint8)(0xB0); // CheckSum8
  sendBuff[1] = (uint8)(0xB0); // Command byte

  // Sending command to U3
  sendChars = LJUSB_Write(hDevice, sendBuff, 2);
  if (sendChars < 2) {
    if (sendChars == 0)
      printf("Error : write failed (StreamStop).\n");
    else
      printf("Error : did not write all of the buffer (StreamStop).\n");
    return -1;
  }

  // Reading response from U3
  recChars = LJUSB_Read(hDevice, recBuff, 4);
  if (recChars < 4) {
    if (recChars == 0)
      printf("Error : read failed (StreamStop).\n");
    else
      printf("Error : did not read all of the buffer (StreamStop).\n");
    return -1;
  }

  if (normalChecksum8(recBuff, 4) != recBuff[0]) {
    printf("Error : read buffer has bad checksum8 (StreamStop).\n");
    return -1;
  }

  if (recBuff[1] != (uint8)(0xB1) || recBuff[3] != (uint8)(0x00)) {
    printf("Error : read buffer has wrong command bytes (StreamStop).\n");
    return -1;
  }

  if (recBuff[2] != 0) {
    printf("Errorcode # %d from StreamStop read.\n", (unsigned int)recBuff[2]);
    return -1;
  }

  return 0;
}
