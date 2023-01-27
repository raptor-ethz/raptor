#ifndef U3STREAMER_H
#define U3STREAMER_H

#include "u3.h"
#include <array>

static const uint8 NumChannels = 4;

class u3Streamer
{
private:
    const uint8 num_channels_ = NumChannels; // For this example to work proper,
                                             // SamplesPerPacket needs to be a multiple of
                                             // NumChannels.
    const uint8 samples_per_packet_ = 4; // Needs to be 25 to read multiple StreamData
                                         // responses in one large packet, otherwise
                                         // can be any value between 1-25 for 1
                                         // StreamData response per packet.

    HANDLE hDevice_;
    u3CalibrationInfo cali_info_;
    int dac1_enabled_;
    int packet_counter_ = 0;
    bool init_flag_ = false;

    // double stream_voltage_[num_channels_];
    // std::array<double, NumChannels> stream_voltages_;

public:
    u3Streamer();
    ~u3Streamer();

    // Sends a ConfigIO low-level command that configures the FIOs, DAC, Timers and
    // Counters for this example
    int ConfigIO(HANDLE hDevice, int *isDAC1Enabled);

    // Sends a StreamConfig low-level command to configure the stream.
    int StreamConfig(HANDLE hDevice);

    // Sends a StreamStart low-level command to start streaming.
    int StreamStart(HANDLE hDevice);

    // Reads the StreamData low-level function response in a loop.  All voltages from
    // the stream are stored in the voltages 2D array.
    int StreamData(HANDLE hDevice, u3CalibrationInfo *caliInfo, int isDAC1Enabled);

    // Sends a StreamStop low-level command to stop streaming.
    int StreamStop(HANDLE hDevice);

    int getStreamData(std::array<double, NumChannels> &voltages);

    bool isInit();

    void stream();
};

#endif
