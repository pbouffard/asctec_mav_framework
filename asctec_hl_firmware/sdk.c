/*

Copyright (c) 2011, Markus Achtelik, ASL, ETH Zurich, Switzerland
You can contact the author at <markus dot achtelik at mavt dot ethz dot ch>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
* Neither the name of ETHZ-ASL nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "main.h"
#include "system.h"
#include "sdk.h"
#include "LL_HL_comm.h"
#include "uart.h"
#include "time.h"
#include "irq.h"
#include "LPC214x.h"
#include "hardware.h"

struct WO_SDK_STRUCT WO_SDK;
struct WO_CTRL_INPUT WO_CTRL_Input;
struct RO_RC_DATA RO_RC_Data;
struct WO_DIRECT_MOTOR_CONTROL WO_Direct_Motor_Control;

#include "gpsmath.h"

#include <ert_main.h>

// time synchronization
volatile int64_t timestamp = 0;
int64_t timeOffset = 0;
unsigned short time_step = 2000;
int64_t time_correction = 0;


short cmdLLValid = 0;
unsigned char cmdLLNew = 0;

short extPositionValid = 0;

unsigned int cpuLoad = 0;
unsigned int sdkCycleStartTime = 0;

HLI_IMU imuData;
HLI_RCDATA rcData;
HLI_GPS gpsData;
HLI_SSDK_STATUS ssdk_status;

// declared external in sdk.h, so that the ssdk can access it
HLI_EXT_POSITION extPosition;
HLI_STATUS statusData;

short motor_state = -1;
short motor_state_count = 0;
unsigned int sdkLoops = 0;

// set up data structures for receiving data ##################################

// create structures containing data
HLI_CMD_LL cmdLL;
PacketInfo *packetCmdLL;

// do that for every other package ...
HLI_MOTORS motors;
PacketInfo *packetMotors;

HLI_EXT_POSITION ext_position_update;
PacketInfo *packetExtPosition;

// extPositionCmd is declared external in sdk.h, so that the ssdk can access it
HLI_CMD_HL extPositionCmd;
PacketInfo *packetCmdHL;

HLI_TIMESYNC timeSync;
PacketInfo *packetTimeSync;

PacketInfo *packetSSDKParams;

// control enable packet
HLI_CMD_LL controlEnable;
PacketInfo *packetControlEnable;

HLI_EKF_STATE ekfState;
PacketInfo *packetEkfState;

HLI_BAUDRATE baudrate;
PacketInfo *packetBaudrate;

// subscription packet
HLI_SUBSCRIPTION subscription;
PacketInfo *packetSubscription;

// config packet
HLI_CONFIG config;
PacketInfo *packetConfig;

// ######################################################################################

void sdkInit(void)
{
  //	UART_setPacketInfo(packetInfo);
  statusData.have_SSDK_parameters = 0;
  ssdk_status.have_parameters = 0;
  baudrate.state = 0;

  extPositionValid = 0;

  config.mode_position_control = HLI_MODE_POSCTRL_OFF;
  config.mode_state_estimation = HLI_MODE_STATE_ESTIMATION_OFF;
  config.position_control_axis_enable = 0;

  // set default packet rates
  subscription.imu = HLI_DEFAULT_PERIOD_IMU;
  subscription.rcdata = HLI_DEFAULT_PERIOD_RCDATA;
  subscription.gps = HLI_DEFAULT_PERIOD_GPS;
  subscription.ssdk_debug = HLI_DEFAULT_PERIOD_SSDK_DEBUG;
  subscription.ekf_state = HLI_DEFAULT_PERIOD_EKF_STATE;

  // register packets to receive
  packetExtPosition = registerPacket(HLI_PACKET_ID_POSITION_UPDATE, &ext_position_update);
  packetCmdHL = registerPacket(HLI_PACKET_ID_CONTROL_HL, &extPositionCmd);
  packetMotors = registerPacket(HLI_PACKET_ID_MOTORS, &motors);
  packetCmdLL = registerPacket(HLI_PACKET_ID_CONTROL_LL, &cmdLL);
  packetTimeSync = registerPacket(HLI_PACKET_ID_TIMESYNC, &timeSync);
  packetSSDKParams = registerPacket(HLI_PACKET_ID_SSDK_PARAMETERS, &ssdk_params); // defined in ert_main.h
  packetControlEnable = registerPacket(HLI_PACKET_ID_CONTROL_ENABLE, &controlEnable);
  packetBaudrate = registerPacket(HLI_PACKET_ID_BAUDRATE, &baudrate);
  packetSubscription = registerPacket(HLI_PACKET_ID_SUBSCRIPTION, &subscription);
  packetEkfState = registerPacket(HLI_PACKET_ID_EKF_STATE, &ekfState);
  packetConfig = registerPacket(HLI_PACKET_ID_CONFIG, &config);

  UART0_rxFlush();
  UART0_txFlush();

  startAutoBaud();
}

/** SDK_mainloop(void) is triggered @ 1kHz.
 *
 * WO_(Write Only) data is written to the LL processor after
 * execution of this function.
 *
 * RO_(Read Only) data is updated before entering this function
 * and can be read to obtain information for supervision or control
 *
 * WO_ and RO_ structs are defined in sdk.h
 *
 * The struct LL_1khz_attitude_data (defined in LL_HL_comm.h) can
 * be used to read all sensor data, results of the data fusion
 * and R/C inputs transmitted from the LL-processor. This struct is
 * automatically updated at 1 kHz.
 * */

void SDK_mainloop(void)
{
  sdkCycleStartTime = T1TC;

  WO_SDK.ctrl_mode = 0x00; //0x00: absolute angle and throttle control

  sdkLoops++;

  parseRxFifo();

  // check for new LL command packet
  if (packetCmdLL->updated)
  {
    cmdLLNew = 1;
    packetCmdLL->updated = 0;
  }

  // check if LL commands arrive at max every CMD_MAX_PERIOD ms
  if ((sdkLoops % CMD_MAX_PERIOD) == 0)
  {

    if (cmdLLNew == 1)
    {
      cmdLLNew = 0;
      cmdLLValid++;
    }
    else
    {
      cmdLLValid--;
    }

    if (cmdLLValid < -2)
      cmdLLValid = -2; // min 3 packets required
    else if (cmdLLValid > 3)
      cmdLLValid = 3; // fall back after 3 missed packets
  }

  // check for motor start/stop packet
  if (packetMotors->updated)
  {
    motor_state = motors.motors;
    motor_state_count = 0;
    packetMotors->updated = 0;
  }

  // check for new HL command packet
  if (packetCmdHL->updated)
  {
    packetCmdHL->updated = 0;
    // SSDK operates in NED, need to convert from ENU
    extPositionCmd.heading = -extPositionCmd.heading + 360000;
    extPositionCmd.y = -extPositionCmd.y;
    extPositionCmd.z = -extPositionCmd.z;
    extPositionCmd.vY = -extPositionCmd.vY;
    extPositionCmd.vZ = -extPositionCmd.vZ;
    extPositionCmd.vYaw = -extPositionCmd.vYaw;
  }

  // handle parameter packet
  if (packetSSDKParams->updated == 1)
  {
    packetSSDKParams->updated = 0;
    statusData.have_SSDK_parameters = 1;
    ssdk_status.have_parameters = 1;
  }

  // decide which position/state input we take for position control
  // SSDK operates in NED --> convert from ENU
  switch(config.mode_state_estimation){
    case HLI_MODE_STATE_ESTIMATION_HL_SSDK:
      extPositionValid = 1;
      extPosition.bitfield = 0;
      extPosition.count = ext_position_update.count;
      extPosition.heading = -ext_position_update.heading + 360000;
      extPosition.x = ext_position_update.x;
      extPosition.y = -ext_position_update.y;
      extPosition.z = -ext_position_update.z;
      extPosition.vX = ext_position_update.vX;
      extPosition.vY = -ext_position_update.vY;
      extPosition.vZ = -ext_position_update.vZ;
      extPosition.qualX = ext_position_update.qualX;
      extPosition.qualY = ext_position_update.qualY;
      extPosition.qualZ = ext_position_update.qualZ;
      extPosition.qualVx = ext_position_update.qualVx;
      extPosition.qualVy = ext_position_update.qualVy;
      extPosition.qualVz = ext_position_update.qualVz;
      break;
    case HLI_MODE_STATE_ESTIMATION_EXT:
      extPositionValid = 1;
      extPosition.bitfield = EXT_POSITION_BYPASS_FILTER;
      extPosition.count = ext_position_update.count;
      extPosition.heading = -ext_position_update.heading + 360000;
      extPosition.x = ext_position_update.x;
      extPosition.y = -ext_position_update.y;
      extPosition.z = -ext_position_update.z;
      extPosition.vX = ext_position_update.vX;
      extPosition.vY = -ext_position_update.vY;
      extPosition.vZ = -ext_position_update.vZ;
      extPosition.qualX = ext_position_update.qualX;
      extPosition.qualY = ext_position_update.qualY;
      extPosition.qualZ = ext_position_update.qualZ;
      extPosition.qualVx = ext_position_update.qualVx;
      extPosition.qualVy = ext_position_update.qualVy;
      extPosition.qualVz = ext_position_update.qualVz;
      break;
    default:
      extPositionValid = 0;
  }

  // execute ssdk - only executed if ssdk parameters are available
  // reads position reference from extPosition
  // reads position/velocity command from extPositionCmd
  // finally writes to WO_CTRL_Input. therefore, make sure to overwrite it after this call if you don't want to have its output
  rt_OneStep();

  // --- write commands to LL ------------------------------------------------

  short motorsRunning = LL_1khz_attitude_data.status2 & 0x1;

  if (motor_state == -1 || motor_state == 2)
  { // motors are either stopped or running --> normal operation

    // commands are always written to LL by the Matlab controller, decide if we need to overwrite them
    if (extPositionValid > 0 && statusData.have_SSDK_parameters == 1 && config.mode_position_control == HLI_MODE_POSCTRL_HL)
    {
      WO_CTRL_Input.ctrl = config.position_control_axis_enable;
      WO_SDK.ctrl_enabled = 1;
    }

    else if (cmdLLValid > 0 && (config.mode_position_control == HLI_MODE_POSCTRL_LL || config.mode_position_control == HLI_MODE_POSCTRL_OFF))
    {
      writeCommand(cmdLL.x, cmdLL.y, cmdLL.yaw, cmdLL.z, config.position_control_axis_enable, 1);
    }
    else
    {
      writeCommand(0, 0, 0, 0, 0, 0);
    }
  }
  // start / stop motors, allow commands max for 1.5 s
  else if (motor_state == 1)
  {
    if (motor_state_count < 1500)
    {
      if (!motorsRunning)
        writeCommand(0, 0, 2047, 0, HLI_YAW_BIT | HLI_THRUST_BIT, 1); // start/stop sequence, set ctrl to acc so that motors will be safely started
      else if (motorsRunning)
      {
        writeCommand(0, 0, 0, 0, HLI_YAW_BIT | HLI_THRUST_BIT, 1);
        motor_state = 2;
      }
    }
    else
    {
      writeCommand(0, 0, 0, 0, HLI_YAW_BIT | HLI_THRUST_BIT, 1);
      motor_state = -1;
    }
    motor_state_count ++;
  }
  else if (motor_state == 0)
  {
    if (motor_state_count < 1500)
    {
      if (motorsRunning)
        writeCommand(0, 0, 2047, 0, HLI_YAW_BIT | HLI_THRUST_BIT, 1); // start/stop sequence, set ctrl to acc so that motors will be safely shut down
      else if (!motorsRunning)
      {
        writeCommand(0, 0, 0, 0, HLI_YAW_BIT | HLI_THRUST_BIT, 1);
        motor_state = -1;
      }
    }
    else
    {
      writeCommand(0, 0, 0, 0, HLI_YAW_BIT | HLI_THRUST_BIT, 1);
      motor_state = -1;
    }
    motor_state_count++;
  }
  else
  {
    // undefined state, disable everything
    writeCommand(0, 0, 0, 0, 0, 0);
  }

  // TODO: thrust limit in case something really goes wrong, may be removed
  if (WO_CTRL_Input.thrust > 4095)
    WO_CTRL_Input.thrust = 4095;

  // ------------------------------------------------------------------------


  // --- send data to UART 0 ------------------------------------------------
  if (checkTxPeriod(subscription.imu))
  {
    sendImuData();
  }
  if (checkTxPeriod(subscription.rcdata))
  {
    sendRcData();
  }
  if (checkTxPeriod(subscription.gps))
  {
    sendGpsData();
  }
  if ((sdkLoops + 20) % 500 == 0)
  {
    sendStatus();
    writePacket2Ringbuffer(HLI_PACKET_ID_SSDK_STATUS, (unsigned char*)&ssdk_status, sizeof(ssdk_status));
  }
  if (checkTxPeriod(subscription.ssdk_debug))
  {
    ssdk_debug.timestamp = timestamp;
    writePacket2Ringbuffer(HLI_PACKET_ID_SSDK_DEBUG, (unsigned char*)&ssdk_debug, sizeof(ssdk_debug));
  }
//
  UART_send_ringbuffer();

  synchronizeTime();

  if (packetBaudrate->updated)
  {
    packetBaudrate->updated = 0;
    while (!UART0_txEmpty())
      ;
  }

  // ------------------------------------------------------------------------

  unsigned int dt;
  if (T1TC < sdkCycleStartTime)
    dt = (processorClockFrequency() - sdkCycleStartTime) + T1TC;
  else
    dt = T1TC - sdkCycleStartTime;

  cpuLoad = ControllerCyclesPerSecond * ((dt * 1e2) / processorClockFrequency()); // cpu load in %

  watchdog();
}

inline void writeCommand(short pitch, short roll, short yaw, short thrust, short ctrl, short enable)
{
  WO_CTRL_Input.pitch = pitch;
  WO_CTRL_Input.roll = roll;
  WO_CTRL_Input.thrust = thrust;
  WO_CTRL_Input.yaw = yaw;
  WO_CTRL_Input.ctrl = ctrl;

  WO_SDK.ctrl_enabled = enable;
}

inline void sendImuData(void)
{
  imuData.timestamp = timestamp;

  // TODO: smoothing of data to make Nyquist happy ;-)
  // acceleration, angular velocities, attitude, height, dheight following the ENU convention (x front, y left, z up)
  imuData.acc_x = -LL_1khz_attitude_data.acc_x;
  imuData.acc_y = -LL_1khz_attitude_data.acc_y;
  imuData.acc_z = -LL_1khz_attitude_data.acc_z;
  imuData.ang_vel_roll = -LL_1khz_attitude_data.angvel_roll;
  imuData.ang_vel_pitch = LL_1khz_attitude_data.angvel_pitch;
  imuData.ang_vel_yaw = -LL_1khz_attitude_data.angvel_yaw;
  imuData.ang_roll = -LL_1khz_attitude_data.angle_roll;
  imuData.ang_pitch = LL_1khz_attitude_data.angle_pitch;
  imuData.ang_yaw = 36000 - LL_1khz_attitude_data.angle_yaw;
  imuData.differential_height = LL_1khz_attitude_data.dheight;
  imuData.height = LL_1khz_attitude_data.height;

  writePacket2Ringbuffer(HLI_PACKET_ID_IMU, (unsigned char*)&imuData, sizeof(imuData));
}

inline void sendGpsData(void)
{

  gpsData.timestamp = timestamp;
  //	gpsData.latitude = LL_1khz_attitude_data.latitude_best_estimate;
  //	gpsData.longitude = LL_1khz_attitude_data.longitude_best_estimate;
  gpsData.latitude = GPS_Data.latitude;
  gpsData.longitude = GPS_Data.longitude;
  gpsData.heading = LL_1khz_attitude_data.angle_yaw;
  gpsData.height = GPS_Data.height;
  gpsData.speedX = GPS_Data.speed_x;
  gpsData.speedY = GPS_Data.speed_y;
  gpsData.horizontalAccuracy = GPS_Data.horizontal_accuracy;
  gpsData.verticalAccuracy = GPS_Data.horizontal_accuracy;
  gpsData.speedAccuracy = GPS_Data.speed_accuracy;
  gpsData.numSatellites = GPS_Data.numSV;
  gpsData.status = GPS_Data.status;

  writePacket2Ringbuffer(HLI_PACKET_ID_GPS, (unsigned char*)&gpsData, sizeof(gpsData));
}

inline void sendStatus(void)
{

  statusData.timestamp = timestamp;
  statusData.battery_voltage = LL_1khz_attitude_data.battery_voltage1;
  statusData.cpu_load = cpuLoad;
  statusData.flight_mode = LL_1khz_attitude_data.flightMode;
  statusData.flight_time = LL_1khz_attitude_data.flight_time;

  if (motor_state == 0 || motor_state == 1)
    statusData.motors = motor_state;
  else if (LL_1khz_attitude_data.status2 & 0x1)
    statusData.motors = 2;
  else if (!(LL_1khz_attitude_data.status2 & 0x1))
    statusData.motors = -1;

  statusData.debug1 = uart0_min_rx_buffer;
  statusData.debug2 = uart0_min_tx_buffer;

  statusData.state_estimation = config.mode_state_estimation;
  statusData.position_control = config.mode_position_control;

  statusData.rx_packets = UART_rxPacketCount;
  statusData.rx_packets_good = UART_rxGoodPacketCount;

  writePacket2Ringbuffer(HLI_PACKET_ID_STATUS, (unsigned char*)&statusData, sizeof(statusData));
}

inline void sendRcData(void)
{
  int i = 0;

  rcData.timestamp = timestamp;
  for (i = 0; i < sizeof(rcData.channel) / sizeof(unsigned short); i++)
    rcData.channel[i] = RO_RC_Data.channel[i];

  writePacket2Ringbuffer(HLI_PACKET_ID_RC, (unsigned char*)&rcData, sizeof(rcData));
}

inline void synchronizeTime()
{

  // check for timesync packet
  if (packetTimeSync->updated)
  {
    timeOffset = (900*timeOffset + 100 * (timeSync.ts1 * 2 - timeSync.tc1 - timestamp) / 2) / 1000;
    statusData.timesync_offset = timeOffset;

    if (timeOffset > 1e7 || timeOffset < -1e7)
    {
      timestamp = timeSync.ts1;
      timeOffset = 0;
    }
    else if (timeOffset > 2000)
      timeOffset = 2000;
    else if (timeOffset < -2000)
      timeOffset = -2000;

    if (timeOffset > 0)
    {
      time_step = 1000 / timeOffset;
      time_correction = 1;
    }
    else if (timeOffset < 0)
    {
      time_step = -1000 / timeOffset;
      time_correction = -1;
    }
    else
    {
      time_step = 1000;
      time_correction = 0;
    }

    packetTimeSync->updated = 0;
  }

  // correct timestamp every step sdkloops by one us
  if (sdkLoops % time_step == 0)
  {
    timestamp += time_correction;
  }

  if (sdkLoops % 2000 == 0)
  {
    timeSync.tc1 = timestamp;
    timeSync.ts1 = 0;
    writePacket2Ringbuffer(HLI_PACKET_ID_TIMESYNC, (unsigned char*)&timeSync, sizeof(timeSync));
    UART_send_ringbuffer();
  }
}

inline void watchdog(void)
{

  static uint32_t lastTxPackets = 0;

  // check if a valid packet arrived in the HLI_COMMUNICATION_TIMEOUT s
  if ((sdkLoops % (ControllerCyclesPerSecond  * HLI_COMMUNICATION_TIMEOUT )) == 0)
  {
    if (UART_rxGoodPacketCount == lastTxPackets)
    {
      startAutoBaud();
    }
    lastTxPackets = UART_rxGoodPacketCount;
  }
}

inline int checkTxPeriod(uint16_t period)
{
  if (period == 0)
    return 0;
  else
    return sdkLoops % period == 0;
}
