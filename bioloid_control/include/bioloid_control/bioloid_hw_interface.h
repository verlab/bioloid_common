/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Example ros_control hardware interface blank template for the Bioloid
           For a more detailed simulation example, see sim_hw_interface.h
*/

#ifndef BIOLOID_CONTROL__BIOLOID_HW_INTERFACE_H
#define BIOLOID_CONTROL__BIOLOID_HW_INTERFACE_H

#include <ros_control_boilerplate/generic_hw_interface.h>
#include <navio/navio_led.h>
#include <std_msgs/Int32.h>

// Dynamixel
#include <ax12ControlTableMacros.h>
#include <dynamixel_syncread.h>

#define NUMBER_OF_JOINTS 18
#define FLOAT_PRECISION_THRESH 0.00001
#define NUMBER_OF_FIELD_PER_MOTOR 1

#define SENSOR_BOARD_ID_START 32
#define SENSOR_BOARD_MAX 2

// Return code
#define ROBOTHW_OK 0
#define ROBOTHW_DXL_INIT_ERROR -1
#define ROBOTHW_INV_DEVICE -2
#define ROBOTHW_INV_BAUDRATE -3
#define ROBOTHW_INV_ACTUATOR_NB -4
#define ROBOTHW_ERROR_RESET -5
#define ROBOTHW_INV_REG -6
#define ROBOTHW_ERROR_SYNC_XFER -7
#define ROBOTHW_ERROR_XFER -8
#define ROBOTHW_INV_ID -9
#define ROBOTHW_INV_SENSOR_NB -10

#define DEBUG
#ifdef DEBUG
#define HW_DBG ROS_INFO
#else
#define HW_DBG(...)                                                                                                    \
  do                                                                                                                   \
  {                                                                                                                    \
  } while (0)
#endif

namespace bioloid_control
{
#define NUMBER_OF_JOINTS 18

// Return code
#define ROBOTHW_OK 0
#define ROBOTHW_DXL_INIT_ERROR -1
#define ROBOTHW_INV_DEVICE -2
#define ROBOTHW_INV_BAUDRATE -3
#define ROBOTHW_INV_ACTUATOR_NB -4
#define ROBOTHW_ERROR_RESET -5
#define ROBOTHW_INV_REG -6
#define ROBOTHW_ERROR_SYNC_XFER -7
#define ROBOTHW_ERROR_XFER -8
#define ROBOTHW_INV_ID -9

typedef struct _XferAX
{
  short dxlID;
  short address;
  short value;
  bool Success;
} XferAX;

typedef struct _XferSyncAX
{
  short dxlIDs[NUMBER_OF_JOINTS];
  short startAddress;
  short numOfValuesPerMotor;
  short values[NUMBER_OF_FIELD_PER_MOTOR][NUMBER_OF_JOINTS];
  bool Success;
} XferSyncAX;

/// \brief Hardware interface for a robot
class BioloidHWInterface : public ros_control_boilerplate::GenericHWInterface
{
public:
  /**
   * \brief Constructor
   * \param nh - Node handle for topics.
   */
  BioloidHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model = NULL);

  /** \brief Initialize the robot hardware interface */
  virtual void init();

  /** \brief Read the state from the robot hardware. */
  virtual void read(ros::Duration &elapsed_time);

  /** \brief Write the command to the robot hardware. */
  virtual void write(ros::Duration &elapsed_time);

  /** \breif Enforce limits for all values before writing */
  virtual void enforceLimits(ros::Duration &period);

  int Initialize(void);
  int initial_position_set(void);
  int sendSyncToAX(XferSyncAX *xfer);
  int receiveSyncFromAX(XferSyncAX *xfer);
  int receiveFromAX(XferAX *xfer);
  int sendToAX(XferAX *xfer);
  int radPerSecToAxSpeed(float oldValue);
  int radToAxPosition(float oldValue);

  /* AX registers */
  static const std::map<int, bool> addressWordMap;
  /*
   *  joint offsets from ROS zero to Robotis zero
   * Robotis zero: "ready stand pose"
   * ROS zero: "fully extended arms/legs"
   */
  double joint_reset_rad[NUMBER_OF_JOINTS];

  double joint_dir_sign[NUMBER_OF_JOINTS];

  static const int directionSign[NUMBER_OF_JOINTS];

  float axSpeedToRadPerSec(int oldValue);
  float axPositionToRad(int oldValue);
  float axTorqueToDecimal(int oldValue);

protected:

  /* Dynamixel packet */
  XferSyncAX xfer;

  // For position controller to estimate velocity
  std::vector<double> joint_position_prev_;

  std_msgs::Int32 led_msg;
  ros::Publisher navio_led_pub_;

  // Name of this class
  std::string name_;
};  // class

}  // namespace

#endif
