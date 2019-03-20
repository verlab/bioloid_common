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
           For a more detailed simulation example, see sim_hw_interface.cpp
*/

#include <bioloid_control/bioloid_hw_interface.h>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace bioloid_control
{
// C++11 init. style
const std::map<int, bool> BioloidHWInterface::addressWordMap = {
	/* register offset ; field length (false = 1 bytes, true = 2 bytes) */
	{ AX12_MODEL_NUMBER_L, true },
	{ AX12_FIRMWARE_VERSION, false },
	{ AX12_ID, false },
	{ AX12_BAUD_RATE, false },
	{ AX12_RETURN_DELAY_TIME, false },
	{ AX12_CW_ANGLE_LIMIT_L, true },
	{ AX12_CCW_ANGLE_LIMIT_L, true },
	{ AX12_HIGH_LIMIT_TEMPERATURE, false },
	{ AX12_LOW_LIMIT_VOLTAGE, false },
	{ AX12_HIGH_LIMIT_VOLTAGE, false },
	{ AX12_MAX_TORQUE_L, true },
	{ AX12_STATUS_RETURN_LEVEL, false },
	{ AX12_ALARM_LED, false },
	{ AX12_ALARM_SHUTDOWN, false },
	{ AX12_TORQUE_ENABLE, false },
	{ AX12_LED, false },
	{ AX12_CW_COMPLIANCE_MARGIN, false },
	{ AX12_CCW_COMPLIANCE_MARGIN, false },
	{ AX12_CW_COMPLIANCE_SLOPE, false },
	{ AX12_CCW_COMPLIANCE_SLOPE, false },
	{ AX12_GOAL_POSITION_L, true },
	{ AX12_MOVING_SPEED_L, true },
	{ AX12_TORQUE_LIMIT_L, true },
	{ AX12_PRESENT_POSITION_L, true },
	{ AX12_PRESENT_SPEED_L, true },
	{ AX12_PRESENT_LOAD_L, true },
	{ AX12_PRESENT_VOLTAGE, false },
	{ AX12_PRESENT_TEMPERATURE, false },
	{ AX12_REGISTERED, false },
	{ AX12_MOVING, false },
	{ AX12_LOCK, false },
	{ AX12_PUNCH_L, true }
};

void BioloidHWInterfaceSigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.

  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

int BioloidHWInterface::radToAxPosition(float oldValue)
{
  // Convert rads to AX-12 position
  if (((-512 * 0.005 - FLOAT_PRECISION_THRESH) <= oldValue) and (oldValue <= (511 * 0.005 + FLOAT_PRECISION_THRESH)))
  {
    return round(oldValue / 0.005 + 512);
  }
  else
  {
    ROS_WARN("Value outside of valid input range, returning 512.");
    return 512;
  }
}

int BioloidHWInterface::receiveFromAX(XferAX *xfer)
{
  // Motor
  if ((1 <= xfer->dxlID) && (xfer->dxlID < 100))
  {
    // Read word
    switch (xfer->address)
    {
      case AX12_MODEL_NUMBER_L:
      case AX12_CW_ANGLE_LIMIT_L:
      case AX12_CCW_ANGLE_LIMIT_L:
      case AX12_MAX_TORQUE_L:
      case AX12_GOAL_POSITION_L:
      case AX12_MOVING_SPEED_L:
      case AX12_TORQUE_LIMIT_L:
      case AX12_PRESENT_POSITION_L:
      case AX12_PRESENT_SPEED_L:
      case AX12_PRESENT_LOAD_L:
      case AX12_PUNCH_L:
      {
        xfer->value = dxl_read_word(xfer->dxlID, xfer->address);
        break;
      }
      // Read byte
      default:
      {
        xfer->value = dxl_read_byte(xfer->dxlID, xfer->address);
        break;
      }
    }
  }
  else
  {
    xfer->Success = false;
    return ROBOTHW_INV_ID;
  }

  int CommStatus = dxl_get_result();
  if (CommStatus == COMM_RXSUCCESS)
  {
    // ROS_DEBUG("Value received: %d", res.value);
    // printErrorCode();
    xfer->Success = true;
    return ROBOTHW_OK;
  }
  else
  {
    // printCommStatus(CommStatus);
    xfer->Success = false;
    return ROBOTHW_ERROR_XFER;
  }
}

int BioloidHWInterface::sendToAX(XferAX *xfer)
{
  // Motor
  if (((1 <= xfer->dxlID) && (xfer->dxlID < 100)) || (xfer->dxlID == BROADCAST_ID))
  {
    switch (xfer->address)
    {
      case AX12_MODEL_NUMBER_L:
      case AX12_CW_ANGLE_LIMIT_L:
      case AX12_CCW_ANGLE_LIMIT_L:
      case AX12_MAX_TORQUE_L:
      case AX12_GOAL_POSITION_L:
      case AX12_MOVING_SPEED_L:
      case AX12_TORQUE_LIMIT_L:
      case AX12_PRESENT_POSITION_L:
      case AX12_PRESENT_SPEED_L:
      case AX12_PRESENT_LOAD_L:
      case AX12_PUNCH_L:
      {
        // 2 bytes
        dxl_write_word(xfer->dxlID, xfer->address, xfer->value);
        break;
      }
      default:
      {
        // 1 byte
        dxl_write_byte(xfer->dxlID, xfer->address, xfer->value);
        break;
      }
    }
  }
  else
  {
    xfer->Success = false;
    return ROBOTHW_INV_ID;
  }

  // No return Status Packet from a broadcast command
  if (xfer->dxlID == BROADCAST_ID)
  {
    xfer->Success = true;
    return ROBOTHW_OK;
  }
  else
  {
    int CommStatus = dxl_get_result();
    if (CommStatus == COMM_RXSUCCESS)
    {
      // ROS_DEBUG("Value sent: %d", val);
      // printErrorCode();
      xfer->Success = true;
      return ROBOTHW_OK;
    }
    else
    {
      // printCommStatus(CommStatus);
      xfer->Success = false;
      return ROBOTHW_ERROR_XFER;
    }
  }
}

int BioloidHWInterface::radPerSecToAxSpeed(float oldValue)
{
  // Convert rads per sec to AX-12 speed
  int newValue = round(fabs(oldValue) / 0.012);
  if ((0.0 <= oldValue) && (oldValue <= (1023 * 0.012 + FLOAT_PRECISION_THRESH)))
    return newValue;
  else if (((-1023 * 0.012 - FLOAT_PRECISION_THRESH) <= oldValue) && (oldValue < 0.0))
    return newValue | 0x400;  // Set bit 10 to 1
  else
  {
    ROS_WARN("Value outside of valid input range, returning 0.");
    return 0;
  }
}

int BioloidHWInterface::receiveSyncFromAX(XferSyncAX *xfer)
{
  // Example: IDs 1, 3, 5 - Get current position, current speed, current torque
  // startAddress:         AX12_PRESENT_POSITION_L (36)
  // numOfValuesPerMotor:  3
  // dxlIDs:               |        1      |        3      |        5      |
  // Returned values:      |  P1,  S1,  T1 |  P2,  S2,  T2 |  P3,  S3,  T3 |
  //
  // rosservice command line example:
  // rosservice call /ReceiveSyncFromAX '[1, 3, 5]' 36 3

  int numOfMotors = num_joints_;

  // Length of data for each motor
  int dataLength = 0;
  std::map<int, bool>::const_iterator it;
  std::vector<bool> isWord(xfer->numOfValuesPerMotor, false);
  for (int j = 0; j < xfer->numOfValuesPerMotor; ++j)
  {
    it = addressWordMap.find(xfer->startAddress + dataLength);
    if (it == addressWordMap.end())
    {
      ROS_ERROR("Address lookup error.");
      xfer->Success = false;
      return false;
    }
    if (it->second == true)
    {
      dataLength = dataLength + 2;
      isWord[j] = true;
    }
    else
    {
      ++dataLength;
      isWord[j] = false;
    }
  }
  if (dataLength > 6)
  {
    ROS_ERROR("Maximum data length must be 6 bytes.");
    xfer->Success = false;
    return ROBOTHW_INV_REG;
  }

  // Generate sync_read command
  dxl_sync_read_start(xfer->startAddress, dataLength);
  for (int i = 0; i < numOfMotors; ++i)
    dxl_sync_read_push_id(xfer->dxlIDs[i]);
  dxl_sync_read_send();

  int CommStatus = dxl_get_result();
  if (CommStatus == COMM_RXSUCCESS)
  {
    int r;
    for (int i = 0; i < numOfMotors; ++i)
    {
      for (int j = 0; j < xfer->numOfValuesPerMotor; ++j)
      {
        if (isWord[j])
          r = dxl_sync_read_pop_word();
        else
          r = dxl_sync_read_pop_byte();

        //                ROS_DEBUG( "i = %d", i );
        //                ROS_DEBUG( "j = %d", j );
        //                ROS_DEBUG( "index = %d", i*req->numOfValuesPerMotor + j );
        //                ROS_DEBUG( "ID %d = %d", req->dxlIDs[i], r );
        xfer->values[j][i] = r;
      }
    }
    // printErrorCode();
    xfer->Success = true;
    return ROBOTHW_OK;
  }
  else
  {
    // printCommStatus(CommStatus);
    xfer->Success = false;
    return ROBOTHW_ERROR_SYNC_XFER;
  }
}

int BioloidHWInterface::sendSyncToAX(XferSyncAX *xfer)
{
  // Example: IDs 1, 3, 5 - Set goal position 100, goal speed 300, max torque 512
  // startAddress:  AX12_GOAL_POSITION_L (30)
  // dxlIDs:        |        1      |        3      |        5      |
  // values:        | 100, 300, 512 | 100, 300, 512 | 100, 300, 512 |
  //
  // rosservice command line example:
  // rosservice call /SendSyncToAX '[1, 3, 5]' 30 '[100, 300, 512, 100, 300, 512, 100, 300, 512]'

  int numOfMotors = NUMBER_OF_JOINTS;
  int numOfValuesPerMotor = xfer->numOfValuesPerMotor;

  // Length of data for each motor
  int dataLength = 0;
  std::map<int, bool>::const_iterator it;
  std::vector<bool> isWord(numOfValuesPerMotor, false);
  for (int j = 0; j < numOfValuesPerMotor; ++j)
  {
    it = addressWordMap.find(xfer->startAddress + dataLength);
    if (it == addressWordMap.end())
    {
      ROS_ERROR("Address lookup error.");
      xfer->Success = false;
      return ROBOTHW_INV_REG;
    }
    if (it->second == true)
    {
      dataLength = dataLength + 2;
      isWord[j] = true;
    }
    else
    {
      ++dataLength;
      isWord[j] = false;
    }
  }

  // Make sync_write packet
  //    ROS_DEBUG( "Packet" );
  //    ROS_DEBUG( "ID:\t\t\t %d", BROADCAST_ID );
  //    ROS_DEBUG( "Instr:\t\t\t %d", INST_SYNC_WRITE );
  //    ROS_DEBUG( "Param 0 (start addr):\t %d", req->startAddress );
  //    ROS_DEBUG( "Param 1 (data length):\t %d", dataLength );
  dxl_set_txpacket_id(BROADCAST_ID);
  dxl_set_txpacket_instruction(INST_SYNC_WRITE);
  dxl_set_txpacket_parameter(0, xfer->startAddress);
  dxl_set_txpacket_parameter(1, dataLength);

  int paramIndex = 2;
  for (int i = 0; i < numOfMotors; ++i)
  {
    //        ROS_DEBUG( "Param %d (dxl %d):\t %d", paramIndex, i, req->dxlIDs[i] );
    dxl_set_txpacket_parameter(paramIndex++, xfer->dxlIDs[i]);
    for (int j = 0; j < numOfValuesPerMotor; ++j)
    {
      //                ROS_DEBUG( "Value: %d", req->values[i*numOfValuesPerMotor + j] );
      if (isWord[j])
      {
        // 2 bytes
        //                ROS_DEBUG( "Param %d (data %dL):\t %d", paramIndex, j,
        //                           dxl_get_lowbyte((int)(req->values[i*numOfValuesPerMotor + j])) );
        //                ROS_DEBUG( "Param %d (data %dH):\t %d", paramIndex + 1, j,
        //                           dxl_get_highbyte((int)(req->values[i*numOfValuesPerMotor + j])) );
        dxl_set_txpacket_parameter(paramIndex++, dxl_get_lowbyte((int)(xfer->values[j][i])));
        dxl_set_txpacket_parameter(paramIndex++, dxl_get_highbyte((int)(xfer->values[j][i])));
      }
      else
      {
        // 1 byte
        //                ROS_DEBUG( "Param %d (data %d):\t %d", paramIndex, j,
        //                           dxl_get_highbyte((int)(req->values[i*numOfValuesPerMotor + j])) );
        dxl_set_txpacket_parameter(paramIndex++, (int)(xfer->values[j][i]));
      }
      //            ROS_DEBUG("--");
    }
  }

  //    ROS_DEBUG( "Length:\t\t\t %d", (dataLength + 1)*numOfMotors + 4 );
  dxl_set_txpacket_length((dataLength + 1) * numOfMotors + 4);

  dxl_txrx_packet();

  int CommStatus = dxl_get_result();
  if (CommStatus == COMM_RXSUCCESS)
  {
    // printErrorCode();
    xfer->Success = true;
    return ROBOTHW_OK;
  }
  else
  {
    // printCommStatus(CommStatus);
    xfer->Success = false;
    return ROBOTHW_ERROR_SYNC_XFER;
  }
}

int BioloidHWInterface::initial_position_set(void)
{
  int ret;
  XferSyncAX xfer;

  xfer.startAddress = AX12_GOAL_POSITION_L;
  xfer.numOfValuesPerMotor = 1;

  for (std::size_t joint_id = 0; joint_id < NUMBER_OF_JOINTS; ++joint_id)
  {
    xfer.dxlIDs[joint_id] = joint_id + 1;

    xfer.values[0][joint_id] = radToAxPosition(joint_reset_rad[joint_id]);
  }

  ret = sendSyncToAX(&xfer);

  ros::Duration(5.0).sleep();

  return ret;
}

int BioloidHWInterface::Initialize(void)
{
  // Note: USB2AX uses a different dxl_hal.c file than the Robotis' Dynamixel SDK for Linux.
  // The Dynamixel SDK assumes the interface is FTDI-based, and thus searches a device named ttyUSBx,
  // while the USB2AX uses the integrated CDC/ACM driver - which names the device ttyACMx.
  // The second problem is that after opening the device, the Dynamixel SDK tries to set parameters which do
  // not exist in the CDC/ACM driver.
  // For more information see:
  // http://www.xevelabs.com/doku.php?id=product:usb2ax:faq#qdynamixel_sdkhow_do_i_use_it_with_the_usb2ax

  int deviceIndex = 0;
  int baudNum = 1;
  int numOfConnectedMotors = 0;
  int numOfConnectedSensorBoard = 0;

  // Initialise comms
  if (dxl_initialize(deviceIndex, baudNum) == 0)
  {
    ROS_ERROR("Failed to open USB2AX.");
    return ROBOTHW_DXL_INIT_ERROR;
  }

  ros::Duration(1.0).sleep();

  // Find motors with IDs [1 to NUM_OF_MOTORS]
  for (int dxlID = 1; dxlID <= NUMBER_OF_JOINTS; ++dxlID)
  {
    dxl_ping(dxlID);
    if (dxl_get_result() == COMM_RXSUCCESS)
    {
      ++numOfConnectedMotors;
      ROS_INFO("PING actuator ID %02d:...[OK]", dxlID);
    }
    else
    {
      ROS_WARN("PING actuator ID %02d:...[FAILED]", dxlID);
    }
    ros::Duration(0.5).sleep();
  }

  if (numOfConnectedMotors != NUMBER_OF_JOINTS)
  {
    ROS_INFO("%d motors connected.", numOfConnectedMotors);
    ROS_ERROR("Number of actuator required %d", NUMBER_OF_JOINTS);
    return ROBOTHW_INV_ACTUATOR_NB;
  }

  // Initial motor settings
  XferAX xfer;

  // Reduce Return Delay Time to speed up comms
  xfer.dxlID = BROADCAST_ID;
  xfer.address = AX12_RETURN_DELAY_TIME;
  xfer.value = 0;
  sendToAX(&xfer);
  ROS_INFO_STREAM("All return delay times set to " << xfer.value << ".");

  // Add compliance margins to reduce motor buzz
  xfer.dxlID = BROADCAST_ID;
  xfer.address = AX12_CW_COMPLIANCE_MARGIN;
  xfer.value = 10;
  sendToAX(&xfer);
  xfer.address = AX12_CCW_COMPLIANCE_MARGIN;
  sendToAX(&xfer);
  ROS_INFO_STREAM("All CW and CCW compliance margins set to " << xfer.value << ".");

  //    // Set torque
  //    paramSet_req->dxlID = BROADCAST_ID;
  //    paramSet_req->value = 0.8;
  //    jointController.setMotorTorqueLimitInDecimal(paramSet_req, paramSet_res);
  //    ros::Duration(0.5).sleep();
  //    ROS_INFO_STREAM("All torque limits set to " << paramSet_req->value/100.0 << "%.");

  // Set slow speed
  float speed = 1.0;
  xfer.dxlID = BROADCAST_ID;
  xfer.address = AX12_MOVING_SPEED_L;
  xfer.value = radPerSecToAxSpeed(speed);
  sendToAX(&xfer);
  ROS_INFO_STREAM("All goal speeds set to " << speed << " rad/s.";);

  //    // Home all motors
  //    jointController.homeAllMotors(empty_req, empty_res);
  //    ros::Duration(3.0).sleep();
  //    ROS_INFO("All motors homed.");

  // Turn off all torques
  xfer.dxlID = BROADCAST_ID;
  xfer.address = AX12_TORQUE_ENABLE;
  xfer.value = 0;
  sendToAX(&xfer);
  ROS_INFO("All motor torques turned off.");

  /* Set actuator to their initial position */
  if (initial_position_set() == ROBOTHW_OK)
    ROS_INFO("All motor set to neutral position.");
  else
    return ROBOTHW_ERROR_RESET;


	// Find sensor boards
	for (int dxlID = SENSOR_BOARD_ID_START; dxlID <= SENSOR_BOARD_MAX; ++dxlID)
	{
		dxl_ping(dxlID);
		if (dxl_get_result() == COMM_RXSUCCESS)
		{
			++numOfConnectedSensorBoard;
			ROS_INFO("PING sensor ID %02d:...[OK]", dxlID);
		}
		else
		{
			ROS_WARN("PING sensor ID %02d:...[FAILED]", dxlID);
		}
		ros::Duration(0.5).sleep();
	}
#if 0
	if (numOfConnectedSensorBoard != SENSOR_BOARD_MAX)
	{
		ROS_INFO("%d sensor board connected.", numOfConnectedSensorBoard);
		ROS_ERROR("Number of sensor board required %d", SENSOR_BOARD_MAX);
		return ROBOTHW_INV_SENSOR_NB;
	}
#endif
  ROS_INFO("Robot HW init done.");

  return ROBOTHW_OK;
}

BioloidHWInterface::BioloidHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
  : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
, name_("bioloid_hw_interface")
{
  // Load rosparams
  ros::NodeHandle rpnh(nh, "hardware_interface");
  std::size_t error = 0;
  double reset_angle;
  double dir_sign;

  error += !rosparam_shortcuts::get(name_, rpnh, "joints", joint_names_);
  rosparam_shortcuts::shutdownIfError(name_, error);
  num_joints_ = joint_names_.size();

  ros::NodeHandle rpnhra(nh, "resetangle");
  for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
  {
    if (rosparam_shortcuts::get(name_, rpnhra, joint_names_[joint_id], reset_angle))
      joint_reset_rad[joint_id] = reset_angle;
    else
      joint_reset_rad[joint_id] = 0;
    HW_DBG("reset angle: #%d: %g", (int)(joint_id + 1), joint_reset_rad[joint_id]);

    /* initialise dynamixel packet */
    xfer.dxlIDs[joint_id] = joint_id + 1;
  }

  ros::NodeHandle rpnhrb(nh, "dirangle");
  for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
  {
    if (rosparam_shortcuts::get(name_, rpnhrb, joint_names_[joint_id], dir_sign))
	joint_dir_sign[joint_id] = dir_sign;
    else
	joint_dir_sign[joint_id] = 1;
    HW_DBG("direction: #%d: %g", (int)(joint_id + 1), joint_dir_sign[joint_id]);
  }

  navio_led_pub_ = rpnh.advertise<std_msgs::Int32>("navio_led", 1);

  // Set LED
  led_msg.data = LED_YELLOW;
  navio_led_pub_.publish(led_msg);

}

float BioloidHWInterface::axSpeedToRadPerSec(int oldValue)
{
  // Convert AX-12 speed to rads per sec
  // v (Hz) = w (rad/s) / 2*pi
  // 1 rpm = 1/60 Hz ~= 0.105 rad/s
  // ~0.111 rpm per unit <=> ~0.012 rad/s per unit
  // Speed range:    0..1023 <=> 0..113.553 rpm (CCW) <=> 0..12.276 rad/s (CCW)
  //              1024..2047 <=> 0..-113.553 rpm (CW)  <=> 0..-12.276 rad/s (CW)
  float newValue = (oldValue & 0x3FF) * 0.012;  // Bits 0-9
  if ((oldValue & 0x400) == 0x0)                // Check bit 10
    return newValue;
  else
    return -newValue;
}

float BioloidHWInterface::axPositionToRad(int oldValue)
{
  // Convert AX-12 position to rads
  // 0.29 degrees per unit <=> ~0.005 rads per unit
  // Position range: 0..1023 <=> 0..296.67 degrees <=> 0..5.115 rad
  // If offset is added so that 0 is the midpoint (0 degrees), then range is:
  // -512..511 <=> -148.48..148.19 degrees <=> -2.56..2.555 rad
  return ((oldValue & 0x3FF) - 512) * 0.005;  // Bits 0-9
}

float BioloidHWInterface::axTorqueToDecimal(int oldValue)
{
  // Convert AX-12 torque to % torque
  // ~0.1% per unit
  // Torque range:    0..1023 <=> 0.0..1.023 (CCW)
  //               1024..2047 <=> 0.0..-1.023 (CW)
  float newValue = (oldValue & 0x3FF) * 0.001;  // Bits 0-9
  if ((oldValue & 0x400) == 0x0)                // Check bit 10
    return newValue;
  else
    return -newValue;
}


void BioloidHWInterface::init()
{
  // Call parent class version of this function
  GenericHWInterface::init();

  if (Initialize() != ROBOTHW_OK)
  {
    ROS_ERROR("Initialization failed.");
    BioloidHWInterfaceSigintHandler(0);
  }

  // Resize vectors
  joint_position_prev_.resize(num_joints_, 0.0);

  ROS_INFO_NAMED(name_, "BioloidHWInterface Ready.");

  // Set LED
  led_msg.data = LED_GREEN;
  navio_led_pub_.publish(led_msg);
}

void BioloidHWInterface::read(ros::Duration &elapsed_time)
{
  std::size_t joint_id;
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // FILL IN YOUR READ COMMAND FROM USB/ETHERNET/ETHERCAT/SERIAL ETC HERE
  for (joint_id = 0; joint_id < num_joints_; ++joint_id)
  {
    joint_position_[joint_id] = std::numeric_limits<double>::quiet_NaN();
  }

  // Get position, speed and torque with a sync_read command
  xfer.startAddress = AX12_PRESENT_POSITION_L;
  xfer.numOfValuesPerMotor = NUMBER_OF_FIELD_PER_MOTOR;

  if (receiveSyncFromAX(&xfer) == ROBOTHW_OK)
  {
    for (joint_id = 0; joint_id < num_joints_; ++joint_id)
    {
      joint_position_[joint_id] =
          joint_dir_sign[joint_id] * axPositionToRad(xfer.values[0][joint_id]) - joint_reset_rad[joint_id];
// joint_velocity_[joint_id] = axSpeedToRadPerSec(xfer.values[1][joint_id]);
/* vel_[id_index] = (new_pos - pos_[id_index])/(double)period.toSec(); */
// joint_effort_[joint_id] = axTorqueToDecimal(xfer.values[2][joint_id]);
#if 0
    		if (joint_position_[joint_id] &&
    				joint_position_[joint_id] != std::numeric_limits<double>::quiet_NaN() &&
					(joint_position_[joint_id] > 0.1 || joint_position_[joint_id] < -0.1))
    			HW_DBG("Pos #%d: %g", (int)(joint_id + 1), joint_position_[joint_id]);
#endif
    }
  }
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}

void BioloidHWInterface::write(ros::Duration &elapsed_time)
{
  xfer.startAddress = AX12_GOAL_POSITION_L;
  xfer.numOfValuesPerMotor = 1;

  // Safety
  enforceLimits(elapsed_time);

  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // FILL IN YOUR WRITE COMMAND TO USB/ETHERNET/ETHERCAT/SERIAL ETC HERE
  //
  // FOR A EASY SIMULATION EXAMPLE, OR FOR CODE TO CALCULATE
  // VELOCITY FROM POSITION WITH SMOOTHING, SEE
  // sim_hw_interface.cpp IN THIS PACKAGE
  //
  // DUMMY PASS-THROUGH CODE
  for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
  {
    if (std::isnan(joint_position_command_[joint_id]))
    {
#if 0
		HW_DBG("Join #%d is NAN, set to %g", (int)(joint_id + 1), joint_position_[joint_id]);
#endif
    }
#if 0
	//joint_position_[joint_id] = joint_position_command_[joint_id];
	if (joint_position_command_[joint_id] &&
		joint_position_command_[joint_id] != std::numeric_limits<double>::quiet_NaN()  &&
		(joint_position_command_[joint_id] > 0.1 || joint_position_command_[joint_id] < -0.1)) {
		HW_DBG("Cmd #%d: %g", (int)(joint_id + 1), joint_position_command_[joint_id]);
		HW_DBG("CPos #%d: %g", (int)(joint_id + 1), position_joint_interface_.getHandle(joint_names_[joint_id]).getPosition());
		HW_DBG("CCmd #%d: %g", (int)(joint_id + 1), position_joint_interface_.getHandle(joint_names_[joint_id]).getCommand());
	}
#endif

    xfer.values[0][joint_id] =
        radToAxPosition(joint_dir_sign[joint_id] * joint_position_command_[joint_id] + joint_reset_rad[joint_id]);

    // Calculate velocity based on change in positions
    if (elapsed_time.toSec() > 0)
    {
      joint_velocity_[joint_id] = (joint_position_[joint_id] - joint_position_prev_[joint_id]) / elapsed_time.toSec();
    }
    else
      joint_velocity_[joint_id] = 0;

    // Save last position
    joint_position_prev_[joint_id] = joint_position_[joint_id];
    // HW_DBG("AX value: %d", xfer.values[0][joint_id]);
  }

  // Actuator sync
  sendSyncToAX(&xfer);

  // END DUMMY CODE
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}

void BioloidHWInterface::enforceLimits(ros::Duration &period)
{
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // CHOOSE THE TYPE OF JOINT LIMITS INTERFACE YOU WANT TO USE
  // YOU SHOULD ONLY NEED TO USE ONE SATURATION INTERFACE,
  // DEPENDING ON YOUR CONTROL METHOD
  //
  // EXAMPLES:
  //
  // Saturation Limits ---------------------------
  //
  // Enforces position and velocity
  pos_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces velocity and acceleration limits
  // vel_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces position, velocity, and effort
  // eff_jnt_sat_interface_.enforceLimits(period);

  // Soft limits ---------------------------------
  //
  // pos_jnt_soft_limits_.enforceLimits(period);
  // vel_jnt_soft_limits_.enforceLimits(period);
  // eff_jnt_soft_limits_.enforceLimits(period);
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}

}  // namespace
