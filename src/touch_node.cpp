/**
 * ROS driver node for 3D Systems Touch / Geomagic Touch haptic device.
 *
 * Uses OpenHaptics HDAPI to read device state (position, orientation,
 * joint angles, buttons) and apply force feedback.
 *
 * Based on bharatm11/Geomagic_Touch_ROS_Drivers, cleaned up for ROS Kinetic.
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>

#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>

#include "touch_driver/TouchButtonEvent.h"
#include "touch_driver/TouchFeedback.h"

#include <pthread.h>
#include <string>
#include <sstream>

// ---------------------------------------------------------------------------
// Device state shared between servo thread and main thread
// ---------------------------------------------------------------------------
struct DeviceState
{
  // Raw from HDAPI
  hduVector3Dd position;       // mm, device frame
  hduVector3Dd velocity;       // mm/s (filtered)
  hduVector3Dd joints;         // joint angles [rad]
  hduVector3Dd gimbal_angles;  // gimbal angles [rad]
  HDdouble transform[16];     // 4x4 column-major transform

  // Velocity filter history (Butterworth 2nd order, 20 Hz cutoff)
  hduVector3Dd inp_vel1, inp_vel2, inp_vel3;
  hduVector3Dd out_vel1, out_vel2, out_vel3;
  hduVector3Dd pos_hist1, pos_hist2;

  // Force feedback
  hduVector3Dd force;          // force to apply [N], device frame

  // Buttons
  int buttons[2];
  int buttons_prev[2];

  // Lock mode
  bool lock;
  hduVector3Dd lock_pos;

  // Units
  double units_ratio;          // divisor: 1.0=mm, 1000.0=m
};

// Calibration
static int g_calibration_style;

// ---------------------------------------------------------------------------
// HDAPI scheduler callback — runs at ~1 kHz in servo thread
// ---------------------------------------------------------------------------
HDCallbackCode HDCALLBACK servoCallback(void *pUserData)
{
  DeviceState *state = static_cast<DeviceState *>(pUserData);

  // Auto-calibration update
  if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE)
  {
    hdUpdateCalibration(g_calibration_style);
  }

  hdBeginFrame(hdGetCurrentDevice());

  // Read state
  hdGetDoublev(HD_CURRENT_TRANSFORM, state->transform);
  hdGetDoublev(HD_CURRENT_JOINT_ANGLES, state->joints);
  hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, state->gimbal_angles);

  // Position from transform (swap Y/Z, invert Z for ROS convention)
  // HD frame: Right +X, Up +Y, Toward user +Z
  // ROS frame: Forward +X, Left +Y, Up +Z
  state->position[0] =  state->transform[12];  // X
  state->position[1] = -state->transform[14];  // -Z -> Y
  state->position[2] =  state->transform[13];  // Y -> Z
  state->position /= state->units_ratio;

  // Velocity estimation: 2nd order backward difference + Butterworth filter (20 Hz)
  hduVector3Dd vel_raw = (state->position * 3.0
                          - 4.0 * state->pos_hist1
                          + state->pos_hist2) / 0.002;

  state->velocity = (0.2196 * (vel_raw + state->inp_vel3)
                     + 0.6588 * (state->inp_vel1 + state->inp_vel2)) / 1000.0
                    - (-2.7488 * state->out_vel1
                       + 2.5282 * state->out_vel2
                       - 0.7776 * state->out_vel3);

  state->pos_hist2 = state->pos_hist1;
  state->pos_hist1 = state->position;
  state->inp_vel3  = state->inp_vel2;
  state->inp_vel2  = state->inp_vel1;
  state->inp_vel1  = vel_raw;
  state->out_vel3  = state->out_vel2;
  state->out_vel2  = state->out_vel1;
  state->out_vel1  = state->velocity;

  // Apply force feedback (map back to HD frame: swap Y/Z, invert Z)
  hduVector3Dd feedback;
  feedback[0] =  state->force[0];       // X
  feedback[1] =  state->force[2];       // Z -> Y
  feedback[2] = -state->force[1];       // -Y -> Z
  hdSetDoublev(HD_CURRENT_FORCE, feedback);

  // Read buttons
  int nButtons = 0;
  hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
  state->buttons[0] = (nButtons & HD_DEVICE_BUTTON_1) ? 1 : 0;
  state->buttons[1] = (nButtons & HD_DEVICE_BUTTON_2) ? 1 : 0;

  hdEndFrame(hdGetCurrentDevice());

  // Check for errors
  HDErrorInfo error;
  if (HD_DEVICE_ERROR(error = hdGetError()))
  {
    hduPrintError(stderr, &error, "Error in servo callback");
    if (hduIsSchedulerError(&error))
      return HD_CALLBACK_DONE;
  }

  return HD_CALLBACK_CONTINUE;
}

// ---------------------------------------------------------------------------
// Calibration
// ---------------------------------------------------------------------------
void autoCalibrate()
{
  int supported;
  hdGetIntegerv(HD_CALIBRATION_STYLE, &supported);

  if (supported & HD_CALIBRATION_AUTO)
  {
    g_calibration_style = HD_CALIBRATION_AUTO;
    ROS_INFO("Calibration: AUTO");
  }
  else if (supported & HD_CALIBRATION_ENCODER_RESET)
  {
    g_calibration_style = HD_CALIBRATION_ENCODER_RESET;
    ROS_INFO("Calibration: ENCODER_RESET");

    HDErrorInfo error;
    do {
      hdUpdateCalibration(g_calibration_style);
      ROS_INFO("Calibrating... (place stylus in inkwell)");
      if (HD_DEVICE_ERROR(error = hdGetError()))
      {
        hduPrintError(stderr, &error, "Calibration failed");
        break;
      }
    } while (hdCheckCalibration() != HD_CALIBRATION_OK);
    ROS_INFO("Calibration complete");
  }
  else if (supported & HD_CALIBRATION_INKWELL)
  {
    g_calibration_style = HD_CALIBRATION_INKWELL;
    ROS_INFO("Calibration: INKWELL");
  }

  // Wait for calibration to be OK
  int wait_count = 0;
  while (hdCheckCalibration() != HD_CALIBRATION_OK && wait_count < 30)
  {
    usleep(500000);
    wait_count++;
    if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_MANUAL_INPUT)
      ROS_INFO("Please place the device into the inkwell for calibration");
    else if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE)
    {
      hdUpdateCalibration(g_calibration_style);
      ROS_INFO("Calibration updated");
    }
  }
}

// ---------------------------------------------------------------------------
// ROS publisher class
// ---------------------------------------------------------------------------
class TouchROS
{
public:
  ros::NodeHandle nh_;
  ros::Publisher pose_pub_;
  ros::Publisher joint_pub_;
  ros::Publisher button_pub_;
  ros::Publisher wrench_pub_;
  ros::Subscriber feedback_sub_;

  std::string omni_name_;
  std::string ref_frame_;
  DeviceState *state_;

  void init(DeviceState *state)
  {
    state_ = state;

    ros::NodeHandle pnh("~");
    pnh.param<std::string>("omni_name", omni_name_, "phantom");
    pnh.param<std::string>("reference_frame", ref_frame_, "world");

    std::string units;
    pnh.param<std::string>("units", units, "m");

    if (units == "mm")        state_->units_ratio = 1.0;
    else if (units == "cm")   state_->units_ratio = 10.0;
    else if (units == "dm")   state_->units_ratio = 100.0;
    else if (units == "m")    state_->units_ratio = 1000.0;
    else {
      ROS_WARN("Unknown units '%s', using 'm'", units.c_str());
      state_->units_ratio = 1000.0;
      units = "m";
    }
    ROS_INFO("Position units: %s (ratio: %.1f)", units.c_str(), state_->units_ratio);

    // Publishers
    pose_pub_   = nh_.advertise<geometry_msgs::PoseStamped>(omni_name_ + "/pose", 10);
    joint_pub_  = nh_.advertise<sensor_msgs::JointState>(omni_name_ + "/joint_states", 10);
    button_pub_ = nh_.advertise<touch_driver::TouchButtonEvent>(omni_name_ + "/button", 10);
    wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>(omni_name_ + "/wrench", 10);

    // Force feedback subscriber
    feedback_sub_ = nh_.subscribe(omni_name_ + "/force_feedback", 1,
                                  &TouchROS::forceCallback, this);

    // Init state
    hduVector3Dd zeros(0, 0, 0);
    state_->velocity  = zeros;
    state_->inp_vel1  = zeros;
    state_->inp_vel2  = zeros;
    state_->inp_vel3  = zeros;
    state_->out_vel1  = zeros;
    state_->out_vel2  = zeros;
    state_->out_vel3  = zeros;
    state_->pos_hist1 = zeros;
    state_->pos_hist2 = zeros;
    state_->force     = zeros;
    state_->lock      = false;
    state_->lock_pos  = zeros;
    state_->buttons[0] = state_->buttons[1] = 0;
    state_->buttons_prev[0] = state_->buttons_prev[1] = 0;
  }

  void forceCallback(const touch_driver::TouchFeedback::ConstPtr& msg)
  {
    // Apply force with small velocity damping for stability
    state_->force[0] = msg->force.x - 0.001 * state_->velocity[0];
    state_->force[1] = msg->force.y - 0.001 * state_->velocity[1];
    state_->force[2] = msg->force.z - 0.001 * state_->velocity[2];

    state_->lock_pos[0] = msg->position.x;
    state_->lock_pos[1] = msg->position.y;
    state_->lock_pos[2] = msg->position.z;
  }

  void publish()
  {
    ros::Time now = ros::Time::now();

    // --- Pose ---
    // Build quaternion from transform matrix
    // The transform is column-major 4x4 from HD_CURRENT_TRANSFORM
    // We remap to ROS convention in the servo callback (position already remapped)
    // For orientation, build rotation matrix with same Y/Z swap
    double* T = state_->transform;
    // HD rotation columns -> ROS rotation
    // ROS_X =  HD_X,  ROS_Y = -HD_Z,  ROS_Z = HD_Y
    double rot[9];
    rot[0] =  T[0];  rot[1] = -T[8];  rot[2] =  T[4];   // row 0
    rot[3] = -T[2];  rot[4] =  T[10]; rot[5] = -T[6];   // row 1
    rot[6] =  T[1];  rot[7] = -T[9];  rot[8] =  T[5];   // row 2

    // Rotation matrix to quaternion
    double trace = rot[0] + rot[4] + rot[8];
    double qw, qx, qy, qz;
    if (trace > 0)
    {
      double s = 0.5 / sqrt(trace + 1.0);
      qw = 0.25 / s;
      qx = (rot[7] - rot[5]) * s;
      qy = (rot[2] - rot[6]) * s;
      qz = (rot[3] - rot[1]) * s;
    }
    else if (rot[0] > rot[4] && rot[0] > rot[8])
    {
      double s = 2.0 * sqrt(1.0 + rot[0] - rot[4] - rot[8]);
      qw = (rot[7] - rot[5]) / s;
      qx = 0.25 * s;
      qy = (rot[1] + rot[3]) / s;
      qz = (rot[2] + rot[6]) / s;
    }
    else if (rot[4] > rot[8])
    {
      double s = 2.0 * sqrt(1.0 + rot[4] - rot[0] - rot[8]);
      qw = (rot[2] - rot[6]) / s;
      qx = (rot[1] + rot[3]) / s;
      qy = 0.25 * s;
      qz = (rot[5] + rot[7]) / s;
    }
    else
    {
      double s = 2.0 * sqrt(1.0 + rot[8] - rot[0] - rot[4]);
      qw = (rot[3] - rot[1]) / s;
      qx = (rot[2] + rot[6]) / s;
      qy = (rot[5] + rot[7]) / s;
      qz = 0.25 * s;
    }

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = now;
    pose.header.frame_id = ref_frame_;
    pose.pose.position.x = state_->position[0];
    pose.pose.position.y = state_->position[1];
    pose.pose.position.z = state_->position[2];
    pose.pose.orientation.x = qx;
    pose.pose.orientation.y = qy;
    pose.pose.orientation.z = qz;
    pose.pose.orientation.w = qw;
    pose_pub_.publish(pose);

    // --- Joint states ---
    sensor_msgs::JointState js;
    js.header.stamp = now;
    js.name.resize(6);
    js.position.resize(6);
    js.name[0] = "waist";
    js.position[0] = -state_->joints[0];
    js.name[1] = "shoulder";
    js.position[1] = state_->joints[1];
    js.name[2] = "elbow";
    js.position[2] = state_->joints[2] - state_->joints[1];
    js.name[3] = "yaw";
    js.position[3] = -state_->gimbal_angles[0] + M_PI;
    js.name[4] = "pitch";
    js.position[4] = -state_->gimbal_angles[1] - 3.0 * M_PI / 4.0;
    js.name[5] = "roll";
    js.position[5] = -state_->gimbal_angles[2] - M_PI;
    joint_pub_.publish(js);

    // --- Wrench (current force being applied) ---
    geometry_msgs::WrenchStamped wrench;
    wrench.header.stamp = now;
    wrench.header.frame_id = ref_frame_;
    wrench.wrench.force.x = state_->force[0];
    wrench.wrench.force.y = state_->force[1];
    wrench.wrench.force.z = state_->force[2];
    wrench_pub_.publish(wrench);

    // --- Buttons ---
    if (state_->buttons[0] != state_->buttons_prev[0] ||
        state_->buttons[1] != state_->buttons_prev[1])
    {
      touch_driver::TouchButtonEvent btn;
      btn.grey_button  = state_->buttons[0];
      btn.white_button = state_->buttons[1];
      button_pub_.publish(btn);

      state_->buttons_prev[0] = state_->buttons[0];
      state_->buttons_prev[1] = state_->buttons[1];
    }
  }
};

// ---------------------------------------------------------------------------
// Publishing thread
// ---------------------------------------------------------------------------
void *publishThread(void *ptr)
{
  TouchROS *touch_ros = static_cast<TouchROS *>(ptr);
  int publish_rate;
  ros::param::param<int>("~publish_rate", publish_rate, 100);
  ROS_INFO("Publishing Touch state at %d Hz", publish_rate);

  ros::Rate rate(publish_rate);
  ros::AsyncSpinner spinner(2);
  spinner.start();

  while (ros::ok())
  {
    touch_ros->publish();
    rate.sleep();
  }
  return NULL;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "touch_node");
  ros::NodeHandle pnh("~");

  // Device name (as configured in Touch Setup utility)
  std::string device_name;
  pnh.param<std::string>("device_name", device_name, "Default Device");

  // --- Init haptic device ---
  HDErrorInfo error;
  HHD hHD = hdInitDevice(device_name.c_str());
  if (HD_DEVICE_ERROR(error = hdGetError()))
  {
    ROS_FATAL("Failed to initialize haptic device '%s'", device_name.c_str());
    return 1;
  }

  ROS_INFO("Found device: %s", hdGetString(HD_DEVICE_MODEL_TYPE));
  ROS_INFO("Device vendor: %s", hdGetString(HD_DEVICE_VENDOR));
  ROS_INFO("Driver version: %s", hdGetString(HD_DEVICE_DRIVER_VERSION));

  hdEnable(HD_FORCE_OUTPUT);
  hdStartScheduler();
  if (HD_DEVICE_ERROR(error = hdGetError()))
  {
    ROS_FATAL("Failed to start scheduler");
    return 1;
  }

  autoCalibrate();

  // --- Init ROS ---
  DeviceState state;
  TouchROS touch_ros;
  touch_ros.init(&state);

  // Schedule servo callback at max priority (runs at ~1 kHz)
  hdScheduleAsynchronous(servoCallback, &state, HD_MAX_SCHEDULER_PRIORITY);

  // --- Run ---
  pthread_t pub_thread;
  pthread_create(&pub_thread, NULL, publishThread, &touch_ros);
  pthread_join(pub_thread, NULL);

  // --- Cleanup ---
  ROS_INFO("Shutting down...");
  hdStopScheduler();
  hdDisableDevice(hHD);

  return 0;
}
