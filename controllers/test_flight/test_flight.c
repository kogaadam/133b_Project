/*
 * File:          test_flight.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 64

#define SIGN(x) ((x) > 0) - ((x) < 0)

/*
IF val < low:
  return low
ELSE:
  IF val > high:
    return high
  ELSE
    return value
*/
#define CLAMP(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();
  int timestep = (int)wb_robot_get_basic_time_step();

  // Get IMU
  WbDeviceTag imu = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(imu, timestep);
  // Get GPS
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, timestep);
  // Get Gyro
  WbDeviceTag gyro = wb_robot_get_device("gyro");
  wb_gyro_enable(gyro, timestep);

  // Get propeller motors and set them to velocity mode.
  WbDeviceTag fr_lt_mot = wb_robot_get_device("front left propeller");
  WbDeviceTag fr_rt_mot = wb_robot_get_device("front right propeller");
  WbDeviceTag bk_lt_mot = wb_robot_get_device("rear left propeller");
  WbDeviceTag bk_rt_mot = wb_robot_get_device("rear right propeller");
  WbDeviceTag motors[4] = {fr_lt_mot, fr_rt_mot, bk_lt_mot, bk_rt_mot};
  int m;
  for (m = 0; m < 4; ++m) {
    wb_motor_set_position(motors[m], INFINITY);
    wb_motor_set_velocity(motors[m], 1.0);
  }

  // Wait one second.
  while (wb_robot_step(timestep) != -1) {
    if (wb_robot_get_time() > 1.0)
      break;
  }

  // Constants, empirically found.
  const double k_vert_thrust = 68.5;  // with this thrust, the drone lifts.
  const double k_vert_offset = 0.6;   // Vertical offset where the robot actually targets to stabilize itself.
  const double k_vert_p = 3.0;        // P constant of the vertical PID.
  const double k_roll_p = 50.0;       // P constant of the roll PID.
  const double k_pitch_p = 30.0;      // P constant of the pitch PID.

  // Variables.
  double target_altitude = 1.0;  // The target altitude
  // double target_x = 0.0;
  // double target_z = 0.0;

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(timestep) != -1) {
    // Get robot position and other characteristics
    const double roll = wb_inertial_unit_get_roll_pitch_yaw(imu)[0] + M_PI / 2.0;
    const double pitch = wb_inertial_unit_get_roll_pitch_yaw(imu)[1];
    // const double x_pos = wb_gps_get_values(gps)[0];
    const double altitude = wb_gps_get_values(gps)[1];
    // const double z_pos = wb_gps_get_values(gps)[2];
    const double roll_acc = wb_gyro_get_values(gyro)[0];
    const double pitch_acc = wb_gyro_get_values(gyro)[1];

    // Process sensor data
    const double roll_change = k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_acc;
    const double pitch_change = k_pitch_p * CLAMP(pitch, -1.0, 1.0) - pitch_acc;
    const double clamped_diff_alt = CLAMP(target_altitude - altitude + k_vert_offset, -1.0, 1.0);
    const double vert_change = k_vert_p * pow(clamped_diff_alt, 3.0);

    // Actuate the motors
    const double fr_lt_mot_inp = k_vert_thrust + vert_change - roll_change - pitch_change;
    const double fr_rt_mot_inp = k_vert_thrust + vert_change + roll_change - pitch_change;
    const double bk_lt_mot_inp = k_vert_thrust + vert_change - roll_change + pitch_change;
    const double bk_rt_mot_inp = k_vert_thrust + vert_change + roll_change + pitch_change;

    // Set new motor velocities
    wb_motor_set_velocity(fr_lt_mot, fr_lt_mot_inp);
    wb_motor_set_velocity(fr_rt_mot, -fr_rt_mot_inp);
    wb_motor_set_velocity(bk_lt_mot, -bk_lt_mot_inp);
    wb_motor_set_velocity(bk_rt_mot, bk_rt_mot_inp);
  };

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
