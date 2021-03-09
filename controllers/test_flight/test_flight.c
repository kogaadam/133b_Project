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
#include <unistd.h>

/*
 * You may want to add macros here.
 */

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


struct datapoints {
  double dp1, dp2, dp3;
};

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {

  // run executable
  int r = system("C:\\School\\_SR\\_WI\\133b\\133b_Project\\prm\\t.exe \
                  prob_road_map_spline main");
  printf("return: %d\n", r);

  sleep(1);

  // read from file
  FILE *fptr;
  const int data_len = 1000;
  struct datapoints dps;

  if ((fptr = fopen("C:\\School\\_SR\\_WI\\133b\\133b_Project\\prm\\path.bin", "rb")) == NULL) {
    printf("Error opening file");
    exit(1);
  }

  // Temp array of destination nodes
  double nodes[1000][3];

  for(int i = 0; i < data_len; i++) {
    fread(&dps, sizeof(struct datapoints), 1, fptr);
    nodes[i][0] = dps.dp1;
    nodes[i][1] = dps.dp3;
    nodes[i][2] = dps.dp2;
    // printf("[%f, %f, %f]\n", dps.dp1, dps.dp2, dps.dp3);
  }
  fclose(fptr);

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
  const double x_offset = -.42;
  const double z_offset = 0;
  const double k_vert_p = 3.0;        // P constant of the vertical PID.
  const double k_roll_p = 50.0;       // P constant of the roll PID.
  const double k_pitch_p = 30.0;      // P constant of the pitch PID.

  // array index for desired node
  int node_index = 0;
  // Threshold for converging to node position [m]
  const double thresh = 0.1;

  // Initialize Target Variables.
  double target_altitude = nodes[0][1];  // The target altitude
  double target_x = nodes[0][0];
  double target_z = nodes[0][2];
  double target_yaw = 0.0;
  // IF target-current < threshold:
  //    array index ++

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(timestep) != -1) {
    // Get robot position and other characteristics
    const double roll = wb_inertial_unit_get_roll_pitch_yaw(imu)[0] + M_PI / 2.0;
    const double pitch = wb_inertial_unit_get_roll_pitch_yaw(imu)[1];
    const double yaw = wb_inertial_unit_get_roll_pitch_yaw(imu)[2];
    const double x_pos = wb_gps_get_values(gps)[0];
    const double altitude = wb_gps_get_values(gps)[1];
    const double z_pos = wb_gps_get_values(gps)[2];
    const double roll_acc = wb_gyro_get_values(gyro)[0];
    const double pitch_acc = wb_gyro_get_values(gyro)[1];
    
    double alt_error = fabs(target_altitude - altitude);
    double x_error = fabs(target_x - x_pos);
    double z_error = fabs(target_z - z_pos);
    
    //printf("Altitude: %f | X: %f | Z: %f\n", altitude, x_pos, z_pos);
    printf("Altitude Error: %f | X Error: %f | Z Error: %f | Node Index: %d\n", alt_error, x_error, z_error, node_index);
    
    // Checks to see if positions are within target threshold then updates to next node.
    if (node_index < 999 && fabs(target_altitude - altitude)<thresh && fabs(target_x - x_pos)<thresh && fabs(target_z - z_pos)<thresh) {
      node_index++;
      target_altitude = nodes[node_index][1];
      target_x = nodes[node_index][0];
      target_z = nodes[node_index][2];
    }

    // Process sensor data
    const double clamped_diff_alt = CLAMP(target_altitude - altitude + k_vert_offset, -1.0, 1.0);
    const double clamped_diff_x = CLAMP(x_pos - target_x + x_offset, -1.0, 1.0);
    const double clamped_diff_z = CLAMP(target_z - z_pos + z_offset, -1.0, 1.0);
    const double clamped_diff_yaw = CLAMP(target_yaw - yaw, -1.0, 1.0);
    const double vert_change = k_vert_p * pow(clamped_diff_alt, 3.0);
    const double x_change = 2 * pow(clamped_diff_x, 3.0);
    const double z_change = 2 * pow(clamped_diff_z, 1.0);
    const double roll_change = k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_acc + z_change;
    const double pitch_change = k_pitch_p * CLAMP(pitch, -1.0, 1.0) - pitch_acc + x_change;
    const double yaw_change = 2 * pow(clamped_diff_yaw, 3.0);

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
