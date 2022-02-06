/*
 * File:          cvtest2_c_vscode.c
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
#include <webots/camera.h>
#include <webots/motor.h>
#include <stdio.h>

//edit from hiruni
#define TIME_STEP 32

int main()
{
  wb_robot_init();
  // Initialize camera
  WbDeviceTag camera = wb_robot_get_device("cam");
  wb_camera_enable(camera, TIME_STEP);
  const int width = wb_camera_get_width(camera);
  const int height = wb_camera_get_height(camera);

  while (wb_robot_step(TIME_STEP) != -1)
  {
    //incoming frames become Mat objects
    // Mat frame = Mat(Size(width, height), CV_8UC4);
    const unsigned char *image = wb_camera_get_image(camera);
    printf("%c\n",image[0]);
    
  }
  wb_robot_cleanup();
  return 0;
}