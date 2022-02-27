/// <summary>
/// Telemetry message model.
/// </summary>
using System.Collections.Generic;

[System.Serializable]
public class TelemetryModel
{
    public int msg_num;
    public string msg_type;
    public string timestamp;
    public string source;
    public string state;

    // zed: camera
    public string camera;
    // TODO: Depth map

    // zed: Position Tracking API
    public float pos_x;
    public float pos_y;
    public float pos_z;

    // zed: IMU - pose
    public float heading;
    public float roll;
    public float pitch;

    // zed: IMU - linear_acceleration
    public float linear_accel_x;
    public float linear_accel_y;
    public float linear_accel_z;

    // zed: IMU - angular_velocity
    public float ang_vel_x;
    public float ang_vel_y;
    public float ang_vel_z;

    // calculated?
    public float vel_x;
    public float vel_y;
    public float vel_z;
}