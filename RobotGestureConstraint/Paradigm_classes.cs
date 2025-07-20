using iM;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Diagnostics.Eventing.Reader;
using System.Net.Sockets;
using System.Windows.Media.Animation;
using System.Windows.Media.Media3D;
using Windows.Networking.Vpn;
using Windows.Storage.Provider;
using YamlDotNet.Core.Tokens;
using Vector = MathNet.Numerics.LinearAlgebra.Vector<double>;



//       rest: 0,
//       open_hand: 1,
//       power: 2,
//       flex: 3,
//       extend: 4,
//       supination: 5,
//       pronation: 6,
//       radial: 7,
//       ulnar: 8

public class paradigm_calculation
{
    /// <summary>
    /// get axis group based on arm position and gesture
    /// </summary>
    /// <param name="gesture">gesture performed in int</param>
    /// <param name="angle">angle between device and gravity</param>
    /// <param name="group1">a list of two angles. all angles in between the two choose for group 1</param>
    /// <param name="group2">a list of two angles. all angles in between the two choose for group 2</param>
    /// <returns></returns>
    /// 
    public (double qw, double qx, double qy, double qz) EulerToQuaternion(double roll, double pitch, double yaw)
    {
        // Assuming roll (phi), pitch (theta), yaw (psi)
        double cy = Math.Cos(yaw * 0.5);
        double sy = Math.Sin(yaw * 0.5);
        double cp = Math.Cos(pitch * 0.5);
        double sp = Math.Sin(pitch * 0.5);
        double cr = Math.Cos(roll * 0.5);
        double sr = Math.Sin(roll * 0.5);

        double qw = cr * cp * cy + sr * sp * sy;
        double qx = sr * cp * cy - cr * sp * sy;
        double qy = cr * sp * cy + sr * cp * sy;
        double qz = cr * cp * sy - sr * sp * cy;

        return (qw, qx, qy, qz);
    }
    public int GetAxisGroup(int gesture, double angle, double[] group1, double[] group2, double[] group3)
    {
        if (gesture != 2)
            return 0;

        if (angle > group1[0] && angle <= group1[1])
            return 1;
        else if (angle > group2[0] && angle <= group2[1])
            return 2;
        else if (angle > group3[0] && angle <= group3[1])
            return 3;

        return 0;
    }


    public Axis ToggleAxis(Axis axisCurrent, int axisGroup, List<Axis> axisBase, List<Axis> axisCarm)
    {
        List<Axis> axisList;

        if (axisGroup == 1)
        {
            axisList = axisBase;
        }
        else
        {
            axisList = axisCarm;
        }

        int currentIndex = axisList.IndexOf(axisCurrent);

        currentIndex = (currentIndex + 1) % axisList.Count;
        return axisList[currentIndex];
    }
    /// <summary>
    /// maps the gesture to a specific axis given the axis group
    /// </summary>
    /// <param name="gesture">performed gesture in int  rest: 0, open_hand: 1, power: 2, flex: 3, extend: 4, supination: 5, pronation: 6, radial: 7, ulnar: 8 </param>
    /// <param name="axisGroup"> int describes group of axes 1 or 2</param>
    /// <returns></returns>
    public Axis GetAxisFromAngle(double angle, Axis[] axisList)
    {
        if (axisList == null || axisList.Length == 0)
            return Axis.None;

        // Normalize angle to [0, 180)
        angle = Math.Max(0, Math.Min(angle, 179.999));

        double segmentSize = 180.0 / axisList.Length;
        int rawIndex = (int)(angle / segmentSize);

        // Clamp index to valid range
        int index = Math.Max(0, Math.Min(rawIndex, axisList.Length - 1));

        return axisList[index];
    }


    public Axis GetAxis(int gesture, int axisGroup)
    {
        if (gesture == 0 || gesture == 2)
        {
            return Axis.None;
        }

        if (axisGroup == 1)
        {
            if (gesture == 3)
            {
                return Axis.X;
            }
            else if (gesture == 4)
            {
                return Axis.Y;
            }
            else if (gesture == 5)
            {
                return Axis.Z;
            }
        }

        if (axisGroup == 2)
        {
            if (gesture == 3)
            {
                return Axis.Phi;
            }
            else if (gesture == 4)
            {
                return Axis.Theta;
            }
            else if (gesture == 5)
            {
                return Axis.Psi;
            }
        }

        return Axis.None;
    }


    /// <summary>
    /// calculates the relative speed and direction for given angles
    /// </summary>
    /// <param name="angle">current angle</param>
    /// <param name="angle_zero"> reference angle zero</param>
    /// <param name="speed_per_step">speed gets incremented per 10 degrees this is the speed that gets added</param>
    /// <returns></returns>
    public double GetSpeed(double angle, double angle_zero, double speed_per_step)
    {
        double difference = (angle - angle_zero);

        // round on decades to archive control in 10 steps
        // steps of 10 degrees. 0.05 more
        double speed = Math.Round(difference / 10);
        speed = speed * speed_per_step;

        return speed;
    }

    public double GetSpeedStatic(double angle, double angle_zero, double speed_per_step)
    {
        double speed = 0;
        double difference = Math.Round(angle - 90, 2);
        speed = difference * 0.004;
        if (difference > 45)
        {
            speed = 0.18;
        }
        else if (difference < -45)
        {
            speed = -0.18;
        }
        else { speed = difference * 0.004; }


        // round on decades to archive control in 10 steps
        // steps of 10 degrees. 0.05 more
        //double speed = Math.Round(difference / 10);
        //speed = speed * speed_per_step;
        //if(angle <=30 | angle <= -30) { speed = speed * 1.5; }


        return speed;
    }


    /// <summary>
    /// construct a motion vector with the speed values of type [x,y,z,roll,pitch,yaw] with zeros for all values exept given axis
    /// </summary>
    /// <param name="speed">speed for axis</param>
    /// <param name="axis">axis to set the speed at</param>
    /// <returns></returns>
    public Vector MotionVector(double speed, Axis axis)
    {
        Vector output = Vector.Build.Dense(6, 0.0);
        if (axis != Axis.None)
        {
            output[(int)axis - 1] = speed;
        }
        return output;
    }
    public Vector MotionVectorCurve(double speed, Axis axis)
    {
        Vector output = Vector.Build.Dense(6, 0.0);
        if (axis != Axis.None)
        {
            if (axis == Axis.Psi)
            {
                //if (speed <= 0) { output[(int)axis - 1] = -1; }
                //else { output[(int)axis - 1] = 1; }
                output[(int)axis - 1] = speed;
                output[0] = speed;
            }
            else
            {
                output[(int)axis - 1] = speed;
            }
        }
        return output;
    }
}

