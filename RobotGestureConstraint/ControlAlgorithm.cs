using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Windows.Forms;
using Vector = MathNet.Numerics.LinearAlgebra.Vector<double>;

namespace iM
{
    enum Algorithms
    {
        None,
        DirectControl,
        StepwiseControl,
        Cartesian
    }

    public enum DOAs
    {
        None=0,
        th_flex,
        th_rot,
        index,
        middle,
        ring,
        little,
        wr_flex_ext,
        wr_uln_rad,
        wr_sup_pron,
        x,
        y,
        z,
        gripper,
        finger,
        phi,    // roll
        theta,  // pitch
        psi,
        screw// yaw
    }

    public class ControlAlgorithm : Block
    {
        public cpControlAlgorithm cp = null;
        Algorithms algorithm = Algorithms.None;

        Dictionary<DOAs, double> controlDict = new Dictionary<DOAs, double>();

        double ds_max = 100 / (25);
        double dead_band = 0.3;

        private DOAs axisLocked = DOAs.None;
        private int gesture = 0;
        public static double[] inputVec = new double[8];  // Accessible across threads
        // Store latest inputs
        private int currentGesture = 0;
        private Vector currentCartesian = null;
        // Store current orientation angles (radians)
        private double currentRoll = 0.0;   // phi
        private double currentPitch = 0.0;  // theta
        private double currentYaw = 0.0;    // psi
        private readonly double X_MIN = 0.35, X_MAX = 0.65;
        private readonly double Y_MIN = -0.25, Y_MAX = 0.25;
        private readonly double Z_MIN = 0.10, Z_MAX = 0.50;

        private bool IsWithinLimits(double x, double y, double z)
        {
            return (x >= X_MIN && x <= X_MAX &&
                    y >= Y_MIN && y <= Y_MAX &&
                    z >= Z_MIN && z <= Z_MAX);
        }

        private void ShowPopup(string message)
        {
            MessageBox.Show(message, "⚠️ Robot Safety Warning",
                MessageBoxButtons.OK, MessageBoxIcon.Warning);
        }

        UdpClient alertListener = new UdpClient(5006);
        Thread alertThread;

        private void StartAlertListener()
        {
            alertThread = new Thread(() =>
            {
                IPEndPoint remoteEP = new IPEndPoint(IPAddress.Any, 0);
                while (true)
                {
                    try
                    {
                        byte[] received = alertListener.Receive(ref remoteEP);
                        string message = Encoding.ASCII.GetString(received);
                        if (message.Contains("collision"))
                        {
                            ShowPopup("⚠️ MuJoCo detected a collision.\nControl is temporarily paused.");
                        }
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine("[AlertListener] Error: " + ex.Message);
                    }
                }
            });
            alertThread.IsBackground = true;
            alertThread.Start();
        }

        public (double qw, double qx, double qy, double qz) EulerToQuaternion(double roll, double pitch, double yaw)
        {
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
        public ControlAlgorithm(string Name, double DesiredRate, List<string> InputCfg, List<string> Params, string Path)
            : base(Name, DesiredRate, InputCfg, Params, Path)
        {
            foreach (var param in Params)
            {
                if (param.Contains("algorithm:"))
                {
                    string _algorithm = param.Split(':')[1];
                    if (!Enum.IsDefined(typeof(Algorithms), _algorithm))
                    {
                        throw new Exception($"Algorithm {_algorithm} is not defined in Enum Algorithms");
                    }
                    Enum.TryParse(_algorithm, out algorithm);
                }
            }
            if (algorithm == Algorithms.None) throw new Exception("No algorithm was provided in YAML");

            switch (algorithm)
            {
                case Algorithms.DirectControl:
                    controlDict.Add(DOAs.th_flex, 0);
                    controlDict.Add(DOAs.th_rot, 0);
                    controlDict.Add(DOAs.index, 0);
                    controlDict.Add(DOAs.middle, 0);
                    controlDict.Add(DOAs.ring, 0);
                    controlDict.Add(DOAs.little, 0);
                    controlDict.Add(DOAs.wr_flex_ext, 50);
                    controlDict.Add(DOAs.wr_uln_rad, 50);
                    controlDict.Add(DOAs.wr_sup_pron, 50);
                    break;

                case Algorithms.StepwiseControl:
                    controlDict.Add(DOAs.th_flex, 0);
                    controlDict.Add(DOAs.th_rot, 0);
                    controlDict.Add(DOAs.index, 0);
                    controlDict.Add(DOAs.middle, 0);
                    controlDict.Add(DOAs.ring, 0);
                    controlDict.Add(DOAs.little, 0);
                    controlDict.Add(DOAs.wr_flex_ext, 50);
                    controlDict.Add(DOAs.wr_uln_rad, 50);
                    controlDict.Add(DOAs.wr_sup_pron, 50);
                    break;

                case Algorithms.Cartesian:
                    controlDict.Add(DOAs.x, 0);
                    controlDict.Add(DOAs.y, 0);
                    controlDict.Add(DOAs.z, 0);
                    controlDict.Add(DOAs.phi, 0);
                    controlDict.Add(DOAs.theta, 0);
                    controlDict.Add(DOAs.psi, 0);
                    break;
            }
        }

        // Now expect **two** inputs: gesture and Cartesian vector
        override public void ConfigureInputs()
        {
            {
                base.ConfigureInputs();

                if (InputBlocks.Count != 2)
                    throw new Exception($"ControlAlgorithm {Name} must have one input block:Cartesian vector.");
            }
        }

        override protected void OnNewInput(Block sender, object value)
        {
            if (!warningListenerStarted)
            {
                StartWarningListener();
                warningListenerStarted = true;
            }
            // Console.WriteLine($"[OnNewInput] sender={sender.Name}, value={value}");
            if (value == null)
            {
                Console.WriteLine("[OnNewInput] Warning: value is null");
                return;
            }

            if (sender == InputBlocks[0]) // Gesture input: expect int (or double cast to int)
            {
                int gestureInput = Convert.ToInt32(value);
                currentGesture = gestureInput;
                //  Console.WriteLine($"[OnNewInput] Gesture input received: {gestureInput}");
                // Update axisLocked based on lock gestures
                switch (currentGesture)
                {
                    case 20: axisLocked = DOAs.x; Console.WriteLine("Axis locked: X"); break;
                    case 21: axisLocked = DOAs.y; Console.WriteLine("Axis locked: Y"); break;
                    case 22: axisLocked = DOAs.z; Console.WriteLine("Axis locked: Z"); break;
                    case 23: axisLocked = DOAs.gripper; Console.WriteLine("Axis locked: Gripper"); break;
                    case 24: axisLocked = DOAs.finger; Console.WriteLine("Axis locked: Finger"); break;
                    case 25: /* axisLocked = DOAs.screw; */ break;
                    case 26: axisLocked = DOAs.phi; Console.WriteLine("Axis locked: Phi"); break;
                    case 27: axisLocked = DOAs.theta; Console.WriteLine("Axis locked: Theta"); break;
                    case 28: axisLocked = DOAs.psi; Console.WriteLine("Axis locked: Psi"); break;
                    case 3: axisLocked = DOAs.None; Console.WriteLine("Axis unlocked"); break;
                    default: //Console.WriteLine($"Gesture {currentGesture} received, no axis lock change."); 
                        break;
                }
                //   Console.WriteLine($"[OnNewInput] axisLocked now: {axisLocked}");
            }
            else if (sender == InputBlocks[1]) // Cartesian control input: expect Vector
            {
                Vector cartesianInput = value as Vector;
                if (cartesianInput == null)
                {
                    //   Console.WriteLine("[OnNewInput] Warning: cannot cast input value to Vector.");
                    return;
                }
                currentCartesian = cartesianInput;
                // Console.WriteLine($"[OnNewInput] Cartesian input received: {cartesianInput}");

                // If no explicit lock, infer axisLocked from vector
                if (axisLocked == DOAs.None)
                {
                    axisLocked = InferAxisFromVector(cartesianInput);
                    //  Console.WriteLine($"[OnNewInput] Inferred axisLocked: {axisLocked}");
                }
            }

            // Only process if Cartesian vector is available
            if (currentCartesian == null)
            {
                //   Console.WriteLine("[OnNewInput] Cartesian input is null, skipping ProcessControl.");
                return;
            }

            // Add debug line here to check axisLocked and gesture before calling ProcessControl
            //  Console.WriteLine($"[Debug] Before ProcessControl: axisLocked={axisLocked}, gesture={currentGesture}");
            // Process control logic now with latest gesture + cartesian vector
            ProcessControl(currentCartesian, currentGesture);
        }

        // Helper method to infer axis locked based on vector values
        private DOAs InferAxisFromVector(Vector prediction)
        {
            double threshold = 1e-5;

            if (Math.Abs(prediction[0]) > threshold) return DOAs.x;
            if (Math.Abs(prediction[1]) > threshold) return DOAs.y;
            if (Math.Abs(prediction[2]) > threshold) return DOAs.z;
            if (Math.Abs(prediction[3]) > threshold) return DOAs.phi;
            if (Math.Abs(prediction[4]) > threshold) return DOAs.theta;
            if (Math.Abs(prediction[5]) > threshold) return DOAs.psi;

            return DOAs.None;
        }

        private void OnNewPoseReceived(double[] receivedPose)
        {
            if (receivedPose.Length >= 8)
            {
                lock (inputVec)  // Optional thread safety
                {
                    Array.Copy(receivedPose, inputVec, 8);
                }
            }
        }
        // State variables
        private double startX = 0, startY = 0, startZ = 0;
        private double deltaX = 0, deltaY = 0, deltaZ = 0;

        private double startRoll = 0, startPitch = 0, startYaw = 0;
        private double deltaRoll = 0, deltaPitch = 0, deltaYaw = 0;
        private UdpClient warningListener;
        private Thread warningThread;
        private bool warningListenerStarted = false;

        private void StartWarningListener()
        {
            try
            {
                warningListener = new UdpClient(5006);
                warningThread = new Thread(() =>
                {
                    while (true)
                    {
                        try
                        {
                            IPEndPoint remoteEP = new IPEndPoint(IPAddress.Any, 0);
                            byte[] data = warningListener.Receive(ref remoteEP);
                            string msg = Encoding.UTF8.GetString(data);

                            if (msg == "LIMIT_EXCEEDED")
                            {
                                Console.WriteLine("[⚠️ WARNING RECEIVED] " + msg);

                                // Show MessageBox safely from UI thread
                                if (Application.OpenForms.Count > 0)
                                {
                                    Form mainForm = Application.OpenForms[0];
                                    mainForm.Invoke((MethodInvoker)delegate
                                    {
                                        if (cp != null)
                                        {
                                            cp.ShowWarningPopup("⚠️ Movement command is beyond safe limits!");
                                        }
                                        else
                                        {
                                            MessageBox.Show("⚠️ Movement command is beyond safe limits!",
                                                            "Safety Warning",
                                                            MessageBoxButtons.OK,
                                                            MessageBoxIcon.Warning);
                                        }
                                    });
                                }
                                else
                                {
                                    MessageBox.Show("⚠️ Movement command is beyond safe limits!",
                                                    "Safety Warning",
                                                    MessageBoxButtons.OK,
                                                    MessageBoxIcon.Warning);
                                }
                            }
                        }
                        catch (SocketException ex)
                        {
                            Console.WriteLine("[UDP WarningListener Error] " + ex.Message);
                            break;
                        }
                    }
                });

                warningThread.IsBackground = true;
                warningThread.Start();
                Console.WriteLine("[UDP] Warning listener started on port 5006.");
            }
            catch (Exception ex)
            {
                Console.WriteLine("[StartWarningListener ERROR] " + ex.Message);
            }
        }
        

        
        private void ProcessControl(Vector Prediction, int gesture)
        {
            // Console.WriteLine($"[ProcessControl] Called with gesture={gesture}, axisLocked={axisLocked}");

            // 🛑 Skip rest gesture
            if (gesture == 0)
            {
                //   Console.WriteLine("🛑 Gesture 0 — holding current desired pose");
                return;
            }

            if (algorithm == Algorithms.Cartesian)
            {
                double step = (axisLocked == DOAs.phi || axisLocked == DOAs.theta || axisLocked == DOAs.psi) ? 0.01 : 0.0008;

                if (axisLocked != DOAs.None)
                {
                    // 🔁 Update only the delta (not the start)
                    if (axisLocked == DOAs.screw)
                    {
                        double dz = 0.0003;  // Z-axis step
                        double dr = 0.03;    // Rotation step

                        if (gesture == 4) // screw
                        {
                            deltaZ = Math.Max(deltaZ - dz, -0.05);     // limit downward motion
                            deltaYaw += dr;
                        }
                        else if (gesture == 5) // unscrew
                        {
                            deltaZ = Math.Min(deltaZ + dz, 0.05);      // limit upward motion
                            deltaYaw -= dr;
                        }
                    }
                    else
                    {
                        if (gesture == 4) // positive
                        {
                            if (axisLocked == DOAs.x) deltaX = Math.Min(deltaX + step, 1.0);
                            else if (axisLocked == DOAs.y) deltaY = Math.Min(deltaY + step, 1.0);
                            else if (axisLocked == DOAs.z) deltaZ = Math.Min(deltaZ + step, 1.0);
                            else if (axisLocked == DOAs.phi) deltaRoll += step;
                            else if (axisLocked == DOAs.theta) deltaPitch += step;
                            else if (axisLocked == DOAs.psi) deltaYaw += step;
                        }
                        else if (gesture == 5) // negative
                        {
                            if (axisLocked == DOAs.x) deltaX = Math.Max(deltaX - step, -1.0);
                            else if (axisLocked == DOAs.y) deltaY = Math.Max(deltaY - step, -1.0);
                            else if (axisLocked == DOAs.z) deltaZ = Math.Max(deltaZ - step, -1.0);
                            else if (axisLocked == DOAs.phi) deltaRoll -= step;
                            else if (axisLocked == DOAs.theta) deltaPitch -= step;
                            else if (axisLocked == DOAs.psi) deltaYaw -= step;
                        }
                    }

                    // ✅ Calculate desired pose = start + delta
                    double desiredX = startX + deltaX;
                    double desiredY = startY + deltaY;
                    double desiredZ = startZ + deltaZ;

                    double desiredRoll = NormalizeAngle(startRoll + deltaRoll);
                    double desiredPitch = NormalizeAngle(startPitch + deltaPitch);
                    double desiredYaw = NormalizeAngle(startYaw + deltaYaw);

                    var q = EulerToQuaternion(desiredRoll, desiredPitch, desiredYaw);

                    double[] outputPose = new double[] { desiredX, desiredY, desiredZ, q.qx, q.qy, q.qz, q.qw };
                    SendOutput(outputPose);

                    //  Console.WriteLine($"[ControlAlgorithm Output] Pos: ({desiredX:F3}, {desiredY:F3}, {desiredZ:F3}), Q: ({q.qw:F3}, {q.qx:F3}, {q.qy:F3}, {q.qz:F3})");
                }
                else
                {
                    // ✳️ DO NOT reset to Prediction — instead, keep current desired pose as new start
                    startX = startX + deltaX;
                    startY = startY + deltaY;
                    startZ = startZ + deltaZ;

                    startRoll = NormalizeAngle(startRoll + deltaRoll);
                    startPitch = NormalizeAngle(startPitch + deltaPitch);
                    startYaw = NormalizeAngle(startYaw + deltaYaw);

                    // 🔁 Reset deltas only
                    deltaX = deltaY = deltaZ = 0;
                    deltaRoll = deltaPitch = deltaYaw = 0;

                    var q = EulerToQuaternion(startRoll, startPitch, startYaw);

                    double[] outputPose = new double[] { startX, startY, startZ, q.qx, q.qy, q.qz, q.qw };
                    SendOutput(outputPose);

                    //  Console.WriteLine($"[Lock Cleared] Holding pose = ({startX:F3}, {startY:F3}, {startZ:F3}), Q = ({q.qw:F3}, {q.qx:F3}, {q.qy:F3}, {q.qz:F3})");
                }

            }
        }



        private double NormalizeAngle(double angle)
        {
            while (angle > Math.PI) angle -= 2 * Math.PI;
            while (angle < -Math.PI) angle += 2 * Math.PI;
            return angle;
        }



        private void SendOutput(double[] pose)
        {
            if (pose == null || pose.Length != 7)
            {
                Console.WriteLine("[SendOutput] Invalid pose array length. Expected 7 doubles.");
                return;
            }

            try
            {
                // (1) Convert to vector for Logger and other blocks
                var outputVector = Vector<double>.Build.DenseOfArray(pose);

                // (2) Send to downstream blocks (e.g., Logger, Joiner, etc.)
                base.SendOutput(outputVector);  // ✅ THIS IS WHAT Logger NEEDS

                // (3) Send over UDP
                using (UdpClient udpClient = new UdpClient())
                {
                    byte[] buffer = new byte[7 * sizeof(double)];

                    for (int i = 0; i < 7; i++)
                    {
                        byte[] bytes = BitConverter.GetBytes(pose[i]);
                        Array.Copy(bytes, 0, buffer, i * sizeof(double), sizeof(double));
                    }

                    udpClient.Send(buffer, buffer.Length, "127.0.0.1", 5005);
                }

                //   Console.WriteLine("[SendOutput] Sent pose: " + string.Join(", ", pose));
            }
            catch (Exception ex)
            {
                //Console.WriteLine("[SendOutput] Error sending UDP packet: " + ex.Message);
            }
        }

        public void SendMotionCommand(DOAs axis, double velocity)
        {
            if (controlDict.ContainsKey(axis))
            {
                controlDict[axis] = velocity;
                //   Console.WriteLine($"[SendMotionCommand] Set {axis} velocity to {velocity:F3}");

                // Prepare 7-element pose array for sending
                double[] pose = new double[7];

                // Position (x, y, z)
                pose[0] = controlDict.ContainsKey(DOAs.x) ? controlDict[DOAs.x] : 0.0;
                pose[1] = controlDict.ContainsKey(DOAs.y) ? controlDict[DOAs.y] : 0.0;
                pose[2] = controlDict.ContainsKey(DOAs.z) ? controlDict[DOAs.z] : 0.0;

                // For orientation quaternion, if you already track currentRoll/Pitch/Yaw, convert here:
                var q = EulerToQuaternion(currentRoll, currentPitch, currentYaw);
                pose[3] = q.qw;
                pose[4] = q.qx;
                pose[5] = q.qy;
                pose[6] = q.qz;

                // Send pose array
                SendOutput(pose);
            }
            else
            {
                //Console.WriteLine($"[SendMotionCommand] Axis {axis} not found in controlDict");
            }
        }

    }
}
