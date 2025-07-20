using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using Vector = MathNet.Numerics.LinearAlgebra.Vector<double>;
using System.Numerics;
using System.Threading;
using MathNet.Numerics.LinearAlgebra.Factorization;
using System.Threading.Tasks;

namespace iM
{
    public partial class cpPanda : ControlPanel
    {
        Panda _SourceBlock;
        static List<Thread> threads = new List<Thread>();

        public cpPanda(Panda _SourceBlock) : base(_SourceBlock)
        {
            InitializeComponent();
            this._SourceBlock = _SourceBlock;
        }

        override protected void cpRefresh(object myObject, EventArgs myEventArgs)
        {
            // Disconnect robot when in reflex
            if (_SourceBlock.robotIsInReflex)
            {
                cbConnectDisconnect.Checked = false;
                cbConnectDisconnect.Enabled = false;

                try
                {
                    // Wait 5 seconds for the robot to overcome the reflex before reenabling connect
                    Thread thread = new Thread(() =>
                    {
                        Thread.Sleep(5000);
                        _SourceBlock.robotIsInReflex = false;
                    });
                    cbConnectDisconnect.Enabled = true;
                    thread.Start();
                }
                catch { }

            }
        }

        private void cpFranka_Load(object sender, EventArgs e)
        {
            cbReadOnce.Enabled = false;
            btnHome.Enabled = false;
            btnStartControl.Enabled = false;
            btnStop.Enabled = false;
            ListControlTypes.Enabled = false;
        }

        private void cbConnectDisconnect_CheckedChanged(object sender, EventArgs e)
        {
            if (cbConnectDisconnect.Checked)
            {
                try
                {
                    Console.WriteLine($"[DEBUG] cbConnectDisconnect_CheckedChanged triggered.");
                    Console.WriteLine($"[DEBUG] Attempting to connect to robot at IP: {_SourceBlock.IP}");
                    _SourceBlock.robot = Panda.createRobot(_SourceBlock.IP);
                    label2.Text = "IP " + _SourceBlock.IP;

                    // Enable GUI elements
                    cbReadOnce.Enabled = true;
                    btnHome.Enabled = true;
                    btnStop.Enabled = true;
                    ListControlTypes.Enabled = true;
                    cbConnectDisconnect.Text = "Disconnect";
                }
                catch
                {
                    Console.WriteLine("Connection problem");
                }
            }
            else
            {
                try
                {
                    // Stop and remove robot
                    Panda.stopRobot(_SourceBlock.robot);
                    Thread.Sleep(100); // sorry. would be better to wait on the robot to stop before deleting it 
                    Panda.deleteRobot(_SourceBlock.robot);

                    // Disnable GUI elements
                    cbReadOnce.Enabled = false;
                    btnHome.Enabled = false;
                    btnStartControl.Enabled = false;
                    btnStop.Enabled = false;
                    ListControlTypes.Enabled = false;
                    cbConnectDisconnect.Text = "Connect";
                }
                catch
                {
                    Console.WriteLine("Disconnect problem");
                }
            }
        }
        private Thread controlThread;
        private bool keepRunning = true;
        private double[] lastTorqueCommand = new double[7];  // For 7 DOF
        private object lockObj = new object();
        private void cbReadOnce_CheckedChanged(object sender, EventArgs e)
        {
            Panda.readOnce(_SourceBlock.robot, ref _SourceBlock.state, true);
        }

        private void btnHome_Click(object sender, EventArgs e)
        {
            Panda.stopRobot(_SourceBlock.robot);
            Thread thread = new Thread(() => Panda.homeRobot(_SourceBlock.robot));
            thread.Start();
            threads.Add(thread);
        }

        private void ListControlTypes_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (ListControlTypes.SelectedIndex == -1)
            {
                btnStartControl.Enabled = false;
            }
            else
            {
                _SourceBlock.activeControl = (string)ListControlTypes.SelectedItems[0];
                btnStartControl.Enabled = true;
            }
        }

        private void btnStop_Click(object sender, EventArgs e)
        {
            Panda.stopRobot(_SourceBlock.robot);
            cbReadOnce.Enabled = true;
            btnHome.Enabled = true;
            ListControlTypes.ClearSelected();
        }

        private void btnStartControl_Click(object sender, EventArgs e)
        {
            cbReadOnce.Enabled = false;
            btnHome.Enabled = false;
            btnStartControl.Enabled = false;
            _SourceBlock.startController();
        }
    }

    public class Panda : Block
    {
        public Panda(string Name, double DesiredRate, List<string> InputCfg, List<string> Params, string Path)
            : base(Name, DesiredRate, InputCfg, Params, Path) { }

        public IntPtr robot;
        public bool robotIsConnected = false;
        public bool controlIsActive = false;
        public bool robotIsInReflex = false;
        public bool robotStopped = false;
        public bool force_feedback_active = false;

        public SimpleState state = new SimpleState();
        public string activeControl;
        public string IP;

        private double[] orientation1;
        private double[] orientation2;
        private double[] orientation_d1;
        private double[] orientation_d2;

        private Vector inputVec;

        override public void ConfigureInputs()
        {
            base.ConfigureInputs();

            // a Myo has one input only, and it must be a timer
            if (InputBlocks.Count != 1) throw new Exception($"Panda {Name} must have one input block only.");

            // a Myo's DesiredRate is the same as the driving Timer
            DesiredRate = InputBlocks[0].DesiredRate;
            cp = new cpPanda(this);

            IP = Params[0];

            orientation1 = new double[4];
            orientation2 = new double[4];

            orientation_d1 = new double[4];
            orientation_d2 = new double[4];
        }
        override protected void OnNewInput(Block sender, object value)
        {
            inputVec = value as Vector;
        }


        #region DLL Imports and marshalling
        public const string DLL_IMPORT_PATH = @"D:\Hafiz_intuitiveRobotControl\master-thesis-mehrkens-florian-devel\libfranka\build\Release\franka.dll";
        [DllImport(DLL_IMPORT_PATH)]
        public static extern IntPtr createRobot(string robot_address);
        [DllImport(DLL_IMPORT_PATH)]
        public static extern void deleteRobot(IntPtr pRobot);
        [DllImport(DLL_IMPORT_PATH)]
        public static extern void stopRobot(IntPtr pRobot);
        [DllImport(DLL_IMPORT_PATH, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void readOnce(IntPtr pRobot, ref SimpleState pState, bool print);

        delegate bool readCallback(SimpleState robot_state);
        [DllImport(DLL_IMPORT_PATH, CallingConvention = CallingConvention.Cdecl)]
        static extern void readRobot(IntPtr pRobot, [MarshalAs(UnmanagedType.FunctionPtr)] readCallback read_callback);
        [DllImport(DLL_IMPORT_PATH)]
        public static extern void getModel(IntPtr pRobot, ref SimpleModel model);
        [DllImport(DLL_IMPORT_PATH)]
        public static extern IntPtr createMotionGenerator(double speed_factor, DoubleArray7 q_goal);
        [DllImport(DLL_IMPORT_PATH)]
        public static extern void setDefaultBehavior(IntPtr pRobot);

        [DllImport(DLL_IMPORT_PATH)]
        public static extern void controlRobot(IntPtr pRobot, IntPtr pMotionGenerator);

        [DllImport(DLL_IMPORT_PATH)]
        public static extern bool homeRobot(IntPtr pRobot);

        [DllImport(DLL_IMPORT_PATH)]
        public static extern void setGuidingMode(IntPtr pRobot, bool elbow);

        [DllImport(DLL_IMPORT_PATH)]
        public static extern void setZeroTorque(IntPtr pRobot);

        [DllImport(DLL_IMPORT_PATH)]
        public static extern IntPtr setCartesianImpedance(IntPtr pRobot, DoubleArray6 k_theta);

        [DllImport(DLL_IMPORT_PATH)]
        public static extern IntPtr setJointImpedance(IntPtr pRobot, DoubleArray7 k_theta);

        [DllImport(DLL_IMPORT_PATH, CallingConvention = CallingConvention.Cdecl)]
        public static extern void setCollisionBehavior(IntPtr pRobot,
            DoubleArray7 lower_torque_thresholds_acceleration,
            DoubleArray7 upper_torque_thresholds_acceleration,
            DoubleArray7 lower_torque_thresholds_nominal,
            DoubleArray7 upper_torque_thresholds_nominal,
            DoubleArray6 lower_force_thresholds_acceleration,
            DoubleArray6 upper_force_thresholds_acceleration,
            DoubleArray6 lower_force_thresholds_nominal,
            DoubleArray6 upper_force_thresholds_nominal);

        public delegate void simpleCallback(ref SimpleState simple_state, ulong miliseconds);
        [DllImport(DLL_IMPORT_PATH, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void positionControl(IntPtr pRobot, ref SimpleState pState, [MarshalAs(UnmanagedType.FunctionPtr)] simpleCallback simple_callback);
        public delegate void impedanceCallback(ref SimpleState simple_state, ref SimpleModel model, ulong miliseconds);
        [DllImport(DLL_IMPORT_PATH, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void impedanceControl(IntPtr pRobot, ref SimpleState pState, ref SimpleModel pModel, [MarshalAs(UnmanagedType.FunctionPtr)] impedanceCallback callback);
        public delegate void torqueCallback(ref SimpleState simple_state, ulong miliseconds);
        [DllImport(DLL_IMPORT_PATH, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void torqueControl(IntPtr pRobot, ref SimpleState pState, [MarshalAs(UnmanagedType.FunctionPtr)] torqueCallback callback);


        public struct DoubleArray7
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 7)]
            public double[] data;
        }

        public struct DoubleArray6
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
            public double[] data;
        }
        public struct DoubleArray42
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 42)]
            public double[] data;
        }

        public class MotionGenerator
        {
            private readonly double[] q_goal_;
            private readonly double[] dq_max_;
            private readonly double[] ddq_max_start_;
            private readonly double[] ddq_max_goal_;
            private double[] q_start_;
            private double[] delta_q_;
            private double[] dq_max_sync_;
            private double[] t_1_sync_;
            private double[] t_2_sync_;
            private double[] t_f_sync_;
            private double[] q_1_;
            private double time_;

            public MotionGenerator(double speed_factor, double[] q_goal)
            {
                q_goal_ = q_goal;
                dq_max_ = new double[] { 2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5 };
                ddq_max_start_ = new double[] { 5, 5, 5, 5, 5, 5, 5 };
                ddq_max_goal_ = new double[] { 5, 5, 5, 5, 5, 5, 5 };

                for (int i = 0; i < dq_max_.Length; i++)
                {
                    dq_max_[i] *= speed_factor;
                    ddq_max_start_[i] *= speed_factor;
                    ddq_max_goal_[i] *= speed_factor;
                }

                dq_max_sync_ = new double[7];
                q_start_ = new double[7];
                delta_q_ = new double[7];
                t_1_sync_ = new double[7];
                t_2_sync_ = new double[7];
                t_f_sync_ = new double[7];
                q_1_ = new double[7];
            }
        }
        #endregion
        #region Robot controls
        public void startController()
        {
            Thread thread;
            stopRobot(robot);

            switch (activeControl)
            {
                case "Cartesian Teleimpedance":
                    if (inputVec != null & inputVec.Count >= 13)
                    {
                        setHardCollisionBehavior(robot);
                        thread = new Thread(() => impedanceControl(robot));
                        thread.Priority = ThreadPriority.Highest;
                        thread.Start();
                    }
                    else
                    {
                        Console.WriteLine("Warning: wrong size for input vector. Failed to start Cartesian Teleimpedance control");
                    }
                    break;
                case "Cartesian Impedance":
                    if (inputVec != null & inputVec.Count >= 7)
                    {
                        Console.WriteLine("[DEBUG] Starting Cartesian Impedance control...");
                        Console.WriteLine("[DEBUG] inputVec Count: " + inputVec.Count);
                        Console.WriteLine("[DEBUG] inputVec Contents: " + string.Join(", ", inputVec));

                        setHardCollisionBehavior(robot);

                        thread = new Thread(() => impedanceControl(robot));
                        thread.Priority = ThreadPriority.Highest;
                        thread.Start();
                    }
                    else
                    {
                        Console.WriteLine("[ERROR] Invalid inputVec! Expected at least 7 elements.");
                        if (inputVec == null)
                        {
                            Console.WriteLine("[ERROR] inputVec is null.");
                        }
                        else
                        {
                            Console.WriteLine("[ERROR] inputVec Count = " + inputVec.Count);
                            Console.WriteLine("[ERROR] inputVec Contents: " + string.Join(", ", inputVec));
                        }

                        Console.WriteLine("Warning: wrong size for input vector. Failed to start Cartesian Impedance control.");
                    }
                    break;
                case "Gravity compensation":
                    setHardCollisionBehavior(robot);
                    thread = new Thread(() => gravityCompensation(robot));
                    thread.Priority = ThreadPriority.Highest;
                    thread.Start();
                    break;
                default:
                    Console.WriteLine("Warning: Could not start control mode. No control mode selected.");
                    break;
            }
        }
        public static void setHardCollisionBehavior(IntPtr pRobot)
        {
            Panda.DoubleArray7 lower_torque_thresholds_acceleration = new Panda.DoubleArray7();
            Panda.DoubleArray7 upper_torque_thresholds_acceleration = new Panda.DoubleArray7();
            Panda.DoubleArray7 lower_torque_thresholds_nominal = new Panda.DoubleArray7();
            Panda.DoubleArray7 upper_torque_thresholds_nominal = new Panda.DoubleArray7();
            Panda.DoubleArray6 lower_force_thresholds_acceleration = new Panda.DoubleArray6();
            Panda.DoubleArray6 upper_force_thresholds_acceleration = new Panda.DoubleArray6();
            Panda.DoubleArray6 lower_force_thresholds_nominal = new Panda.DoubleArray6();
            Panda.DoubleArray6 upper_force_thresholds_nominal = new Panda.DoubleArray6();
            lower_torque_thresholds_acceleration.data = new double[] { 1000, 1000, 1000, 1000, 1000, 1000, 1000 };
            upper_torque_thresholds_acceleration.data = new double[] { 1000, 1000, 1000, 1000, 1000, 1000, 1000 };
            lower_torque_thresholds_nominal.data = new double[] { 1000, 1000, 1000, 1000, 1000, 1000, 1000 };
            upper_torque_thresholds_nominal.data = new double[] { 1000, 1000, 1000, 1000, 1000, 1000, 1000 };
            lower_force_thresholds_acceleration.data = new double[] { 1000, 1000, 1000, 1000, 1000, 1000 };
            upper_force_thresholds_acceleration.data = new double[] { 1000, 1000, 1000, 1000, 1000, 1000 };
            lower_force_thresholds_nominal.data = new double[] { 1000, 1000, 1000, 1000, 1000, 1000 };
            upper_force_thresholds_nominal.data = new double[] { 1000, 1000, 1000, 1000, 1000, 1000 };

            Panda.setCollisionBehavior(pRobot,
                                        lower_torque_thresholds_acceleration,
                                        upper_torque_thresholds_acceleration,
                                        lower_torque_thresholds_nominal,
                                        upper_torque_thresholds_nominal,
                                        lower_force_thresholds_acceleration,
                                        upper_force_thresholds_acceleration,
                                        lower_force_thresholds_nominal,
                                        upper_force_thresholds_nominal);
        }

        public void gravityCompensation(IntPtr pRobot)
        {
            try
            {
                double[] zeros = { 0, 0, 0, 0, 0, 0, 0 };

                void callback(ref SimpleState robot_state, ulong miliseconds)
                {
                    // command zero torque to robot
                    robot_state.tau_J_d = zeros;

                    // iM Blocks data logging
                    Vector w_measured = Vector.Build.DenseOfArray(robot_state.O_F_ext_hat_K);
                    Matrix4x4 transform = Create4x4FromArray(robot_state.O_T_EE);
                    Quaternion orientation = Quaternion.CreateFromRotationMatrix(transform);

                    Vector robot_data = Vector.Build.Dense(13);
                    robot_data[0] = w_measured[0];
                    robot_data[1] = w_measured[1];
                    robot_data[2] = w_measured[2];
                    robot_data[3] = w_measured[3];
                    robot_data[4] = w_measured[4];
                    robot_data[5] = w_measured[5];
                    robot_data[6] = transform.Translation.X;
                    robot_data[7] = transform.Translation.Y;
                    robot_data[8] = transform.Translation.Z;
                    robot_data[9] = orientation.X;
                    robot_data[10] = orientation.Y;
                    robot_data[11] = orientation.Z;
                    robot_data[12] = orientation.W;

                    SendOutput(robot_data); // iM Blocks export
                }

                controlIsActive = true;
                torqueControl(pRobot, ref state, callback);
                controlIsActive = false;

                readOnce(pRobot, ref state, false);
                if (state.robot_mode.Equals(RobotMode.kReflex))
                {
                    robotIsInReflex = true;
                }
            }
            catch (Exception e)
            {
                Console.WriteLine(e.Message);
            }
        }

        public void impedanceControl(IntPtr pRobot)
        {
            double[] received_doubles = state.q;

            // Compliance parameters
            double translationalStiffness = 1500; //2000
            double rotationalStiffness = 45; // 50
            Matrix<double> stiffness = Matrix<double>.Build.Dense(6, 6, 0.0);
            Matrix<double> damping = Matrix<double>.Build.Dense(6, 6, 0.0);

            var translationalIdentity = translationalStiffness * Matrix<double>.Build.DenseIdentity(3, 3);
            var rotationalIdentity = rotationalStiffness * Matrix<double>.Build.DenseIdentity(3, 3);

            stiffness.SetSubMatrix(0, 3, 0, 3, translationalIdentity);
            stiffness.SetSubMatrix(3, 3, 3, 3, rotationalIdentity);

            var translationalDamping = (0.5 * Math.Sqrt(translationalStiffness) * Matrix<double>.Build.DenseIdentity(3, 3));
            var rotationalDamping = (0.5 * Math.Sqrt(rotationalStiffness) * Matrix<double>.Build.DenseIdentity(3, 3));

            damping.SetSubMatrix(0, 3, 0, 3, translationalDamping);
            damping.SetSubMatrix(3, 3, 3, 3, rotationalDamping);


            try
            {
                readOnce(pRobot, ref state, false);
                double[] zeros = { 0, 0, 0, 0, 0, 0, 0 };

                // equilibrium point is the initial position
                Matrix4x4 initial_transform = Create4x4FromArray(state.O_T_EE);


                Vector3 position_d; // Translation component
                Quaternion orientation_d; // Rotational component

                double[] initial_pose = new double[7];

                double time = 0.0;
                double prev_deltax = 0;
                double prev_deltay = 0;
                double prev_deltaz = 0;

                Matrix4x4 transform_master = Create4x4FromArray(state.O_T_EE);
                Vector3 position_master = transform_master.Translation;
                float offsetx = position_master.X;
                float offsety = position_master.Y;
                float offsetz = position_master.Z;

                Quaternion rotation_master;
                Quaternion initial_orientation_follower = Quaternion.CreateFromRotationMatrix(initial_transform);

                Quaternion rot_offset = new Quaternion();

                time = 0;
                double alpha = 0.7;


                // define callback for the torque control loop
                void Impedance_control_callback(ref SimpleState robot_state, ref SimpleModel model, ulong miliseconds)
                {
                    // convert to Eigen
                    Matrix<double> coriolis = ArrayToMatrix<double>(model.coriolis, 7, 1);
                    Matrix<double> jacobian = ArrayToMatrix<double>(model.zeroJacobian, 6, 7);
                    Matrix<double> q = ArrayToMatrix<double>(robot_state.q, 7, 1);
                    Matrix<double> dq = ArrayToMatrix<double>(robot_state.dq, 7, 1);

                    Matrix4x4 transform = Create4x4FromArray(robot_state.O_T_EE);
                    Vector3 position = new Vector3(transform.M41, transform.M42, transform.M43); // Translation component
                    Quaternion orientation = Quaternion.CreateFromRotationMatrix(transform); // Rotational component

                    position_master.X = (float)inputVec[0];
                    position_master.Y = (float)inputVec[1];
                    position_master.Z = (float)inputVec[2];

                    rotation_master.X = (float)inputVec[3];
                    rotation_master.Y = (float)inputVec[4];
                    rotation_master.Z = (float)inputVec[5];

                    rotation_master.W = (float)inputVec[6];

                    if (time == 0)
                    {
                        initial_pose = state.O_T_EE;

                        prev_deltax = initial_pose[12];
                        prev_deltay = initial_pose[13];
                        prev_deltaz = initial_pose[14];


                        offsetx = position_master.X;
                        offsety = position_master.Y;
                        offsetz = position_master.Z;

                        rot_offset = new Quaternion(rotation_master.X, rotation_master.Y, rotation_master.Z, rotation_master.W);

                        Console.WriteLine("Initials set");

                    }

                    time += 1;

                    float pos_x = (float)(((position_master.X - offsetx) + initial_pose[12]) * alpha + (1 - alpha) * (prev_deltax));
                    float pos_y = (float)(((position_master.Y - offsety) + initial_pose[13]) * alpha + (1 - alpha) * (prev_deltay));
                    float pos_z = (float)(((position_master.Z - offsetz) + initial_pose[14]) * alpha + (1 - alpha) * (prev_deltaz));

                    //float rot_x = (float)(rotation_master.X * alpha + (1 - alpha) * rot_offset.X);
                    //float rot_y = (float)(rotation_master.Y * alpha + (1 - alpha) * rot_offset.Y);
                    //float rot_z = (float)(rotation_master.Z * alpha + (1 - alpha) * rot_offset.Z);
                    //float rot_w = (float)(rotation_master.W * alpha + (1 - alpha) * rot_offset.W);


                    prev_deltax = pos_x;
                    prev_deltay = pos_y;
                    prev_deltaz = pos_z;

                    position_d = new Vector3(pos_x, pos_y, pos_z);

                    Quaternion orient_input = new Quaternion(rotation_master.X, rotation_master.Y, rotation_master.Z, rotation_master.W);

                    //orient_input.W = rot_w;
                    //orient_input.X = rot_x;
                    //orient_input.Y = rot_y;
                    //orient_input.Z = rot_z;

                    orientation_d = orient_input * Quaternion.Inverse(rot_offset);

                    orientation_d = orientation_d * initial_orientation_follower;

                    Quaternion.Normalize(orientation_d);

                    // compute error to desired equilibrium pose
                    // position error
                    Matrix<double> error = Matrix<double>.Build.Dense(6, 1, 0.0);
                    Vector3 error_vector = (position - position_d);

                    error[0, 0] = error_vector.X;
                    error[1, 0] = error_vector.Y;
                    error[2, 0] = error_vector.Z;

                    orientation1[0] = orientation.X;
                    orientation1[1] = orientation.Y;
                    orientation1[2] = orientation.Z;
                    orientation1[3] = orientation.W;

                    avoidJumps(orientation1, orientation2);

                    orientation.X = (float)orientation1[0];
                    orientation.Y = (float)orientation1[1];
                    orientation.Z = (float)orientation1[2];
                    orientation.W = (float)orientation1[3];


                    orientation_d1[0] = orientation_d.X;
                    orientation_d1[1] = orientation_d.Y;
                    orientation_d1[2] = orientation_d.Z;
                    orientation_d1[3] = orientation_d.W;

                    avoidJumps(orientation_d1, orientation_d2);

                    orientation_d.X = (float)orientation_d1[0];
                    orientation_d.Y = (float)orientation_d1[1];
                    orientation_d.Z = (float)orientation_d1[2];
                    orientation_d.W = (float)orientation_d1[3];

                    // Ensure that two quaternions are the closest along the path
                    if (Quaternion.Dot(orientation_d, orientation) < 0.0)
                    {
                        orientation = new Quaternion(-orientation.X, -orientation.Y, -orientation.Z, -orientation.W);
                    }



                    // "difference" quaternion
                    Quaternion error_quaternion = Quaternion.Inverse(orientation) * orientation_d;
                    //Quaternion error_quaternion = orient_input;


                    error[3, 0] = error_quaternion.X;
                    error[4, 0] = error_quaternion.Y;
                    error[5, 0] = error_quaternion.Z;


                    // Transform to base frame
                    Matrix<double> rotationMatrix = Matrix<double>.Build.DenseOfArray(new double[,] {
                                { transform.M11, transform.M21, transform.M31 },
                                { transform.M12, transform.M22, transform.M32 },
                                { transform.M13, transform.M23, transform.M33 }
                            });
                    error.SetSubMatrix(3, 3, 0, 1, (-rotationMatrix * error.SubMatrix(3, 3, 0, 1)));

                    //Console.WriteLine(error_quaternion.X + "||" + error_quaternion.Y + "||" + error_quaternion.Z + "||" + error[3, 0].ToString() + "||" + error[4, 0].ToString() + "||" + error[5, 0].ToString() + "|");

                    // compute control
                    var tau_task = Vector<double>.Build.Dense(7);
                    var tau_d = Vector<double>.Build.Dense(7);

                    // Spring damper system with damping ratio=1
                    tau_task = (jacobian.Transpose() * (-stiffness * error - damping * (jacobian * dq))).Column(0);
                    tau_d = tau_task + coriolis.Column(0);
                    

                    Double[] tau_d_array = tau_d.ToArray();

                    robot_state.tau_J_d = tau_d.ToArray();

                    Vector w_measured = Vector.Build.DenseOfArray(robot_state.O_F_ext_hat_K);
                    //Console.WriteLine("[RobotCallback] Received target position: " + position_d);
                    //Console.WriteLine("[RobotCallback] Output torque command: " + string.Join(", ", tau_d_array));

                    Vector robot_data = Vector.Build.Dense(13);
                    robot_data[0] = w_measured[0];
                    robot_data[1] = w_measured[1];
                    robot_data[2] = w_measured[2];
                    robot_data[3] = w_measured[3];
                    robot_data[4] = w_measured[4];
                    robot_data[5] = w_measured[5];

                    robot_data[6] = position.X;
                    robot_data[7] = position.Y;
                    robot_data[8] = position.Z;

                    orientation1[0] = orientation.X;
                    orientation1[1] = orientation.Y;
                    orientation1[2] = orientation.Z;
                    orientation1[3] = orientation.W;

                    avoidJumps(orientation1, orientation2);

                    robot_data[9] = orientation1[0];
                    robot_data[10] = orientation1[1];
                    robot_data[11] = orientation1[2];
                    robot_data[12] = orientation1[3];

                    SendOutput(robot_data);
                };

                SimpleModel simple_model = new SimpleModel();
                getModel(pRobot, ref simple_model);
                impedanceControl(pRobot, ref state, ref simple_model, Impedance_control_callback);
            }

            catch (Exception e)
            {
                Console.WriteLine(e.Message);
            }
        }

        public void deleteRobotPointer(IntPtr pRobot)
        {
            robot = IntPtr.Zero;
        }
        // function to perform spherical linear interpolation (slerp) between two quaternions
        void slerp(double[] q1, double[] q2, float t, double[] result)
        {
            double dot = 0;
            for (int i = 0; i < 4; i++)
            {
                dot += q1[i] * q2[i];
            }

            if (dot < 0.0)
            {
                for (int i = 0; i < 4; i++)
                {
                    q1[i] = -q1[i];
                }
                dot = -dot;
            }

            const float DOT_THRESHOLD = 0.9995f;
            if (dot > DOT_THRESHOLD)
            {
                for (int i = 0; i < 4; i++)
                {
                    result[i] = q1[i] + t * (q2[i] - q1[i]);
                }
            }
            else
            {
                float theta_0 = (float)Math.Acos(dot);
                float sin_theta_0 = (float)Math.Sqrt(1 - (dot * dot));

                for (int i = 0; i < 4; i++)
                {
                    result[i] = (double)((Math.Sin((1.0 - t) * theta_0) * q1[i] + Math.Sin(t * theta_0) * q2[i]) / sin_theta_0);
                }
            }
        }

        // function to avoid flipping the sign of quaternions
        void avoidJumps(double[] q1, double[] q2)
        {
            double[] slerpResult = new double[4];
            slerp(q1, q2, 0.5f, slerpResult); // perform slerp interpolation with equal weighting

            for (int i = 0; i < 4; i++)
            {
                q1[i] = slerpResult[i];
            }

            // update previous quaternion values
            for (int i = 0; i < 4; i++)
            {
                q2[i] = q1[i];
            }
        }
        #endregion
        #region Helper functions
        public static Matrix4x4 Create4x4FromArray(double[] array)
        {
            if (array.Length != 16)
            {
                throw new ArgumentException("Pose must be a 4x4 array.");
            }

            Matrix4x4 m = new Matrix4x4(
            (float)array[0], (float)array[1], (float)array[2], (float)array[3],
            (float)array[4], (float)array[5], (float)array[6], (float)array[7],
            (float)array[8], (float)array[9], (float)array[10], (float)array[11],
            (float)array[12], (float)array[13], (float)array[14], (float)array[15]);

            return m;
        }

        // Column-major order is the default in MathNet.Numerics
        public static Matrix<T> ArrayToMatrix<T>(T[] array, int rowCount, int columnCount) where T : struct, IEquatable<T>, IFormattable
        {
            if (array.Length != rowCount * columnCount)
                throw new ArgumentException("Array length does not match the product of row and column counts.");

            if (typeof(T) == typeof(double))
            {
                var doubleArray = Array.ConvertAll(array, item => (double)(object)item);
                return Matrix<double>.Build.Dense(rowCount, columnCount, doubleArray) as Matrix<T>;
            }
            else if (typeof(T) == typeof(float))
            {
                var floatArray = Array.ConvertAll(array, item => (float)(object)item);
                return Matrix<float>.Build.Dense(rowCount, columnCount, floatArray) as Matrix<T>;
            }
            else
            {
                throw new ArgumentException("Unsupported type. Only float and double are supported.");
            }
        }
        #endregion
    }
}