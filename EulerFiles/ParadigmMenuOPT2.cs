using System;
using System.Drawing;
using System.Drawing.Imaging;
using System.Collections.Generic;
using System.Linq;
using System.Net.Sockets;
using System.Reflection.Emit;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Media.Media3D;
using System.Xml.Serialization;
using Vector = MathNet.Numerics.LinearAlgebra.Vector<double>;
using System.Speech.Synthesis;
using System.Windows.Forms;
using Windows.System;
using System.IO;
using static System.Windows.Forms.VisualStyles.VisualStyleElement.Window;
using WinForms = System.Windows.Forms;
using System.Runtime.InteropServices;
using System.Threading;


// This block expects two input blocks 
// 1. Orientaton value 
// 2. Gesture
// and two input params
// 1. IP adress for first menue (base of c-arm)
// 2. Port
// 3. IP adress for the second menu (the c-arm part)
// 4. Port 


//       rest: 0,
//       open_hand: 1,
//       power: 2,
//       flex: 3,
//       extend: 4,
//       supination: 5,
//       pronation: 6,
//       radial: 7,
//       ulnar: 8

namespace iM
{
    public enum MenuState
    {
        AxisGroup = 0,
        Axis = 1,
        DirectionSpeed = 2,
        Inactive = -1
    }
    public enum Axis
    {
        None = 0,
        X = 1,
        Y = 2,
        Z = 3,
        Phi = 4,        // Roll
        Theta = 5,      // Pitch
        Psi = 6,         // Yaw
        Gripper = 7,
        Finger = 8,
        Screw = 9
    }

    public class GestureAngleChangedEventArgs : EventArgs
    {
        public int Gesture { get; }
        public double Angle { get; }

        public GestureAngleChangedEventArgs(int gesture, double angle)
        {
            Gesture = gesture;
            Angle = angle;
        }
    }

    public partial class cpParadigmMenuOPT2 : ControlPanel
    {
        private Panda pandaInstance;

        public cpParadigmMenuOPT2(ParadigmMenuOPT2 sourceBlock, Panda panda)
    : base(sourceBlock)
        {
            InitializeComponent();
            this._SourceBlock = sourceBlock;
            this.pandaInstance = panda;  // Store the Panda instance here

            this.KeyPreview = true;
            this.KeyDown += CpParadigmMenuOPT2_KeyDown;
            this.Load += (s, e) => this.Focus();
        }

        private void StartPandaGestureControl()
        {
            try
            {
                System.Diagnostics.Debug.WriteLine("🟡 StartPandaGestureControl() was called.");

                if (pandaInstance != null && pandaInstance.StartGestureControl != null)
                {
                    pandaInstance.StartGestureControl.Invoke();
                    System.Diagnostics.Debug.WriteLine("✅ Panda gesture control started successfully.");
                }
                else
                {
                    System.Diagnostics.Debug.WriteLine("❌ Panda instance or StartGestureControl delegate is null.");
                    System.Windows.Forms.MessageBox.Show(
                        "Panda robot control is not initialized.",
                        "Error",
                        System.Windows.Forms.MessageBoxButtons.OK,
                        System.Windows.Forms.MessageBoxIcon.Error);
                }
            }
            catch (Exception ex)
            {
                System.Windows.Forms.MessageBox.Show(
                    "❌ Failed to start Panda gesture control:\n" + ex.Message,
                    "Control Error",
                    System.Windows.Forms.MessageBoxButtons.OK,
                    System.Windows.Forms.MessageBoxIcon.Error);
            }
        }
    





    public Dictionary<Axis, string> filePaths = new Dictionary<Axis, string>()
{
    { Axis.X, @"D:\Hafiz_intuitiveRobotControl\Recorded data\menus\cartesianx.png" },
    { Axis.Y, @"D:\Hafiz_intuitiveRobotControl\Recorded data\menus\cartesiany.png" },
    { Axis.Z, @"D:\Hafiz_intuitiveRobotControl\Recorded data\menus\cartesianz.png" },
    { Axis.Theta, @"D:\Hafiz_intuitiveRobotControl\Recorded data\menus\roll.png" },
    { Axis.Phi, @"D:\Hafiz_intuitiveRobotControl\Recorded data\menus\Pitch.png" },
    { Axis.Psi, @"D:\Hafiz_intuitiveRobotControl\Recorded data\menus\yaw.png" }
};


        static string pathCur = @"D:\Hafiz_intuitiveRobotControl\Recorded data\menus\Menu.png";


        Image image1 = Image.FromFile(@"D:\Hafiz_intuitiveRobotControl\Recorded data\menus\Menu.png");
        Image image2 = Image.FromFile(@"D:\Hafiz_intuitiveRobotControl\Recorded data\menus\cartesian.png");

        Axis current_axis = Axis.None;

        ParadigmMenuOPT2 _SourceBlock;
        bool isOn = false;
        bool change = false;
        bool change_state = false;
        MenuState current_state = MenuState.Inactive;
        public cpParadigmMenuOPT2(ParadigmMenuOPT2 _SourceBlock) : base(_SourceBlock)
        {
            InitializeComponent();
            this._SourceBlock = _SourceBlock;
            this.KeyPreview = true; // 👈 ADD THIS
            this.KeyDown += CpParadigmMenuOPT2_KeyDown; // 👈 ADD THIS TOO
            this.Load += (s, e) => this.Focus();  // ✅ ADD THIS
        }

        private bool muJoCoStarted = false;
        private cpParadigmMenuOPT2 cp;
        private void UpdateMenuUI()
        {
            if (_SourceBlock.gesture == 1)
            {
                image1 = Image.FromFile(@"D:\Hafiz_intuitiveRobotControl\Recorded data\menus\Menu.png");
            }
            else if (_SourceBlock.gesture == 2)
            {
                image1 = Image.FromFile(@"D:\Hafiz_intuitiveRobotControl\Recorded data\menus\cartesian.png");
            }
            else if (_SourceBlock.gesture == 3)
            {
                image1 = Image.FromFile(@"D:\Hafiz_intuitiveRobotControl\Recorded data\menus\gripperfinger.png");
            }

            pictureBox2.Image = image1;
        }
        public int gesture = 0;
        

        public int gestureOverride = -1;

        private void CpParadigmMenuOPT2_KeyDown(object sender, KeyEventArgs e)
        {
            switch (e.KeyCode)
            {
                case Keys.S:
                    _SourceBlock.gestureOverride = 1; break;
                case Keys.E:
                    _SourceBlock.gestureOverride = 2; break;
                case Keys.Q:
                    _SourceBlock.gestureOverride = 3; break;
                case Keys.NumPad6:
                    _SourceBlock.gestureOverride = 4; break;
                case Keys.NumPad4:
                    _SourceBlock.gestureOverride = 5; break;
            }

            UpdateMenuUI(); // update visual state
            e.Handled = true;
        }
        private void SendTestGestureToMuJoCo()
        {
            try
            {
                using (var udpClient = new System.Net.Sockets.UdpClient())
                {
                    string message = "4"; // Example test gesture
                    byte[] bytes = System.Text.Encoding.ASCII.GetBytes(message);
                    udpClient.Send(bytes, bytes.Length, "127.0.0.1", 5005);
                    System.Diagnostics.Debug.WriteLine("✅ Sent test gesture '4' to MuJoCo.");
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine("❌ Failed to send gesture to MuJoCo: " + ex.Message);
            }
        }


        public bool muJoCoLaunched = false;
        public System.Windows.Forms.Label StateLabel => state;
        public System.Windows.Forms.Label OrientationLabel => orientation;

        private double angle_zero; // ✅ OK (keep private)

        private void button1_Click(object sender, EventArgs e)
        {
            isOn = !isOn;
            if (!isOn)
            {
                button1.Text = "Start";
                _SourceBlock.state = MenuState.Inactive;
                motion_activated.Hide();
            }
            else
            {
                button1.Text = "Stop";
                _SourceBlock.state = MenuState.AxisGroup;
                _SourceBlock.Speak("Select axis group");

                current_axis = Axis.None;
                _SourceBlock.axisLocked = Axis.None;         // ✅ allow angle-based scrolling again
                _SourceBlock.muJoCoLaunched = false;         // ✅ allow relaunching MuJoCo
            }
        }

        override protected void cpRefresh(object o, EventArgs e)
        {


            if (_SourceBlock.axisLocked != current_axis)
            {
                current_axis = _SourceBlock.axisLocked;
                change = true;
            }
            else { change = false; }
            if (_SourceBlock.state != current_state)
            {
                current_state = _SourceBlock.state;
                change_state = true;
            }
            else { change_state = false; }
            switch (_SourceBlock.state)
            {
                case MenuState.Inactive:
                    {
                        state.Text = "Inactive";
                        image1 = Image.FromFile(@"D:\Hafiz_intuitiveRobotControl\Recorded data\menus\Menu.png");
                        break;
                    }

                case MenuState.AxisGroup:
                    {
                        state.Text = "Select Axis-group";

                        // 💡 Live update of image based on angle
                        if (_SourceBlock.angle < 90)
                        {
                            image1 = Image.FromFile(@"D:\Hafiz_intuitiveRobotControl\Recorded data\menus\cartesian.png");
                        }
                        else
                        {
                            image1 = Image.FromFile(@"D:\Hafiz_intuitiveRobotControl\Recorded data\menus\gripperfinger.png");
                        }
                        break;
                    }




                case MenuState.Axis:
                    {
                        string head;

                        // 1️⃣ Select from appropriate axis list
                        Axis[] axisList = (_SourceBlock.axisGroup == 1)
                            ? new Axis[] { Axis.X, Axis.Y, Axis.Z }
                            : new Axis[] { Axis.Phi, Axis.Theta, Axis.Psi };

                        // 2️⃣ Only scroll by angle if NOT locked
                        if (_SourceBlock.axisLocked == Axis.None && _SourceBlock.gesture != 2)

                        {
                            Axis axisFromAngle = _SourceBlock.pc.GetAxisFromAngle(_SourceBlock.angle, axisList);
                            if (current_axis != axisFromAngle)
                            {
                                current_axis = axisFromAngle;
                                change = true;
                            }
                        }

                        // 3️⃣ Lock current_axis on gesture 2 (lock), only once
                        if (_SourceBlock.gesture == 2 && _SourceBlock.axisLocked == Axis.None && current_axis != Axis.None)
                        {
                            _SourceBlock.axisLocked = current_axis;
                            _SourceBlock.state = _SourceBlock.ChangeState(1); // motion control
                            _SourceBlock.SetAngleZero(_SourceBlock.angle);
                            change = true;

                            if (!_SourceBlock.muJoCoLaunched)
                            {
                                StartPandaGestureControl();
                                _SourceBlock.muJoCoLaunched = true;
                            }

                            if (pandaInstance != null)
                            {
                                pandaInstance.LockAxis(current_axis);
                            }
                        }




                        // 4️⃣ Unlock axis on gesture 3 (unlock)
                        if (_SourceBlock.gesture == 3 && _SourceBlock.axisLocked != Axis.None)
                        {
                            _SourceBlock.axisLocked = Axis.None;

                            // Notify Panda about unlock, if you implement such method:
                            if (pandaInstance != null)
                            {
                                pandaInstance.UnlockAxis();
                            }


                            change = true;
                        }

                        if ((_SourceBlock.gesture == 4 || _SourceBlock.gesture == 5) && _SourceBlock.axisLocked != Axis.None)
                        {
                            double speed = (_SourceBlock.gesture == 4) ? 1.0 : -1.0;

                            if (pandaInstance != null)
                            {
                                pandaInstance.MoveLockedAxis(_SourceBlock.axisLocked, speed);
                            }
                        }

                        // 5️⃣ Image updates
                        if (change_state)
                        {
                            motion_activated.Hide();
                            if (current_axis != Axis.None && filePaths.ContainsKey(current_axis))
                            {
                                image1 = Image.FromFile(filePaths[current_axis]);
                            }
                            else if (current_axis == Axis.None && _SourceBlock.axisGroup == 1)
                            {
                                image1 = Image.FromFile(@"D:\Hafiz_intuitiveRobotControl\Recorded data\menus\grippersubmenu.png");
                            }
                            else
                            {
                                image1 = Image.FromFile(@"D:\Hafiz_intuitiveRobotControl\Recorded data\menus\cartesianmenu.png");
                            }
                        }

                        if (change && filePaths.ContainsKey(current_axis))
                        {
                            image1 = Image.FromFile(filePaths[current_axis]);
                        }
                        // 7️⃣ Update text depending on lock state
                        if (_SourceBlock.axisLocked != Axis.None)
                        {
                            // Show locked axis ready message
                            state.Text = $"{_SourceBlock.axisLocked} axis ready to move";
                        }
                        else
                        {
                            head = (_SourceBlock.axisGroup == 1)
                            ? "Select Axis from Cartesian"
                            : "Select Axis from Gripper";

                        state.Text = head;
                        }
                        break;

                    }



                case MenuState.DirectionSpeed:
                    {
                        motion_activated.Show();
                        state.Text = $"Ready to move axis {_SourceBlock.axisLocked}";
                        motion_activated.Text = $"Speed {-_SourceBlock.speed}";
                        break;
                    }
            }

            // Always executed after the switch
            pictureBox1.Image = image2;
            pictureBox2.Image = image1;
            output.Text = _SourceBlock.cp_output;
            orientation.Text = $"Your arm is at angle {Convert.ToString(Math.Round(_SourceBlock.angle))}, input gesture {_SourceBlock.gesture} ";
        }
    }

    public class ParadigmMenuOPT2 : Block
    {
        public void SetAngleZero(double value)
        {
            angle_zero = value;
        }

        // Input blocks
        Block get_orientation;
        Block get_gesture;
        Block get_Syncstation;
        public MenuState state = MenuState.Inactive;

        // paradigm classes
        public paradigm_calculation pc = new paradigm_calculation();

        // Speech synthesizer
        SpeechSynthesizer synth = new SpeechSynthesizer();

        public int gestureOverride = -1;

        public bool muJoCoLaunched = false;

        // parameters
        public double angle = 0;
        public int gesture = 0;
        public void UnlockAxis()
        {
            // Add logic here to handle axis unlocking in the real robot
            Console.WriteLine("[INFO] Axis unlocked on Panda robot.");
            // For example, reset any internal flags or stop motion on locked axis
            // You can also notify robot or reset any control state if needed
        }

        // Inside ParadigmMenuOPT2 class:
        public event EventHandler<GestureAngleChangedEventArgs> GestureAngleChanged;

        // Raise this event whenever you update gesture or angle internally:
        private void RaiseGestureAngleChanged()
        {
            GestureAngleChanged?.Invoke(this, new GestureAngleChangedEventArgs(gesture, angle));
        }

        public int axisGroup = 0;
        double angle_zero;
        public Axis axis = Axis.None;
        public bool active = false;
        public double speed;
        bool talk = true;
        public string cp_output;
        bool changedMenue;
        public Axis axisLocked = Axis.None;
        DateTime startTime;
        int counter_null = 0;
        
        public string current_mode = "";
        private int selectedAxisGesture = 0;  // Stores last gesture used for axis selection



        public ParadigmMenuOPT2(string Name, double DesiredRate, List<string> InputCfg, List<string> Params, string Path)
            : base(Name, DesiredRate, InputCfg, Params, Path) { }
        override public void ConfigureInputs()
        {
            base.ConfigureInputs();
            if (InputBlocks.Count != 2) throw new Exception($"Block Menue must have two inputs blocks 1.gesture 2.orientation");
            get_gesture = InputBlocks[0];
            get_orientation = InputBlocks[1];
            //get_Syncstation = InputBlocks[2];

            cp = new cpParadigmMenuOPT2(this);
        }
        private void SendGestureToMuJoCo(int gestureId)
        {
            try
            {
                using (var udpClient = new System.Net.Sockets.UdpClient())
                {
                    byte[] bytes = BitConverter.GetBytes((double)gestureId);
                    byte[] payload = new byte[20];  // Fake 12-byte header + 8-byte payload
                    System.Buffer.BlockCopy(bytes, 0, payload, 12, 8);  // ✅ FIXED HERE
                    udpClient.Send(payload, payload.Length, "127.0.0.1", 5005);
                    System.Diagnostics.Debug.WriteLine($"✅ Sent gesture {gestureId} to MuJoCo.");
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine("❌ Failed to send gesture to MuJoCo: " + ex.Message);
            }
        }


        private DateTime lastGestureTime = DateTime.MinValue;
        private readonly TimeSpan gestureDelay = TimeSpan.FromMilliseconds(150);

        /// <summary>
        /// 
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="value"></param>
        override protected void OnNewInput(Block sender, object value)
        {
            if (sender == get_orientation)
            {
                angle = (double)value;
                counter_null = 0;
            }
            else if (sender == get_gesture)
            {
                int incomingGesture = (int)value;
                counter_null = 0;

                // Gesture throttling
                if (DateTime.Now - lastGestureTime < gestureDelay)
                    return;

                lastGestureTime = DateTime.Now;

                if (gestureOverride != -1)
                {
                    gesture = gestureOverride;
                    gestureOverride = -1;
                }
                else
                {
                    gesture = incomingGesture;
                }
            }

            StateMachine();
            // Notify subscribers after updating gesture and angle
            RaiseGestureAngleChanged();
        }


        private void StateMachine()
        {
            Console.WriteLine($"[StateMachine] Current state: {state}");
            switch (state)
            {

                case MenuState.Inactive:
                    {
                        axisLocked = Axis.None;
                        SendZeros(); break;
                    }
                case MenuState.AxisGroup:
                    {

                        double[] group1 = new double[] { 0, 90 };
                        double[] group2 = new double[] { 90, 180 };
                        // get the axisgroup
                        axisGroup = pc.GetAxisGroup(gesture, angle, group1, group2);

                        // advance option
                        if (axisGroup != 0 && gesture == 2)
                        {
                            state = ChangeState(1);
                            Speak($"group {axisGroup}");
                        }

                        SendZeros();
                        break;
                    }
                case MenuState.Axis:
                    {
                        Console.WriteLine($"🚪 Entering Cartesian submenu. Current axis: {axis}, current_mode: {current_mode}, axisLocked: {axisLocked}, gesture: {gesture}");

                        // 🔙 Return option: go back if gesture 1 and not in changed menu state
                        if (gesture == 1 && !changedMenue)
                        {
                            Console.WriteLine("🔙 Gesture 1 detected and menu not changed, returning to previous menu...");
                            state = ChangeState(-1);
                            axisLocked = Axis.None;
                            Speak("Returned");
                            Console.WriteLine("✅ Returned from submenu, axisUnlocked.");
                            break;
                        }

                        // 🧠 Update axis selection continuously based on angle if no axis locked
                        if (axisLocked == Axis.None)
                        {
                            Console.WriteLine($"🔄 No axis locked, updating axis selection based on angle: {angle}");

                            // Select axis list depending on current axis group
                            Axis[] axisList = (axisGroup == 1)
                                ? new Axis[] { Axis.X, Axis.Y, Axis.Z }
                                : new Axis[] { Axis.Phi, Axis.Theta, Axis.Psi };

                            Console.WriteLine($"🧩 Axis group: {axisGroup}, using axis list: {string.Join(", ", axisList)}");

                            // Update axis from angle
                            Axis axisFromAngle = pc.GetAxisFromAngle(angle, axisList);
                            if (axisFromAngle != Axis.None && axisFromAngle != axis)
                            {
                                axis = axisFromAngle;
                                current_mode = axis.ToString().ToLower();
                                Console.WriteLine($"📐 Axis selected by angle: {axis}, current_mode updated to: {current_mode}");
                                Speak($"Selected {axis}");
                            }
                            else
                            {
                                Console.WriteLine($"⏸️ Axis not changed. Current axis: {axis}");
                            }
                        }
                        else
                        {
                            Console.WriteLine($"⛔ Axis locked: {axisLocked}, skipping axis update.");
                        }

                        // 🔄 Scroll override: use gestures 4 or 5 to manually select axis if no lock yet
                        if ((gesture == 4 || gesture == 5) && axisLocked == Axis.None)
                        {
                            Axis scrolledAxis = pc.GetAxis(gesture, axisGroup);
                            if (scrolledAxis != Axis.None && scrolledAxis != axis)
                            {
                                axis = scrolledAxis;
                                current_mode = axis.ToString().ToLower();
                                Speak($"{axis}");
                                Console.WriteLine($"🔄 Gesture {gesture} manually selected axis = {axis}, current_mode = {current_mode}");
                            }
                            else
                            {
                                Console.WriteLine($"⏸️ Scroll gesture {gesture} did not change axis.");
                            }
                        }

                        Console.WriteLine($"🔎 Checking lock conditions: gesture={gesture}, axisLocked={axisLocked}, axis={axis}");

                        // 🟡 Lock axis on gesture 2 if axis selected and not locked yet
                        Console.WriteLine($"🧪 Trying to lock... gesture={gesture}, axisLocked={axisLocked}, axis={axis}");
                        if (gesture == 2 && axisLocked == Axis.None && axis != Axis.None)
                        {
                            axisLocked = axis;
                            state = ChangeState(1);
                            angle_zero = angle;
                            Speak($"Locked {axisLocked}");
                            Console.WriteLine($"🔒 Axis LOCKED: {axisLocked}");

                            if (!muJoCoLaunched)
                            {
                                muJoCoLaunched = true;
                                Console.WriteLine("🚀 MuJoCo launched flag set to true.");
                            }

                            Console.WriteLine($"📤 Sending lock mode: {current_mode}");
                            SendModeGesture(current_mode);
                            Console.WriteLine($"✅ Sent {current_mode}");
                        }
                        else
                        {
                            Console.WriteLine($"⏸️ Lock not triggered: gesture={gesture}, axisLocked={axisLocked}, axis={axis}");
                        }

                        // 🔓 Unlock axis with gesture 3 if locked
                        if (gesture == 3 && axisLocked != Axis.None)
                        {
                            Console.WriteLine($"🔓 Axis UNLOCKED: {axisLocked}");
                            axisLocked = Axis.None;
                            Speak("Unlocked");
                        }

                        if (axisLocked == Axis.None)
                        {
                            SendZeros();
                            Console.WriteLine("📤 Sent zeros for no motion because no axis is locked.");
                        }

                        break;
                    }





                case MenuState.DirectionSpeed:
                    {
                        double speed = 0.0;

                        // Unlock and return to previous state on gesture 3
                        if (gesture == 3)
                        {
                            axisLocked = Axis.None;
                            state = ChangeState(-1);  // Go back to previous menu
                            Speak("Unlocked");
                            Console.WriteLine("🔓 Axis unlocked, returning to Axis selection");
                        }
                        else if (axisLocked != Axis.None)
                        {
                            // Calculate speed for gestures 4 (positive) and 5 (negative)
                            if (gesture == 4)
                            {
                                speed = pc.GetSpeedStatic(angle, angle_zero, 0.02);
                            }
                            else if (gesture == 5)
                            {
                                speed = -pc.GetSpeedStatic(angle, angle_zero, 0.02);
                            }
                            else
                            {
                                // No movement for other gestures but stay in this state
                                speed = 0.0;
                            }

                            int gestureToSend = 0;

                            switch (axisLocked)
                            {
                                case Axis.X:
                                    gestureToSend = (speed > 0) ? 4 : 5; break;
                                case Axis.Y:
                                    gestureToSend = (speed > 0) ? 6 : 7; break;
                                case Axis.Z:
                                    gestureToSend = (speed > 0) ? 8 : 9; break;
                                case Axis.Finger:
                                    gestureToSend = (speed > 0) ? 10 : 11; break;
                                case Axis.Screw:
                                    gestureToSend = (speed > 0) ? 12 : 13; break;
                                case Axis.Gripper:
                                    gestureToSend = (speed > 0) ? 14 : 15; break;
                                case Axis.Theta:
                                    gestureToSend = (speed > 0) ? 16 : 17; break;
                                case Axis.Phi:
                                    gestureToSend = (speed > 0) ? 18 : 19; break;
                                case Axis.Psi:
                                    gestureToSend = (speed > 0) ? 20 : 21; break;
                                default:
                                    gestureToSend = 0; break;
                            }

                            if (gestureToSend != 0)
                                SendGestureToMuJoCo(gestureToSend);
                        }
                        else
                        {
                            // No axis locked, no motion and return to previous state
                            speed = 0.0;
                            state = ChangeState(-1);
                        }

                        var motionvector = pc.MotionVector(speed, axisLocked);
                        SendOutput(motionvector);

                        Console.WriteLine($"[ParadigmMenuOPT2] State: {state}, Gesture: {gesture}, AxisLocked: {axisLocked}");
                        Console.WriteLine($"[ParadigmMenuOPT2] MotionVector: {motionvector}");

                        break;
                    }


            }
        }

        public void SendModeGesture(string mode)
        {
            System.Diagnostics.Debug.WriteLine($"📤 Trying to send lock mode: {mode}");
            int id = 0;
            switch (mode)
            {
                case "x": id = 20; break;
                case "y": id = 21; break;
                case "z": id = 22; break;
                case "gripper": id = 23; break;
                case "finger": id = 24; break;
                case "screw": id = 25; break;
                case "phi": id = 26; break;     // added phi
                case "theta": id = 27; break;   // added theta
                case "psi": id = 28; break;     // added psi
                default:
                    System.Diagnostics.Debug.WriteLine("⚠️ Invalid mode. No gesture sent.");
                    return;
            }
            Console.WriteLine($"[Sender] Sending lock gesture id: {id} for mode '{mode}'");
            byte[] bytes = BitConverter.GetBytes((double)id);
            byte[] payload = new byte[20];
            Array.Copy(bytes, 0, payload, 12, 8);

            for (int i = 0; i < 6; i++) // simulate holding gesture
            {
                using (var udpClient = new UdpClient())
                {
                    udpClient.Send(payload, payload.Length, "127.0.0.1", 5005);
                }
                System.Diagnostics.Debug.WriteLine($"✅ Sent gesture {id} for mode '{mode}' — repeat {i + 1}/5");
                Thread.Sleep(100);  // delay to mimic stable input
            }
        }


        public MenuState ChangeState(int change_amount)
        {
            MenuState new_state = (MenuState)((int)state + change_amount);
            return new_state;
        }
        public void Speak(string text)
        {
            synth.Volume = 100; //0 ... 100
            synth.Rate = 0;    // -10 ... 10

            synth.SpeakAsync(text);
        }
        // speak completed event
        public void SendZeros()
        {
            Vector output = pc.MotionVector(0, Axis.None);
            SendOutput(output);
        }
    }
}

