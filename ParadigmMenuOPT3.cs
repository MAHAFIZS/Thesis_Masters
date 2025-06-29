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

namespace iM
{
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

    //public enum MenuState
    //{
    //    AxisGroup = 0,
    //    Axis = 1,
    //    DirectionSpeed = 2
    //}
    //public enum Axis
    //{
    //    None = 0,
    //    X = 1,
    //    Y = 2,
    //    Z = 3,
    //    Phi = 4,        // Roll
    //    Theta = 5,      // Pitch
    //    Psi = 6         // Yaw
    //}

    public partial class cpParadigmMenuOPT3 : ControlPanel
    {
        ParadigmMenuOPT3 _SourceBlock;
        bool isOn = false;

        public cpParadigmMenuOPT3(ParadigmMenuOPT3 _SourceBlock) : base(_SourceBlock) { InitializeComponent(); this._SourceBlock = _SourceBlock; }

        private void button1_Click(object sender, EventArgs e)
        {
            isOn = !isOn;
            if (!isOn) { button1.Text = "Start"; _SourceBlock.state = MenuState.Inactive; }
            else { button1.Text = "Stop"; _SourceBlock.state = MenuState.AxisGroup; _SourceBlock.Speak("Select axis group"); }
        }
        override protected void cpRefresh(object o, EventArgs e)
        {
            switch (_SourceBlock.state)
            {
                case MenuState.Inactive:
                    {

                        state.Text = "Inactive";
                        pictureBox2.Image = Image.FromFile("C:\\Users\\z004zrxh\\Development\\GIT\\Git_repository\\Visualization\\OPT3\\wholeMenu_noZ.png");
                        pictureBox1.Image = Image.FromFile("C:\\Users\\z004zrxh\\OneDrive - Siemens Healthineers\\Master_Thesis\\Struktur\\visualization\\carm_axes.png");
                        motion_activated.Hide();

                        break;
                    }
                case MenuState.AxisGroup:
                    {
                        if (_SourceBlock.angle < 90)
                        {
                            pictureBox2.Image = Image.FromFile("C:\\Users\\z004zrxh\\Development\\GIT\\Git_repository\\Visualization\\OPT2\\selectGroupCarm_noZ.png");
                        }
                        else
                        {
                            pictureBox2.Image = Image.FromFile("C:\\Users\\z004zrxh\\Development\\GIT\\Git_repository\\Visualization\\OPT2\\selectGroupBase_noZ.png");
                        }
                        state.Text = "Select Axis-group";
                        //pictureBox2.Image = image2;
                        break;
                    }
                case MenuState.Axis:
                    {
                        String head;
                        Image image1;
                        Image image2;
                        motion_activated.Hide();
                        if (_SourceBlock.axisLocked == Axis.X)
                        {
                            image1 = Image.FromFile("C:\\Users\\z004zrxh\\Development\\GIT\\Git_repository\\Visualization\\OPT3\\x.png");
                        }
                        else if (_SourceBlock.axisLocked == Axis.Y)
                        {
                            image1 = Image.FromFile("C:\\Users\\z004zrxh\\Development\\GIT\\Git_repository\\Visualization\\OPT3\\y.png");
                        }
                        //else if (_SourceBlock.axisLocked == Axis.Z)
                        //{
                        //    image1 = Image.FromFile("C:\\Users\\z004zrxh\\OneDrive - Siemens Healthineers\\Master_Thesis\\Struktur\\visualization\\Toggle_visualization\\z.png");
                        //}
                        else if (_SourceBlock.axisLocked == Axis.Phi)
                        {
                            image1 = Image.FromFile("C:\\Users\\z004zrxh\\Development\\GIT\\Git_repository\\Visualization\\OPT3\\phi_noZ.png");
                        }
                        else if (_SourceBlock.axisLocked == Axis.Psi)
                        {
                            image1 = Image.FromFile("C:\\Users\\z004zrxh\\Development\\GIT\\Git_repository\\Visualization\\OPT3\\psi.png");
                        }
                        else
                        {
                            image1 = Image.FromFile("C:\\Users\\z004zrxh\\Development\\GIT\\Git_repository\\Visualization\\OPT3\\theta_noZ.png");
                        }
                        image2 = Image.FromFile("C:\\Users\\z004zrxh\\Development\\GIT\\Git_repository\\Visualization\\OPT3\\motion_trigger_supination.png");

                        if (_SourceBlock.axisGroup == 1)
                        {
                            head = $"Select Axis from BASE";
                        }
                        else
                        {
                            head = $"Select Axis from CARM";
                        }
                        state.Text = head;
                        pictureBox2.Image = image1;
                        pictureBox1.Image = image2;
                        break;
                    }
                case MenuState.DirectionSpeed:
                    {
                        pictureBox1.Image = Image.FromFile("..\\..\\..\\Demo\\lock.png");
                        motion_activated.Show();
                        state.Text = $"Ready to move axis {_SourceBlock.axisLocked}";
                        motion_activated.Text = $"Speed {_SourceBlock.speed}";
                        break;
                    }
            }


            output.Text = _SourceBlock.cp_output;
            orientation.Text = $"Your arm is at angle {Convert.ToString(Math.Round(_SourceBlock.angle))}, input gesture {_SourceBlock.gesture} ";
        }
    }
    public class ParadigmMenuOPT3 : Block
    {
        // Input blocks
        Block get_orientation;
        Block get_gesture;
        Block get_Syncstation;
        public MenuState state = MenuState.Inactive;

        // paradigm classes
        paradigm_calculation pc = new paradigm_calculation();

        // Speech synthesizer
        SpeechSynthesizer synth = new SpeechSynthesizer();



        // parameters
        int counter_null = 0;
        public double angle = 0;
        public int gesture = 0;
        public int axisGroup = 0;
        double angle_zero;
        public bool active = false;
        public double speed;
        bool talk = true;
        public string cp_output;
        bool changedMenue;
        public Axis axisLocked = Axis.None;
        DateTime startTime;
        bool toggle = false;
        List<Axis> axisBase = new List<Axis> { Axis.Psi, Axis.X, Axis.Y };
        List<Axis> axisCarm = new List<Axis> { Axis.Phi, Axis.Theta };


        public ParadigmMenuOPT3(string Name, double DesiredRate, List<string> InputCfg, List<string> Params, string Path)
            : base(Name, DesiredRate, InputCfg, Params, Path) { }
        override public void ConfigureInputs()
        {
            base.ConfigureInputs();
            if (InputBlocks.Count < 2) throw new Exception($"Block Menue must have two inputs blocks 1.gesture 2.orientation");
            get_gesture = InputBlocks[0];
            get_orientation = InputBlocks[1];
            //get_Syncstation = InputBlocks[2];

            cp = new cpParadigmMenuOPT3(this);
        }

        override protected void OnNewInput(Block sender, object value)
        {
            //if (sender == get_Syncstation)
            //{
            //    if (value == null)
            //    {
            //        counter_null += 1;
            //        if (counter_null > 3)
            //        {
            //            SendZeros();
            //            return;
            //        }

            //    }
            //    return;
            //}
            if (sender == get_orientation)
            {
                counter_null = 0;
                angle = (double)value;
            }
            else if (sender == get_gesture)
            {
                counter_null = 0;
                gesture = (int)value;
            }

            StateMachine();


        }


        private void StateMachine()
        {
            switch (state)
            {
                case MenuState.Inactive:
                    {
                        SendZeros(); break;
                    }
                case MenuState.AxisGroup:
                    {
                        double[] group2 = new double[] { 0, 90 };
                        double[] group1 = new double[] { 90, 180 };
                        // get the axisgroup
                        axisGroup = pc.GetAxisGroup(gesture, angle, group1, group2);

                        // advance option
                        if (axisGroup != 0)
                        {
                            if (axisGroup == 1) { axisLocked = axisBase[0]; }
                            if (axisGroup == 2) { axisLocked = axisCarm[0]; }
                            state = ChangeState(1);
                            toggle = false;
                            Speak($"Group {axisGroup}");

                        }
                        SendZeros();
                        break;
                    }
                case MenuState.Axis:
                    {

                        // return option
                        if (gesture == 4 & !changedMenue)
                        {
                            state = ChangeState(-1);
                            axisLocked = Axis.None;
                            Speak("Returned");
                            break;
                        }

                        if (gesture == 2 & toggle == true)
                        {
                            axisLocked = pc.ToggleAxis(axisLocked, axisGroup, axisBase, axisCarm);
                            toggle = false;
                            Speak($"{axisLocked}");
                            break;
                        }
                        if (toggle == false & gesture != 2)
                        {
                            toggle = true;
                        }

                        // advance option
                        // set zero level for speed and activate motion
                        if (axisLocked != Axis.None & gesture == 6)
                        {
                            state = ChangeState(1);
                            //Speak($"Lock");
                            angle_zero = angle;
                        }
                        SendZeros();
                        break;
                    }
                case MenuState.DirectionSpeed:
                    {
                        toggle = true;
                        speed = 0.0;
                        // calculate speed and direction when active
                        if (gesture == 6)
                        {
                            speed = pc.GetSpeedStatic(angle, angle_zero, 0.02);
                        }
                        // stop motion (and return to previous state)
                        if (gesture != 6)
                        {
                            // return option
                            state = ChangeState(-1);
                            //Speak("Release");
                        }

                        // generate vector [x,y,z,roll,pitch,yaw] and send as output
                        Vector motionvector = pc.MotionVector(speed, axisLocked);
                        SendOutput(motionvector);

                        break;
                    }

            }
        }


        private MenuState ChangeState(int change_amount)
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
        private void SendZeros()
        {
            Vector output = pc.MotionVector(0, Axis.None);
            SendOutput(output);
        }
    }
}

