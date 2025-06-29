using System;
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
    
    public partial class cpParadigmMenu : ControlPanel
    {
        ParadigmMenu _SourceBlock;
        public bool isOn = false;
        public cpParadigmMenu(ParadigmMenu _SourceBlock) : base(_SourceBlock) { InitializeComponent(); this._SourceBlock = _SourceBlock; }
        private void button1_Click(object sender, EventArgs e)
        {
            isOn = !isOn;
            if (!isOn) { button1.Text = "Start"; _SourceBlock.state = MenuState.Inactive; }
            else { button1.Text = "Stop"; _SourceBlock.state = MenuState.AxisGroup; }
        }
        override protected void cpRefresh(object o, EventArgs e)
        {
            string text;
            if (_SourceBlock.state == MenuState.Inactive)
            {
                state.Text = "Inactive";
                motion_activated.Hide();

                axis_info.Hide();
            }


            else if (_SourceBlock.state == MenuState.AxisGroup)
            {
                state.Text = "1. Select Axis group.";
                motion_activated.Hide();
                axis_info.Hide();

            }
            else if (_SourceBlock.state == MenuState.Axis)
            {
                state.Text = $"2. Select Axis.";
                if (_SourceBlock.axisGroup == 1)
                {
                    text = "X = radial/ulnar \n Y = flex/extend \n psi = supination/pronation";
                }
                else
                {
                    text = "Z = supination/pronation \n theta = flex/extend\n phi = radial/ulnar";
                }

                axis_info.Text = text;
                motion_activated.Hide();
            }
            else
            {
                state.Text = $"3. Control speed and direction";
                axis_info.Text = $"Axis: {_SourceBlock.axis}";
                motion_activated.Text = $"Speed: {_SourceBlock.speed}";
            }
            output.Text = _SourceBlock.cp_output;
            orientation.Text = $"Your arm is at {Convert.ToString(Math.Round(_SourceBlock.angle))} degrees";
        }
    }
    public class ParadigmMenu : Block
    {
        // Input blocks
        Block get_orientation;
        Block get_gesture;
        public MenuState state = MenuState.Inactive;

        // paradigm classes
        paradigm_calculation pc = new paradigm_calculation();

        // Speech synthesizer
        SpeechSynthesizer synth = new SpeechSynthesizer();



        // parameters
        public double angle = 0;
        public int gesture = 0;
        public int axisGroup = 0;
        double angle_zero;
        public Axis axis = Axis.None;
        public bool active = false;
        public double speed;
        bool talk = true;
        public string cp_output;
        bool changedMenue;
        DateTime startTime;


        public ParadigmMenu(string Name, double DesiredRate, List<string> InputCfg, List<string> Params, string Path)
            : base(Name, DesiredRate, InputCfg, Params, Path) { }
        override public void ConfigureInputs()
        {
            base.ConfigureInputs();
            if (InputBlocks.Count != 2) throw new Exception($"Block Menue must have two inputs blocks 1.gesture 2.orientation");
            get_gesture = InputBlocks[0];
            get_orientation = InputBlocks[1];

            cp = new cpParadigmMenu(this);
        }

        override protected void OnNewInput(Block sender, object value)
        {

            if (sender == get_orientation)
            {
                angle = (double)value;
            }
            else if (sender == get_gesture)
            {
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
                        SendZeros();
                        break;
                    }
                case MenuState.AxisGroup:
                    {
                        if (talk)
                        {
                            Speak("Select group");
                            talk = false;
                        }

                        double[] group2 = new double[] { 0, 90 };
                        double[] group1 = new double[] { 90, 180 };
                        // get the axisgroup
                        axisGroup = pc.GetAxisGroup(gesture, angle, group1, group2);

                        // advance option
                        if (axisGroup != 0)
                        {
                            state = ChangeState(1);
                            Speak($"Select axis from group {axisGroup}");
                        }
                        SendZeros();
                        break;
                    }
                case MenuState.Axis:
                    {

                        // return option
                        if (gesture == 1 & !changedMenue)
                        {
                            state = ChangeState(-1);
                            axis = Axis.None;
                            Speak("Returned to Axis Group selection");
                            break;
                        }
                        else if (changedMenue & (DateTime.Now >= startTime.AddSeconds(5)))
                        {
                            changedMenue = false;
                            Console.Write("unlocked");
                        }

                        // get the axis
                        axis = pc.GetAxis(gesture, axisGroup);

                        // advance option
                        if (axis != Axis.None)
                        {
                            state = ChangeState(1);
                            Speak($"Ready to move axis {axis}");
                        }
                        SendZeros();
                        break;
                    }
                case MenuState.DirectionSpeed:
                    {
                        Console.WriteLine($"[DirectionSpeed] Current gesture: {gesture}, active: {active}");

                        // Set zero level for speed and activate motion on gesture 2
                        if (gesture == 2 && !active)
                        {
                            angle_zero = angle;
                            active = true;
                            Console.WriteLine($"[DirectionSpeed] Gesture 2 detected, locking angle_zero at {angle_zero} and setting active = true");
                            // Do NOT break here; allow processing to continue and send motion vector
                        }
                        // Stop motion and possibly return to previous state on gestures other than 2
                        else if (gesture != 2)
                        {
                            if (gesture == 1)
                            {
                                state = ChangeState(-1);
                                changedMenue = true;
                                startTime = DateTime.Now;
                                Speak("Returned to Axis selection");
                                Console.WriteLine("[DirectionSpeed] Gesture 1 detected, returning to previous menu and unlocking");
                            }

                            active = false;
                            Console.WriteLine("[DirectionSpeed] Motion stopped, active set to false");
                        }

                        double speed = 0.0;

                        // Calculate speed and direction only if active
                        if (active)
                        {
                            speed = pc.GetSpeed(angle, angle_zero, 0.5);
                            Console.WriteLine($"[DirectionSpeed] Calculated speed: {speed}");
                        }
                        else
                        {
                            Console.WriteLine("[DirectionSpeed] Not active, speed set to 0");
                        }

                        // Generate and send motion vector
                        Vector motionvector = pc.MotionVector(speed, axis);
                        SendOutput(motionvector);
                        Console.WriteLine($"[DirectionSpeed] Sent motion vector: {motionvector}");

                        break;
                    }


            }
        }


        public MenuState ChangeState(int change_amount)
        {
            MenuState new_state = (MenuState)((int)state + change_amount);
            return new_state;
        }
        private void Speak(string text)
        {
            synth.Volume = 100; //0 ... 100
            synth.Rate = -2;    // -10 ... 10

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

