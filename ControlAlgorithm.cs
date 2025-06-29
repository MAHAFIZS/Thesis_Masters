using System;
using System.Collections.Generic;
using System.Threading;
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
        psi     // yaw
    }

    public class ControlAlgorithm : Block
    {
        Algorithms algorithm = Algorithms.None;

        Dictionary<DOAs, double> controlDict = new Dictionary<DOAs, double>();

        double ds_max = 100 / (25);
        double dead_band = 0.3;

        private DOAs axisLocked = DOAs.None;
        private int gesture = 0;

        // Store latest inputs
        private int currentGesture = 0;
        private Vector currentCartesian = null;

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
                    throw new Exception($"ControlAlgorithm {Name} must have two input blocks: gesture and cartesian vector.");
            }
        }

        override protected void OnNewInput(Block sender, object value)
        {
            if (value == null)
            {
                Console.WriteLine("[OnNewInput] Warning: value is null");
                return;
            }

            if (sender == InputBlocks[0]) // Gesture input: expect int (or double cast to int)
            {
                int gestureInput = Convert.ToInt32(value);
                currentGesture = gestureInput;
                Console.WriteLine($"[OnNewInput] Gesture input received: {gestureInput}");
                // Update axisLocked based on lock gestures
                switch (currentGesture)
                {
                    case 20: axisLocked = DOAs.x; break;
                    case 21: axisLocked = DOAs.y; break;
                    case 22: axisLocked = DOAs.z; break;
                    case 23: axisLocked = DOAs.gripper; break;
                    case 24: axisLocked = DOAs.finger; break;
                    case 25: /* axisLocked = DOAs.screw; */ break; // optionally handle screw
                    case 26: axisLocked = DOAs.phi; break;    // added phi
                    case 27: axisLocked = DOAs.theta; break;  // added theta
                    case 28: axisLocked = DOAs.psi; break;    // added psi
                    case 3: axisLocked = DOAs.None; break;  // unlock
                    default: break; // no change
                }

            }
            else if (sender == InputBlocks[1]) // Cartesian control input: expect Vector
            {
                Vector cartesianInput = value as Vector;
                if (cartesianInput == null)
                {
                    Console.WriteLine("[OnNewInput] Warning: cannot cast input value to Vector.");
                    return;
                }
                currentCartesian = cartesianInput;
                Console.WriteLine($"[OnNewInput] Cartesian input received: {cartesianInput}");
            }

            // Only process if Cartesian vector is available
            if (currentCartesian == null)
            {
                Console.WriteLine("[OnNewInput] Cartesian input is null, skipping ProcessControl.");
                return;
            }
            // Process control logic now with latest gesture + cartesian vector
            ProcessControl(currentCartesian, currentGesture);
        }

        private void ProcessControl(Vector Prediction, int gesture)
        {
            Console.WriteLine($"Hafiz");
            Console.WriteLine($"[ProcessControl] Called with gesture={gesture}, axisLocked={axisLocked}");
            // For demonstration, only Cartesian algorithm shown here
            if (algorithm == Algorithms.Cartesian)
            {
                double step = 0.01;

                if (axisLocked != null)
                {
                    if (gesture == 4)  // move positive
                    {
                        controlDict[axisLocked] = Math.Min(controlDict[axisLocked] + step, 1.0);
                        Console.WriteLine($"[ControlAlgorithm] Moving locked axis {axisLocked} positive to {controlDict[axisLocked]:F3}");
                        // call method here
                        SendMotionCommand(axisLocked, controlDict[axisLocked]);
                    }
                    else if (gesture == 5) // move negative
                    {
                        controlDict[axisLocked] = Math.Max(controlDict[axisLocked] - step, -1.0);
                        Console.WriteLine($"[ControlAlgorithm] Moving locked axis {axisLocked} negative to {controlDict[axisLocked]:F3}");
                        // call method here
                        SendMotionCommand(axisLocked, controlDict[axisLocked]);
                    }
                    else
                    {
                        controlDict[axisLocked] = 0;
                        Console.WriteLine("[ControlAlgorithm] No motion gesture received for locked axis, sending zero velocity");
                        SendMotionCommand(axisLocked, 0);
                    }

                }
                else
                {
                    controlDict[DOAs.x] = Prediction[0];
                    controlDict[DOAs.y] = Prediction[1];
                    controlDict[DOAs.z] = Prediction[2];
                    controlDict[DOAs.phi] = Prediction[3];
                    controlDict[DOAs.theta] = Prediction[4];
                    controlDict[DOAs.psi] = Prediction[5];
                }

                // Log output
                var entries = new List<string>();
                foreach (var kvp in controlDict)
                    entries.Add($"{kvp.Key}={kvp.Value:F3}");

                string logString = string.Join(", ", entries);
                Console.WriteLine($"[ControlAlgorithm Output] {logString}");

                // Send out
                //SendOutput(controlDict);
            }
            else
            {
                // Add other algorithms as needed here
            }
        }
        public void SendMotionCommand(DOAs axis, double velocity)
        {
            if (controlDict.ContainsKey(axis))
            {
                controlDict[axis] = velocity;
                Console.WriteLine($"[SendMotionCommand] Set {axis} velocity to {velocity:F3}");

                // Send the updated control dictionary to the output
                SendOutput(controlDict);
            }
            else
            {
                Console.WriteLine($"[SendMotionCommand] Axis {axis} not found in controlDict");
            }
        }
    }
}
