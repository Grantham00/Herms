using System;
using System.Threading;
using Microsoft.SPOT;
using DriveStraightVelocityAuxiliary.Platform;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.Sensors;

namespace DriveStraightVelocityAuxiliary
{
    public class Program
    {

        public static void Main()
        {
            //Init a new array of 2 values. This is XY destination
            float[] coord = new float[2] {3, 3 };

            //new empty array for storing IMU yaw-pitch-roll data
            float[] ypr = new float[3];

            //Initialize all hardware to have factory default config
            Hardware._pidgey.ConfigFactoryDefault();
            Hardware._rightTalon.ConfigFactoryDefault();
            Hardware._leftTalon.ConfigFactoryDefault();

            /* Disable drivetrain/motors */
            Hardware._rightTalon.Set(ControlMode.PercentOutput, 0);
            // Hardware._leftVictor.Set(ControlMode.PercentOutput, 0);
            Hardware._leftTalon.Set(ControlMode.PercentOutput, 0);

            /* Set Neutral Mode */
            //Hardware._rightTalon.SetNeutralMode(NeutralMode.Coast)
            Hardware._rightTalon.SetNeutralMode(NeutralMode.Brake);
            Hardware._leftTalon.SetNeutralMode(NeutralMode.Brake);


            /** Feedback Sensor Configuration [Remote Sum and Yaw] */

            /* Configure the left Talon's selected sensor as local QuadEncoder */
            //Constants.BLANK accesses a list of constants stored in Platform.cs
            Hardware._leftTalon.ConfigSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,    // Local Feedback Source
                                                                Constants.PID_PRIMARY,      // PID Slot for Source [0, 1]
                                                                Constants.kTimeoutMs);      // Configuration Timeout

            /* Configure the Remote Talon's selected sensor as a remote sensor for the right Talon */
            Hardware._rightTalon.ConfigRemoteFeedbackFilter(Hardware._leftTalon.GetDeviceID(),                              // Device ID of Source
                                                            RemoteSensorSource.RemoteSensorSource_TalonSRX_SelectedSensor,  // Remote Feedback Source
                                                            Constants.REMOTE_0,                                             // Source number [0, 1]
                                                            Constants.kTimeoutMs);                                          // Configuration Timeout

            /* Setup Sum signal to be used for Distance */
            //Sum/Diff Signals This is part of the example DriveStraightVelocityAuxiliary[Quad] but unused in our implementation
            Hardware._rightTalon.ConfigSensorTerm(SensorTerm.SensorTerm_Sum0, FeedbackDevice.RemoteSensor0, Constants.kTimeoutMs); // Feedback Device of Remote Talon
            Hardware._rightTalon.ConfigSensorTerm(SensorTerm.SensorTerm_Sum1, FeedbackDevice.QuadEncoder, Constants.kTimeoutMs);   // Quadrature Encoder of current Talon

            /* Setup Difference signal to be used for Turn */
            Hardware._rightTalon.ConfigSensorTerm(SensorTerm.SensorTerm_Diff1, FeedbackDevice.RemoteSensor0, Constants.kTimeoutMs);
            Hardware._rightTalon.ConfigSensorTerm(SensorTerm.SensorTerm_Diff0, FeedbackDevice.QuadEncoder, Constants.kTimeoutMs);

            /* Configure Sum [Sum of both QuadEncoders] to be used for Primary PID Index */
            Hardware._rightTalon.ConfigSelectedFeedbackSensor(FeedbackDevice.SensorSum, Constants.PID_PRIMARY, Constants.kTimeoutMs);

            /* Scale Feedback by 0.5 to half the sum of Distance */
            Hardware._rightTalon.ConfigSelectedFeedbackCoefficient(0.5f,                   // Coefficient
                                                                    Constants.PID_PRIMARY,  // PID Slot of Source 
                                                                    Constants.kTimeoutMs);  // Configuration Timeout

            /* Configure Difference [Difference between both QuadEncoders] to be used for Auxiliary PID Index */
            Hardware._rightTalon.ConfigSelectedFeedbackSensor(FeedbackDevice.SensorDifference, Constants.PID_TURN, Constants.kTimeoutMs);

            /* Scale the Feedback Sensor using a coefficient */
            Hardware._rightTalon.ConfigSelectedFeedbackCoefficient(Constants.kTurnTravelUnitsPerRotation / Constants.kEncoderUnitsPerRotation,
                                                                    Constants.PID_TURN,
                                                                    Constants.kTimeoutMs);

            /* Configure output and sensor direction */
            //Inverted reverses motor direction. SensorPhase reverses encoder pos/neg direction
            Hardware._rightTalon.SetInverted(false);
            //Hardware._leftVictor.SetInverted(false);    // Output on victor
            Hardware._rightTalon.SetSensorPhase(false);
            Hardware._leftTalon.SetSensorPhase(true);   // Talon only used for sensor

            /* Set status frame periods to ensure we don't have stale data */
            Hardware._rightTalon.SetStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 10, Constants.kTimeoutMs);
            Hardware._rightTalon.SetStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
            Hardware._rightTalon.SetStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 10, Constants.kTimeoutMs);
            Hardware._pidgey.SetStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 5, Constants.kTimeoutMs);
            Hardware._leftTalon.SetStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, Constants.kTimeoutMs);

            /* Configure Neutral Deadband */
            Hardware._rightTalon.ConfigNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);
            Hardware._leftTalon.ConfigNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);

            /* Configure max peak output [Open and closed loop modes]
             * Can use configClosedLoopPeakOutput() for only closed Loop modes
             */
            Hardware._rightTalon.ConfigPeakOutputForward(+1.0f, Constants.kTimeoutMs);
            Hardware._rightTalon.ConfigPeakOutputReverse(-1.0f, Constants.kTimeoutMs);
            Hardware._leftTalon.ConfigPeakOutputForward(+1.0f, Constants.kTimeoutMs);
            Hardware._leftTalon.ConfigPeakOutputReverse(-1.0f, Constants.kTimeoutMs);

            /* FPID Gains for distance closed loop */
            Hardware._rightTalon.Config_kP(Constants.kSlot_Velocit, Constants.kGains_Velocit.kP, Constants.kTimeoutMs);
            Hardware._rightTalon.Config_kI(Constants.kSlot_Velocit, Constants.kGains_Velocit.kI, Constants.kTimeoutMs);
            Hardware._rightTalon.Config_kD(Constants.kSlot_Velocit, Constants.kGains_Velocit.kD, Constants.kTimeoutMs);
            Hardware._rightTalon.Config_kF(Constants.kSlot_Velocit, Constants.kGains_Velocit.kF, Constants.kTimeoutMs);
            Hardware._rightTalon.Config_IntegralZone(Constants.kSlot_Velocit, (int)Constants.kGains_Velocit.kIzone, Constants.kTimeoutMs);
            Hardware._rightTalon.ConfigClosedLoopPeakOutput(Constants.kSlot_Velocit, Constants.kGains_Velocit.kPeakOutput, Constants.kTimeoutMs);

            /* Allow an error of 0, always enforce closed loop */
            Hardware._rightTalon.ConfigAllowableClosedloopError(Constants.kSlot_Velocit, 0, Constants.kTimeoutMs);

            /* FPID Gains for turn closed loop */
            Hardware._rightTalon.Config_kP(Constants.kSlot_Turning, Constants.kGains_Turning.kP, Constants.kTimeoutMs);
            Hardware._rightTalon.Config_kI(Constants.kSlot_Turning, Constants.kGains_Turning.kI, Constants.kTimeoutMs);
            Hardware._rightTalon.Config_kD(Constants.kSlot_Turning, Constants.kGains_Turning.kD, Constants.kTimeoutMs);
            Hardware._rightTalon.Config_kF(Constants.kSlot_Turning, Constants.kGains_Turning.kF, Constants.kTimeoutMs);
            Hardware._rightTalon.Config_IntegralZone(Constants.kSlot_Turning, (int)Constants.kGains_Turning.kIzone, Constants.kTimeoutMs);
            Hardware._rightTalon.ConfigClosedLoopPeakOutput(Constants.kSlot_Turning, Constants.kGains_Turning.kPeakOutput, Constants.kTimeoutMs);

            /* Allow an error of 10 units on a 3600 unit resolution for yaw (1 Degree of Error) */
            Hardware._rightTalon.ConfigAllowableClosedloopError(Constants.kSlot_Turning, 10, Constants.kTimeoutMs);

            /* 1ms per loop.  PID loop can be slowed down if need be. */
            int closedLoopTimeMs = 1;
            Hardware._rightTalon.ConfigSetParameter(CTRE.Phoenix.LowLevel.ParamEnum.ePIDLoopPeriod, closedLoopTimeMs, 0x00, 0, Constants.kTimeoutMs);   // Primary
            Hardware._rightTalon.ConfigSetParameter(CTRE.Phoenix.LowLevel.ParamEnum.ePIDLoopPeriod, closedLoopTimeMs, 0x00, 1, Constants.kTimeoutMs);   // Turn (Auxiliary)

            /* False means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
             * True means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
             */
            Hardware._rightTalon.ConfigAuxPIDPolarity(false, Constants.kTimeoutMs);

            /* Latched values to detect on-press events for buttons */
            //Reason for having a btns and _btns unclear
            bool[] _btns = new bool[Constants.kNumButtonsPlusOne];
            bool[] btns = new bool[Constants.kNumButtonsPlusOne];

            /* Initialize */
            bool _state = false;
            bool _firstCall = true;
            //float _targetAngle = 0;

            //set encoder and IMU values to zero
            ZeroSensors();

            while (true)
            {
                /* Enable motor controllers if gamepad connected */
                if (Hardware._gamepad.GetConnectionStatus() == CTRE.Phoenix.UsbDeviceConnection.Connected)
                    CTRE.Phoenix.Watchdog.Feed();

                /* Gamepad value processing */
                float forward = -1 * Hardware._gamepad.GetAxis(1);
                float turn = 1 * Hardware._gamepad.GetAxis(0);
                //Deadband creates area of rejecting small gamepad inputs
                CTRE.Phoenix.Util.Deadband(ref forward);
                CTRE.Phoenix.Util.Deadband(ref turn);

                /* Button processing */
                //Pass GetButtons() an array (btns), it will fill it with the button values from gamepad
                Hardware._gamepad.GetButtons(btns);

                //When you press button 2, toggle the program state (between joystick drive and go-to-coordinates)
                if (btns[2] && !_btns[2])
                {
                    _state = !_state;           // Toggle state
                    _firstCall = true;          // State change, do first call operation
                    //_targetAngle = Hardware._rightTalon.GetSelectedSensorPosition(1);
                    ZeroSensors();
                    ZeroPosition();
                    Debug.Print("btns[2] && !_btns[2]");
                }
                //Zero sensors when button 1
                else if (btns[1] && !_btns[1])
                {
                    ZeroSensors();              // Zero sensors
                    Debug.Print("btns[1] && !_btns[1]");
                }
                //Copy btns to _btns
                System.Array.Copy(btns, _btns, Constants.kNumButtonsPlusOne);

                if (!_state)
                {
                    if (_firstCall)
                    {
                        Debug.Print("This is basic Arcade Drive with Arbitrary Feed-forward.\n");
                        //ZeroSensors();
                    }
                    Hardware._rightTalon.SetInverted(false);
                    Hardware._leftTalon.SetInverted(false);
                    /* Use Arbitrary FeedForward to create an Arcade Drive Control by modifying the forward output */
                    //Talon.Set(ControlMode[PercentOutput, current, pos, velocity], output number, DemandType[feedforward, PID], output number)
                    Hardware._rightTalon.Set(ControlMode.PercentOutput, forward / 2, DemandType.ArbitraryFeedForward, -turn / 2);
                    Hardware._leftTalon.Set(ControlMode.PercentOutput, forward / 2, DemandType.ArbitraryFeedForward, +turn / 2);
                    //Debug.Print("RightEncoder: " + Hardware._rightTalon.GetSelectedSensorPosition().ToString());
                    Hardware._pidgey.GetYawPitchRoll(ypr);
                    //Debug.Print("Magnetometer: " + ypr[0]);
                    //Debug.Print((forward/2).ToString());
                    //Debug.Print((turn / 2).ToString());
                }
                else
                {
                    if (_firstCall)
                    {
                        ZeroPosition();
                        //Puase .25 sec
                        Thread.Sleep(250);

                        /* Determine which slot affects which PID */
                        Hardware._rightTalon.SelectProfileSlot(Constants.kSlot_Velocit, Constants.PID_PRIMARY);
                        Hardware._rightTalon.SelectProfileSlot(Constants.kSlot_Turning, Constants.PID_TURN);

                        Debug.Print("Starting rotateDrive...");
                        rotateDrive(coord);
                        _state = !_state;
                    }

                    /* Calculate targets from gamepad inputs */

                    //Hardware._leftTalon.Set(ControlMode.Velocity, target_unitsPer100ms_left, DemandType.AuxPID, 0);

                }
                _firstCall = false;

                Thread.Sleep(10);
            }
        }

        static void Forward()
        {
            float target_RPM = 125; /* +- 500 RPM */
            float target_unitsPer100ms = target_RPM * Constants.kSensorUnitsPerRotation / 600.0f;

            //float target_turn = _targetAngle;
            Hardware._rightTalon.SetInverted(false);
            Hardware._leftTalon.SetInverted(false);

            /* Configured for Velocity Closed Loop on Quad Encoders' Sum and Auxiliary PID on Quadrature Encoders' Difference*/
            Hardware._rightTalon.Set(ControlMode.Velocity, target_unitsPer100ms, DemandType.AuxPID, 0);

            Hardware._leftTalon.Follow(Hardware._rightTalon, FollowerType.AuxOutput1);
        }
        static void ForwardDistance(double dist)
        {
            //convert feet to encodervalues
            double encoderDistance = (8000 / 31.41) * ((dist * 12)-3);
            //loop while left or right talon are less than encoderDistance
            while ((Hardware._leftTalon.GetSelectedSensorPosition() < encoderDistance || Hardware._rightTalon.GetSelectedSensorPosition() < encoderDistance) && !Hardware._gamepad.GetButton(3))
            {
                //Send robot forward at 35%
                Forward();
                CTRE.Phoenix.Watchdog.Feed();

                Debug.Print("dist: " + dist);
                Debug.Print("EncoderDistance: " + encoderDistance.ToString());
                Debug.Print("\n");

                Debug.Print("leftEncoder: " + Hardware._leftTalon.GetSelectedSensorPosition().ToString());
                Debug.Print("\n");

                Debug.Print("rightEncoder: " + Hardware._rightTalon.GetSelectedSensorPosition().ToString());
                Debug.Print("\n");
                Thread.Sleep(10);
            }
            Debug.Print("DUN");
            //After arriving, Change controlmode to 0% (stop)
            Hardware._rightTalon.Set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, 0);
            Hardware._leftTalon.Set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, 0);
        }

        static void TurnLeftTheta(double theta)
        {
            Debug.Print("TurnLeftTheta");
            //new empty ypr array
            float[] ypr = new float[3];

            //SetYaw of IMU to zero (also in ZeroSensors())
            Hardware._pidgey.SetYaw(0);
            //Fill ypr array with YawPitchRoll from IMU
            Hardware._pidgey.GetYawPitchRoll(ypr);
            //print current yaw (zero) and destination theta
            Debug.Print("ypr[0] = " + ypr[0].ToString());
            Debug.Print("theta = " + theta.ToString());
            Hardware._pidgey.GetYawPitchRoll(ypr);
            
            //While ypr[0] (Yaw) is less than destination theta then loop this. Kill when Button3
            while (ypr[0] < (theta - 3.5) && !Hardware._gamepad.GetButton(3))
            {
                //float target_RPM = 125; /* +- 500 RPM */
                //float target_unitsPer100ms = target_RPM * Constants.kSensorUnitsPerRotation / 600.0f;

                //Instead of doing math, hardcode a velocity for rotating
                float target_unitsPer100ms = 853.33F;

                //Invert Right, Uninvert left to turn left
                Hardware._rightTalon.SetInverted(true);
                Hardware._rightTalon.Set(ControlMode.Velocity, target_unitsPer100ms, DemandType.AuxPID, 0);
                Hardware._leftTalon.SetInverted(false);
                //set left to follow right talon(but inverted)
                Hardware._leftTalon.Follow(Hardware._rightTalon, FollowerType.AuxOutput1);

                //Update ypr values
                Hardware._pidgey.GetYawPitchRoll(ypr);

                //Hardware._pidgey.GetYawPitchRoll(ypr);

                //Watcdog.Feed gives motor control 
                CTRE.Phoenix.Watchdog.Feed();
                //Print the following to the debug screen
                Debug.Print("TurnRightTheta\n");
                //print yaw
                Debug.Print("Magnetometer " + (ypr[0].ToString()));
                Debug.Print("\n");

                Debug.Print("\n");
                Thread.Sleep(10);
            }
        }
        //same as TurnLeftTheta, but inverted motor config and negative target_unitsPer100ms
        static void TurnRightTheta(double theta)
        {
            Debug.Print("TurnRightTheta");

            float[] ypr = new float[3];

            Hardware._pidgey.SetYaw(0);
            Hardware._pidgey.GetYawPitchRoll(ypr);
            Debug.Print("ypr[0] = " + ypr[0].ToString());
            Debug.Print("theta = " + theta.ToString());

            while (ypr[0] > -(theta - 3.5) && !Hardware._gamepad.GetButton(3))
            {
                //float target_RPM = 125; /* +- 500 RPM */
                //float target_unitsPer100ms = target_RPM * Constants.kSensorUnitsPerRotation / 600.0f;
                float target_unitsPer100ms = -853.33F;

                Hardware._rightTalon.SetInverted(false);
                Hardware._rightTalon.Set(ControlMode.Velocity, target_unitsPer100ms, DemandType.AuxPID, 0);
                Hardware._leftTalon.SetInverted(true);
                Hardware._leftTalon.Follow(Hardware._rightTalon, FollowerType.AuxOutput1);

                Hardware._pidgey.GetYawPitchRoll(ypr);

                //Hardware._pidgey.GetYawPitchRoll(ypr);
                CTRE.Phoenix.Watchdog.Feed();
                //Print the following to the debug screen
                Debug.Print("TurnRightTheta\n");
                Debug.Print("Magnetometer " + (ypr[0].ToString()));
                Debug.Print("\n");

                Debug.Print("\n");
                Thread.Sleep(10);
            }
            Debug.Print("DUN");
            Hardware._rightTalon.Set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, 0);
            Hardware._leftTalon.Set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, 0);
        }

        
        static void Turn180()
        {
            TurnLeftTheta(180);
        }

        static void rotateDrive(float[] coord)
        {
            //Do trig to convert X and Y to polar
            double theta = (System.Math.Atan(coord[1] / coord[0])) * (180 / System.Math.PI);
            double radius = System.Math.Sqrt((coord[0] * coord[0]) + (coord[1] * coord[1]));
            ZeroPosition();
            ZeroSensors();
            //If theta is positive, turn right
            if (theta > 0)
            {
                TurnRightTheta(theta);
                ZeroPosition();

                ForwardDistance(radius);

            }
            //if theta is negative, turn left
            else if (theta < 0)
            {
                TurnLeftTheta(theta);
                ZeroPosition();

                ForwardDistance(radius);
            }


        }

        /** Zero all sensors used in Auxiliary Example */
        static void ZeroSensors()
        {
            Hardware._rightTalon.GetSensorCollection().SetQuadraturePosition(0, Constants.kTimeoutMs);
            Hardware._leftTalon.GetSensorCollection().SetQuadraturePosition(0, Constants.kTimeoutMs);
            Hardware._pidgey.SetYaw(0, Constants.kTimeoutMs);
            Hardware._pidgey.SetAccumZAngle(0, Constants.kTimeoutMs);
            Debug.Print("[Sensors] All sensors are zeroed.\n");
        }

        /** Zero all QuadEncoder Positions used in Auxiliary Example */
        static void ZeroPosition()
        {
            Hardware._rightTalon.GetSensorCollection().SetQuadraturePosition(0, Constants.kTimeoutMs);
            Hardware._leftTalon.GetSensorCollection().SetQuadraturePosition(0, Constants.kTimeoutMs);
            Debug.Print("[Position] All sensors are zeroed.\n");
        }
    }
}

