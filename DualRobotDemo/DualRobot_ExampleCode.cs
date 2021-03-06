using System;
using System.Threading;
using DualRobotLib;

namespace DualRobotDemo
{
    internal class DualRobot_ExampleCode
    {
        /// Methods
        public void Connect()
        {
            DualRobotLib.Core core = new Core();

            // 1. establish connection
            // core.Connect(Model.CR7, "192.168.3.124", 60008);
            var result = core.Connect(Model.CR7, "127.0.0.1", 60008);

            Console.WriteLine("result:" + result);
        }

        public void Disconnect()
        {
            DualRobotLib.Core core = new Core();

            // 1. establish connection
            // core.Connect(Model.CR7, "192.168.3.124", 60008);
            var result = core.Connect(Model.CR7, "127.0.0.1", 60008);
            result = core.Connect(Model.CR7, "127.0.0.1", 60008);
            result = core.Disconnect(Model.CR7);
            result = core.Connect(Model.CR7, "127.0.0.1", 60008);
            result = core.Disconnect(Model.CR7);
            result = core.Connect(Model.CR7, "127.0.0.1", 60008);
            result = core.Disconnect(Model.CR7);
            result = core.Connect(Model.CR7, "127.0.0.1", 60008);
            result = core.Disconnect(Model.CR7);
            result = core.Connect(Model.CR7, "127.0.0.1", 60008);


            core.Connect(Model.CR15, "127.0.0.1", 9021);
            core.Disconnect(Model.CR15);
            core.Connect(Model.CR15, "127.0.0.1", 9021);

            Console.WriteLine("result:" + result);
        }

        public void GetTcpDistance()
        {
            // (1) prerequisite: finish step3
            // (2) define tcp
            // (3) get tcp distance in real time

            // (1)
            // step3:
            // a. uninstall calibration pin.
            // b. 3 point method.
            // c. record calibrated user frame data. 
            //  c.1. robot connection.
            //  c.2. record calibrated user frame data. 

            DualRobotLib.Core core = new Core();
            core.Connect(Model.CR15, "127.0.0.1", 9021);
            core.Connect(Model.CR7, "127.0.0.1", 60008);
            // core.Connect(Model.CR15, "192.168.3.125", 60008);
            // core.Connect(Model.CR7, "192.168.3.124", 60008);

            // examples: calibrated user frame data
            double[] Pos_Cr7_CalliBase = { 708.92, 1.98, 51.15, -2.11, 46.69, -0.4 };
            double[] Pos_Cr15_CalliBase = { 1337.41, -1.24, -386.08, -178.4, 44.25, -177.79 };
            core.RobotBaseCalibrationInit(Pos_Cr7_CalliBase, Pos_Cr15_CalliBase);

            // (2)
            float[] tcp_cr7 = { 9, 0, 123, 0, -45, 0 };
            float[] tcp_cr15 = { 0, 0, 140, 0, 45, -90 };
            core.SetTCP(Model.CR15, tcp_cr15);
            core.SetTCP(Model.CR7, tcp_cr7);

            // (3)
            var tcp_distance = core.GetTcpDistance();

            Console.WriteLine("tcp_distance: " + tcp_distance);
        }

        public void GetSpanningRange()
        {
            // 0, 200, 180, 100, 90, 0
            // 200, 500, 300, 250, 150, 0
            // 400, 600, 500, 300, 250, 0
            // 500, 600, 500, 300, 250, 0
            // 800, 800, 600, 400, 300, 0
            // 1000, 800, 400, 400, 200, 0


            DualRobotLib.Core core = new Core();

            var h = core.GetSpanningHeight(1000);

            var w = core.GetSpanningWidth(1000);

            Console.WriteLine("width: " + w);
            Console.WriteLine("height: " + h);

        }

        public void GetMoveFlag()
        {
            // Thread th1 = new Thread(() => core.thread_MoveFlag(Model.CR15));
            // th1.Start();
        }

        public void BasicMovementDemo()
        {
            DualRobotLib.Core core = new Core();
            core.Connect(Model.CR15, "127.0.0.1", 9021);
            core.Connect(Model.CR7, "127.0.0.1", 60008);

            // For Cr15

            // 1. Get UF Cur Pos
            var uf_pos = core.GetUFCurPos(Model.CR15);

            // 2. Get RB Cur Pos
            var rb_pos = core.GetRBCurPos(Model.CR15);

            // 3. UFMove
            float[] tcp_cr15 = { 0, 0, 140, 0, 45, -90 };
            core.SetTCP(Model.CR15, tcp_cr15);
            core.SetUserFrame(Model.CR15);

            double[] pos = new[] { 0.0, 0.0, 200, 0.0, 0.0, 0.0 };
            core.UFMove(Model.CR15, pos);

            // 4. RBMove
            double[] pos1 = new[] { 1277.247, 4.829875, 100.56811, -178.5574, 36.90373, -178.0444 };
            core.UFMove(Model.CR15, pos1);

            // For Cr7

            // 1. Get UF Cur Pos
            var uf_pos1 = core.GetUFCurPos(Model.CR7);

            // 2. Get RB Cur Pos
            var rb_pos1 = core.GetRBCurPos(Model.CR7);

            // 3. UFMove
            float[] tcp_cr7 = { 9, 0, 123, 0, -45, 0 };
            core.SetTCP(Model.CR7, tcp_cr7);
            core.SetUserFrame(Model.CR7);

            double[] pos2 = new[] { 0.0, 0.0, 50, 0.0, 0.0, 0.0 };
            core.UFMove(Model.CR7, pos2);

            // 4. RBMove
            double[] pos3 = new[] { 624.72, 4.60, 270.40, -1.87, 39.35, -0.05 };
            core.UFMove(Model.CR7, pos3);
            
        }

        /// CR7
        public void CR7_MoveTo_Demo()
        {
            DualRobotLib.Core core = new Core();

            // 1. establish connection
            // core.Connect(Model.CR7, "192.168.3.124", 60008);
            core.Connect(Model.CR7, "127.0.0.1", 60008);

            // 2. movement test

            core.MoveTo(Model.CR7, Position.Home);

            core.MoveTo(Model.CR7, Position.InitPos_Scene1B);

            core.MoveTo(Model.CR7, Position.Home);

            core.MoveTo(Model.CR7, Position.InitPos_Scene1B);
        }

        public void CR7_BasicMovement_Demo()
        {
            DualRobotLib.Core core = new Core();

            // 1. establish connection
            core.Connect(Model.CR7, "192.168.3.124", 60008);
            // core.Connect(Model.CR7, "127.0.0.1", 60008);

            // 2. movement test

            // 2.1 define the tcp
            float[] tcp_cr7 = { 9, 0, 123, 0, -45, 0 };
            core.SetTCP(Model.CR7, tcp_cr7);

            // 2.2 define the tcp speed
            core.SetSpeed(Model.CR7, 100);

            // 2.3 define the user frame
            core.SetUserFrame(Model.CR7);

            // 2.4.1 Set Single Point
            float[] pos_cr7 = { 50, 0, 0, 0, 0, 0 };
            core.SetSinglePoint(Model.CR7, pos_cr7);

            // 2.4.2 Move
            core.MoveSinglePoint(Model.CR7);

            // 2.4.3 Set Single Point
            float[] pos_cr7_2 = { -50, 0, 0, 0, 0, 0 };
            core.SetSinglePoint(Model.CR7, pos_cr7_2);

            // 2.4.4 Move
            core.MoveSinglePoint(Model.CR7);

            // 2.4.5 Set Single Point
            float[] pos_cr7_3 = { 0, 0, 0, 0, 0, 0 };
            core.SetSinglePoint(Model.CR7, pos_cr7_3);

            // 2.4.6 Move
            core.MoveSinglePoint(Model.CR7);
        }

        /// CR15
        public void CR15_MoveTo_Demo()
        {
            DualRobotLib.Core core = new Core();

            // 1. establish connection
            core.Connect(Model.CR15, "127.0.0.1", 9021);
            // core.Connect(Model.CR15, "192.168.3.125", 60008);

            // 2. movement test

            core.MoveTo(Model.CR15, Position.Home);

            core.MoveTo(Model.CR15, Position.InitPos_Scene1B);

            core.MoveTo(Model.CR15, Position.Home);

            core.MoveTo(Model.CR15, Position.InitPos_Scene1B);
        }

        public void CR15_BasicMovement_Demo()
        {
            DualRobotLib.Core core = new Core();
            core.Connect(Model.CR15, "127.0.0.1", 9021);
            core.Connect(Model.CR7, "127.0.0.1", 60008);

            // Get UF Cur Pos
            var uf_pos = core.GetUFCurPos(Model.CR15);

            // Get RB Cur Pos
            var rb_pos = core.GetRBCurPos(Model.CR15);

            // UFMove
            float[] tcp_cr15 = { 0, 0, 140, 0, 45, -90 };
            core.SetTCP(Model.CR15, tcp_cr15);
            core.SetUserFrame(Model.CR15);

            double[] pos = new[] { 0.0, 0.0, 200, 0.0, 0.0, 0.0 };
            core.UFMove(Model.CR15, pos);

            // RBMove
            double[] pos1 = new[] { 1277.247, 4.829875, 100.56811, -178.5574, 36.90373, -178.0444 };
            core.UFMove(Model.CR15, pos1);
        }

        /// DualRobot
        public void DualRobot_Scene1A_Demo_WithCalibration()
        {
            // step1: Robot Installation & Calibration. (skip)
            // step2: TCP Calibration.
            // step3: RobotBase Calibration.
            // step4: Measurement Initialization.
            // step5: Scene2

            // step2:
            // a. install calibration tool.
            // b. install calibration plate & calibration pin.
            // c. 6 points method.
            // d. record calibrated tcp data.

            // step3:
            // a. uninstall calibration pin.
            // b. 3 point method.
            // c. record calibrated user frame data. 
            //  c.1. robot connection.
            //  c.2. record calibrated user frame data. 
            //**c.3. record calibrated rotate plate center data.

            DualRobotLib.Core core = new Core();
            core.Connect(Model.CR15, "127.0.0.1", 9021);
            core.Connect(Model.CR7, "127.0.0.1", 60008);
            // core.Connect(Model.CR15, "192.168.3.125", 60008);
            // core.Connect(Model.CR7, "192.168.3.124", 60008);

            // examples: calibrated user frame data
            double[] Pos_Cr7_CalliBase = { 825.25, 6.38, 134.67, -2.11, 46.69, -0.40 };
            double[] Pos_Cr15_CalliBase = { 1223.50, -1.77, -304.52, 1.61, 44.25, -177.78 };
            core.RobotBaseCalibrationInit(Pos_Cr7_CalliBase, Pos_Cr15_CalliBase);

            // examples: rotate plate center data.
            double[] Pos_Cr7_PlateCenter = { 828.51, -50, 561.10, 2.74, 1.11, -90.00 };
            core.RotationPlateCalibrationInit(Pos_Cr7_PlateCenter);

            // step4:
            // a. install testing tools.
            // b. tools setting, tcp speed.
            // c. define scene params.
            // d. robots move to init position.
            // e. define user frame
            //
            // b. examples: tcp data
            float[] tcp_cr7 = { 9, 0, 123, 0, -45, 0 };
            float[] tcp_cr15 = { 0, 134, 182, 0, 0, 90 };
            core.SetTCP(Model.CR15, tcp_cr15);
            core.SetTCP(Model.CR7, tcp_cr7);
            core.SetSpeed(Model.CR15, 100);
            core.SetSpeed(Model.CR7, 100);
            // c. examples.
            double[] param = { 300, 180, 10, 250, 90, 45 };
            core.SceneParamInit(SceneName.Scene1A, param);
            // d.
            core.SceneRobotInit(SceneName.Scene1A);
            // e.
            core.SetUserFrame(Model.CR15);
            core.SetUserFrame(Model.CR7);

            // step5: Scene1A
            core.Scene1A(MovementType.QuickCheck);
        }

        public void DualRobot_Scene1B_Demo_WithCalibration()
        {
            // step1: Robot Installation & Calibration. (skip)
            // step2: TCP Calibration.
            // step3: RobotBase Calibration.
            // step4: Measurement Initialization.
            // step5: Scene1B

            // step2:
            // a. install calibration tool.
            // b. install calibration plate & calibration pin.
            // c. 4 points method.
            // d. record calibrated tcp data.

            // step3:
            // a. uninstall calibration pin.
            // b. 3 point method.
            // c. record calibrated user frame data. 
            //  c.1. robot connection.
            //  c.2. record calibrated user frame data. 

            DualRobotLib.Core core = new Core();
            core.Connect(Model.CR15, "127.0.0.1", 9021);
            core.Connect(Model.CR7, "127.0.0.1", 60008);
            // core.Connect(Model.CR15, "192.168.3.125", 60008);
            // core.Connect(Model.CR7, "192.168.3.124", 60008);

            // examples: calibrated user frame data
            double[] Pos_Cr7_CalliBase = { 825.25, 6.38, 134.67, -2.11, 46.69, -0.40 };
            double[] Pos_Cr15_CalliBase = { 1223.50, -1.77, -304.52, 1.61, 44.25, -177.78 };
            core.RobotBaseCalibrationInit(Pos_Cr7_CalliBase, Pos_Cr15_CalliBase);

            // step4:
            // a. install testing tools.
            // b. tools setting, tcp speed.
            // c. define scene params.
            // d. robots move to init position.
            // e. define user frame
            //
            // b. examples: tcp data
            float[] tcp_cr7 = { 9, 0, 123, 0, -45, 0 };
            float[] tcp_cr15 = { 0, 0, 140, 0, 45, -90 };
            core.SetTCP(Model.CR15, tcp_cr15);
            core.SetTCP(Model.CR7, tcp_cr7);
            core.SetSpeed(Model.CR15, 100);
            core.SetSpeed(Model.CR7, 100);
            // c. examples.
            // double[] param = { 1000, 130, 10, 180, 90, 180, 45 };
            // double[] param = { 1000, 130, 0.5, 180, 90, 180, 45 };
            double[] param = { 200, 102, 34, 102, 34, 180, 60 };
            core.SceneParamInit(SceneName.Scene1B, param);
            // d.
            core.SceneRobotInit(SceneName.Scene1B);
            // e.
            core.SetUserFrame(Model.CR15);
            core.SetUserFrame(Model.CR7);

            // step5: Scene1B
            core.Scene1B(MovementType.QuickCheck);
        }

        public void DualRobot_Scene1C_Demo_WithCalibration()
        {
            // step1: Robot Installation & Calibration. (skip)
            // step2: TCP Calibration.
            // step3: RobotBase Calibration.
            // step4: Measurement Initialization.
            // step5: Scene1B

            // step2:
            // a. install calibration tool.
            // b. install calibration plate & calibration pin.
            // c. 4 points method.
            // d. record calibrated tcp data.

            // step3:
            // a. uninstall calibration pin.
            // b. 3 point method.
            // c. record calibrated user frame data. 
            //  c.1. robot connection.
            //  c.2. record calibrated user frame data. 

            DualRobotLib.Core core = new Core();
            core.Connect(Model.CR15, "127.0.0.1", 9021);
            core.Connect(Model.CR7, "127.0.0.1", 60008);
            // core.Connect(Model.CR15, "192.168.3.125", 60008);
            // core.Connect(Model.CR7, "192.168.3.124", 60008);

            // examples: calibrated user frame data
            double[] Pos_Cr7_CalliBase = { 825.25, 6.38, 134.67, -2.11, 46.69, -0.40 };
            double[] Pos_Cr15_CalliBase = { 1223.50, -1.77, -304.52, 1.61, 44.25, -177.78 };
            core.RobotBaseCalibrationInit(Pos_Cr7_CalliBase, Pos_Cr15_CalliBase);

            // step4:
            // a. install testing tools.
            // b. tools setting, tcp speed.
            // c. define scene params.
            // d. robots move to init position.
            // e. define user frame
            //
            // b. examples: tcp data
            float[] tcp_cr7 = { 9, 0, 123, 0, -45, 0 };
            float[] tcp_cr15 = { 0, 0, 140, 0, 45, -90 };
            core.SetTCP(Model.CR15, tcp_cr15);
            core.SetTCP(Model.CR7, tcp_cr7);
            core.SetSpeed(Model.CR15, 100);
            core.SetSpeed(Model.CR7, 100);
            // c. examples.
            double[] param = { 250, 300, 300, 100, 100, 0 };
            core.SceneParamInit(SceneName.Scene1C, param);
            // d.
            core.SceneRobotInit(SceneName.Scene1C);
            // e.
            core.SetUserFrame(Model.CR15);
            core.SetUserFrame(Model.CR7);

            // step5: Scene1C
            core.Scene1C(MovementType.QuickCheck);
        }

        public void DualRobot_Scene2_Demo_WithCalibration()
        {
            // step1: Robot Installation & Calibration. (skip)
            // step2: TCP Calibration.
            // step3: RobotBase Calibration.
            // step4: Measurement Initialization.
            // step5: Scene2

            // step2:
            // a. install calibration tool.
            // b. install calibration plate & calibration pin.
            // c. 6 points method.
            // d. record calibrated tcp data.

            // step3:
            // a. uninstall calibration pin.
            // b. 3 point method.
            // c. record calibrated user frame data. 
            //  c.1. robot connection.
            //  c.2. record calibrated user frame data. 
            //**c.3. record calibrated rotate plate center data.

            DualRobotLib.Core core = new Core();
            core.Connect(Model.CR15, "127.0.0.1", 9021);
            core.Connect(Model.CR7, "127.0.0.1", 60008);
            // core.Connect(Model.CR15, "192.168.3.125", 60008);
            // core.Connect(Model.CR7, "192.168.3.124", 60008);

            // examples: calibrated user frame data
            double[] Pos_Cr7_CalliBase = { 825.25, 6.38, 134.67, -2.11, 46.69, -0.40 };
            double[] Pos_Cr15_CalliBase = { 1223.50, -1.77, -304.52, 1.61, 44.25, -177.78 };
            core.RobotBaseCalibrationInit(Pos_Cr7_CalliBase, Pos_Cr15_CalliBase);

            // examples: rotate plate center data.
            double[] Pos_Cr7_PlateCenter = { 828.51, -50, 61.10, 2.74, 1.11, -90.00 };
            core.RotationPlateCalibrationInit(Pos_Cr7_PlateCenter);

            // step4:
            // a. install testing tools.
            // b. tools setting, tcp speed.
            // c. define scene params.
            // d. robots move to init position.
            // e. define user frame
            //
            // b. examples: tcp data
            float[] tcp_cr7 = { -125, 50, 45, 90, 0, 180 };
            float[] tcp_cr15 = { 0, 134, 182, 0, 0, 90 };
            core.SetTCP(Model.CR15, tcp_cr15);
            core.SetTCP(Model.CR7, tcp_cr7);
            core.SetSpeed(Model.CR15, 100);
            core.SetSpeed(Model.CR7, 100);
            // c. examples.
            double[] param = { 430, 100, 10, 250, 90, 45 };
            core.SceneParamInit(SceneName.Scene2, param);
            // d.
            core.SceneRobotInit(SceneName.Scene2);
            // e.
            core.SetUserFrame(Model.CR15);
            core.SetUserFrame(Model.CR7);

            // step5: Scene2
            core.Scene2(MovementType.QuickCheck);
        }

        /// Motor
        public void SerialPortTest()
        {
            DualRobotLib.Core core = new Core();

            core.MotorInitTest();
        }

        public void threadDemo()
        {
            // public delegate void SumOfNumbersCallback(int SumOfNumbers);
            //
            // class Number
            // {
            //     private int _target;
            //     SumOfNumbersCallback _callbackMethod;
            //
            //     public Number(int target, SumOfNumbersCallback callbackMethod)
            //     {
            //         this._target = target;
            //         _callbackMethod = callbackMethod;
            //     }
            //
            //     public void PrintSumOfNumbers()
            //     {
            //         int sum = 0;
            //         for (int i = 1; i <= _target; i++)
            //         {
            //             sum += i;
            //         }
            //
            //         if (_callbackMethod != null)
            //             _callbackMethod(sum);
            //     }
            // }
            //
            // public static void PrintSum(int sum)
            // {
            //     Console.WriteLine("Sum of numbers = " + sum);
            // }
            //
            // public MainWindow()
            // {
            //     int target = 5;
            //
            //     SumOfNumbersCallback callback = new SumOfNumbersCallback(PrintSum);
            //
            //     Number number = new Number(target, callback);
            //
            //     Thread T1 = new Thread(new ThreadStart(number.PrintSumOfNumbers));
            //
            //     T1.Start();
            // }
        }
    }
}
