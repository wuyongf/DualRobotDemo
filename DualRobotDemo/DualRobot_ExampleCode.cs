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

            core.MoveTo(Model.CR7, Position.Ready_Scene1B);

            // DRA Installation ... //

            core.MoveTo(Model.CR7, Position.Home);

            core.MoveTo(Model.CR7, Position.InitPos_Scene1B);

            // 3. ready position 2
            core.MoveTo(Model.CR7, Position.Ready_Scene2);
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

            // 3. move to ready position
            core.MoveTo(Model.CR15, Position.Ready_Scene1B);

            core.MoveTo(Model.CR15, Position.Ready_Scene2);
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
        public void PoseTransformation()
        {
            // (1) Connection
            DualRobotLib.Core core = new Core();
            core.Connect(Model.CR15, "127.0.0.1", 9021);
            core.Connect(Model.CR7, "127.0.0.1", 60008);

            DualRobotLib.Trans trans = new Trans();

            double[] relPos = { 0, 0, 0, 0, 0, 90 };
            double[] curPos = core.GetCurPos(Model.CR15);

            var t1 = trans.pos2T(ref relPos);
            var t2 = trans.pos2T(ref curPos);
            var t3 = t2 * t1;

            // float[] targetPosFloat = trans.T2pos(t3);
            double[] targetPos = trans.T2posd(t3);
        }

        public void ListAllTCPs()
        {
            // (1) Connection
            DualRobotLib.Core core = new Core();
            core.Connect(Model.CR15, "127.0.0.1", 9021);
            core.Connect(Model.CR7, "127.0.0.1", 60008);

            // ....
            // ....
            // Please run the following code after step (7) Scene Initialization

            var cal_pos = core.GetRobotBaseCalibrationPose(); // relation between cr7 and cr15. formula: T_cr15 = T_cal * T_cr7
            var lttt_tcp = core.GetStationAntennaTCP_Cr7(); // record in cr7 base coordinate
            var cr7_tcp = core.GetCurPos(Model.CR7);        // record in cr7 base coordinate
            var cr15_tcp = core.GetCurPos(Model.CR15);      // record in cr15 base coordinate

            Console.WriteLine("cal_pos = " + string.Join(",", cal_pos));
            Console.WriteLine("lttt_tcp = " + string.Join(",", lttt_tcp));
            Console.WriteLine("cr7_tcp = " + string.Join(",", cr7_tcp));
            Console.WriteLine("cr15_tcp = " + string.Join(",", cr15_tcp));


            var dis1 = core.GetTcpDistance(SceneName.Scene1A_Sim, Model.LiftTable, Model.CR15); // get distance between lttt_tcp and cr15_tcp
            var dis2 = core.GetTcpDistance(SceneName.Scene1A_Sim, Model.LiftTable, Model.CR7);  // get distance between lttt_tcp and cr7_tcp
            var dis3 = core.GetTcpDistance(SceneName.Scene1A_Sim, Model.CR15, Model.CR7);       // get distance between cr15_tcp and cr7_tcp

            Console.WriteLine("distance between lttt_tcp and cr15_tcp: " + dis1);
            Console.WriteLine("distance between lttt_tcp and cr7_tcp: " + dis2);
            Console.WriteLine("distance between cr15_tcp and cr7_tcp: " + dis3);
        }
        // CityU-Demo
        public void DualRobot_Scene1B_CityU()
        {
            // (1) Connection
            DualRobotLib.Core core = new Core();
            core.Connect(Model.CR15, "127.0.0.1", 9021);
            core.Connect(Model.CR7, "127.0.0.1", 60008);
            // core.Connect(Model.CR15, "192.168.0.125", 60008);
            // core.Connect(Model.CR7, "192.168.0.124", 60008);

            // (2) Get Calibrated Co-Frame Data
            // % Pos_Cr7_CalliBase 
            // % cal-0: 779.422, -37.794, -339.305, 0.351, 0.523, -92.267
            // % cal-1: 779.282, -38.284, -286.598, 0.588, 0.510, -92.060
            // % cal-2: 778.281, -38.520, -336.517, 0.810, 0.687, -92.389
            //
            // % Pos_Cr15_CalliBase 
            // % cal-0: 1249.821, 22.977, -774.474, 0.149, 0.109, 89.261 //error: 3-4mm
            // % cal-1: 1252.218, 23.278, -722.122, 0.082, 0.179, 89.359 //error: 2.5mm
            // % cal-2: 1252.171, 24.657, -770.478, 0.269, 0.423, 89.225 //error: 0.2mm
            double[] Pos_Cr7_CalliBase = { 778.281, -38.520, -336.517, 0.810, 0.687, -92.389 };
            double[] Pos_Cr15_CalliBase = { 1252.171, 24.657, -770.478, 0.269, 0.423, 89.225 };
            core.RobotBaseCalibrationInit(Pos_Cr7_CalliBase, Pos_Cr15_CalliBase);

            // (3) Get Tool Antenna TCP Data 

            float[] origin_wpr_cr7 = { 176.753f, 8.741f, 174.332f };
            float[] default_wpr_cr7 = { -3.884f, 53.543f, -9.229f };

            float[] origin_wpr_cr15 = { -0.596f, 29.023f, -2.64f };
            float[] default_wpr_cr15 = { 105.071f, -0.865f, 89.288f };

            var cal_wpr_cr7 = core.GetToolFixtureWPR(SceneName.Scene1B, Model.CR7, origin_wpr_cr7, default_wpr_cr7);
            var cal_wpr_cr15 = core.GetToolFixtureWPR(SceneName.Scene1B, Model.CR15, origin_wpr_cr15, default_wpr_cr15);

            float[] cal_pin_tcp_cr7 = { -61.97f, 1.016f, 193.006f, cal_wpr_cr7[0], cal_wpr_cr7[1], cal_wpr_cr7[2] };
            float[] cal_pin_tcp_cr15 = { -1.946f, -35.828f, 174.092f, cal_wpr_cr15[0], cal_wpr_cr15[1], cal_wpr_cr15[2] };
            float cal_pin_length_cr7 = 45.23f + 5.02f;
            float cal_pin_length_cr15 = 45.33f + 5.02f;
            var fixture_tcp_cr7 = core.GetToolFixtureTCP(Model.CR7, cal_pin_tcp_cr7, cal_pin_length_cr7);
            var fixture_tcp_cr15 = core.GetToolFixtureTCP(Model.CR15, cal_pin_tcp_cr15, cal_pin_length_cr15);

            // offset-1: cr7: 6.27f;  cr15:6.24f;
            // offset-2: cr7: 16.05f; cr15: 16.03f;
            float[] antenna_offset_cr7 = { 0.0f, 0.0f, 6.27f, 0.0f, 0.0f, 0.0f };
            float[] antenna_offset_cr15 = { 0.0f, 0.0f, 6.24f, 0.0f, 0.0f, 0.0f };
            var tcp_cr7 = core.GetToolAntennaTCP(Model.CR7, fixture_tcp_cr7, antenna_offset_cr7);
            var tcp_cr15 = core.GetToolAntennaTCP(Model.CR15, fixture_tcp_cr15, antenna_offset_cr15);
            Console.WriteLine("tcp_cr7: " + tcp_cr7);
            Console.WriteLine("tcp_cr15: " + tcp_cr15);

            // (4) Robot Initialization
            core.SetTCP(Model.CR15, tcp_cr15);
            core.SetTCP(Model.CR7, tcp_cr7);
            core.SetSpeed(Model.CR15, 100);
            core.SetSpeed(Model.CR7, 100);
            // c. examples.
            // double[] param = { 1000, 130, 10, 180, 90, 180, 45 };
            // double[] param = { 200, 102, 34, 102, 34, 180, 60 };
            double[] param = { 200, 130, 10, 50, 25, 180, 45 };
            core.SceneParamInit(SceneName.Scene1B, param);
            // d.
            core.SceneRobotInit(SceneName.Scene1B);
            // e.
            core.SetUserFrame(Model.CR15);
            core.SetUserFrame(Model.CR7);

            // step5: Scene1B
            core.Scene1B(MovementType.QuickCheck);
        }
        public void DualRobot_Scene1C_CityU()
        {
            // (1) Connection
            DualRobotLib.Core core = new Core();
            core.Connect(Model.CR15, "127.0.0.1", 9021);
            core.Connect(Model.CR7, "127.0.0.1", 60008);
            // core.Connect(Model.CR15, "192.168.0.125", 60008);
            // core.Connect(Model.CR7, "192.168.0.124", 60008);

            // (2) Get Calibrated Co-Frame Data
            // % Pos_Cr7_CalliBase 
            // % cal-0: 779.422, -37.794, -339.305, 0.351, 0.523, -92.267
            // % cal-1: 779.282, -38.284, -286.598, 0.588, 0.510, -92.060
            // % cal-2: 778.281, -38.520, -336.517, 0.810, 0.687, -92.389
            //
            // % Pos_Cr15_CalliBase 
            // % cal-0: 1249.821, 22.977, -774.474, 0.149, 0.109, 89.261 //error: 3-4mm
            // % cal-1: 1252.218, 23.278, -722.122, 0.082, 0.179, 89.359 //error: 2.5mm
            // % cal-2: 1252.171, 24.657, -770.478, 0.269, 0.423, 89.225 //error: 0.2mm
            double[] Pos_Cr7_CalliBase = { 778.281, -38.520, -336.517, 0.810, 0.687, -92.389 };
            double[] Pos_Cr15_CalliBase = { 1252.171, 24.657, -770.478, 0.269, 0.423, 89.225 };
            core.RobotBaseCalibrationInit(Pos_Cr7_CalliBase, Pos_Cr15_CalliBase);

            // (3) Get Tool Antenna TCP Data 

            float[] origin_wpr_cr7 = { 176.753f, 8.741f, 174.332f };
            float[] default_wpr_cr7 = { -3.884f, 53.543f, -9.229f };

            float[] origin_wpr_cr15 = { -0.596f, 29.023f, -2.64f };
            float[] default_wpr_cr15 = { 105.071f, -0.865f, 89.288f };

            var cal_wpr_cr7 = core.GetToolFixtureWPR(SceneName.Scene1B, Model.CR7, origin_wpr_cr7, default_wpr_cr7);
            var cal_wpr_cr15 = core.GetToolFixtureWPR(SceneName.Scene1B, Model.CR15, origin_wpr_cr15, default_wpr_cr15);

            float[] cal_pin_tcp_cr7 = { -61.97f, 1.016f, 193.006f, cal_wpr_cr7[0], cal_wpr_cr7[1], cal_wpr_cr7[2] };
            float[] cal_pin_tcp_cr15 = { -1.946f, -35.828f, 174.092f, cal_wpr_cr15[0], cal_wpr_cr15[1], cal_wpr_cr15[2] };
            float cal_pin_length_cr7 = 45.23f + 5.02f;
            float cal_pin_length_cr15 = 45.33f + 5.02f;
            var fixture_tcp_cr7 = core.GetToolFixtureTCP(Model.CR7, cal_pin_tcp_cr7, cal_pin_length_cr7);
            var fixture_tcp_cr15 = core.GetToolFixtureTCP(Model.CR15, cal_pin_tcp_cr15, cal_pin_length_cr15);

            // offset-1: cr7: 6.27f;  cr15:6.24f;
            // offset-2: cr7: 16.05f; cr15: 16.03f;
            // offset-3: cr7: 71.53f; cr15: 16.02f;
            // offset-4: cr7: 16.05f; cr15: 16.02f;
            // offset-5: cr7: 71.53f; cr15: 105.13f;
            // offset-6: cr7: 16.05f; cr15: 105.13f;
            // offset-7: cr7: 45.14f; cr15: 105.13f;
            float[] antenna_offset_cr7 = { 0.0f, 0.0f, 45.14f, 0.0f, 0.0f, 0.0f };
            float[] antenna_offset_cr15 = { 0.0f, 0.0f, 105.13f, 0.0f, 0.0f, 0.0f };
            var tcp_cr7 = core.GetToolAntennaTCP(Model.CR7, fixture_tcp_cr7, antenna_offset_cr7);
            var tcp_cr15 = core.GetToolAntennaTCP(Model.CR15, fixture_tcp_cr15, antenna_offset_cr15);
            Console.WriteLine("tcp_cr7: " + tcp_cr7);
            Console.WriteLine("tcp_cr15: " + tcp_cr15);

            // (4) Robot Initialization
            // examples: tcp data
            //float[] tcp_cr7 = { -61.7107f, 0, 193.7107f, 0, -45, 0 };
            //float[] tcp_cr15 = { 0, 0, 140, 0, 45, -90 };
            core.SetTCP(Model.CR15, tcp_cr15);
            core.SetTCP(Model.CR7, tcp_cr7);
            core.SetSpeed(Model.CR15, 100);
            core.SetSpeed(Model.CR7, 100);

            // c. examples.
            //double[] param = { 200, 350, 350, 4, 4, 0 };
            double[] param = { 100, 350, 350, 4, 4, 0 };
            core.SceneParamInit(SceneName.Scene1C, param);
            // d.
            core.SceneRobotInit(SceneName.Scene1C);
            // e.
            core.SetUserFrame(Model.CR15);
            core.SetUserFrame(Model.CR7);

            // step5: Scene1C
            // Thread th_scene1c = new Thread(() => core.Scene1C(MovementType.QuickCheck));
            // th_scene1c.Start();

            // Thread th1 = new Thread(() => core.thread_MoveFlag(Model.CR15));
            // th1.Start();

            core.Scene1C(MovementType.QuickCheck, MovementStage.One);

            core.Scene1C(MovementType.QuickCheck, MovementStage.Two);
        }
        public void DualRobot_Scene2_CityU_Sim()
        {
            // (1) Connection
            DualRobotLib.Core core = new Core();
            core.Connect(Model.CR15, "127.0.0.1", 9021);
            core.Connect(Model.CR7, "127.0.0.1", 60008);
            // core.Connect(Model.CR15, "192.168.0.125", 60008);
            // core.Connect(Model.CR7, "192.168.0.124", 60008);

            // (2) Get Calibrated Co-Frame Data
            // % cal-2: current error: 0.2mm
            // double[] Pos_Cr7_CalliBase = { 778.281, -38.520, -336.517, 0.810, 0.687, -92.389 };
            // double[] Pos_Cr15_CalliBase = { 1252.171, 24.657, -770.478, 0.269, 0.423, 89.225 };
            double[] Pos_Cr7_CalliBase = { 860.0, 0, 247.0, 0, 0, -90.0 };
            double[] Pos_Cr15_CalliBase = { 1190.0, 0, -193.0, 0, 0, 90.0 };
            core.RobotBaseCalibrationInit(Pos_Cr7_CalliBase, Pos_Cr15_CalliBase);

            // (3) Get Tool Antenna TCP Data 
            float[] origin_wpr_cr7 = { -9.082f, 89.472f, -9.403f };
            float[] default_wpr_cr7 = { 89.529f, -1.760f, 88.218f };

            float[] origin_wpr_cr15 = { -17.024f, 87.760f, -17.928f };
            float[] default_wpr_cr15 = { -91.714f, 0.772f, -90.444f };

            var cal_wpr_cr7 = core.GetToolFixtureWPR(SceneName.Scene2_Sim, Model.CR7, origin_wpr_cr7, default_wpr_cr7);
            var cal_wpr_cr15 = core.GetToolFixtureWPR(SceneName.Scene2_Sim, Model.CR15, origin_wpr_cr15, default_wpr_cr15);

            float[] cal_pin_tcp_cr7 = { -120.609f, 180.878f, 134.808f, cal_wpr_cr7[0], cal_wpr_cr7[1], cal_wpr_cr7[2] };
            float[] cal_pin_tcp_cr15 = { -1.831f, 0.474f, 299.920f, cal_wpr_cr15[0], cal_wpr_cr15[1], cal_wpr_cr15[2] };
            float cal_pin_length_cr7 = 45.23f + 5.02f;
            float cal_pin_length_cr15 = 45.33f + 5.02f;
            var fixture_tcp_cr7 = core.GetToolFixtureTCP(Model.CR7, cal_pin_tcp_cr7, cal_pin_length_cr7);
            var fixture_tcp_cr15 = core.GetToolFixtureTCP(Model.CR15, cal_pin_tcp_cr15, cal_pin_length_cr15);

            // offset-1: cr7: 6.27f;  cr15:6.24f;
            // offset-2: cr7: 16.05f; cr15: 16.03f;
            float[] antenna_offset_cr7 = { 0.0f, 0.0f, 16.05f, 0.0f, 0.0f, 0.0f };
            float[] antenna_offset_cr15 = { 0.0f, 0.0f, 105.13f, 0.0f, 0.0f, 0.0f };
            var tcp_cr7 = core.GetToolAntennaTCP(Model.CR7, fixture_tcp_cr7, antenna_offset_cr7);
            var tcp_cr15 = core.GetToolAntennaTCP(Model.CR15, fixture_tcp_cr15, antenna_offset_cr15);

            // (4) Robot Initialization
            core.SetTCP(Model.CR15, tcp_cr15);
            core.SetTCP(Model.CR7, tcp_cr7);
            core.SetSpeed(Model.CR15, 100);
            core.SetSpeed(Model.CR7, 100);

            // (5) Set Station Antenna TCP (Cr7) (Cal. tool + UF: 0)
            float[] station_cal_pin_tcp_cr7 = { 832.066f, -102.571f, 277.335f, 0.953f, 0.102f, -92.282f }; // switch to tool:5 user frame:0
            float station_cal_pin_length = 41.05f;
            var station_center_zero_tcp = core.GetStationCenterZeroTCP(station_cal_pin_tcp_cr7, station_cal_pin_length);

            float[] antenna_offset_station = { 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f };
            float station_offset = 0; // 0-290

            var station_antenna_tcp_cr7 = core.GetStationAntennaTCP(station_center_zero_tcp, antenna_offset_station, station_offset);
            Console.WriteLine("station_antenna_tcp_cr7: " + string.Join(",", station_antenna_tcp_cr7));

            core.SetStationAntennaTCP_Cr7(station_antenna_tcp_cr7);


            // (6) Align LiftTable Height with station_center_zero_tcp
            double lift_table_align_error = 1.025;
            double stage34_fixture_height = 127;

            // (7) Scene Initialization
            // a. examples.
            double[] param = { 160, 90, 10, 180, 30, 100, 45, 0, 140, lift_table_align_error, stage34_fixture_height };
            
            core.SceneParamInit(SceneName.Scene2_Sim, param);
            // b.
            core.SceneRobotInit(SceneName.Scene2_Sim);
            // c.
            core.SetUserFrame(Model.CR15);
            core.SetUserFrame(Model.CR7);

            // (8) Execute Scene2_Sim

            // stage - 1
            core.Scene2_Sim(MovementType.QuickCheck, MovementStage.One);

            // stage - 2
            core.SceneRobotInit(SceneName.Scene2_Sim);
            core.Scene2_Sim(MovementType.QuickCheck, MovementStage.Two);

            // stage - 3
            param[1] = 20;
            param[8] = 13;
            core.SceneParamInit(SceneName.Scene2_Sim, param);
            core.SceneRobotInit(SceneName.Scene2_Sim, Model.Null, MovementStage.Three);
            core.Scene2_Sim(MovementType.QuickCheck, MovementStage.Three);

            // stage - 4
            param[1] = 20;
            param[8] = 13;
            core.SceneParamInit(SceneName.Scene2_Sim, param);
            core.SceneRobotInit(SceneName.Scene2_Sim, Model.Null, MovementStage.Four);
            core.Scene2_Sim(MovementType.QuickCheck, MovementStage.Four);
        }
        public void DualRobot_Scene2_CityU()
        {
            // (1) Connection
            DualRobotLib.Core core = new Core();
            //core.Connect(Model.CR15, "127.0.0.1", 9021);
            //core.Connect(Model.CR7, "127.0.0.1", 60008);
            core.Connect(Model.CR15, "192.168.0.125", 60008);
            core.Connect(Model.CR7, "192.168.0.124", 60008);

            core.Connect(Model.LiftTable, "192.168.0.119", 50000, "COM4");
            core.Connect(Model.Motor, "COM3");

            core.MotorInit();
            core.LiftTableInit();

            // (2) Get Calibrated Co-Frame Data
            // % cal-2: current error: 0.2mm
            double[] Pos_Cr7_CalliBase = { 778.281, -38.520, -336.517, 0.810, 0.687, -92.389 };
            double[] Pos_Cr15_CalliBase = { 1252.171, 24.657, -770.478, 0.269, 0.423, 89.225 };
            core.RobotBaseCalibrationInit(Pos_Cr7_CalliBase, Pos_Cr15_CalliBase);

            // (3) Get Tool Antenna TCP Data 
            float[] origin_wpr_cr7 = { -9.082f, 89.472f, -9.403f };
            float[] default_wpr_cr7 = { 89.529f, -1.760f, 88.218f };

            float[] origin_wpr_cr15 = { -17.024f, 87.760f, -17.928f };
            float[] default_wpr_cr15 = { -91.714f, 0.772f, -90.444f };

            var cal_wpr_cr7 = core.GetToolFixtureWPR(Model.CR7, origin_wpr_cr7, default_wpr_cr7);
            var cal_wpr_cr15 = core.GetToolFixtureWPR(Model.CR15, origin_wpr_cr15, default_wpr_cr15);

            float[] cal_pin_tcp_cr7 = { -120.609f, 180.878f, 134.808f, cal_wpr_cr7[0], cal_wpr_cr7[1], cal_wpr_cr7[2] };
            float[] cal_pin_tcp_cr15 = { -1.831f, 0.474f, 299.920f, cal_wpr_cr15[0], cal_wpr_cr15[1], cal_wpr_cr15[2] };
            float cal_pin_length_cr7 = 45.23f + 5.02f;
            float cal_pin_length_cr15 = 45.33f + 5.02f;
            var fixture_tcp_cr7 = core.GetToolFixtureTCP(Model.CR7, cal_pin_tcp_cr7, cal_pin_length_cr7);
            var fixture_tcp_cr15 = core.GetToolFixtureTCP(Model.CR15, cal_pin_tcp_cr15, cal_pin_length_cr15);

            float[] antenna_offset_cr7 = { 0.0f, 0.0f, 50.18f, 0.0f, 0.0f, 0.0f };
            float[] antenna_offset_cr15 = { 0.0f, 0.0f, 71.53f, 0.0f, 0.0f, 0.0f };
            var tcp_cr7 = core.GetToolAntennaTCP(Model.CR7, fixture_tcp_cr7, antenna_offset_cr7);
            var tcp_cr15 = core.GetToolAntennaTCP(Model.CR15, fixture_tcp_cr15, antenna_offset_cr15);

            // (4) Robot Initialization
            core.SetTCP(Model.CR15, tcp_cr15);
            core.SetTCP(Model.CR7, tcp_cr7);
            core.SetSpeed(Model.CR15, 100);
            core.SetSpeed(Model.CR7, 100);

            // (5) Set Station Antenna TCP (Cr7) (Cal. tool + UF: 0)
            float[] station_cal_pin_tcp_cr7 = { 832.244f, -102.672f, 272.135f, 0.841f, 0.826f, -92.135f }; // switch to tool:5 user frame:0
            float station_cal_pin_length = 40.05f + 0.8f;
            var station_center_zero_tcp = core.GetStationCenterZeroTCP(station_cal_pin_tcp_cr7, station_cal_pin_length);

            float[] antenna_offset_station = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
            float station_offset = 0;

            var station_antenna_tcp_cr7 = core.GetStationAntennaTCP(station_center_zero_tcp, antenna_offset_station, station_offset);
            Console.WriteLine("station_antenna_tcp_cr7: " + string.Join(",", station_antenna_tcp_cr7));

            core.SetStationAntennaTCP_Cr7(station_antenna_tcp_cr7);

            // (6) Align LiftTable Height with station_center_zero_tcp
            double lift_table_align_error = 1.13;
            double stage34_fixture_height = 0;

            // (7) Scene Initialization
            // a. examples.
            double[] param = { 145, 90, 45, 180, 60, 100, 45, 0, 140, lift_table_align_error, stage34_fixture_height };
            core.SceneParamInit(SceneName.Scene2, param);
            // b.
            core.SceneRobotInit(SceneName.Scene2);
            // c.
            core.SetUserFrame(Model.CR15);
            core.SetUserFrame(Model.CR7);

            // (8) Execute Scene2

            // stage - 1
            core.Scene2(MovementType.QuickCheck, MovementStage.One);

            // stage - 2
            core.SceneRobotInit(SceneName.Scene2);
            core.Scene2(MovementType.QuickCheck, MovementStage.Two);

            // stage - 3
            param[1] = 20;
            param[8] = 13;
            core.SceneParamInit(SceneName.Scene2, param);
            core.SceneRobotInit(SceneName.Scene2, Model.Null, MovementStage.Three);
            core.Scene2(MovementType.QuickCheck, MovementStage.Three);

            // stage - 4
            param[1] = 20;
            param[8] = 13;
            core.SceneParamInit(SceneName.Scene2, param);
            core.SceneRobotInit(SceneName.Scene2, Model.Null, MovementStage.Four);
            core.Scene2(MovementType.QuickCheck, MovementStage.Four);

        }
        public void DualRobot_Scene1A_CityU_Sim()
        {
            // (1) Connection
            DualRobotLib.Core core = new Core();
            core.Connect(Model.CR15, "127.0.0.1", 9021);
            core.Connect(Model.CR7, "127.0.0.1", 60008);
            // core.Connect(Model.CR15, "192.168.0.125", 60008);
            // core.Connect(Model.CR7, "192.168.0.124", 60008);

            // (2) Get Calibrated Co-Frame Data
            // % cal-2: current error: 0.2mm
            double[] Pos_Cr7_CalliBase = { 778.281, -38.520, -336.517, 0.810, 0.687, -92.389 };
            double[] Pos_Cr15_CalliBase = { 1252.171, 24.657, -770.478, 0.269, 0.423, 89.225 };
            core.RobotBaseCalibrationInit(Pos_Cr7_CalliBase, Pos_Cr15_CalliBase);

            // (3) Get Tool Antenna TCP Data 
            float[] origin_wpr_cr15 = { -17.024f, 87.760f, -17.928f };
            float[] default_wpr_cr15 = { -91.714f, 0.772f, -90.444f };

            var cal_wpr_cr15 = core.GetToolFixtureWPR(SceneName.Scene2_Sim, Model.CR15, origin_wpr_cr15, default_wpr_cr15);

            float[] cal_pin_tcp_cr15 = { -1.831f, 0.474f, 299.920f, cal_wpr_cr15[0], cal_wpr_cr15[1], cal_wpr_cr15[2] };
            float cal_pin_length_cr15 = 45.33f + 5.02f;
            var fixture_tcp_cr15 = core.GetToolFixtureTCP(Model.CR15, cal_pin_tcp_cr15, cal_pin_length_cr15);

            // offset
            float[] antenna_offset_cr15 = { 0.0f, 0.0f, 50.35f, 0.0f, 0.0f, 0.0f };
            var tcp_cr15 = core.GetToolAntennaTCP(Model.CR15, fixture_tcp_cr15, antenna_offset_cr15);

            // (4) Robot Initialization
            core.SetTCP(Model.CR15, tcp_cr15);
            core.SetSpeed(Model.CR15, 100);
            core.SetSpeed(Model.CR7, 100);

            // (5) Set Station Antenna TCP (Calibrated by Cr7 with Cal. tool) (Cal. tool + UF: 0)
            float[] station_cal_pin_tcp_cr7 = { 832.066f, -102.571f, 277.335f, 0.953f, 0.102f, -92.282f }; // switch to tool:5 user frame:0
            float station_cal_pin_length = 41.05f;
            var station_center_zero_tcp = core.GetStationCenterZeroTCP(station_cal_pin_tcp_cr7, station_cal_pin_length);

            float[] antenna_offset_station = { 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f };
            float station_offset = 0; // 0-290

            var station_antenna_tcp_cr7 = core.GetStationAntennaTCP(station_center_zero_tcp, antenna_offset_station, station_offset);

            core.SetStationAntennaTCP_Cr7(station_antenna_tcp_cr7);


            // (6) Align LiftTable Height with station_center_zero_tcp
            double lift_table_align_error = 1.025;
            double stage34_fixture_height = 0;
            double antenna_height = antenna_offset_station[2];

            // (7) Scene Initialization
            // a. examples.
            double[] param = { 250, 180, 10, 180, 90, 140, lift_table_align_error, stage34_fixture_height, antenna_height };
            core.SceneParamInit(SceneName.Scene1A_Sim, param);
            // b.
            core.SceneRobotInit(SceneName.Scene1A_Sim);
            // c.
            core.SetUserFrame(Model.CR15);
            core.SetUserFrame(Model.CR7);

            // (8) Execute Scene1A_Sim

            // stage - 1
            core.Scene1A_Sim(MovementType.QuickCheck, MovementStage.One);

            // stage - 2
            core.SceneRobotInit(SceneName.Scene1A_Sim);
            core.Scene1A_Sim(MovementType.QuickCheck, MovementStage.Two);
        }
        public void DualRobot_Scene1A_CityU()
        {
            DualRobotLib.Core core = new Core();
            // core.Connect(Model.CR15, "127.0.0.1", 9021);
            // core.Connect(Model.CR7, "127.0.0.1", 60008);
            core.Connect(Model.CR15, "192.168.0.125", 60008);
            core.Connect(Model.CR7, "192.168.0.124", 60008);

            core.Connect(Model.LiftTable, "192.168.0.119", 50000, "COM4");
            core.Connect(Model.Motor, "COM3");

            core.MotorInit();
            core.LiftTableInit();

            // (2) Get Calibrated Co-Frame Data
            // % cal-2: current error: 0.2mm
            double[] Pos_Cr7_CalliBase = { 778.281, -38.520, -336.517, 0.810, 0.687, -92.389 };
            double[] Pos_Cr15_CalliBase = { 1252.171, 24.657, -770.478, 0.269, 0.423, 89.225 };
            core.RobotBaseCalibrationInit(Pos_Cr7_CalliBase, Pos_Cr15_CalliBase);

            // (3) Get Tool Antenna TCP Data 
            float[] origin_wpr_cr7 = { -9.082f, 89.472f, -9.403f };
            float[] default_wpr_cr7 = { 89.529f, -1.760f, 88.218f };

            float[] origin_wpr_cr15 = { -17.024f, 87.760f, -17.928f };
            float[] default_wpr_cr15 = { -91.714f, 0.772f, -90.444f };

            var cal_wpr_cr7 = core.GetToolFixtureWPR(Model.CR7, origin_wpr_cr7, default_wpr_cr7);
            var cal_wpr_cr15 = core.GetToolFixtureWPR(Model.CR15, origin_wpr_cr15, default_wpr_cr15);

            float[] cal_pin_tcp_cr7 = { -120.609f, 180.878f, 134.808f, cal_wpr_cr7[0], cal_wpr_cr7[1], cal_wpr_cr7[2] };
            float[] cal_pin_tcp_cr15 = { -1.831f, 0.474f, 299.920f, cal_wpr_cr15[0], cal_wpr_cr15[1], cal_wpr_cr15[2] };
            float cal_pin_length_cr7 = 50.25f;
            float cal_pin_length_cr15 = 50.35f;
            var fixture_tcp_cr7 = core.GetToolFixtureTCP(Model.CR7, cal_pin_tcp_cr7, cal_pin_length_cr7);
            var fixture_tcp_cr15 = core.GetToolFixtureTCP(Model.CR15, cal_pin_tcp_cr15, cal_pin_length_cr15);

            float[] antenna_offset_cr15 = { 0.0f, 0.0f, 4.99f, 0.0f, 0.0f, 0.0f };//Scene 1A Testing Cr15 Cal Pin = 5.00
            var tcp_cr15 = core.GetToolAntennaTCP(Model.CR15, fixture_tcp_cr15, antenna_offset_cr15);

            // (4) Robot Initialization
            core.SetTCP(Model.CR15, tcp_cr15);
            core.SetSpeed(Model.CR15, 100);
            core.SetSpeed(Model.CR7, 100);

            // (5) Set Station Antenna TCP (Cr7) (Cal. tool + UF: 0) *** need to update***
            float[] station_cal_pin_tcp_cr7 = { 832.975f, -101.911f, 280.829f, 0.484f, 0.529f, -92.352f }; // zero position
            // float lift_table_input = 135.793
            // float lift_table_output = 136.395
            float station_cal_pin_length = 50.2f;
            var station_center_zero_tcp = core.GetStationCenterZeroTCP(station_cal_pin_tcp_cr7, station_cal_pin_length);

            //float[] antenna_offset_station = { 0.0f, 0.0f, 50.35f, 0.0f, 0.0f, 0.0f };//Scene 1A Testing Station Cal Pin = 50.35
            float[] antenna_offset_station = { 0.0f, 0.0f, 4.95f, 0.0f, 0.0f, 0.0f };//Scene 1A Testing Station Cal Plate = 5.00
            float station_offset = 0;

            var station_antenna_tcp_cr7 = core.GetStationAntennaTCP(station_center_zero_tcp, antenna_offset_station, station_offset);
            Console.WriteLine("station_antenna_tcp_cr7: " + string.Join(",", station_antenna_tcp_cr7));

            core.SetStationAntennaTCP_Cr7(station_antenna_tcp_cr7);

            // (6) Align LiftTable Height with station_center_zero_tcp
            double lift_table_align_error = 0.247;
            double stage34_fixture_height = 0;
            double antenna_height = antenna_offset_station[2];

            // (7) Scene Initialization
            // a. examples.
            double[] param = { 150, 180, 45, 180, 90, 135.793, lift_table_align_error, stage34_fixture_height, antenna_height }; //Scene 1A Testing with station fixture offset
            core.SceneParamInit(SceneName.Scene1A, param);
            // b.
            core.SceneRobotInit(SceneName.Scene1A);
            // c.
            core.SetUserFrame(Model.CR15);
            core.SetUserFrame(Model.CR7);

            var tcp_distance = core.GetTcpDistance(SceneName.Scene1A, Model.LiftTable, Model.CR15);
            Console.WriteLine("tcp_distance: " + tcp_distance);

            // (8) Execute Scene1A

            // stage1: main
            core.Scene1A(MovementType.QuickCheck, MovementStage.One);

            // stage2: init
            core.SetStationAntennaTCP_Cr7(station_antenna_tcp_cr7);
            core.SceneRobotInit(SceneName.Scene1A);
            core.SetUserFrame(Model.CR15);
            core.SetUserFrame(Model.CR7);
            // stage2: main
            core.Scene1A(MovementType.QuickCheck, MovementStage.Two);
        }
        public void DualRobot_Scene3_CityU_Sim()
        {
            // (1) Connection
            DualRobotLib.Core core = new Core();
            core.Connect(Model.CR15, "127.0.0.1", 9021);
            core.Connect(Model.CR7, "127.0.0.1", 60008);
            // core.Connect(Model.CR15, "192.168.0.125", 60008);
            // core.Connect(Model.CR7, "192.168.0.124", 60008);

            // (2) Get Calibrated Co-Frame Data
            // % cal-2: current error: 0.2mm
            // double[] Pos_Cr7_CalliBase = { 778.281, -38.520, -336.517, 0.810, 0.687, -92.389 };
            // double[] Pos_Cr15_CalliBase = { 1252.171, 24.657, -770.478, 0.269, 0.423, 89.225 };
            double[] Pos_Cr7_CalliBase = { 860.0, 0, 247.0, 0, 0, -90.0 };
            double[] Pos_Cr15_CalliBase = { 1190.0, 0, -193.0, 0, 0, 90.0 };
            core.RobotBaseCalibrationInit(Pos_Cr7_CalliBase, Pos_Cr15_CalliBase);

            // (3) Get Tool Antenna TCP Data 
            float[] origin_wpr_cr7 = { -9.082f, 89.472f, -9.403f };
            float[] default_wpr_cr7 = { 89.529f, -1.760f, 88.218f };

            float[] origin_wpr_cr15 = { -17.024f, 87.760f, -17.928f };
            float[] default_wpr_cr15 = { -91.714f, 0.772f, -90.444f };

            var cal_wpr_cr7 = core.GetToolFixtureWPR(SceneName.Scene2_Sim, Model.CR7, origin_wpr_cr7, default_wpr_cr7);
            var cal_wpr_cr15 = core.GetToolFixtureWPR(SceneName.Scene2_Sim, Model.CR15, origin_wpr_cr15, default_wpr_cr15);

            float[] cal_pin_tcp_cr7 = { -120.609f, 180.878f, 134.808f, cal_wpr_cr7[0], cal_wpr_cr7[1], cal_wpr_cr7[2] };
            float[] cal_pin_tcp_cr15 = { -1.831f, 0.474f, 299.920f, cal_wpr_cr15[0], cal_wpr_cr15[1], cal_wpr_cr15[2] };
            float cal_pin_length_cr7 = 45.23f + 5.02f;
            float cal_pin_length_cr15 = 45.33f + 5.02f;
            var fixture_tcp_cr7 = core.GetToolFixtureTCP(Model.CR7, cal_pin_tcp_cr7, cal_pin_length_cr7);
            var fixture_tcp_cr15 = core.GetToolFixtureTCP(Model.CR15, cal_pin_tcp_cr15, cal_pin_length_cr15);

            // offset-1: cr7: 6.27f;  cr15:6.24f;
            // offset-2: cr7: 16.05f; cr15: 16.03f;
            float[] antenna_offset_cr7 = { 0.0f, 0.0f, 16.05f, 0.0f, 0.0f, 0.0f };
            float[] antenna_offset_cr15 = { 0.0f, 0.0f, 105.13f, 0.0f, 0.0f, 0.0f };
            // var tcp_cr7 = core.GetToolAntennaTCP(Model.CR7, fixture_tcp_cr7, antenna_offset_cr7);
            // var tcp_cr15 = core.GetToolAntennaTCP(Model.CR15, fixture_tcp_cr15, antenna_offset_cr15);

            // (4) Robot Initialization
            float[] tcp_cr7 = { 0.0f, -75.0f, 245.0f, 0, -45, -90 };
            float[] tcp_cr15 = { 0, 0, 505, 0, 0, 90 };
            core.SetTCP(Model.CR15, tcp_cr15);
            core.SetTCP(Model.CR7, tcp_cr7);
            core.SetSpeed(Model.CR15, 100);
            core.SetSpeed(Model.CR7, 100);

            // (5) Set Station Antenna TCP (Cr7) (Cal. tool + UF: 0)
            // float[] station_cal_pin_tcp_cr7 = { 832.066f, -102.571f, 277.335f, 0.953f, 0.102f, -92.282f }; // switch to tool:5 user frame:0
            float[] station_cal_pin_tcp_cr7 = { 832.066f, -102.571f, 187.335f, 0.953f, 0.102f, -92.282f }; // switch to tool:5 user frame:0
            float station_cal_pin_length = 41.05f;
            var station_center_zero_tcp = core.GetStationCenterZeroTCP(station_cal_pin_tcp_cr7, station_cal_pin_length);

            float[] antenna_offset_station = { 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f };
            float station_offset = 0; // 0-290

            var station_antenna_tcp_cr7 = core.GetStationAntennaTCP(station_center_zero_tcp, antenna_offset_station, station_offset);
            Console.WriteLine("station_antenna_tcp_cr7: " + string.Join(",", station_antenna_tcp_cr7));

            core.SetStationAntennaTCP_Cr7(station_antenna_tcp_cr7);


            // (6) Align LiftTable Height with station_center_zero_tcp
            double lift_table_align_error = 1.025;
            double stage34_fixture_height = 127;

            // (7) Scene Initialization
            // a. examples.
            double[] param = { 20, 100, 100, 4, 4, 20, 15, 0, 140, lift_table_align_error, stage34_fixture_height };

            core.SceneParamInit(SceneName.Scene3_Sim, param);
            // b.
            core.SceneRobotInit(SceneName.Scene3_Sim);
            // c.
            core.SetUserFrame(Model.CR15);
            core.SetUserFrame(Model.CR7);

            // (8) Execute Scene3_Sim

            // stage - 1
            core.Scene3_Sim(MovementType.QuickCheck, MovementStage.One);

            // stage - 2
            core.SceneRobotInit(SceneName.Scene3_Sim);
            // c.
            core.SetUserFrame(Model.CR15);
            core.SetUserFrame(Model.CR7);
            core.Scene3_Sim(MovementType.QuickCheck, MovementStage.Two);
        }

        public void DRA_S4_Sim()
        {
            // (1) Connection
            DualRobotLib.Core core = new Core();
            core.Connect(Model.CR15, "127.0.0.1", 9021);
            core.Connect(Model.CR7, "127.0.0.1", 60008);
            // core.Connect(Model.CR15, "192.168.0.125", 60008);
            // core.Connect(Model.CR7, "192.168.0.124", 60008);

            // (2) Get Calibrated Co-Frame Data
            double[] Pos_Cr7_CalliBase = { 860.0, 0, 247.0, 0, 0, -90.0 };
            double[] Pos_Cr15_CalliBase = { 1190.0, 0, -193.0, 0, 0, 90.0 };
            core.RobotBaseCalibrationInit(Pos_Cr7_CalliBase, Pos_Cr15_CalliBase);

            // (3) Get Tool Antenna TCP Data 
            float[] origin_wpr_cr7 = { -9.082f, 89.472f, -9.403f };
            float[] default_wpr_cr7 = { 89.529f, -1.760f, 88.218f };

            float[] origin_wpr_cr15 = { -17.024f, 87.760f, -17.928f };
            float[] default_wpr_cr15 = { -91.714f, 0.772f, -90.444f };

            var cal_wpr_cr7 = core.GetToolFixtureWPR(SceneName.Scene2_Sim, Model.CR7, origin_wpr_cr7, default_wpr_cr7);
            var cal_wpr_cr15 = core.GetToolFixtureWPR(SceneName.Scene2_Sim, Model.CR15, origin_wpr_cr15, default_wpr_cr15);

            float[] cal_pin_tcp_cr7 = { -120.609f, 180.878f, 134.808f, cal_wpr_cr7[0], cal_wpr_cr7[1], cal_wpr_cr7[2] };
            float[] cal_pin_tcp_cr15 = { -1.831f, 0.474f, 299.920f, cal_wpr_cr15[0], cal_wpr_cr15[1], cal_wpr_cr15[2] };
            float cal_pin_length_cr7 = 45.23f + 5.02f;
            float cal_pin_length_cr15 = 45.33f + 5.02f;
            var fixture_tcp_cr7 = core.GetToolFixtureTCP(Model.CR7, cal_pin_tcp_cr7, cal_pin_length_cr7);
            var fixture_tcp_cr15 = core.GetToolFixtureTCP(Model.CR15, cal_pin_tcp_cr15, cal_pin_length_cr15);

            float[] antenna_offset_cr7 = { 0.0f, 0.0f, 16.05f, 0.0f, 0.0f, 0.0f };
            float[] antenna_offset_cr15 = { 0.0f, 0.0f, 105.13f, 0.0f, 0.0f, 0.0f };
            // var tcp_cr7 = core.GetToolAntennaTCP(Model.CR7, fixture_tcp_cr7, antenna_offset_cr7);
            // var tcp_cr15 = core.GetToolAntennaTCP(Model.CR15, fixture_tcp_cr15, antenna_offset_cr15);

            // (4) Robot Initialization
            float[] tcp_cr7 = { 0.0f, -75.0f, 245.0f, 0, -45, -90 };
            float[] tcp_cr15 = { 0, 0, 260, 0, 0, 0 };
            core.SetTCP(Model.CR15, tcp_cr15);
            core.SetTCP(Model.CR7, tcp_cr7);
            core.SetSpeed(Model.CR15, 100);
            core.SetSpeed(Model.CR7, 100);

            // (5) Set Station Antenna TCP (Cr7)
            //float[] station_cal_pin_tcp_cr7 = { 860.365f, -49.229f, 135.271f, 0.0f, 0.0f, -88.872f }; // simulator - @450mm
            float[] station_cal_pin_tcp_cr7 = { 860.365f, -49.229f, 340.271f, 0.0f, 0.0f, -88.872f };   // simulator - @655mm
            float station_cal_pin_length = 0.0f;                                                     // simulation default value
            var station_center_zero_tcp = core.GetStationCenterZeroTCP(station_cal_pin_tcp_cr7, station_cal_pin_length);
            float[] antenna_offset_station = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };                    // simulation default value
            float station_offset = 0;                                                                 // simulation default value
            var station_antenna_tcp_cr7 = core.GetStationAntennaTCP(station_center_zero_tcp, antenna_offset_station, station_offset);
            //Console.WriteLine("station_antenna_tcp_cr7: " + string.Join(",", station_antenna_tcp_cr7));
            core.SetStationAntennaTCP_Cr7(station_antenna_tcp_cr7);


            // (7) Scene Initialization
            double[] param = { 0, 200, 3, 200, 3, 300, 2 };
            core.SceneParamInit(SceneName.Scene4_Sim, param);

            // (*) stage1
            core.SceneRobotInit(SceneName.Scene4_Sim);
            core.SetUserFrame(Model.CR15);
            core.SetUserFrame(Model.CR7);
            core.Scene4_Sim(MovementType.QuickCheck, MovementStage.One);

            // (*) stage2
            // core.SceneRobotInit(SceneName.Scene4_Sim);
            // core.SetUserFrame(Model.CR15);
            // core.SetUserFrame(Model.CR7);
            // core.Scene4_Sim(MovementType.QuickCheck, MovementStage.Two);
        }
        public void DRA_S4()
        {
            // (1) Connection
            DualRobotLib.Core core = new Core();
            core.Connect(Model.CR15, "127.0.0.1", 9021);
            core.Connect(Model.CR7, "127.0.0.1", 60008);

            // core.Connect(Model.CR15, "192.168.0.125", 60008);
            // core.Connect(Model.CR7, "192.168.0.124", 60008);
            // core.Connect(Model.LiftTable, "192.168.0.119", 50000, "COM4");
            // core.Connect(Model.Motor, "COM3");
            // core.MotorInit();
            // core.LiftTableInit();

            // (2) Get Calibrated Co-Frame Data
            double[] Pos_Cr7_CalliBase = { 778.281, -38.520, -336.517, 0.810, 0.687, -92.389 };
            double[] Pos_Cr15_CalliBase = { 1252.171, 24.657, -770.478, 0.269, 0.423, 89.225 };
            core.RobotBaseCalibrationInit(Pos_Cr7_CalliBase, Pos_Cr15_CalliBase);

            // (3) Get Tool Antenna TCP Data 
            float[] origin_wpr_cr7 = { -9.082f, 89.472f, -9.403f };
            float[] default_wpr_cr7 = { 89.529f, -1.760f, 88.218f };

            float[] origin_wpr_cr15 = { -17.024f, 87.760f, -17.928f };
            float[] default_wpr_cr15 = { -91.714f, 0.772f, -90.444f };

            var cal_wpr_cr7 = core.GetToolFixtureWPR(SceneName.Scene2_Sim, Model.CR7, origin_wpr_cr7, default_wpr_cr7);
            var cal_wpr_cr15 = core.GetToolFixtureWPR(SceneName.Scene2_Sim, Model.CR15, origin_wpr_cr15, default_wpr_cr15);

            float[] cal_pin_tcp_cr7 = { -120.609f, 180.878f, 134.808f, cal_wpr_cr7[0], cal_wpr_cr7[1], cal_wpr_cr7[2] };
            float[] cal_pin_tcp_cr15 = { -1.831f, 0.474f, 299.920f, cal_wpr_cr15[0], cal_wpr_cr15[1], cal_wpr_cr15[2] };
            float cal_pin_length_cr7 = 45.23f + 5.02f;
            float cal_pin_length_cr15 = 45.33f + 5.02f;
            var fixture_tcp_cr7 = core.GetToolFixtureTCP(Model.CR7, cal_pin_tcp_cr7, cal_pin_length_cr7);
            var fixture_tcp_cr15 = core.GetToolFixtureTCP(Model.CR15, cal_pin_tcp_cr15, cal_pin_length_cr15);

            float[] antenna_offset_cr7 = { 0.0f, 0.0f, 71.53f, 0.0f, 0.0f, 0.0f };
            float[] antenna_offset_cr15 = { 0.0f, 0.0f, 71.53f, 0.0f, 0.0f, 0.0f };
            var tcp_cr7 = core.GetToolAntennaTCP(Model.CR7, fixture_tcp_cr7, antenna_offset_cr7);
            var tcp_cr15 = core.GetToolAntennaTCP(Model.CR15, fixture_tcp_cr15, antenna_offset_cr15);

            // (4) Robot Initialization
            core.SetTCP(Model.CR15, tcp_cr15);
            core.SetTCP(Model.CR7, tcp_cr7);
            core.SetSpeed(Model.CR15, 100);
            core.SetSpeed(Model.CR7, 100);

            // (5) Set Station Antenna TCP (Cr7)
            // float[] station_cal_pin_tcp_cr7 = { 832.244f, -102.672f, 272.135f, 0.841f, 0.826f, -92.135f }; // lab version
            float[] station_cal_pin_tcp_cr7 = { 832.244f, -102.672f, 272.135f, 0.841f, 0.826f, -92.135f }; // lab version
            float station_cal_pin_length = 40.05f + 0.8f;
            var station_center_zero_tcp = core.GetStationCenterZeroTCP(station_cal_pin_tcp_cr7, station_cal_pin_length);
            float[] antenna_offset_station = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
            float station_offset = 0;
            var station_antenna_tcp_cr7 = core.GetStationAntennaTCP(station_center_zero_tcp, antenna_offset_station, station_offset);
            core.SetStationAntennaTCP_Cr7(station_antenna_tcp_cr7);


            // (7) Scene
            double[] param = { 0, 200, 3, 200, 3, 300, 2 };
            core.SceneParamInit(SceneName.Scene4, param);

            // (*) stage1
            core.SceneRobotInit(SceneName.Scene4);
            core.SetUserFrame(Model.CR15);
            core.SetUserFrame(Model.CR7);
            core.Scene4(MovementType.QuickCheck, MovementStage.One);

            // (*) stage2
            // core.SceneRobotInit(SceneName.Scene4_Sim);
            // core.SetUserFrame(Model.CR15);
            // core.SetUserFrame(Model.CR7);
            // core.Scene4_Sim(MovementType.QuickCheck, MovementStage.Two);
        }

        // CityU-Demo - StepRun
        public void DualRobot_Scene2_Sim_StepRun()
        {
            // (1) Connection
            DualRobotLib.Core core = new Core();
            core.Connect(Model.CR15, "127.0.0.1", 9021);
            core.Connect(Model.CR7, "127.0.0.1", 60008);
            // core.Connect(Model.CR15, "192.168.0.125", 60008);
            // core.Connect(Model.CR7, "192.168.0.124", 60008);

            // (2) Get Calibrated Co-Frame Data
            // % cal-2: current error: 0.2mm
            double[] Pos_Cr7_CalliBase = { 778.281, -38.520, -336.517, 0.810, 0.687, -92.389 };
            double[] Pos_Cr15_CalliBase = { 1252.171, 24.657, -770.478, 0.269, 0.423, 89.225 };
            // double[] Pos_Cr7_CalliBase = { 860.0, 0, 247.0, 0, 0, -90.0 };
            // double[] Pos_Cr15_CalliBase = { 1190.0, 0, -193.0, 0, 0, 90.0 };
            core.RobotBaseCalibrationInit(Pos_Cr7_CalliBase, Pos_Cr15_CalliBase);

            // (3) Get Tool Antenna TCP Data 
            float[] origin_wpr_cr7 = { -9.082f, 89.472f, -9.403f };
            float[] default_wpr_cr7 = { 89.529f, -1.760f, 88.218f };

            float[] origin_wpr_cr15 = { -17.024f, 87.760f, -17.928f };
            float[] default_wpr_cr15 = { -91.714f, 0.772f, -90.444f };

            var cal_wpr_cr7 = core.GetToolFixtureWPR(SceneName.Scene2_Sim, Model.CR7, origin_wpr_cr7, default_wpr_cr7);
            var cal_wpr_cr15 = core.GetToolFixtureWPR(SceneName.Scene2_Sim, Model.CR15, origin_wpr_cr15, default_wpr_cr15);

            float[] cal_pin_tcp_cr7 = { -120.609f, 180.878f, 134.808f, cal_wpr_cr7[0], cal_wpr_cr7[1], cal_wpr_cr7[2] };
            float[] cal_pin_tcp_cr15 = { -1.831f, 0.474f, 299.920f, cal_wpr_cr15[0], cal_wpr_cr15[1], cal_wpr_cr15[2] };
            float cal_pin_length_cr7 = 45.23f + 5.02f;
            float cal_pin_length_cr15 = 45.33f + 5.02f;
            var fixture_tcp_cr7 = core.GetToolFixtureTCP(Model.CR7, cal_pin_tcp_cr7, cal_pin_length_cr7);
            var fixture_tcp_cr15 = core.GetToolFixtureTCP(Model.CR15, cal_pin_tcp_cr15, cal_pin_length_cr15);

            // offset-1: cr7: 6.27f;  cr15:6.24f;
            // offset-2: cr7: 16.05f; cr15: 16.03f;
            float[] antenna_offset_cr7 = { 0.0f, 0.0f, 16.05f, 0.0f, 0.0f, 0.0f };
            float[] antenna_offset_cr15 = { 0.0f, 0.0f, 105.13f, 0.0f, 0.0f, 0.0f };
            var tcp_cr7 = core.GetToolAntennaTCP(Model.CR7, fixture_tcp_cr7, antenna_offset_cr7);
            var tcp_cr15 = core.GetToolAntennaTCP(Model.CR15, fixture_tcp_cr15, antenna_offset_cr15);

            // (4) Robot Initialization
            core.SetTCP(Model.CR15, tcp_cr15);
            core.SetTCP(Model.CR7, tcp_cr7);
            core.SetSpeed(Model.CR15, 100);
            core.SetSpeed(Model.CR7, 100);

            // (5) Set Station Antenna TCP (Cr7) (Cal. tool + UF: 0)
            float[] station_cal_pin_tcp_cr7 = { 832.066f, -102.571f, 277.335f, 0.953f, 0.102f, -92.282f }; // switch to tool:5 user frame:0
            float station_cal_pin_length = 41.05f;
            var station_center_zero_tcp = core.GetStationCenterZeroTCP(station_cal_pin_tcp_cr7, station_cal_pin_length);

            float[] antenna_offset_station = { 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f };
            float station_offset = 0; // 0-290

            var station_antenna_tcp_cr7 = core.GetStationAntennaTCP(station_center_zero_tcp, antenna_offset_station, station_offset);
            Console.WriteLine("station_antenna_tcp_cr7: " + string.Join(",", station_antenna_tcp_cr7));

            core.SetStationAntennaTCP_Cr7(station_antenna_tcp_cr7);

            // (6) Align LiftTable Height with station_center_zero_tcp
            double lift_table_align_error = 1.025;
            double stage34_fixture_height = 127;

            // (7) Scene Initialization
            // a. examples.
            double[] param = { 160, 90, 10, 180, 30, 100, 45, 0, 140, lift_table_align_error, stage34_fixture_height };

            core.SceneParamInit(SceneName.Scene2_Sim, param);
            // b.
            core.SceneRobotInit(SceneName.Scene2_Sim);
            // c.
            core.SetUserFrame(Model.CR15);
            core.SetUserFrame(Model.CR7);

            // (8) Scene2_Sim - StepRun Debug

            Thread th1 = new Thread(() => core.thread_GetViaPoints());
            th1.Start();
            
            // stage - 1
            core.Scene2_Sim(MovementType.StepRun, MovementStage.One);
            
            // stage - 2
            core.SceneRobotInit(SceneName.Scene2_Sim);
            core.Scene2_Sim(MovementType.StepRun, MovementStage.Two);
            
            // stage - 3
            param[1] = 20;
            param[8] = 13;
            core.SceneParamInit(SceneName.Scene2_Sim, param);
            core.SceneRobotInit(SceneName.Scene2_Sim, Model.Null, MovementStage.Three);
            core.Scene2_Sim(MovementType.StepRun, MovementStage.Three);
            
            // stage - 4
            param[1] = 20;
            param[8] = 13;
            core.SceneParamInit(SceneName.Scene2_Sim, param);
            core.SceneRobotInit(SceneName.Scene2_Sim, Model.Null, MovementStage.Four);
            core.Scene2_Sim(MovementType.StepRun, MovementStage.Four);
        }
        public void DualRobot_Scene1A_Sim_StepRun()
        {
            // (1) Connection
            DualRobotLib.Core core = new Core();
            core.Connect(Model.CR15, "127.0.0.1", 9021);
            core.Connect(Model.CR7, "127.0.0.1", 60008);
            // core.Connect(Model.CR15, "192.168.0.125", 60008);
            // core.Connect(Model.CR7, "192.168.0.124", 60008);

            // (2) Get Calibrated Co-Frame Data
            // % cal-2: current error: 0.2mm
            double[] Pos_Cr7_CalliBase = { 778.281, -38.520, -336.517, 0.810, 0.687, -92.389 };
            double[] Pos_Cr15_CalliBase = { 1252.171, 24.657, -770.478, 0.269, 0.423, 89.225 };
            core.RobotBaseCalibrationInit(Pos_Cr7_CalliBase, Pos_Cr15_CalliBase);

            // (3) Get Tool Antenna TCP Data 
            float[] origin_wpr_cr7 = { -9.082f, 89.472f, -9.403f };
            float[] default_wpr_cr7 = { 89.529f, -1.760f, 88.218f };

            float[] origin_wpr_cr15 = { -17.024f, 87.760f, -17.928f };
            float[] default_wpr_cr15 = { -91.714f, 0.772f, -90.444f };

            var cal_wpr_cr7 = core.GetToolFixtureWPR(SceneName.Scene2_Sim, Model.CR7, origin_wpr_cr7, default_wpr_cr7);
            var cal_wpr_cr15 = core.GetToolFixtureWPR(SceneName.Scene2_Sim, Model.CR15, origin_wpr_cr15, default_wpr_cr15);

            float[] cal_pin_tcp_cr7 = { -120.609f, 180.878f, 134.808f, cal_wpr_cr7[0], cal_wpr_cr7[1], cal_wpr_cr7[2] };
            float[] cal_pin_tcp_cr15 = { -1.831f, 0.474f, 299.920f, cal_wpr_cr15[0], cal_wpr_cr15[1], cal_wpr_cr15[2] };
            float cal_pin_length_cr7 = 45.23f + 5.02f;
            float cal_pin_length_cr15 = 45.33f + 5.02f;
            var fixture_tcp_cr7 = core.GetToolFixtureTCP(Model.CR7, cal_pin_tcp_cr7, cal_pin_length_cr7);
            var fixture_tcp_cr15 = core.GetToolFixtureTCP(Model.CR15, cal_pin_tcp_cr15, cal_pin_length_cr15);

            // offset-1: cr7: 6.27f;  cr15:6.24f;
            // offset-2: cr7: 16.05f; cr15: 16.03f;
            float[] antenna_offset_cr7 = { 0.0f, 0.0f, 16.05f, 0.0f, 0.0f, 0.0f };
            float[] antenna_offset_cr15 = { 0.0f, 0.0f, 50.35f, 0.0f, 0.0f, 0.0f };
            var tcp_cr7 = core.GetToolAntennaTCP(Model.CR7, fixture_tcp_cr7, antenna_offset_cr7);
            var tcp_cr15 = core.GetToolAntennaTCP(Model.CR15, fixture_tcp_cr15, antenna_offset_cr15);

            // (4) Robot Initialization
            core.SetTCP(Model.CR15, tcp_cr15);
            core.SetTCP(Model.CR7, tcp_cr7);
            core.SetSpeed(Model.CR15, 100);
            core.SetSpeed(Model.CR7, 100);

            // (5) Set Station Antenna TCP (Cr7) (Cal. tool + UF: 0)
            float[] station_cal_pin_tcp_cr7 = { 832.066f, -102.571f, 277.335f, 0.953f, 0.102f, -92.282f }; // switch to tool:5 user frame:0
            float station_cal_pin_length = 41.05f;
            var station_center_zero_tcp = core.GetStationCenterZeroTCP(station_cal_pin_tcp_cr7, station_cal_pin_length);

            float[] antenna_offset_station = { 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f };
            float station_offset = 0; // 0-290

            var station_antenna_tcp_cr7 = core.GetStationAntennaTCP(station_center_zero_tcp, antenna_offset_station, station_offset);
            Console.WriteLine("station_antenna_tcp_cr7: " + string.Join(",", station_antenna_tcp_cr7));

            core.SetStationAntennaTCP_Cr7(station_antenna_tcp_cr7);

            // (6) Align LiftTable Height with station_center_zero_tcp
            double lift_table_align_error = 1.025;
            double stage34_fixture_height = 127;

            // (7) Scene Initialization
            // a. examples.
            double[] param = { 250, 180, 10, 180, 90, 140, lift_table_align_error, stage34_fixture_height };
            core.SceneParamInit(SceneName.Scene1A_Sim, param);
            // b.
            core.SceneRobotInit(SceneName.Scene1A_Sim);
            // c.
            core.SetUserFrame(Model.CR15);
            core.SetUserFrame(Model.CR7);

            // (8) Scene2_Sim - StepRun Debug

            Thread th1 = new Thread(() => core.thread_GetViaPoints());
            th1.Start();

            Console.WriteLine(core.GetViaPointsInfo(SceneName.Scene1A_Sim)[0]);
            Console.WriteLine(core.GetViaPointsInfo(SceneName.Scene1A_Sim)[1]);
            Console.WriteLine(core.GetViaPointsInfo(SceneName.Scene1A_Sim)[2]);

            // stage - 1
            core.Scene1A_Sim(MovementType.StepRun, MovementStage.One);

            // stage - 2
            core.SceneRobotInit(SceneName.Scene1A_Sim);
            core.Scene1A_Sim(MovementType.StepRun, MovementStage.Two);
        }

        // Motor // LiftTable Demo
        public void Motor_Demo()
        {
            DualRobotLib.Core core = new Core();
            
            // 1. Connect
            core.Connect(Model.Motor, "COM3");

            // 2. Init(init params + move to zero)
            core.MotorInit();

            // 3. Absolute Movement
            core.MotorAbsMoveTo(30.5);

            // Optional Methods
            //
            // (1). Disconnect
            // core.Disconnect(Model.Motor);
            // (2). IsConnected
            // core.MotorIsConnected();
            // (3). IsMoving
            // var isMoving = core.MotorIsMoving();
            // (4). Get Current Degree
            // var curDegree = core.MotorGetDegree();
            // (5). Relative Move
            // core.MotorRelMoveTo(14.5);
        }
        public void LiftTable_Demo()
        {
            DualRobotLib.Core core = new Core();

            // 1. Connect
            core.Connect(Model.LiftTable, "192.168.0.119", 50000, "COM4");

            // 2. Init
            core.LiftTableInit();

            // 3. Absolute Move
            core.LiftTableAbsMoveTo(100);

            // Optional Methods
            //
            // (1). Disconnect
            // core.Disconnect(Model.LiftTable);
            // (2). IsConnected
            // core.LiftTableIsConnected();
            // (3). IsMoving
            // core.LiftTableIsMoving();
            // (4). Get Current Height
            // core.LiftTableGetCurHeight();

            // todo: Relative Movement
            // core.LiftTableRelMoveTo();
        }
        public void LiftTable_Demo2()
        {
            // (1) Connection
            DualRobotLib.Core core = new Core();
            //core.Connect(Model.CR15, "127.0.0.1", 9021);
            //core.Connect(Model.CR7, "127.0.0.1", 60008);
            core.Connect(Model.CR15, "192.168.0.125", 60008);
            core.Connect(Model.CR7, "192.168.0.124", 60008);

            core.Connect(Model.LiftTable, "192.168.0.119", 50000, "COM4");
            // core.Connect(Model.Motor, "COM3");

            // 2. Init
            core.LiftTableInit();

            Thread.Sleep(5000);
        }

        // TCP
        public void ListAllTCPs(ref DualRobotLib.Core core)
        {
            var cal_pos = core.GetRobotBaseCalibrationPose(); // relation between cr7 and cr15. formula: T_cr15 = T_cal * T_cr7
            var lttt_tcp = core.GetStationAntennaTCP_Cr7(); // record in cr7 base coordinate
            var cr7_tcp = core.GetCurPos(Model.CR7);        // record in cr7 base coordinate
            var cr15_tcp = core.GetCurPos(Model.CR15);      // record in cr15 base coordinate

            Console.WriteLine("cal_pos = " + string.Join(",", cal_pos));
            Console.WriteLine("lttt_tcp = " + string.Join(",", lttt_tcp));
            Console.WriteLine("cr7_tcp = " + string.Join(",", cr7_tcp));
            Console.WriteLine("cr15_tcp = " + string.Join(",", cr15_tcp));


            var dis1 = core.GetTcpDistance(SceneName.Scene1A_Sim, Model.LiftTable, Model.CR15); // get distance between lttt_tcp and cr15_tcp
            var dis2 = core.GetTcpDistance(SceneName.Scene1A_Sim, Model.LiftTable, Model.CR7);  // get distance between lttt_tcp and cr7_tcp
            var dis3 = core.GetTcpDistance(SceneName.Scene1A_Sim, Model.CR15, Model.CR7);       // get distance between cr15_tcp and cr7_tcp

            Console.WriteLine("distance between lttt_tcp and cr15_tcp: " + dis1);
            Console.WriteLine("distance between lttt_tcp and cr7_tcp: " + dis2);
            Console.WriteLine("distance between cr15_tcp and cr7_tcp: " + dis3);
        }
    }
}
