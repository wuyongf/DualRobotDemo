//DualRobotLib.Core core = new Core();

//// 1. Connect
//core.Connect(Model.LiftTable, "192.168.0.119", 50000, "COM4");

//// 2. Init
//core.LiftTableInit();

//// 3. Absolute Move
//core.LiftTableAbsMoveTo(140);


#if (SCENE1A_TEST)
            //################################################################################################################################################
            // YF Testing Scene1A - Start
            /// Scene1A
            ///YF Testing///
            // (1) Connection
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

            var cal_wpr_cr7 = core.GetToolFixtureWPR(SceneName.Scene2_Sim, Model.CR7, origin_wpr_cr7, default_wpr_cr7);
            var cal_wpr_cr15 = core.GetToolFixtureWPR(SceneName.Scene2_Sim, Model.CR15, origin_wpr_cr15, default_wpr_cr15);

            float[] cal_pin_tcp_cr7 = { -120.609f, 180.878f, 134.808f, cal_wpr_cr7[0], cal_wpr_cr7[1], cal_wpr_cr7[2] };
            float[] cal_pin_tcp_cr15 = { -1.831f, 0.474f, 299.920f, cal_wpr_cr15[0], cal_wpr_cr15[1], cal_wpr_cr15[2] };
            float cal_pin_length_cr7 = 50.25f;
            float cal_pin_length_cr15 = 50.35f;
            var fixture_tcp_cr7 = core.GetToolFixtureTCP(Model.CR7, cal_pin_tcp_cr7, cal_pin_length_cr7);
            var fixture_tcp_cr15 = core.GetToolFixtureTCP(Model.CR15, cal_pin_tcp_cr15, cal_pin_length_cr15);

            //float[] antenna_offset_cr15 = { 0.0f, 0.0f, 50.35f, 0.0f, 0.0f, 0.0f };//Scene 1A Testing Cr15 Cal Pin = 50.35
            float[] antenna_offset_cr15 = { 0.0f, 0.0f, 4.99f, 0.0f, 0.0f, 0.0f };//Scene 1A Testing Cr15 Cal Pin = 5.00

            var tcp_cr15 = core.GetToolAntennaTCP(Model.CR15, fixture_tcp_cr15, antenna_offset_cr15);

            // (4) Robot Initialization
            core.SetTCP(Model.CR15, tcp_cr15);
            core.SetSpeed(Model.CR15, 100);
            core.SetSpeed(Model.CR7, 100);

            // (5) Set Station Antenna TCP (Cr7) (Cal. tool + UF: 0) *** need to update***
            //float[] station_cal_pin_tcp_cr7 = { 834.256f, -101.723f, 144.610f, 0.937f, -4.023f, -95.016f }; // zero position
            //float[] station_cal_pin_tcp_cr7 = { 833.261f, -102.093f, 281.305f, 0.833f, -5.159f, -95.258f }; // 135.793 - (height difference) position
            float[] station_cal_pin_tcp_cr7 = { 832.975f, -101.911f, 280.829f, 0.484f, 0.529f, -92.352f }; // 135.793(Ideal)|136.04(Real) - (height difference) position
            float station_cal_pin_length = 50.2f;
            var station_center_zero_tcp = core.GetStationCenterZeroTCP(station_cal_pin_tcp_cr7, station_cal_pin_length);

            //float[] antenna_offset_station = { 0.0f, 0.0f, 50.35f, 0.0f, 0.0f, 0.0f };//Scene 1A Testing Station Cal Pin = 50.35
            float[] antenna_offset_station = { 0.0f, 0.0f, 4.95f, 0.0f, 0.0f, 0.0f };//Scene 1A Testing Station Cal Plate = 5.00
                                                                                     //float[] antenna_offset_station = { 0.0f, 0.0f, 71.53f, 0.0f, 0.0f, 0.0f };//Scene 1A Testing Station DUT = 71.53
            float station_offset = 0;

            var station_antenna_tcp_cr7 = core.GetStationAntennaTCP(station_center_zero_tcp, antenna_offset_station, station_offset);
            Console.WriteLine("station_antenna_tcp_cr7: " + string.Join(",", station_antenna_tcp_cr7));

            core.SetStationAntennaTCP_Cr7(station_antenna_tcp_cr7);

            // (6) Align LiftTable Height with station_center_zero_tcp
            double lift_table_align_error = 0.247; //136.04(Real for Cal)-135.793(Ideal)
            double stage34_fixture_height = 0;
            double antenna_height = antenna_offset_station[2];

            // (7) Scene Initialization
            // a. examples.
            //double[] param = { 160, 180, 10, 180, 90, 13, lift_table_align_error, stage34_fixture_height, antenna_height };//Scene 1A Testing without station fixture offset

            double[] param = { 250, 180, 45, 180, 90, 135.793, lift_table_align_error, stage34_fixture_height, antenna_height }; //Scene 1A Testing with station fixture offset
            core.SceneParamInit(SceneName.Scene1A, param);
            // b.
            core.SceneRobotInit(SceneName.Scene1A);
            // c.
            core.SetUserFrame(Model.CR15);
            core.SetUserFrame(Model.CR7);

            //var tcp_distance = core.GetTcpDistance();
            //Console.WriteLine("tcp_distance: " + tcp_distance);
            //tcp_distance = core.GetTcpDistance_Scene2(Model.CR7);
            //Console.WriteLine("tcp_distance: " + tcp_distance);
            //tcp_distance = core.GetTcpDistance_Scene2(Model.CR15);
            //Console.WriteLine("tcp_distance: " + tcp_distance);

            var tcp_distance = core.GetTcpDistance(SceneName.Scene1A, Model.LiftTable, Model.CR15);
            Console.WriteLine("tcp_distance: " + tcp_distance);

            // (8) Execute Scene1A
            //
            core.Scene1A(MovementType.QuickCheck, MovementStage.One);

            //core.SetStationAntennaTCP_Cr7(station_antenna_tcp_cr7);
            core.SceneRobotInit(SceneName.Scene1A);
            core.Scene1A(MovementType.QuickCheck, MovementStage.Two);

            if (core.GetMoveFlag(Model.CR15) == 2)
            {
                double[] temp_val = { 0, 0, 0 };
                temp_val = core.GetViaPointLocation();
                Console.WriteLine("Arrived GetViaPointLocation = " + temp_val[0] + temp_val[1] + temp_val[2]);
            }
            // YF Testing Scene1A - End


            //################################################################################################################################################

#endif

#if (SCENE2)
            /////// <summary>
            /////// Scene 2///
            /////// </summary>
            /////// 
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
            //float[] antenna_offset_cr7 = { 0.0f, 0.0f, 50.25f, 0.0f, 0.0f, 0.0f };
            //float[] antenna_offset_cr15 = { 0.0f, 0.0f, 50.35f, 0.0f, 0.0f, 0.0f };
            //float[] antenna_offset_cr7 = { 0.0f, 0.0f, 50.18f, 0.0f, 0.0f, 0.0f };
            float[] antenna_offset_cr7 = { 0.0f, 0.0f, 71.53f, 0.0f, 0.0f, 0.0f };
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
            Console.WriteLine("station_antenna_tcp_cr7: " + station_antenna_tcp_cr7);

            core.SetStationAntennaTCP_Cr7(station_antenna_tcp_cr7);

            // (6) Align LiftTable Height with station_center_zero_tcp
            double lift_table_align_error = 1.13;
            double stage34_fixture_height = 0;

            // (7) Scene Initialization
            // a. examples.
            double[] param = { 250, 90, 45, 180, 90, 100, 45, 0, 140, lift_table_align_error, stage34_fixture_height };
            core.SceneParamInit(SceneName.Scene2, param);
            // b.
            core.SceneRobotInit(SceneName.Scene2);
            // c.
            core.SetUserFrame(Model.CR15);
            core.SetUserFrame(Model.CR7);

            //(8) Execute Scene2

            //stage - 1
            core.Scene2(MovementType.QuickCheck, MovementStage.One);

            ////stage - 2
            core.SceneRobotInit(SceneName.Scene2);
            core.Scene2(MovementType.QuickCheck, MovementStage.Two);

            ////// stage - 3
            ////param[1] = 20;
            ////param[8] = 13;
            ////core.SceneParamInit(SceneName.Scene2, param);
            ////core.SceneRobotInit(SceneName.Scene2, Model.Null, MovementStage.Three);
            ////core.Scene2(MovementType.QuickCheck, MovementStage.Three);

            ////// stage - 4
            ////param[1] = 20;
            ////param[8] = 13;
            ////core.SceneParamInit(SceneName.Scene2, param);
            ////core.SceneRobotInit(SceneName.Scene2, Model.Null, MovementStage.Four);
            ////core.Scene2(MovementType.QuickCheck, MovementStage.Four);

            ///// < summary >
            ///// Scene 2///
            ///// </ summary >

#endif

#if (SCENE1A)



            //################################################################################################################################################
            /// <summary>
            /// Scene 1A///
            /// </summary>
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
            float[] antenna_offset_cr7 = { 0.0f, 0.0f, 50.25f, 0.0f, 0.0f, 0.0f };
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
            float station_offset = 0;

            var station_antenna_tcp_cr7 = core.GetStationAntennaTCP(station_center_zero_tcp, antenna_offset_station, station_offset);
            Console.WriteLine("station_antenna_tcp_cr7: " + station_antenna_tcp_cr7);

            core.SetStationAntennaTCP_Cr7(station_antenna_tcp_cr7);

            // (6) Align LiftTable Height with station_center_zero_tcp
            double lift_table_align_error = 1.025;
            double stage34_fixture_height = 127;

            // (7) Scene Initialization
            // a. examples.
            double[] param = { 250, 180, 10, 180, 90, 13, lift_table_align_error, stage34_fixture_height };
            core.SceneParamInit(SceneName.Scene1A, param);
            // b.
            core.SceneRobotInit(SceneName.Scene1A);
            // c.
            core.SetUserFrame(Model.CR15);
            core.SetUserFrame(Model.CR7);

            // (8) Execute Scene1A
            //
            core.Scene1A(MovementType.QuickCheck, MovementStage.One);

            //core.SceneRobotInit(SceneName.Scene1A);
            //core.Scene1A(MovementType.QuickCheck, MovementStage.Two);

            /// <summary>
            /// Scene 1A///
            /// </summary>

#endif

#if (SCENE1C)
            /// <summary>
            /// Scene 1C///
            /// </summary>
            // (1) Connection
            DualRobotLib.Core core = new Core();
            //core.Connect(Model.CR15, "127.0.0.1", 9021);
            //core.Connect(Model.CR7, "127.0.0.1", 60008);
            core.Connect(Model.CR15, "192.168.0.125", 60008);
            core.Connect(Model.CR7, "192.168.0.124", 60008);

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
            //float[] antenna_offset_cr7 = { 0.0f, 0.0f, 45.14f, 0.0f, 0.0f, 0.0f };
            //float[] antenna_offset_cr15 = { 0.0f, 0.0f, 105.13f, 0.0f, 0.0f, 0.0f };
            float[] antenna_offset_cr7 = { 0.0f, 0.0f, 16.05f, 0.0f, 0.0f, 0.0f };
            float[] antenna_offset_cr15 = { 0.0f, 0.0f, 16.05f, 0.0f, 0.0f, 0.0f };
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
            //double[] param = { 100, 350, 350, 4, 4, 0 };
            double[] param = { 100, 350, 350, 4, 4, 0 };//Testing 230105
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
            /// <summary>
            /// Scene 1C///
            /// </summary>
#endif
