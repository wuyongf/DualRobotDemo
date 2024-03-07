using System;
using System.Collections.Generic;
using System.Linq;
using System.Security.Cryptography.X509Certificates;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using DualRobotLib;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace DualRobotDemo
{
    /// <summary>
    /// Dual Robot Example Code
    /// </summary>
    public partial class MainWindow : Window
    {
        public MainWindow()
        {
            // (1) Connection
            DualRobotLib.Core core = new Core();
            core.Connect(Model.CR15, "127.0.0.1", 9021);
            core.Connect(Model.CR7, "127.0.0.1", 60008);

            // (2) Get Calibrated Co-Frame Data
            double[] Pos_Cr7_CalliBase = { 778.281, -38.520, -336.517, 0.810, 0.687, -92.389 };
            double[] Pos_Cr15_CalliBase = { 1252.171, 24.657, -770.478, 0.269, 0.423, 89.225 };
            core.RobotBaseCalibrationInit(Pos_Cr7_CalliBase, Pos_Cr15_CalliBase);

            // (3) Set Probe TCP
            float[] tcp_cr7 = { 0.0f, 0.0f, 0.0f, 0, 0, 0 };
            float[] tcp_cr15 = { 0, 0, 260, 0, 0, 0 };

            // (4) Robot Initialization
            core.SetTCP(Model.CR15, tcp_cr15);
            core.SetTCP(Model.CR7, tcp_cr7);
            core.SetSpeed(Model.CR15, 100);
            core.SetSpeed(Model.CR7, 100);

            // (5) Set Station Antenna TCP (Cr7)
            // float[] station_cal_pin_tcp_cr7 = { 832.244f, -102.672f, 272.135f, 0.841f, 0.826f, -92.135f }; // lab version
            float[] dutCalPinTCP = { 832.244f, -102.672f, 127.135f, 0.841f, 0.826f, -92.135f }; // manual cal version

            float dutCalPinLength = 40.05f; // x + 0.8f
            var dutFixtureTCP = core.GetStationCenterZeroTCP(dutCalPinTCP, dutCalPinLength);

            float dutFixtureLength = 439.5f;
            var turntableCenterTcp = core.GetStationCenterZeroTCP(dutFixtureTCP, dutFixtureLength);

            float[] antenna_offset_station = { 0.0f, 0.0f, 655.0f, 0.0f, 0.0f, 0.0f };
            float station_offset = 0;
            var station_antenna_tcp_cr7 = core.GetStationAntennaTCP(turntableCenterTcp, antenna_offset_station, station_offset);
            core.SetStationAntennaTCP_Cr7(station_antenna_tcp_cr7);

            // (7) Scene
            // double[] param = { 0, 200, 2, 200, 2, 300, 2 };
            double[] param = { 45, 200, 3, 200, 3, 255, 2 };
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
    }
}
