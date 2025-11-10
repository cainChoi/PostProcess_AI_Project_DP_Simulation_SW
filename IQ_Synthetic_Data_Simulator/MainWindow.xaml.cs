using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Numerics;
using System.Text;
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

namespace IQ_Synthetic_Data_Simulator
{
    /// <summary>
    /// MainWindow.xaml에 대한 상호 작용 논리
    /// </summary>
    public partial class MainWindow : Window
    {
        BackgroundWorker worker = new BackgroundWorker();
        public MainWindow()
        {
            InitializeComponent();
            worker.DoWork += Worker_DoWork;
            worker.RunWorkerAsync();
        }

        private void UpdateState(double dCur, double dEnd)
        {
            Dispatcher.BeginInvoke(System.Windows.Threading.DispatcherPriority.Normal, new Action(() =>
            {
                pb_State.Value = dCur;
                pb_State.Maximum = dEnd;
            }));
        }

        private void Worker_DoWork(object sender, DoWorkEventArgs e)
        {
            SimulatorEngine.SimulatorEngine engine = new SimulatorEngine.SimulatorEngine();
            engine.m_deleUpdateProcess += UpdateState;
            engine.CenterFrequency_CW_Hz = 10_450_000_000;
            engine.StartFrequency_FMCW_Hz = 10_500_000_000;
            engine.Bandwidth_B1_Hz = 9_934_000;
            engine.Bandwidth_B2_Hz = 15_894_000;
            string strPath = Environment.CurrentDirectory + "\\";
            engine.LoadTargetModule(strPath + "BallisticModule.dll", new BallisticModule.TargetParameters() { LaunchElevation = 25, LaunchAzimuth = 70 });
            engine.LoadPlatformModule(strPath + "ShipModule.dll", new ShipModule.PlatformParameters() );
            engine.LoadAntennaModule(strPath + "AntennaModule.dll", new AntennaModule.AntennaParameters());
            engine.LoadHeaderModule(strPath + "DopplerSensorModule.dll", new DopplerSensorModule.HeaderParameters() { dtTime = DateTime.Now, abSTX = BitConverter.GetBytes((ulong)0xDEF09ABC56781234) });

            engine.AddClutterModule(strPath + "SeaClutterModule.dll", new SeaClutterModule.SeaParameters());
            

            engine.RunSimulation(50, 0.04, true, true);
            //double d45Deg = engine.RunSimulation_GetFlightTime(0.04);

            List<object> results = RadarSim.Core.PropertyExtractor.ExtractNestedProperty(engine.dataStorages, "Trajectory.Position.Y");
            return;

            engine.LoadTargetModule(strPath + "BallisticModule.dll", new BallisticModule.TargetParameters() { LaunchElevation = 15 });
            engine.LoadPlatformModule(strPath + "ShipModule.dll", new ShipModule.PlatformParameters());
            engine.LoadAntennaModule(strPath + "AntennaModule.dll", new AntennaModule.AntennaParameters());
            System.Diagnostics.Debug.Print("15도");
            double d15Deg = engine.RunSimulation_GetFlightTime(0.04, false);
        }
    }
}
