using RadarSim.Core;
using System.Numerics;
using System; // Math
using System.IO; // 파일 쓰기
using System.Linq; // Linq
using System.Collections.Generic; // List
using System.Reflection; // Assembly
using AntennaModule;
using System.Threading.Tasks;

namespace SimulatorEngine
{
    public class SimulatorEngine
    {
        //저장소
        public class DataStorage
        {
            public double m_dTime;
            public RadarSim.Core.Trajectory.SnapShot m_ssTrajectory;
            public RadarSim.Core.Platform.SnapShot m_ssPlatform;
            public RadarSim.Core.Antenna.SnapShot m_ssAntenna;
            public RadarSim.Core.Header.SnapShot m_ssHeader;

            public double Time { get => m_dTime; }
            public RadarSim.Core.Trajectory.SnapShot Trajectory { get => m_ssTrajectory; }
            public RadarSim.Core.Platform.SnapShot Platform { get => m_ssPlatform; }
            public RadarSim.Core.Antenna.SnapShot Antenna { get => m_ssAntenna; }
            public RadarSim.Core.Header.SnapShot Header { get => m_ssHeader; }
        }


        public List<DataStorage> dataStorages = new List<DataStorage>();

        // --- 모듈 ---
        private RadarSim.Core.Trajectory.ITrajectoryModel trajectoryModule;
        private RadarSim.Core.Platform.IPlatformModel platformModule;
        private RadarSim.Core.Antenna.IAntennaModel antennaModule;
        private RadarSim.Core.Header.IHeaderModel headerModule;
        private List<RadarSim.Core.Clutter.IClutterModel> clutterModels = new List<RadarSim.Core.Clutter.IClutterModel>();

        // 잡음/클러터용 난수 생성기
        private Random noiseRand = new Random();
        private Random clutterRand = new Random();

        // --- 시뮬레이션 파라미터 (WPF UI에서 설정) ---
        //public double CenterFrequency_Hz { get; set; } = 10.5e9;

        /// <summary>
        /// CW 채널의 중심 주파수 (Hz)
        /// </summary>
        public double CenterFrequency_CW_Hz { get; set; } = 10.450e9; // 10.500 GHz

        ///// <summary>
        ///// FMCW 채널의 중심 주파수 (Hz)
        ///// </summary>
        //public double CenterFrequency_FMCW_Hz { get; set; } = 10.501e9; // 10.501 GHz

        public double StartFrequency_FMCW_Hz { get; set; } = 10.500e9; // 10.501 GHz

        //public double CenterFrequency_FMCW1_Hz { get; set; } = 10.505e9; // 10.501 GHz

        //public double CenterFrequency_FMCW2_Hz { get; set; } = 10.5075e9; // 10.501 GHz

        public double Bandwidth_B1_Hz { get; set; } = 10e6;
        public double Bandwidth_B2_Hz { get; set; } = 15e6;
        public double ChirpDuration_sec { get; set; } = 0.040; // 40ms
        public double AdcSampleRate_Hz { get; set; } = 1e6; // 1MHz

        // RRE 파라미터 (레이더 방정식)
        public double TxPower_W { get; set; } = 1.0;
        public double TxGain_Linear { get; set; } = 100.0; // 30dBi
        public double RxGain_Linear { get; set; } = 100.0; // 30dBi
        public double NoiseFigure_Linear { get; set; } = 1.58; // 2dB
        public double SystemTemp_K { get; set; } = 290.0;

        /// <summary>
        /// CW 채널의 유효 잡음 대역폭 (Hz)
        /// (예: 도플러 처리 대역폭 1kHz)
        /// </summary>
        public double NoiseBandwidth_CW_Hz { get; set; } = 1000.0;

        // --- 시뮬레이션 상태 변수 ---
        private double simulationTime = 0.0;
        private double timeSinceLastChirp = 0.0;
        private int chirpCounter = 0;
        

        // 빛의 속도 (공용)
        private const double C = 299792458.0;



        /// <summary>
        /// Loads the target module.(플러그인 구조)
        /// </summary>
        /// <param name="dllPath">The DLL path.</param>
        public void LoadTargetModule(string dllPath, RadarSim.Core.Trajectory.ITargetParameters parameters)
        {
            // 1. 선택된 DLL 파일을 런타임에 불러옴
            // 예: "C:\Plugins\BallisticModule.dll"
            Assembly pluginAssembly = Assembly.LoadFrom(dllPath);

            // 2. DLL 내부를 뒤져서 'ITrajectoryModel' 약속을 지킨
            //    클래스를 찾아냄
            Type moduleType = pluginAssembly.GetTypes()
                .FirstOrDefault(t => typeof(RadarSim.Core.Trajectory.ITrajectoryModel).IsAssignableFrom(t));

            if (moduleType != null)
            {
                // 3. 해당 클래스의 인스턴스를 생성
                //    (WPF는 이게 'BallisticModule'인지 'DroneModule'인지 모름)
                trajectoryModule = (RadarSim.Core.Trajectory.ITrajectoryModel)Activator.CreateInstance(moduleType);

                trajectoryModule.Initialize(parameters);
            }
        }

        public void LoadPlatformModule(string dllPath, RadarSim.Core.Platform.IPlatformParameters parameters)
        {
            // 1. 선택된 DLL 파일을 런타임에 불러옴
            // 예: "C:\Plugins\BallisticModule.dll"
            Assembly pluginAssembly = Assembly.LoadFrom(dllPath);

            // 2. DLL 내부를 뒤져서 'IPlatformModel' 약속을 지킨
            //    클래스를 찾아냄
            Type moduleType = pluginAssembly.GetTypes()
                .FirstOrDefault(t => typeof(RadarSim.Core.Platform.IPlatformModel).IsAssignableFrom(t));

            if (moduleType != null)
            {
                // 3. 해당 클래스의 인스턴스를 생성
                platformModule = (RadarSim.Core.Platform.IPlatformModel)Activator.CreateInstance(moduleType);

                platformModule.Initialize(parameters);
            }
        }

        public void LoadAntennaModule(string dllPath, RadarSim.Core.Antenna.IAntennaParameters parameters)
        {
            // 1. 선택된 DLL 파일을 런타임에 불러옴
            // 예: "C:\Plugins\BallisticModule.dll"
            Assembly pluginAssembly = Assembly.LoadFrom(dllPath);

            // 2. DLL 내부를 뒤져서 'IAntennaModel' 약속을 지킨
            //    클래스를 찾아냄
            Type moduleType = pluginAssembly.GetTypes()
                .FirstOrDefault(t => typeof(RadarSim.Core.Antenna.IAntennaModel).IsAssignableFrom(t));

            if (moduleType != null)
            {
                // 3. 해당 클래스의 인스턴스를 생성
                antennaModule = (RadarSim.Core.Antenna.IAntennaModel)Activator.CreateInstance(moduleType);

                antennaModule.Initialize(parameters);
            }
        }

        public void AddClutterModule(string dllPath, RadarSim.Core.Clutter.IClutterParameters parameters)
        {
            // 1. 선택된 DLL 파일을 런타임에 불러옴
            // 예: "C:\Plugins\BallisticModule.dll"
            Assembly pluginAssembly = Assembly.LoadFrom(dllPath);

            // 2. DLL 내부를 뒤져서 'IClutterModel' 약속을 지킨
            //    클래스를 찾아냄
            Type moduleType = pluginAssembly.GetTypes()
                .FirstOrDefault(t => typeof(RadarSim.Core.Clutter.IClutterModel).IsAssignableFrom(t));

            if (moduleType != null)
            {
                // 3. 해당 클래스의 인스턴스를 생성
                var cluttermodule = (RadarSim.Core.Clutter.IClutterModel)Activator.CreateInstance(moduleType);
                cluttermodule.Initialize(parameters);
                clutterModels.Add(cluttermodule);
            }
        }

        public void LoadHeaderModule(string dllPath, RadarSim.Core.Header.IHeaderParameters parameters)
        {
            // 1. 선택된 DLL 파일을 런타임에 불러옴
            // 예: "C:\Plugins\BallisticModule.dll"
            Assembly pluginAssembly = Assembly.LoadFrom(dllPath);

            // 2. DLL 내부를 뒤져서 'IHeaderParameters' 약속을 지킨
            //    클래스를 찾아냄
            Type moduleType = pluginAssembly.GetTypes()
                .FirstOrDefault(t => typeof(RadarSim.Core.Header.IHeaderModel).IsAssignableFrom(t));

            if (moduleType != null)
            {
                // 3. 해당 클래스의 인스턴스를 생성
                headerModule = (RadarSim.Core.Header.IHeaderModel)Activator.CreateInstance(moduleType);

                headerModule.Initialize(parameters);
            }
        }


        private List<FileStream> fileStreams;
        private List<BinaryWriter> fileWriters;

        public delegate void deleUpdateProcess(double dCur, double dEnd);
        public deleUpdateProcess m_deleUpdateProcess = null;

        /// <summary>
        /// 메인 시뮬레이션 루프를 시작합니다.
        /// </summary>
        /// <param name="totalTime">총 시뮬레이션 시간(초)</param>
        /// <param name="dt">물리 업데이트 시간 간격(초)</param>
        public void RunSimulation(double totalTime, double dt, bool bFileSave, bool bHeaderAdd)
        {
            if (trajectoryModule == null || platformModule == null || antennaModule == null)
            { 
                throw new InvalidOperationException("모든 모듈이 로드되어야 합니다.");
            }
            if (bHeaderAdd)
            {
                if (headerModule == null)
                {
                    throw new InvalidOperationException("헤더 모듈이 로드되어야 합니다.");
                }
            }

            if (bFileSave)
            { 
                this.fileStreams = new List<FileStream>();
                this.fileWriters = new List<BinaryWriter>();
                string strDate = DateTime.Now.ToString("yyyyMMddHHmmss");
                DirectoryInfo dirInfo = new DirectoryInfo(strDate + "\\");
                dirInfo.Create();
                string strDateFileName = DateTime.Now.ToString("yyyyMMdd_HHmmss");
                for (int i = 0; i < 8; i++) // 8개 채널
                {
                    // (헤더가 필요하면 여기서 먼저 Write)

                    // I 채널 파일
                    FileStream fs_i = new FileStream($"{strDate}\\CH{i + 1}_I_{strDateFileName}_00001.bin", FileMode.Create);
                    this.fileStreams.Add(fs_i);
                    this.fileWriters.Add(new BinaryWriter(fs_i));

                    // Q 채널 파일
                    FileStream fs_q = new FileStream($"{strDate}\\CH{i + 1}_Q_{strDateFileName}_00001.bin", FileMode.Create);
                    this.fileStreams.Add(fs_q);
                    this.fileWriters.Add(new BinaryWriter(fs_q));
                }
            }

            // 상태 초기화
            simulationTime = 0.0;
            timeSinceLastChirp = 0.0;
            chirpCounter = 0;
            dataStorages.Clear();
            while (simulationTime <= totalTime)
            {
                // 이 함수가 메인 루프의 핵심입니다.
                RunSimulationStep(dt, bFileSave);

                //System.Diagnostics.Debug.Print(string.Format("{0}", trajectoryModule.Position.Y));

                dataStorages.Add(new DataStorage() { m_dTime = dt, m_ssTrajectory = trajectoryModule.GetSnapshot(), m_ssPlatform = platformModule.GetSnapshot(), m_ssAntenna = antennaModule.GetSnapshot() });

                simulationTime += dt;
                timeSinceLastChirp += dt;

                m_deleUpdateProcess?.Invoke(simulationTime, totalTime);
            }

            if (bFileSave)
            {
                // ⬇️ 4. 시뮬레이션이 끝나면 모든 파일을 안전하게 닫음
                foreach (var writer in this.fileWriters)
                {
                    writer.Close(); // (Writer를 닫으면 Stream도 닫힘)
                }

                this.fileStreams.Clear();
                this.fileWriters.Clear();
            }
        }

        /// <summary>
        /// 메인 시뮬레이션 루프를 시작하고,
        /// 표적이 해수면(Y=0)에 닿을 때까지의
        /// 총 비행 시간을 반환합니다.
        /// </summary>
        /// <param name="dt">물리 업데이트 시간 간격(초)</param>
        /// <returns>총 비행 시간(초)</returns>        
        public double RunSimulation_GetFlightTime(double dt, bool bFileSave)
        {
            if (trajectoryModule == null || platformModule == null || antennaModule == null)
                throw new InvalidOperationException("모든 모듈이 로드되어야 합니다.");

            // 상태 초기화
            simulationTime = 0.0;
            timeSinceLastChirp = 0.0;
            chirpCounter = 0;

            do
            {
                // 이 함수가 메인 루프의 핵심입니다.
                RunSimulationStep(dt, bFileSave);

                //System.Diagnostics.Debug.Print(trajectoryModule.Position.Y.ToString());
                //System.Diagnostics.Debug.Print(platformModule.Attitude.Y.ToString());
                System.Diagnostics.Debug.Print(platformModule.Attitude.Y.ToString());

                simulationTime += dt;
                timeSinceLastChirp += dt;
            } while (trajectoryModule.Position.Y > 0);

            return simulationTime;
        }

        /// <summary>
        /// 단일 물리 시간(dt)만큼 시뮬레이션을 진행합니다.
        /// 처프 주기가 되면 I/Q 데이터 생성을 트리거합니다.
        /// </summary>
        /// <param name="dt">물리 시간 간격(초)</param>
        public void RunSimulationStep(double dt, bool bFileSave)
        {
            // ---------------------------------------------
            // STAGE 1: Ground Truth 업데이트 (매 스텝 실행)
            // ---------------------------------------------
            trajectoryModule.Update(dt);
            
            platformModule.Update(dt);

            // 안테나는 다른 모듈의 최신 상태를 기반으로 업데이트
            antennaModule.Update(
                dt,
                platformModule.Attitude,
                trajectoryModule.Position,
                platformModule.Position
            );


            // ---------------------------------------------
            // STAGE 2 & 3: I/Q 생성 (주기적으로 실행)
            // ---------------------------------------------
            if (timeSinceLastChirp >= this.ChirpDuration_sec)
            {
                // ⬇️ 3. 표적이 활성 상태인지 확인
                bool targetIsActive = trajectoryModule.IsActive;
                RadarPhysicsSnapshot physics;
                double rcs = 0, gain = 0;

                if (targetIsActive)
                {
                    // 4. 활성 상태일 때만 물리량 계산
                    (physics, rcs, gain) = CalculateRadarPhysics();
                }
                else
                {
                    // 5. 비활성 상태면 빈 스냅샷 생성
                    //    (클러터/잡음 계산에 필요할 수 있는 값들만 채움)
                    (physics, rcs, gain) = GetInactivePhysicsSnapshot();
                }

                if(bFileSave)
                {
                    GenerateAndSaveHeader(dt, chirpCounter);
                }

                // 6. '활성' 상태와 함께 I/Q 생성 함수 호출
                GenerateAndSaveIQ(physics, rcs, gain, targetIsActive, bFileSave);

                //System.Diagnostics.Debug.Print(physics.FmcwBeat_B1.ToString() + ", " + physics.RadialVelocity.ToString());

                // 7. 타이머 리셋
                timeSinceLastChirp -= this.ChirpDuration_sec;
                chirpCounter++;
            }
        }

        /// <summary>
        /// 표적이 비활성(IsActive == false)일 때
        /// GenerateAndSaveIQ에 전달할 빈 스냅샷을 생성합니다.
        /// (RCS/Gain은 0, Noise 계산을 위한 Bandwidth는 유효)
        /// </summary>
        private (RadarPhysicsSnapshot physics, double rcs, double gain) GetInactivePhysicsSnapshot()
        {
            // 1. RCS와 Gain은 0으로 설정
            double rcs = 0.0;
            double gain = 0.0;

            // 2. 잡음(Noise) 계산에 필요한 '현재 첩의 대역폭'은 
            //    'CalculateRadarPhysics'와 동일한 로직으로 계산합니다.
            double currentBandwidth;
            if (chirpCounter % 2 == 0)
            {
                currentBandwidth = Bandwidth_B1_Hz;
            }
            else
            {
                currentBandwidth = Bandwidth_B2_Hz;
            }

            // 3. "빈" 스냅샷 객체 생성
            var snapshot = new RadarPhysicsSnapshot
            {
                // 표적 관련 값은 모두 0 또는 기본값
                RelativeRange = 0,
                RadialVelocity = 0,
                DopplerFrequency_CW = 0,
                DopplerFrequency_FMCW = 0,
                FmcwBeat_B1 = 0,
                FmcwBeat_B2 = 0,
                LosUnitVector_Global = Vector3.UnitZ, // (기본값)

                // ⬇️ 잡음 계산을 위해 이 값은 올바르게 설정
                Bandwidth = currentBandwidth
            };

            // 4. (physics, rcs, gain) 튜플 반환
            return (snapshot, rcs, gain);
        }

        private (RadarPhysicsSnapshot physics, double rcs, double gain) CalculateRadarPhysics()
        {
            // 모듈에서 최신 상태 가져오기
            Vector3 targetPos = trajectoryModule.Position;
            Vector3 targetVel = trajectoryModule.Velocity;
            Quaternion targetAtt = trajectoryModule.Orientation;

            Vector3 platformPos = platformModule.Position;
            Vector3 platformVel = platformModule.Velocity;
            Quaternion platformAtt = platformModule.Attitude;

            Vector3 boresight_Plat = antennaModule.BoresightVector_PlatformCoords;
            // (안테나 마운트 오프셋 적용 필요 - 단순화를 위해 일단 플랫폼 위치 사용)
            Vector3 antennaPos = platformPos;

            // A. 상대 거리 및 LOS(Line-of-Sight) 벡터
            Vector3 relativePos = targetPos - antennaPos;
            double range = relativePos.Length();
            Vector3 losUnitVector = Vector3.Normalize(relativePos);

            // B. 상대 속도 (v_target - v_platform) ⋅ LOS
            Vector3 relativeVel = targetVel - platformVel;
            double radialVelocity = Vector3.Dot(relativeVel, losUnitVector);

            // C. 동적 RCS 계산 (표적의 로컬 좌표계 기준 입사각)
            Vector3 lookVector_Global = -relativePos; // 레이더가 표적을 보는 방향
            Vector3 lookVector_Local = Vector3.Transform(lookVector_Global, Quaternion.Conjugate(targetAtt));
            double rcs = trajectoryModule.GetRCS(lookVector_Local);

            // D. 동적 Gain 계산 (오프보어사이트 각)
            Vector3 boresight_Global = Vector3.Transform(boresight_Plat, platformAtt);
            float dot = Vector3.Dot(boresight_Global, losUnitVector);
            //double offBoresightAngle_Rad = Math.Acos(Math.Clamp(dot, -1.0f, 1.0f));
            float clampedDot = Math.Max(-1.0f, Math.Min(1.0f, dot));
            double offBoresightAngle_Rad = Math.Acos(clampedDot);
            double gain = antennaModule.GetGain(offBoresightAngle_Rad);

            // E. 비트 주파수 계산
            double f_dop_cw = (2 * radialVelocity * CenterFrequency_CW_Hz) / C;
            //double f_dop_fmcw = (2 * radialVelocity * CenterFrequency_FMCW_Hz) / C;

            // Chirp 1 (B1)용
            double f_c_b1 = this.StartFrequency_FMCW_Hz + (Bandwidth_B1_Hz / 2.0); // (예: 10.505 GHz)
            double f_dop_b1 = (2 * radialVelocity * f_c_b1) / C;

            // Chirp 2 (B2)용
            double f_c_b2 = this.StartFrequency_FMCW_Hz + (Bandwidth_B2_Hz / 2.0); // (예: 10.5075 GHz)
            double f_dop_b2 = (2 * radialVelocity * f_c_b2) / C;

            //// B1용 거리 주파수
            double f_range_b1 = (2 * range * Bandwidth_B1_Hz) / (C * ChirpDuration_sec);
            //// B2용 거리 주파수
            double f_range_b2 = (2 * range * Bandwidth_B2_Hz) / (C * ChirpDuration_sec);
            double f_beat_fmcw_b1 = f_range_b1 - f_dop_b1;
            double f_beat_fmcw_b2 = f_range_b2 - f_dop_b2;



            // 현재 첩의 대역폭과 FMCW 비트 주파수 결정
            double currentBandwidth;
            double current_f_c_fmcw; // 현재 첩의 실제 f_c
            double current_f_dop_fmcw; // 현재 첩의 실제 f_d

            //Odd/Even 뒤집기
            //if (chirpCounter % 2 == 0)
            if (chirpCounter % 2 != 0)
            {
                currentBandwidth = Bandwidth_B1_Hz;
                current_f_c_fmcw = f_c_b1;
                current_f_dop_fmcw = f_dop_b1;
            }
            else
            {
                currentBandwidth = Bandwidth_B2_Hz;
                current_f_c_fmcw = f_c_b2;
                current_f_dop_fmcw = f_dop_b2;
            }

            // 결과 스냅샷 생성
            var snapshot = new RadarPhysicsSnapshot
            {
                RelativeRange = range,
                RadialVelocity = radialVelocity,
                DopplerFrequency_CW = f_dop_cw, // CW용
                DopplerFrequency_FMCW = current_f_dop_fmcw, // ⬅️ 현재 첩의 f_d
                FmcwBeat_B1 = f_beat_fmcw_b1, // B1용 (참고용)
                FmcwBeat_B2 = f_beat_fmcw_b2, // B2용 (참고용)
                Bandwidth = currentBandwidth,
                ActualCarrierFrequency_Hz = current_f_c_fmcw, // ⬅️ 현재 첩의 f_c
                LosUnitVector_Global = losUnitVector // 글로벌 LOS 벡터 전달
            };

            return (snapshot, rcs, gain);
        }

        // --- STAGE 3 헬퍼: I/Q 생성 및 저장 ---

        private void GenerateAndSaveIQ(RadarPhysicsSnapshot physics, double rcs, double gain, bool targetIsActive, bool bFileSave)
        { 
            // 1. 신호 및 잡음 전력 계산
            double R = physics.RelativeRange;
            double k_Boltzmann = 1.380649e-23;


            int numSamples = (int)(this.ChirpDuration_sec * this.AdcSampleRate_Hz);
            double timeStep = 1.0 / this.AdcSampleRate_Hz;

            // ⬇️ 1. 잡음 대역폭을 ADC 샘플링 속도로 고정
            //    (Nyquist = AdcSampleRate_Hz / 2 이지만,
            //     ADC 앞단 필터가 보통 샘플링 속도와 비슷하므로 AdcSampleRate_Hz를 사용)
            double adcNoiseBandwidth = this.AdcSampleRate_Hz;

            // ⬇️ 2. 잡음 계산을 루프 밖으로 이동 (모든 채널 공통)
            double noisePower = k_Boltzmann * SystemTemp_K * adcNoiseBandwidth * NoiseFigure_Linear;
            double noiseStdDev = Math.Sqrt(noisePower);

            // ⬇️ 3. 양자화 기준값도 루프 밖으로 이동 (모든 채널 공통)
            double maxQuantVal = 3.0 * noiseStdDev;


            AntennaParameters antennaParams = (AntennaParameters)antennaModule.GetParameters();

            // 안테나의 현재 지향각 (클러터 계산용)
            Vector3 boresight_Global = Vector3.Transform(
                antennaModule.BoresightVector_PlatformCoords,
                platformModule.Attitude);

            Parallel.For(0, antennaParams.ElementPositions.Length,
                i =>
                {

                    //for (int i = 0; i < antennaParams.ElementPositions.Length; i++)
                    //{
                    double current_f_beat;
                    double current_f_carrier;
                    //double current_noise_bandwidth;

                    // A. 채널 타입 결정
                    if (antennaParams.CwChannelIndices.Contains(i))
                    {
                        // CW 채널
                        current_f_beat = physics.DopplerFrequency_CW;
                        current_f_carrier = this.CenterFrequency_CW_Hz;
                        //current_noise_bandwidth = this.NoiseBandwidth_CW_Hz; // ⬇️ CW 잡음 대역폭
                    }
                    else // (FMCW 채널)
                    {
                        // FMCW 채널
                        //current_f_beat = (chirpCounter % 2 == 0)
                        //Odd/Even 뒤집기
                        current_f_beat = (chirpCounter % 2 != 0)
                                         ? physics.FmcwBeat_B1
                                         : physics.FmcwBeat_B2;
                        current_f_carrier = physics.ActualCarrierFrequency_Hz;
                        //current_noise_bandwidth = physics.Bandwidth; // ⬇️ FMCW 첩 대역폭
                    }


                    // B. 신호 전력 계산 (Signal Power)
                    // B-1. 현재 채널의 파장(lambda) 계산
                    double lambda = C / current_f_carrier;

                    // B-2. 레이더 방정식으로 신호 전력 계산
                    double signalPower = (TxPower_W * TxGain_Linear * RxGain_Linear * rcs * gain * gain * lambda * lambda)
                                       / (Math.Pow(4 * Math.PI, 3) * Math.Pow(R, 4));

                    // B-3. 신호 진폭 계산
                    double signalAmplitude = Math.Sqrt(signalPower * 2);

                    //// C. 잡음 전력 계산
                    //double noisePower = k_Boltzmann * SystemTemp_K * current_noise_bandwidth * NoiseFigure_Linear;
                    //double noiseStdDev = Math.Sqrt(noisePower);


                    // D. 채널별 위상 편이 계산 (핵심!)
                    // D-1. 기준 위상 (도플러 위상)
                    // (안테나 마운트 기준점까지의 왕복 위상)
                    double basePhase = (-4 * Math.PI * physics.RelativeRange * current_f_carrier) / C;

                    // D-2. 배열 위상 편이 (각 소자의 위치 차이)
                    // (플랫폼 자세가 반영된 글로벌 소자 위치 필요)
                    Vector3 elementOffset_Global = Vector3.Transform(
                        antennaParams.ElementPositions[i],
                        platformModule.Attitude
                    );

                    // (신호가 들어오는 경로 차이: d * cos(theta))
                    //'i번째' 소자와 기준점 간의 "경로 길이 차이" 계산 (소자 오프셋을 '표적 방향(LOS)' 벡터에 투영)
                    double pathDifference = Vector3.Dot(elementOffset_Global, physics.LosUnitVector_Global);

                    // (왕복 경로 차이에 대한 위상 편이) "경로 길이 차이"를 "위상 편이"로 변환
                    double arrayPhaseShift = (-2 * Math.PI * current_f_carrier * (2 * pathDifference)) / C;

                    // D-3. 최종 초기 위상(채널마다 고유한 값을 가짐)
                    double totalInitialPhase = basePhase + arrayPhaseShift;


                    // ----------------------------------------------------
                    // ⬇️ E. 클러터(Clutter) 신호 생성 (신규 추가) ⬇️
                    // ----------------------------------------------------
                    Complex[] clutterSignal = new Complex[numSamples]; // 0으로 자동 초기화

                    if (this.clutterModels.Any())
                    {
                        // (1) 현재 채널의 컨텍스트 생성
                        RadarSim.Core.Clutter.ClutterContext context = new RadarSim.Core.Clutter.ClutterContext
                        {
                            Platform = this.platformModule,
                            Antenna = this.antennaModule,
                            CenterFrequency_Hz = current_f_carrier,
                            Bandwidth_Hz = this.NoiseBandwidth_CW_Hz,
                            ChirpDuration_sec = this.ChirpDuration_sec,
                            AdcSampleRate_Hz = this.AdcSampleRate_Hz,
                            TxPower_W = this.TxPower_W,
                            TxGain_Linear = this.TxGain_Linear,
                            RxGain_Linear = this.RxGain_Linear,
                            ChannelIndex = i,
                            AntennaBoresight_Global = boresight_Global // 안테나 지향각 전달
                        };

                        // (2) 모든 클러터 모듈의 신호를 합산
                        foreach (var model in this.clutterModels)
                        {
                            Complex[] c = model.GenerateClutterSignal(context, this.clutterRand);
                            for (int s = 0; s < numSamples; s++)
                            {
                                clutterSignal[s] += c[s]; // ⬅️ 합산
                            }
                        }
                    }
                    // ----------------------------------------------------

                    // --- F. 샘플 생성 및 최종 합산 ---
                    short[] i_data = new short[numSamples];
                    short[] q_data = new short[numSamples];
                    double currentTime = 0.0;
                    //double maxQuantVal = 3.0 * noiseStdDev;

                    // ⬇️ --- 마이크로 도플러 구현 (수정) --- ⬇️
                    double[] phaseNoiseArray;
                    if (targetIsActive)
                    {
                        // (1) 엔진은 플러그인에게 위상 배열을 '요청'
                        phaseNoiseArray = trajectoryModule.GetMicroDopplerPhaseNoise(
                            numSamples,
                            timeStep,
                            current_f_carrier,
                            this.noiseRand // (엔진의 공용 Random 객체 사용)
                        );
                    }
                    else
                    {
                        // (2) 표적이 비활성이면 빈 배열
                        phaseNoiseArray = new double[numSamples]; // 0으로 채워짐
                    }
                    // ⬆️ ----------------------------------- ⬆️

                    for (int s = 0; s < numSamples; s++)
                    {
                        // (1) 표적 신호
                        double i_sig = 0.0; // 기본값 0
                        double q_sig = 0.0; // 기본값 0

                        // 표적이 활성 상태일 때만 표적 신호(i_sig, q_sig)를 계산
                        if (targetIsActive)
                        {
                            double phase_noise = phaseNoiseArray[s];

                            double signalPhase = (2 * Math.PI * current_f_beat * currentTime)
                                         + totalInitialPhase
                                         + phase_noise; // ⬅️ 스핀/프로펠러 효과 적용

                            i_sig = signalAmplitude * Math.Cos(signalPhase);
                            q_sig = signalAmplitude * Math.Sin(signalPhase);

                            //double signalPhase = (2 * Math.PI * current_f_beat * currentTime) + totalInitialPhase;
                            //i_sig = signalAmplitude * Math.Cos(signalPhase);
                            //q_sig = signalAmplitude * Math.Sin(signalPhase);
                        }

                        //double signalPhase = (2 * Math.PI * current_f_beat * currentTime) + totalInitialPhase;
                        //double i_sig = signalAmplitude * Math.Cos(signalPhase);
                        //double q_sig = signalAmplitude * Math.Sin(signalPhase);

                        // (2) 클러터 신호
                        double i_clutter = clutterSignal[s].Real;
                        double q_clutter = clutterSignal[s].Imaginary;

                        // (3) 잡음 신호
                        double u1 = 1.0 - noiseRand.NextDouble();
                        double u2 = 1.0 - noiseRand.NextDouble();
                        double randStdNormal = Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Sin(2.0 * Math.PI * u2);
                        double i_noise = noiseStdDev * randStdNormal;

                        u1 = 1.0 - noiseRand.NextDouble();
                        u2 = 1.0 - noiseRand.NextDouble();
                        randStdNormal = Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Sin(2.0 * Math.PI * u2);
                        double q_noise = noiseStdDev * randStdNormal;

                        // ⬇️ 최종 합산 (Target + Clutter + Noise)
                        double i_total = i_sig + i_clutter + i_noise;
                        double q_total = q_sig + q_clutter + q_noise;

                        // (4) 양자화
                        double i_val = i_total / maxQuantVal * 32767.0;
                        double q_val = q_total / maxQuantVal * 32767.0;

                        i_data[s] = (short)Math.Max(short.MinValue, Math.Min(short.MaxValue, i_val));
                        q_data[s] = (short)Math.Max(short.MinValue, Math.Min(short.MaxValue, q_val));

                        currentTime += timeStep;
                    }

                    // --- E. 채널별 파일 저장 ---

                    if (bFileSave)
                    {
                        // (1) 멤버 변수에서 올바른 Writer 선택
                        BinaryWriter writer_I = this.fileWriters[i * 2];
                        BinaryWriter writer_Q = this.fileWriters[i * 2 + 1];

                        // (2) short[] 배열을 byte[] 배열로 변환 (가장 빠름)
                        byte[] i_bytes = new byte[numSamples * 2]; // short는 2바이트
                        Buffer.BlockCopy(i_data, 0, i_bytes, 0, i_bytes.Length);

                        byte[] q_bytes = new byte[numSamples * 2];
                        Buffer.BlockCopy(q_data, 0, q_bytes, 0, q_bytes.Length);

                        // (3) 변환된 byte[]를 파일에 한 번에 쓰기
                        writer_I.Write(i_bytes);
                        writer_Q.Write(q_bytes);
                    }
                    //}
                });
        }

        private void GenerateAndSaveHeader(double dTime, int nSequence)
        {
            AntennaParameters antennaParams = (AntennaParameters)antennaModule.GetParameters();
            Parallel.For(0, antennaParams.ElementPositions.Length,
                i =>
                {
                    // (1) 멤버 변수에서 올바른 Writer 선택
                    BinaryWriter writer_I = this.fileWriters[i * 2];
                    BinaryWriter writer_Q = this.fileWriters[i * 2 + 1];

                    // (2) short[] 배열을 byte[] 배열로 변환 (가장 빠름)
                    byte[] i_bytes = new byte[128];
                    byte[] q_bytes = new byte[128];
                    int[] arg_i = new int[6];
                    int[] arg_q = new int[6];
                    int nArgIdx = 0;
                    arg_i[nArgIdx] = arg_q[nArgIdx] = i + 1;//CH
                    nArgIdx++;
                    if (i >= 4)
                    {
                        arg_i[nArgIdx] = arg_q[nArgIdx] = 4;//FMCW
                    }
                    else
                    {
                        arg_i[nArgIdx] = arg_q[nArgIdx] = 1;//CW
                    }
                    nArgIdx++;
                    //MFCW Profile Skip
                    nArgIdx++;
                    //I or Q Identifier
                    arg_i[nArgIdx] = 0;
                    arg_q[nArgIdx] = 0;
                    nArgIdx++;
                    //COLLECTION MODE
                    arg_i[nArgIdx] = arg_q[nArgIdx] = 2;

                    Buffer.BlockCopy(headerModule.GenerateFileHeader(dTime, nSequence, arg_i), 0, i_bytes, 0, i_bytes.Length);
                    Buffer.BlockCopy(headerModule.GenerateFileHeader(dTime, nSequence, arg_q), 0, q_bytes, 0, q_bytes.Length);

                    // (3) 변환된 byte[]를 파일에 한 번에 쓰기
                    writer_I.Write(i_bytes);
                    writer_Q.Write(q_bytes);
                });
        }

        // 계산된 물리량을 담아둘 내부 구조체
        public struct RadarPhysicsSnapshot
        {
            public double RelativeRange;
            public double RadialVelocity;

            // 비트 주파수를 분리
            public double DopplerFrequency_CW;     // CW 채널용
            public double DopplerFrequency_FMCW;   // FMCW 채널용
            public double FmcwBeat_B1;          // FMCW (B1) 채널용
            public double FmcwBeat_B2;          // FMCW (B2) 채널용

            public double Bandwidth;            // 현재 첩의 대역폭
            public Vector3 LosUnitVector_Global; // 위상 계산용 (글로벌 좌표계)

            /// <summary>
            /// 이 첩(Chirp)을 계산하는 데 사용된
            /// '실제' 중심 주파수 (f_c) (Hz)
            /// </summary>
            public double ActualCarrierFrequency_Hz;
        }
    }
}
