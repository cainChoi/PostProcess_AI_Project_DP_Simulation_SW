using AntennaModule;
using RadarSim.Core.Clutter;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace SeaClutterModule
{
    public class SeaClutterModel : IClutterModel
    {
        private SeaParameters parameters;
        /// <summary>
        /// 유효 지구 반경 (4/3 * 실제 지구 반경, 미터 단위)
        /// (표준 대기 굴절률 고려)
        /// </summary>
        private const double EffectiveEarthRadius_m = 8500000.0; // 8,500 km

        public void Initialize(IClutterParameters parameters)
        {
            if (parameters is SeaParameters seaParams)
            {
                this.parameters = seaParams;
            }
            else
            {
                throw new ArgumentException("Invalid parameter type for SeaClutterModel");
            }
        }

        /// <summary>
        /// 한 첩(chirp) 길이의 바다 클러터 I/Q 신호를 생성합니다.
        /// </summary>
        /// <param name="context">계산에 필요한 현재 레이더/플랫폼 상태</param>
        /// <param name="rand">엔진이 제공하는 공용 Random 객체</param>
        /// <returns>복소수 I/Q 샘플 배열</returns>
        public Complex[] GenerateClutterSignal(ClutterContext context, Random rand)
        {
            // --- 1. 지오메트리 계산 ---
            // 플랫폼 고도(Height)와 안테나 지향각(Boresight)으로
            // 빔이 바다에 닿는 '입사각(Grazing Angle)'을 계산합니다.
            double altitude = context.Platform.Position.Y;
            double grazingAngle_rad = CalculateGrazingAngle(context.Platform, context.AntennaBoresight_Global);

            // 빔이 바다를 비추지 않으면(수평선 위) 빈 신호 반환
            if (grazingAngle_rad <= 0 || altitude <= 0)
            {
                int numSamples_zero = (int)(context.ChirpDuration_sec * context.AdcSampleRate_Hz);
                return new Complex[numSamples_zero]; // 0으로 채워진 배열
            }

            // --- 2. 핵심 물리량 계산 (인수 준비) ---
            // 'range' 인수 준비
            double slantRange = altitude / Math.Sin(grazingAngle_rad);
            //'lambda' 인수 준비
            const double C = 299792458.0;
            double lambda = C / context.CenterFrequency_Hz;

            // --- 3. 클러터 반사도(σ⁰) 계산 ---
            // '입사각'과 '해상 상태'를 기반으로, 단위 면적당 반사도(σ⁰, "시그마-너트")를
            // 표준 모델(예: Georgia Tech, NRL 모델)을 사용해 계산합니다.
            double sigmaNought = CalculateSigmaNought(grazingAngle_rad,
                                                     this.parameters.SeaState,
                                                     context.CenterFrequency_Hz);

            // --- 4. 클러터 면적(Patch) 계산 ---
            // 빔폭과 거리(고도/sin(각도))를 이용해, 빔이 실제 바다를
            // 비추는 타원형 면적(Area)을 계산합니다.
            double clutterArea = CalculateClutterPatchArea(context.Antenna, grazingAngle_rad, slantRange);

            // 'rcs' 인수 준비
            double totalClutterRCS = sigmaNought * clutterArea;

            // --- 5. 총 클러터 RCS 및 전력 계산 ---
            // (RCS = σ⁰ * 면적, RRE로 평균 전력 계산)
            double meanClutterPower = CalculateScaledClutterPower(
                    totalClutterRCS,  // 클러터의 실제 RCS
                    slantRange,       // 클러터의 실제 거리
                    context           // 보정 기준값이 담긴 컨텍스트
                );
            double clutterStdDev = Math.Sqrt(meanClutterPower);

            // --- 6. 클러터 I/Q 샘플 생성 (통계 + 도플러) ---
            int numSamples = (int)(context.ChirpDuration_sec * context.AdcSampleRate_Hz);
            Complex[] clutterSamples = new Complex[numSamples];

            // (단순화된 모델: Rayleigh 분포 클러터)
            // (Rayleigh = I/Q가 독립적인 가우시안 분포)

            for (int i = 0; i < numSamples; i++)
            {
                // Box-Muller 변환으로 가우시안 난수 생성
                double u1 = 1.0 - rand.NextDouble(); // 0에 가까운 값 방지
                double u2 = 1.0 - rand.NextDouble();
                double i_rand = Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Sin(2.0 * Math.PI * u2);

                u1 = 1.0 - rand.NextDouble();
                u2 = 1.0 - rand.NextDouble();
                double q_rand = Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Sin(2.0 * Math.PI * u2);

                // 계산된 클러터 전력(표준편차)만큼 스케일링
                clutterSamples[i] = new Complex(i_rand * clutterStdDev,
                                                q_rand * clutterStdDev);
            }

            return clutterSamples;
        }

        /// <summary>
        /// 빔이 바다 표면에 닿는 입사각(Grazing Angle)을 계산합니다.
        /// (4/3 유효 지구 곡률 모델 적용)
        /// </summary>
        /// <param name="platform">플랫폼 모듈 (고도 정보용)</param>
        /// <param name="boresightGlobal">안테나의 현재 글로벌 지향각 (Normalized)</param>
        /// <returns>입사각 (Radians, 0 ~ PI/2)</returns>
        private double CalculateGrazingAngle(RadarSim.Core.Platform.IPlatformModel platform, Vector3 boresightGlobal)
        {
            // 1. 플랫폼 고도(h)
            double h = platform.Position.Y;

            // 2. 빔의 '내림각' (Depression Angle, θ_d) 계산
            //    (수평선 기준, 아래로 향하면 양수)
            double depressionAngle_rad = Math.Asin(-boresightGlobal.Y);

            // 3. 빔이 수평선 위를 향하면(각도가 0 이하) 클러터 없음
            if (depressionAngle_rad <= 0 || h <= 0)
            {
                return 0.0;
            }

            // 4. 지구 곡률이 적용된 입사각(γ) 계산
            //    (h가 Re에 비해 매우 작다는 표준 근사식 사용)
            //    γ ≈ arcsin( (h/R_s) - (R_s / (2*Re)) )
            //    여기서 R_s (경사 거리) ≈ h / sin(θ_d)

            // (4-1) 평평한 지구 기준 경사 거리(Slant Range) 계산
            double slantRange = h / Math.Sin(depressionAngle_rad);

            // (4-2) 곡률 보정 항 계산
            double correctionTerm = slantRange / (2.0 * EffectiveEarthRadius_m);

            // (4-3) 최종 입사각의 sin 값 계산
            double sin_gamma = (h / slantRange) - correctionTerm;
            // (sin_gamma = sin(θ_d) - (h / (2*Re*sin(θ_d))))

            // 5. 수평선 너머(sin_gamma < 0)로 빔이 향하면 클러터 없음
            if (sin_gamma <= 0)
            {
                return 0.0;
            }

            return Math.Asin(sin_gamma);
        }

        /// <summary>
        /// 클러터 반사도(sigma-nought, σ⁰)를 계산합니다.
        /// (매우 단순화된 경험적 모델 예시)
        /// </summary>
        /// <param name="grazingAngle_rad">입사각 (Radians)</param>
        /// <param name="seaState">해상 상태 (0-9)</param>
        /// <param name="frequency_Hz">중심 주파수 (Hz)</param>
        /// <returns>선형 단위의 sigma-nought 값</returns>
        private double CalculateSigmaNought(double grazingAngle_rad, int seaState, double frequency_Hz)
        {
            if (grazingAngle_rad <= 0) return 0.0;

            // 1. 해상 상태에 따른 기본 값 (dB)
            // (이 값들은 완전히 임의의 예시입니다)
            double baseSigma_dB = -40.0 + (seaState * 2.5); // 해상 상태가 나쁠수록 반사도 증가

            // 2. 주파수 의존성 (dB) (주파수가 높을수록 반사도 증가)
            double freq_GHz = frequency_Hz / 1.0e9;
            double freqDependency_dB = 10 * Math.Log10(freq_GHz / 10.0); // 10GHz 기준

            // 3. 입사각 의존성 (dB) (입사각이 클수록 반사도 증가)
            double angleDependency_dB = 10 * Math.Log10(Math.Sin(grazingAngle_rad));

            // 4. 총 dB 값 합산
            double totalSigma_dB = baseSigma_dB + freqDependency_dB + angleDependency_dB;

            // 5. dB -> 선형(Linear) 값으로 변환
            return Math.Pow(10, totalSigma_dB / 10.0);
        }

        /// <summary>
        /// 빔이 바다 표면을 비추는 클러터 면적(Patch Area)을 계산합니다.
        /// (CW/FMCW 빔-제한(Beam-limited) 시나리오 가정)
        /// </summary>
        /// <param name="antenna">안테나 모듈 (빔폭 정보용)</param>
        /// <param name="grazingAngle_rad">입사각 (Radians)</param>
        /// <param name="altitude">플랫폼 고도 (m)</param>
        /// <returns>클러터 면적 (m^2)</returns>
        private double CalculateClutterPatchArea(RadarSim.Core.Antenna.IAntennaModel antenna, double grazingAngle_rad, double altitude)
        {
            if (grazingAngle_rad <= 0 || altitude <= 0) return 0.0;

            // 1. 안테나 빔폭 (Radians)
            // (IAntennaModel에 GetParameters()가 있다고 가정)
            var antennaParams = (AntennaParameters)antenna.GetParameters();
            double beamwidth_rad = antennaParams.Beamwidth * (Math.PI / 180.0);

            // (빔이 대칭적이라고 가정, Azimuth = Elevation 빔폭)
            double beamwidth_az_rad = beamwidth_rad;
            double beamwidth_el_rad = beamwidth_rad;

            // 2. 바다 표면까지의 경사 거리 (Slant Range)
            double R = altitude / Math.Sin(grazingAngle_rad);

            // 3. 빔-제한 면적 계산 (Area = (R * Az_Beamwidth) * (R * El_Beamwidth / sin(GrazingAngle)))
            double area = (R * R * beamwidth_az_rad * beamwidth_el_rad) / Math.Sin(grazingAngle_rad);

            return area;
        }

        /// <summary>
        /// 엔진의 '기준 전력'을 '클러터의 실제 상황'에 맞게 스케일링합니다.
        /// (엔진의 RRE 스케일링 로직과 동일)
        /// </summary>
        private double CalculateScaledClutterPower(double totalClutterRCS, double clutterRange, ClutterContext context)
        {
            // (1) 엔진의 "기준점" 신호 전력 가져오기
            double referenceSignalPower = context.ReferenceSignalPower_W;

            // (2) 거리(R)에 대한 스케일링
            double R_ref = context.Reference_Range_m;
            double R_clutter = clutterRange;
            double scale_Range = (R_ref == 0 || R_clutter == 0) ? 1.0 : Math.Pow(R_ref / R_clutter, 4);

            // (3) RCS에 대한 스케일링
            double RCS_ref = context.Reference_RCS_m2;
            double scale_RCS = (RCS_ref == 0) ? 1.0 : (totalClutterRCS / RCS_ref);

            // (4) Gain에 대한 스케일링
            // (클러터는 빔 중심(Gain=1.0)에 있다고 가정. 
            //  표적의 Gain(gain)이 아닌 기준 Gain(Gain_ref)과 비교)
            double Gain_ref = context.Reference_Gain_Linear;
            double Gain_clutter = 1.0;
            double scale_Gain = (Gain_ref == 0) ? 1.0 : Math.Pow(Gain_clutter / Gain_ref, 2);

            // (5) 최종 스케일링된 클러터 전력
            return referenceSignalPower * scale_Range * scale_RCS * scale_Gain;
        }
    }
}
