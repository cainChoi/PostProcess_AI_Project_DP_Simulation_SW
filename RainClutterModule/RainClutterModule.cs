using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using RadarSim.Core.Clutter;
using System.Numerics;
using AntennaModule;

namespace RainClutterModule
{
    public class RainClutterModule
    {
        private ClutterParameters parameters;
        private const double C = 299792458.0;

        public void Initialize(IClutterParameters parameters)
        {
            if (parameters is ClutterParameters rainParams)
            {
                this.parameters = rainParams;
            }
            else
            {
                throw new ArgumentException("Invalid parameter type for RainClutterModel");
            }
        }

        /// <summary>
        /// 한 첩(chirp) 길이의 강우 클러터 I/Q 신호를 생성합니다.
        /// </summary>
        public Complex[] GenerateClutterSignal(ClutterContext context, Random rand)
        {
            int numSamples = (int)(context.ChirpDuration_sec * context.AdcSampleRate_Hz);
            Complex[] clutterSamples = new Complex[numSamples]; // 0으로 초기화

            // --- 1. 지오메트리 계산 (빔과 비구름의 교차) ---
            // (안테나 고도, 빔 방향, 구름 고도를 기반으로
            //  빔이 비구름을 통과하는 거리 R_min, R_max 계산)
            (double R_min, double R_max) = CalculateBeamIntersection(
                context.Platform.Position.Y + context.Antenna.GetParameters().MountOffset.Y, // 안테나 실제 고도
                context.AntennaBoresight_Global,
                this.parameters.CloudBase_m,
                this.parameters.CloudTop_m
            );

            // 빔이 비구름과 교차하지 않으면 빈 신호 반환
            if (R_min >= R_max)
            {
                return clutterSamples;
            }

            // --- 2. 체적 반사율 (Eta, η) 계산 ---
            // Marshall-Palmer 관계식 (Z = 200 * R^1.6)
            double R = this.parameters.RainRate_mm_hr;
            double Z_linear = 200.0 * Math.Pow(R, 1.6);

            double lambda_m = C / context.CenterFrequency_Hz;
            double K_sq = 0.93; // 물의 유전 상수

            // Eta (m^2 / m^3) = (pi^5 * |K|^2 * Z_linear) / (lambda^4 * 10^18)
            double eta = (Math.Pow(Math.PI, 5) * K_sq * Z_linear) /
                       (Math.Pow(lambda_m, 4) * 1.0e18);

            // --- 3. 클러터 체적(Volume) 계산 ---
            // (빔폭과 교차 거리를 이용해 빔이 비구름 내에서 차지하는 체적 계산)
            // (단순화: 빔폭 * (R_max^2 - R_min^2) / 2)
            var antennaParams = (AntennaParameters)context.Antenna.GetParameters();
            double beamwidth_rad = antennaParams.Beamwidth * (Math.PI / 180.0);
            double solidAngle = (Math.PI / 4) * beamwidth_rad * beamwidth_rad; // 원뿔 근사

            // V = (SolidAngle/3) * (R_max^3 - R_min^3)
            double clutterVolume = (solidAngle / 3.0) * (Math.Pow(R_max, 3) - Math.Pow(R_min, 3));

            // --- 4. 총 클러터 RCS 및 전력 계산 ---
            double totalClutterRCS = eta * clutterVolume;
            double avgRange = (R_min + R_max) / 2.0;

            //double meanClutterPower = CalculatePowerFromRCS(
            //    totalClutterRCS,
            //    avgRange,
            //    lambda_m,
            //    context
            //);
            double meanClutterPower = CalculateScaledClutterPower(
                totalClutterRCS,  // 클러터의 실제 RCS
                avgRange,       // 클러터의 실제 거리
                context           // 보정 기준값이 담긴 컨텍스트
                );

            double clutterStdDev = Math.Sqrt(meanClutterPower); // Rayleigh 분포의 표준편차

            // --- 5. 평균 클러터 도플러(Mean Doppler) 계산 ---
            Vector3 rainFallVector = new Vector3(0, (float)this.parameters.FallSpeed_mps, 0);
            Vector3 rainVelocity_Global = this.parameters.WindVector_Global + rainFallVector;
            Vector3 platformVelocity_Global = context.Platform.Velocity;

            // 레이더 빔 방향 (LOS)으로의 상대 속도
            Vector3 v_relative = rainVelocity_Global - platformVelocity_Global;
            Vector3 los_vector = context.AntennaBoresight_Global;
            double v_radial = Vector3.Dot(v_relative, los_vector);

            double meanDopplerFreq = (2 * v_radial * context.CenterFrequency_Hz) / C;

            // --- 6. I/Q 샘플 생성 (도플러가 적용된 Rayleigh 잡음) ---
            double timeStep = 1.0 / context.AdcSampleRate_Hz;
            double currentTime = 0.0;

            for (int i = 0; i < numSamples; i++)
            {
                // (1) 기본 Rayleigh 잡음 생성 (I/Q가 독립 가우시안)
                double u1 = 1.0 - rand.NextDouble();
                double u2 = 1.0 - rand.NextDouble();
                double i_rand = Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Sin(2.0 * Math.PI * u2) * clutterStdDev;

                u1 = 1.0 - rand.NextDouble();
                u2 = 1.0 - rand.NextDouble();
                double q_rand = Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Sin(2.0 * Math.PI * u2) * clutterStdDev;

                // (2) 평균 도플러 주파수로 회전(Modulation)
                double signalPhase = 2 * Math.PI * meanDopplerFreq * currentTime;
                double cosPhase = Math.Cos(signalPhase);
                double sinPhase = Math.Sin(signalPhase);

                clutterSamples[i] = new Complex(
                    i_rand * cosPhase - q_rand * sinPhase,
                    i_rand * sinPhase + q_rand * cosPhase
                );

                currentTime += timeStep;
            }

            return clutterSamples;
        }

        // --- 내부 헬퍼 함수들 ---

        // ⬇️ --- 4. 헬퍼 함수 수정 (이름 및 로직 변경) --- ⬇️
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

        private (double R_min, double R_max) CalculateBeamIntersection(double antennaAltitude, Vector3 boresightGlobal, double cloudBase, double cloudTop)
        {
            // 빔 지향각의 수직 성분 (Vy)
            float vy = boresightGlobal.Y;

            // 빔이 위를 향하거나 수평이면 (Vy >= 0)
            if (vy >= 0)
            {
                // 빔이 위를 향하는데 안테나가 구름 하단보다 낮으면 교차
                if (antennaAltitude < cloudTop)
                {
                    double R_min = (antennaAltitude < cloudBase) ? (cloudBase - antennaAltitude) / vy : 0;
                    double R_max = (cloudTop - antennaAltitude) / vy;
                    return (R_min, R_max);
                }
                return (0, 0); // 교차 없음
            }
            else // 빔이 아래를 향하면 (Vy < 0)
            {
                // 빔이 아래를 향하는데 안테나가 구름 상단보다 높으면 교차
                if (antennaAltitude > cloudBase)
                {
                    double R_min = (antennaAltitude > cloudTop) ? (antennaAltitude - cloudTop) / -vy : 0;
                    double R_max = (antennaAltitude - cloudBase) / -vy;
                    return (R_min, R_max);
                }
                return (0, 0); // 교차 없음
            }
        }

        //private double CalculatePowerFromRCS(double rcs, double range, double lambda, ClutterContext context)
        //{
        //    // (엔진의 RRE 로직과 동일)
        //    double signalPower = (context.TxPower_W * context.TxGain_Linear * context.RxGain_Linear * rcs * lambda * lambda)
        //                       / (Math.Pow(4 * Math.PI, 3) * Math.Pow(range, 4));
        //    return signalPower;
        //}

    }
}
