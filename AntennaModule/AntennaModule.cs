using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using RadarSim.Core.Antenna;
using System.Numerics;

namespace AntennaModule
{
    public class AntennaModule : IAntennaModel
    {
        private AntennaParameters parameters;

        // --- 파라미터 저장 변수 ---
        private Vector3 mountOffset_Platform; // 플랫폼 좌표계 기준 마운트 위치
        private double beamwidth_Rad; // 라디안 단위 빔폭

        // 가우시안 빔 패턴 근사를 위한 상수 (k)
        // Gain = exp(-k * angle^2)
        // 3dB 지점 (Gain=0.5, angle=Beamwidth/2)을 기준으로 k 계산
        // k = ln(2) / (Beamwidth/2)^2 ~= 2.772 / Beamwidth^2
        private double gainConstant_k;

        // --- 내부 상태 변수 ---
        private Vector3 boresightVector_Platform; // 플랫폼 좌표계 기준 지향각
        private Vector3 angularVelocity_Global; // 글로벌 좌표계 기준 각속도
        private Vector3 previousBoresightVector_Global; // 각속도 계산용

        // --- 인터페이스 속성 구현 ---
        public Vector3 BoresightVector_PlatformCoords => this.boresightVector_Platform;
        public Vector3 AngularVelocity_Global => this.angularVelocity_Global;

        private double maxSlewRate_Rad_s; // 라디안/초
        private double trackingNoiseStdDev_Rad; // 라디안
        private Random jitterRand = new Random();

        // (BoresightVector_Platform가 이제 '실제' 짐벌의 기계적 방향이 됨)

        public SnapShot GetSnapshot()
        {
            return new SnapShot(BoresightVector_PlatformCoords, AngularVelocity_Global, true);
        }


        // --- 인터페이스 메서드 구현 ---

        public void Initialize(IAntennaParameters parameters)
        {
            if (parameters is AntennaParameters antennaParams)
            {

                // ⬇️ 2. 파라미터를 멤버 변수에 저장
                this.parameters = antennaParams;

                this.mountOffset_Platform = antennaParams.MountOffset;

                // Degree -> Radian 변환하여 저장
                this.beamwidth_Rad = antennaParams.Beamwidth * (Math.PI / 180.0);

                // 빔 패턴 상수 계산
                if (this.beamwidth_Rad > 0)
                {
                    // k = 2.772 / Beamwidth^2
                    this.gainConstant_k = 2.7725887 / (this.beamwidth_Rad * this.beamwidth_Rad);
                }
                else
                {
                    this.gainConstant_k = 0; // 빔폭이 0이면 게인 계산 안 함
                }

                // 상태 변수 초기화
                this.boresightVector_Platform = Vector3.UnitZ; // 초기엔 플랫폼 정면
                this.angularVelocity_Global = Vector3.Zero;
                this.previousBoresightVector_Global = Vector3.UnitZ; // Z축=North

                this.maxSlewRate_Rad_s = antennaParams.MaxSlewRateDegreesPerSec * (Math.PI / 180.0);
                this.trackingNoiseStdDev_Rad = antennaParams.TrackingNoiseStdDevDegrees * (Math.PI / 180.0);
            }
            else
            {
                throw new ArgumentException("Invalid parameter type for GimbalTrackerModel");
            }
        }

        public void Update(double timeDelta,
                           Quaternion platformAttitude,
                           Vector3 targetPosition_Global,
                           Vector3 platformPosition_Global)
        {
            // 1. 안테나 마운트의 '글로벌' 위치 계산
            //    (플랫폼 위치 + 플랫폼 자세에 따라 회전된 마운트 오프셋)
            Vector3 rotatedMountOffset = Vector3.Transform(this.mountOffset_Platform, platformAttitude);
            Vector3 antennaMountPosition_Global = platformPosition_Global + rotatedMountOffset;

            // 2. 안테나 -> 표적을 향하는 '글로벌' 벡터 계산
            Vector3 targetVector_Global = targetPosition_Global - antennaMountPosition_Global;
            Vector3 targetDirection_Global = Vector3.Normalize(targetVector_Global);

            // 3. (핵심) 안정화된 '플랫폼 좌표계'의 지향 벡터 계산
            //    글로벌 지향 벡터를 플랫폼의 자세(Attitude)로 "되돌려" 놓습니다.
            //    (플랫폼이 흔들려도 이 벡터는 표적을 가리킴)
            this.boresightVector_Platform = Vector3.Transform(
                targetDirection_Global,
                Quaternion.Conjugate(platformAttitude)
            );

            // 4. (보너스) 글로벌 각속도 계산 (도플러 계산용)
            if (timeDelta > 0)
            {
                Vector3 currentBoresight_Global = targetDirection_Global;

                // 두 벡터 사이의 회전 각도 및 축 계산
                Vector3 rotationAxis = Vector3.Cross(this.previousBoresightVector_Global, currentBoresight_Global);
                float rotationAngle = (float)Math.Acos(
                    Math.Max(-1.0f, Math.Min(1.0f, // Clamp
                    Vector3.Dot(this.previousBoresightVector_Global, currentBoresight_Global)))
                );

                // 각속도 = (회전축 * 회전각) / 시간
                this.angularVelocity_Global = (rotationAxis.LengthSquared() > 0)
                    ? (Vector3.Normalize(rotationAxis) * rotationAngle / (float)timeDelta)
                    : Vector3.Zero;

                // 다음 프레임을 위해 현재 값 저장
                this.previousBoresightVector_Global = currentBoresight_Global;
            }
        }

        public double GetGain(double offBoresightAngle_Rad)
        {
            if (this.parameters.UseSidelobe)
            {
                double x = this.gainConstant_k * offBoresightAngle_Rad;
                double gain = Math.Abs(Math.Sin(x) / x); // 사이드로브가 생김
                return gain;
            }
            else
            {
                if (this.gainConstant_k == 0) return 1.0; // 빔폭 0이면 항상 1.0

                // 5. 가우시안 빔 패턴 근사: Gain = exp(-k * (angle)^2)
                //    offBoresightAngle_Rad는 이미 라디안 단위
                double gain = Math.Exp(-this.gainConstant_k * offBoresightAngle_Rad * offBoresightAngle_Rad);

                return gain;
            }
        }

        public IAntennaParameters GetParameters()
        {
            // 저장해둔 멤버 변수를 그대로 반환
            return this.parameters;
        }
    }
}
