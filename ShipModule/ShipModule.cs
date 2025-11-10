using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using RadarSim.Core.Platform;
using System.Numerics;

namespace ShipModule
{
    public class ShipModule : IPlatformModel
    {
        // --- 내부 상태 변수 ---
        private Vector3 currentPosition;
        private Vector3 currentVelocity;
        private Quaternion currentAttitude;
        private double simulationTime = 0.0;

        // --- 파라미터 저장 변수 ---
        private double pitchAmplitude_rad;
        private double pitchFrequency_rad_s; // (2*PI / Period)
        private double rollAmplitude_rad;
        private double rollFrequency_rad_s;
        private double initialYaw_rad; // 함선의 기본 침로(방향)

        // --- 인터페이스 속성 구현 ---

        /// <summary>
        /// 현재 시간의 플랫폼 3D 위치 (글로벌 좌표계)
        /// </summary>
        public Vector3 Position => this.currentPosition;

        /// <summary>
        /// 현재 시간의 플랫폼 3D 속도 (글로벌 좌표계)
        /// </summary>
        public Vector3 Velocity => this.currentVelocity;

        /// <summary>
        /// 현재 시간의 플랫폼 3D 자세 (Roll, Pitch, Yaw)
        /// </summary>
        public Quaternion Attitude => this.currentAttitude;

        public SnapShot GetSnapshot()
        {
            return new SnapShot(Position, Velocity, Attitude, true);
        }


        // --- 인터페이스 메서드 구현 ---

        public void Initialize(IPlatformParameters parameters)
        {
            if (parameters is PlatformParameters shipParams)
            {
                // 1. 초기 위치 설정
                this.currentPosition = shipParams.InitialPosition;

                // 2. 초기 속도 벡터 설정 (침로와 속력 기반)
                double courseRad = shipParams.CourseDirection * (Math.PI / 180.0);
                float vx = (float)(shipParams.CourseSpeed * Math.Sin(courseRad));
                float vz = (float)(shipParams.CourseSpeed * Math.Cos(courseRad));
                this.currentVelocity = new Vector3(vx, 0, vz); // Z축=North, X축=East

                // 3. 흔들림(Oscillation) 파라미터 저장 (라디안, 주파수로 변환)
                this.pitchAmplitude_rad = shipParams.PitchAmplitude * (Math.PI / 180.0);
                this.pitchFrequency_rad_s = (shipParams.PitchPeriod > 0) ? (2 * Math.PI / shipParams.PitchPeriod) : 0;

                this.rollAmplitude_rad = shipParams.RollAmplitude * (Math.PI / 180.0);
                this.rollFrequency_rad_s = (shipParams.RollPeriod > 0) ? (2 * Math.PI / shipParams.RollPeriod) : 0;

                // 4. 기본 자세(Yaw) 설정
                this.initialYaw_rad = courseRad;
                this.currentAttitude = Quaternion.CreateFromYawPitchRoll((float)this.initialYaw_rad, 0, 0);

                // 5. 시뮬레이션 시간 리셋
                this.simulationTime = 0.0;
            }
            else
            {
                throw new ArgumentException("Invalid parameter type for ShipModel");
            }
        }

        public void Update(double timeDelta)
        {
            // 1. 모듈 내부 시뮬레이션 시간 누적
            this.simulationTime += timeDelta;

            // 2. 선형 위치 업데이트 (속도는 일정하다고 가정)
            this.currentPosition = this.currentPosition + this.currentVelocity * (float)timeDelta;

            // 3. 자세(Attitude) 업데이트 (흔들림 시뮬레이션)

            // Pitch (X축 기준 회전)
            float pitch = (float)(this.pitchAmplitude_rad * Math.Sin(this.simulationTime * this.pitchFrequency_rad_s));

            // Roll (Z축 기준 회전 - 함선 진행 방향 기준)
            // (Pitch와 90도 위상 차이를 주기 위해 Cos 사용)
            float roll = (float)(this.rollAmplitude_rad * Math.Cos(this.simulationTime * this.rollFrequency_rad_s));

            // Yaw (Y축 기준 회전) - 기본 침로(initialYaw_rad)는 유지
            float yaw = (float)this.initialYaw_rad; // (단순 모델에서는 Yaw 흔들림은 0으로 가정)

            // 4. 최종 자세 쿼터니언 생성
            // (Yaw가 먼저 적용되고, 그 다음 Pitch, 그 다음 Roll)
            this.currentAttitude = Quaternion.CreateFromYawPitchRoll(yaw, pitch, roll);
        }

    }
}
