using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using RadarSim.Core.Platform;
using System.Numerics;

namespace ShipModule
{
    public class PlatformParameters : IPlatformParameters
    {
        // 1. IPlatformParameters 인터페이스 구현
        public Vector3 InitialPosition { get; set; } = System.Numerics.Vector3.Zero;

        // 2. 함선(Ship) 모듈 전용 파라미터

        /// <summary>
        /// 함선의 항해 속도 (m/s)
        /// </summary>
        public double CourseSpeed { get; set; } = 10;

        /// <summary>
        /// 함선의 침로 (방향) (Degrees, 0 = +Z축 North)
        /// </summary>
        public double CourseDirection { get; set; } = 0;

        // --- 해상 상태(Sea State)에 따른 흔들림 파라미터 ---
        // (간단한 Sine파 진동으로 시뮬레이션합니다)

        /// <summary>
        /// Pitch(상하 흔들림)의 최대 진폭 (Degrees)
        /// </summary>
        public double PitchAmplitude { get; set; } = 3;

        /// <summary>
        /// Pitch(상하 흔들림)의 주기 (Seconds)
        /// </summary>
        public double PitchPeriod { get; set; } = 2;

        /// <summary>
        /// Roll(좌우 흔들림)의 최대 진폭 (Degrees)
        /// </summary>
        public double RollAmplitude { get; set; } = 3;

        /// <summary>
        /// Roll(좌우 흔들림)의 주기 (Seconds)
        /// </summary>
        public double RollPeriod { get; set; } = 3;

        /// <summary>
        /// 예: 0=잔잔, 5=거친파도
        /// </summary>
        public int SeaState { get; set; } = 2;
    }
}
