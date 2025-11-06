using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Numerics;

namespace RadarSim.Core.Platform
{
    /// <summary>
    /// 레이더 플랫폼(함선, 차량 등)의 기동을 정의하는 인터페이스
    /// </summary>
    public interface IPlatformModel
    {
        // --- 초기 설정 ---

        /// <summary>
        /// 플랫폼의 초기 파라미터로 모듈을 설정합니다.
        /// </summary>
        /// <param name="parameters">플랫폼 초기값 (예: 항로, 해상 상태)</param>
        void Initialize(IPlatformParameters parameters);


        // --- 매 프레임 업데이트 ---

        /// <summary>
        /// 경과 시간(초)만큼 플랫폼의 상태를 업데이트합니다.
        /// </summary>
        /// <param name="timeDelta">프레임 간 경과 시간 (예: 0.01초)</param>
        void Update(double timeDelta);


        // --- 핵심 출력 (읽기 전용) ---

        /// <summary>
        /// 현재 시간의 플랫폼 기준점 3D 위치 (글로벌 좌표계) [m]
        /// (예: 함선의 무게 중심)
        /// </summary>
        Vector3 Position { get; }

        /// <summary>
        /// 현재 시간의 플랫폼 3D 속도 (글로벌 좌표계) [m/s]
        /// </summary>
        Vector3 Velocity { get; }

        /// <summary>
        /// 현재 시간의 플랫폼 3D 자세 (Roll, Pitch, Yaw)
        /// (이 값이 안테나 안정화의 기준이 됩니다.)
        /// </summary>
        Quaternion Attitude { get; }


        SnapShot GetSnapshot();
    }

    public class SnapShot
    {
        public SnapShot(Vector3 _pos, Vector3 _vel, Quaternion _atti, bool _isActive)
        {
            m_vecPos = _pos;
            m_vecVel = _vel;
            m_quaAttitude = _atti;
            m_bIsActive = _isActive;
        }
        private Vector3 m_vecPos;
        private Vector3 m_vecVel;
        private Quaternion m_quaAttitude;
        private bool m_bIsActive;

        /// <summary>
        /// 현재 시간의 플랫폼 기준점 3D 위치 (글로벌 좌표계) [m]
        /// (예: 함선의 무게 중심)
        /// </summary>
        Vector3 Position { get; }

        /// <summary>
        /// 현재 시간의 플랫폼 3D 속도 (글로벌 좌표계) [m/s]
        /// </summary>
        Vector3 Velocity { get; }

        /// <summary>
        /// 현재 시간의 플랫폼 3D 자세 (Roll, Pitch, Yaw)
        /// (이 값이 안테나 안정화의 기준이 됩니다.)
        /// </summary>
        Quaternion Attitude { get; }

        /// <summary>
        /// 이 표적이 현재 유효한지 여부를 반환합니다.
        /// (Y <= 0이 되면 false가 됨)
        /// </summary>
        public bool IsActive { get => m_bIsActive; }
    }

    /// <summary>
    /// 플랫폼 파라미터를 위한 기본 인터페이스
    /// </summary>
    public interface IPlatformParameters
    {
        Vector3 InitialPosition { get; set; }
        double CourseSpeed { get; set; }
        double CourseDirection { get; set; }
        int SeaState { get; set; } // 예: 0=잔잔, 5=거친파도
    }
}
