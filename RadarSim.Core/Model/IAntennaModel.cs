using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Numerics;

namespace RadarSim.Core.Antenna
{
    /// <summary>
    /// 안테나 구동기(Gimbal) 또는 빔 조향(Phased Array)을 정의하는 인터페이스
    /// </summary>
    public interface IAntennaModel
    {
        // --- 초기 설정 ---

        /// <summary>
        /// 안테나의 고유 파라미터로 모듈을 설정합니다.
        /// </summary>
        /// <param name="parameters">안테나 고유값 (예: 마운트 위치, 빔폭)</param>
        void Initialize(IAntennaParameters parameters);


        // --- 매 프레임 업데이트 ---

        /// <summary>
        /// 플랫폼의 흔들림과 표적의 위치를 기반으로 안테나의 지향각을 업데이트합니다.
        /// </summary>
        /// <param name="timeDelta">프레임 간 경과 시간</param>
        /// <param name="platformAttitude">
        /// 현재 플랫폼의 자세 (IPlatformModel의 출력값)
        /// </param>
        /// <param name="targetPosition_Global">
        /// 현재 표적의 3D 위치 (ITrajectoryModel의 출력값)
        /// </param>
        /// <param name="platformPosition_Global">
        /// 현재 플랫폼의 3D 위치 (IPlatformModel의 출력값)
        /// </param>
        void Update(double timeDelta,
                      Quaternion platformAttitude,
                      Vector3 targetPosition_Global,
                      Vector3 platformPosition_Global);


        // --- 핵심 출력 (읽기 전용) ---

        /// <summary>
        /// 안테나가 지향하는 방향 벡터 (Boresight)
        /// (주의: '플랫폼 좌표계' 기준. 안정화 로직이 포함된 결과)
        /// </summary>
        Vector3 BoresightVector_PlatformCoords { get; }

        /// <summary>
        /// 안테나 빔의 3D 각속도 (글로벌 좌표계 기준) [rad/s]
        /// (도플러 계산 시 안테나 회전 성분으로 사용됨)
        /// </summary>
        Vector3 AngularVelocity_Global { get; }

        

        // --- 레이더 상호작용 ---

        /// <summary>
        /// 특정 '오프보어사이트 각'에 대한 안테나 이득(Gain)을 반환합니다.
        /// (내부적으로 빔 패턴 함수를 조회합니다.)
        /// </summary>
        /// <param name="offBoresightAngle">
        /// 빔의 중심축(Boresight)과 실제 표적 방향 간의 각도 [rad]
        /// </param>
        /// <returns>선형 이득 값 (예: 1.0 = 0dB, 0.5 = -3dB)</returns>
        double GetGain(double offBoresightAngle);
        IAntennaParameters GetParameters();

        SnapShot GetSnapshot();
    }

    public class SnapShot
    {
        public SnapShot(Vector3 _BoresightVector_PlatformCoords, Vector3 _AngularVelocity_Global, bool _isActive)
        {
            m_vecBoresightVector_PlatformCoords = _BoresightVector_PlatformCoords;
            m_vecAngularVelocity_Global = _AngularVelocity_Global;
            m_bIsActive = _isActive;
        }
        private Vector3 m_vecBoresightVector_PlatformCoords;
        private Vector3 m_vecAngularVelocity_Global;
        private bool m_bIsActive;

        /// <summary>
        /// 안테나가 지향하는 방향 벡터 (Boresight)
        /// (주의: '플랫폼 좌표계' 기준. 안정화 로직이 포함된 결과)
        /// </summary>
        public Vector3 BoresightVector_PlatformCoords { get => m_vecBoresightVector_PlatformCoords; }

        /// <summary>
        /// 안테나 빔의 3D 각속도 (글로벌 좌표계 기준) [rad/s]
        /// (도플러 계산 시 안테나 회전 성분으로 사용됨)
        /// </summary>
        public Vector3 AngularVelocity_Global { get => m_vecAngularVelocity_Global; }

        /// <summary>
        /// 이 표적이 현재 유효한지 여부를 반환합니다.
        /// (Y <= 0이 되면 false가 됨)
        /// </summary>
        public bool IsActive { get => m_bIsActive; }
    }

    /// <summary>
    /// 안테나 파라미터를 위한 기본 인터페이스
    /// </summary>
    public interface IAntennaParameters
    {
        /// <summary>
        /// 플랫폼 기준점 대비 안테나의 설치 위치 [m]
        /// </summary>
        Vector3 MountOffset { get; set; }

        /// <summary>
        /// 안테나의 3dB 빔폭 [rad]
        /// </summary>
        double Beamwidth { get; set; }

        Vector3[] ElementPositions { get; set; }

        int[] CwChannelIndices { get; set; }
        int[] FmcwChannelIndices { get; set; }


        /// <summary>
        /// 짐벌의 최대 구동 속도 (Slew Rate) [Degrees / sec]
        /// 1초에 몇 도까지 회전할 수 있는지
        /// </summary>
        double MaxSlewRateDegreesPerSec { get; set; }

        /// <summary>
        /// 안정화된 상태에서의 미세한 추적 오차 (Jitter) [Degrees, 1-sigma]
        /// 0이면 완벽하게 추적
        /// </summary>
        double TrackingNoiseStdDevDegrees { get; set; }
    }
}
