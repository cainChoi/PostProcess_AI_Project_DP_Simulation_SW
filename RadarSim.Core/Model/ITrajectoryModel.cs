using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Numerics;

namespace RadarSim.Core.Trajectory
{
    /// <summary>
    /// 표적의 궤적과 물리적 특성을 정의하는 인터페이스
    /// </summary>
    public interface ITrajectoryModel
    {
        // --- 초기 설정 ---

        /// <summary>
        /// 시뮬레이션 시작 전, 표적의 초기 파라미터로 모듈을 설정합니다.
        /// (이 파라미터 객체는 WPF UI에서 채워집니다.)
        /// </summary>
        /// <param name="parameters">표적별 초기값 (예: BallisticParameters)</param>
        void Initialize(ITargetParameters parameters);


        // --- 매 프레임 업데이트 ---

        /// <summary>
        /// 경과 시간(초)만큼 표적의 물리 상태를 업데이트합니다.
        /// </summary>
        /// <param name="timeDelta">프레임 간 경과 시간 (예: 0.01초)</param>
        void Update(double timeDelta);


        // --- 핵심 출력 (읽기 전용) ---

        /// <summary>
        /// 현재 시간의 표적 3D 위치 (글로벌 좌표계) [m]
        /// </summary>
        Vector3 Position { get; }

        /// <summary>
        /// 현재 시간의 표적 3D 속도 (글로벌 좌표계) [m/s]
        /// </summary>
        Vector3 Velocity { get; }

        /// <summary>
        /// 현재 시간의 표적 3D 자세 (방향) (글로벌 좌표계)
        /// (예: 포탄의 경우 속도 벡터와 일치)
        /// </summary>
        Quaternion Orientation { get; }


        // --- 레이더 상호작용 ---

        /// <summary>
        /// 특정 '입사각'에 대한 RCS 값을 반환합니다.
        /// (내부적으로 LUT를 조회합니다.)
        /// </summary>
        /// <param name="aspectAngleVector">
        /// 레이더가 표적을 바라보는 방향 벡터 (표적의 로컬 좌표계 기준)
        /// </param>
        /// <returns>RCS 값 [m^2]</returns>
        double GetRCS(Vector3 aspectAngleVector);

        /// <summary>
        /// 이 표적이 현재 유효한지(예: 파괴되지 않았는지) 여부를 반환합니다.
        /// (Y <= 0이 되면 false가 됨)
        /// </summary>
        bool IsActive { get; }

        SnapShot GetSnapshot();

        /// <summary>
        /// 이 표적의 고유한 마이크로 도플러 물리 현상을
        /// '샘플별 위상 오차' 배열로 계산하여 반환합니다.
        /// </summary>
        /// <param name="numSamples">생성할 샘플 개수</param>
        /// <param name="timeStep">샘플당 시간 간격 (1 / AdcSampleRate)</param>
        /// <param name="carrierFrequency">현재 채널의 중심 주파수</param>
        /// <param name="rand">난수 생성기</param>
        /// <returns>numSamples 크기의 위상 오차 배열 (Radians)</returns>
        double[] GetMicroDopplerPhaseNoise(int numSamples,
                                           double timeStep,
                                           double carrierFrequency,
                                           Random rand);




    }


    public class SnapShot
    {
        public SnapShot(Vector3 _pos, Vector3 _vel, Quaternion _ori, bool _isActive)
        {
            m_vecPos = _pos;
            m_vecVel = _vel;
            m_quaOri = _ori;
            m_bIsActive = _isActive;
        }
        private Vector3 m_vecPos;
        private Vector3 m_vecVel;
        private Quaternion m_quaOri;
        private bool m_bIsActive;

        /// <summary>
        /// 현재 시간의 표적 3D 위치 (글로벌 좌표계) [m]
        /// </summary>
        public Vector3 Position { get => m_vecPos; }

        /// <summary>
        /// 현재 시간의 표적 3D 속도 (글로벌 좌표계) [m/s]
        /// </summary>
        public Vector3 Velocity { get => m_vecVel; }

        /// <summary>
        /// 현재 시간의 표적 3D 자세 (방향) (글로벌 좌표계)
        /// (예: 포탄의 경우 속도 벡터와 일치)
        /// </summary>
        public Quaternion Orientation { get => m_quaOri; }

        /// <summary>
        /// 이 표적이 현재 유효한지(예: 파괴되지 않았는지) 여부를 반환합니다.
        /// (Y <= 0이 되면 false가 됨)
        /// </summary>
        public bool IsActive { get => m_bIsActive; }
    }


    /// <summary>
    /// 표적 파라미터를 위한 기본 인터페이스 (확장용)
    /// </summary>
    public interface ITargetParameters
    {
        // 예: 공통 파라미터
        Vector3 InitialPosition { get; set; }
    }

   

}
