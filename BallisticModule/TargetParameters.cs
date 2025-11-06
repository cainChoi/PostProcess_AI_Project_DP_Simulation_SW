using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using RadarSim.Core.Trajectory;
using System.Numerics;
namespace BallisticModule
{
    public class TargetParameters : ITargetParameters
    {
        // 1. ITargetParameters 인터페이스 구현

        public Vector3 InitialPosition { get; set; } = new Vector3(5000, 0, 1000) + new Vector3(0, 20, 0);//앞에는 발사함의 위치 뒤에 것은 함포의 마운트 오프셋(높이)

        // 2. 포탄(Ballistic) 모듈 전용 파라미터
        // (WPF UI의 텍스트박스 등에서 이 값들을 채워줍니다)

        /// <summary>
        /// 초기 발사 속도 [m/s]
        /// </summary>
        public double InitialSpeed { get; set; } = 900.0;//일반적인 포탄의 마하 2.6~2.7 수준

        /// <summary>
        /// 초기 발사 방위각 (Azimuth) [degrees]
        /// </summary>
        public double LaunchAzimuth { get; set; } = 0.0;

        /// <summary>
        /// 초기 발사 고각 (Elevation) [degrees]
        /// </summary>
        public double LaunchElevation { get; set; } = 45.0;//진공에서 최대 사거리를 갖는 고전적인 테스트 각도

        /// <summary>
        /// 항력 계수 (Drag Coefficient) (단위 없음)
        /// </summary>
        public double DragCoefficient { get; set; } = 0.25;//초음속 포탄의 일반적인 근사치

        /// <summary>
        /// 포탄의 질량 [kg]
        /// </summary>
        public double Mass { get; set; } = 45.0;//질량: 45 kg (155mm 포탄의 평균 질량 과 유사)

        /// <summary>
        /// 포탄의 단면적 (항력 계산용) [m^2]
        /// </summary>
        public double CrossSectionalArea { get; set; } = 0.01887; //단면적: 0.01887 m^2 (155mm 직경(0.155m) 기준), (계산: Area = pi * (0.155 / 2)^2 = 0.01887 m^2)

        /// <summary>
        /// 포탄의 축 회전 속도 (초당 회전수, Hz)
        /// (예: 50 Hz = 3000 RPM)
        /// </summary>
        public double SpinRate_Hz { get; set; } = 50.0;
    }


}
