using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RadarSim.Core
{
    /// <summary>
    /// 시뮬레이션에 필요한 공용 물리 및 환경 계산 함수를 제공하는
    /// 정적 유틸리티 클래스입니다.
    /// </summary>
    public static class PhysicsUtils
    {
        /// <summary>
        /// 고도(Altitude)에 따른 공기 밀도(rho)를 근사 계산합니다.
        /// (U.S. Standard Atmosphere의 단순화된 지수 모델 사용)
        /// </summary>
        /// <param name="altitudeMeters">해수면 기준 고도 (m)</param>
        /// <returns>공기 밀도 (kg/m^3)</returns>
        public static double GetAirDensity(double altitudeMeters)
        {
            // 해수면에서의 표준 공기 밀도 (kg/m^3)
            const double SeaLevelDensity = 1.225;

            // 대기의 스케일 높이 (Scale Height) (m)
            const double ScaleHeight = 8500.0;

            // 수식: rho(h) = rho_0 * exp(-h / H)
            return SeaLevelDensity * Math.Exp(-altitudeMeters / ScaleHeight);
        }

        // (여기에 나중에 중력 상수, 빛의 속도 등
        //  다른 공용 상수나 함수를 추가할 수 있습니다.)
    }
}
