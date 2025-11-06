using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using RadarSim.Core.Clutter;
using System.Numerics;

namespace RainClutterModule
{
    /// <summary>
    /// '강우 클러터' 모듈을 초기화하는 데 필요한 파라미터
    /// </summary>
    public class ClutterParameters : IClutterParameters
    {
        /// <summary>
        /// 강우 강도 (mm/hr)
        /// </summary>
        public double RainRate_mm_hr { get; set; } = 5.0; // 5mm/hr (보통 비)

        /// <summary>
        /// 비구름의 하단 고도 (m)
        /// </summary>
        public double CloudBase_m { get; set; } = 500.0;

        /// <summary>
        /// 비구름의 상단 고도 (m)
        /// </summary>
        public double CloudTop_m { get; set; } = 3000.0;

        /// <summary>
        /// 바람의 3D 속도 벡터 (글로벌 좌표계) (m/s)
        /// (예: 서쪽에서 15m/s로 부는 바람)
        /// </summary>
        public Vector3 WindVector_Global { get; set; } = new Vector3(15, 0, 0);

        /// <summary>
        /// 빗방울의 평균 낙하 속도 (m/s) (음수 = 아래로)
        /// </summary>
        public double FallSpeed_mps { get; set; } = -6.0;
    }
}
