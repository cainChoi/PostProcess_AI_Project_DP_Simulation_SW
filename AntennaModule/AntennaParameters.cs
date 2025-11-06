using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using RadarSim.Core;
using System.Numerics;

namespace AntennaModule
{
    /// <summary>
    /// '안테나' 모듈을 초기화하는 데 필요한
    /// 구체적인 파라미터 세트입니다.
    /// IAntennaParameters 인터페이스를 구현합니다.
    /// </summary>
    public class AntennaParameters : RadarSim.Core.Antenna.IAntennaParameters
    {
        // 1. IAntennaParameters 인터페이스 구현

        /// <summary>
        /// 플랫폼 기준점(예: 무게중심) 대비 
        /// 안테나 짐벌의 설치 위치 오프셋 [m]
        /// </summary>
        public Vector3 MountOffset { get; set; } = new Vector3(0, 10, 0);//Centerline 기준 (좌(-)우(+),상(+)하(-),함수방향(+)함미방향(-) 

        /// <summary>
        /// 안테나의 3dB 빔폭 (메인 로브의 폭) [Degrees]
        /// </summary>
        public double Beamwidth { get; set; } = 10;

        /// <summary>
        /// 8개 안테나 소자의 3D 위치 오프셋 배열 (m)
        /// (안테나 마운트 지점 기준, 플랫폼 좌표계)
        /// </summary>
        public Vector3[] ElementPositions { get; set; } = new Vector3[]
        {
            // --- Top 4: CW (Indices 0-3) ---
            // (X, Y, Z)
            new Vector3(0.925f, 0.225f, 0.0f), // Ch 0: CW, Top-Left
            new Vector3(1.075f, 0.225f, 0.0f), // Ch 1: CW, Top-Right
            new Vector3(0.925f, 0.075f, 0.0f), // Ch 2: CW, Bottom-Left
            new Vector3(1.075f, 0.075f, 0.0f), // Ch 3: CW, Bottom-Right

            // --- Bottom 4: FMCW (Indices 4-7) ---
            new Vector3(0.925f, -0.075f, 0.0f), // Ch 4: FMCW, Top-Left
            new Vector3(1.075f, -0.075f, 0.0f), // Ch 5: FMCW, Top-Right
            new Vector3(0.925f, -0.225f, 0.0f), // Ch 6: FMCW, Bottom-Left
            new Vector3(1.075f, -0.225f, 0.0f)  // Ch 7: FMCW, Bottom-Right
        };

    //초깃값은 다음과 같이 설계하였다.
    //안테나 중간지점: 포지셔너 중심축(원점)에서 오른쪽으로 1미터
    //Array Center = (X: 1.0, Y: 0.0, Z: 0.0)
    //(모든 소자가 동일한 Z 평면에 있다고 가정, Z=0)
    //X축 위치 (가로 2줄)
    //두 줄의 간격이 15cm (0.15m)이므로, 중간지점(X=1.0)을 기준으로 ±(0.15 / 2) 만큼 떨어져 있음.
    //X_Left = 1.0 - 0.075 = 0.925 m
    //X_Right = 1.0 + 0.075 = 1.075 m
    //Y축 위치 (세로 4줄)
    //4줄의 간격이 15cm (0.15m)입니다. 중간지점(Y=0.0)은 상단 FMCW 안테나와 하단 CW 안테나 사이가 됩니다.
    //Y_CW_Top = 0.075 + 0.15 = +0.225 m (가장 위)
    //Y_CW_Bottom = 0.075 = +0.075 m (두 번째)
    //Y_FMCW_Top = -0.075 m (세 번째)
    //Y_FMCW_Bottom = -0.075 - 0.15 = -0.225 m (가장 아래)
    //

    /// <summary>
    /// CW 채널의 인덱스 목록 (예: 0, 1, 2, 3)
    /// </summary>
    public int[] CwChannelIndices { get; set; } = { 0, 1, 2, 3 };

        /// <summary>
        /// FMCW 채널의 인덱스 목록 (예: 4, 5, 6, 7)
        /// </summary>
        public int[] FmcwChannelIndices { get; set; } = { 4, 5, 6, 7 };

        /// <summary>
        /// 짐벌의 최대 구동 속도 (Slew Rate) [Degrees / sec]
        /// 1초에 몇 도까지 회전할 수 있는지
        /// </summary>
        public double MaxSlewRateDegreesPerSec { get; set; } = 60.0; // 예: 90도/초

        /// <summary>
        /// 안정화된 상태에서의 미세한 추적 오차 (Jitter) [Degrees, 1-sigma]
        /// 0이면 완벽하게 추적
        /// </summary>
        public double TrackingNoiseStdDevDegrees { get; set; } = 1.1; // 예: 0.1도 오차
    }
}
