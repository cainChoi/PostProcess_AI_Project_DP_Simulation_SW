using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Numerics;

namespace RadarSim.Core.Clutter
{
    public interface IClutterParameters { }

    // 2. 클러터 컨텍스트 (계산에 필요한 정보)
    public struct ClutterContext
    {
        public Platform.IPlatformModel Platform;
        public Antenna.IAntennaModel Antenna;
        public double CenterFrequency_Hz;
        public double Bandwidth_Hz;
        public double ChirpDuration_sec;
        public double AdcSampleRate_Hz;
        public int ChannelIndex;

        public double TxPower_W { get; set; }
        public double TxGain_Linear { get; set; }
        public double RxGain_Linear { get; set; }

        /// <summary>
        /// 안테나 빔의 중심(Boresight)이
        /// 현재 가리키고 있는 '글로벌' 방향 벡터
        /// </summary>
        public Vector3 AntennaBoresight_Global;
    }

    // 3. 클러터 모듈 인터페이스
    public interface IClutterModel
    {
        void Initialize(IClutterParameters parameters);

        /// <summary>
        /// 한 첩(chirp) 길이의 클러터 I/Q 신호를 생성합니다.
        /// </summary>
        /// <param name="context">계산에 필요한 현재 레이더/플랫폼 상태</param>
        /// <param name="rand">일관된 난수 생성을 위한 Random 객체</param>
        /// <returns>복소수 I/Q 샘플 배열</returns>
        Complex[] GenerateClutterSignal(ClutterContext context, Random rand);
    }
}
