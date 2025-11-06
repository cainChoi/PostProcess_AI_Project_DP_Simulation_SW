using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RadarSim.Core.Header
{
    public interface IHeaderModel
    {
        void Initialize(IHeaderParameters parameters);
        byte[] GenerateFileHeader(double dTime, int nSequence, int[] anArg);

        SnapShot GetSnapshot();

    }

    public class SnapShot
    {
        public SnapShot(double dTime, int nSeqeunce, DateTime dtCur, int[] anArg, byte[] abyBinaryData)
        {
            this.dTime = dTime;
            this.nSeqeunce = nSeqeunce;
            this.dtCurTime = dtCur;
            this.anArg = anArg;
            this.abyBinaryData = abyBinaryData;
        }

        public double dTime { get; set; }
        public int nSeqeunce { get; set; }
        public DateTime dtCurTime { get; set; }
        public int[] anArg { get; set; }
        public byte[] abyBinaryData { get; set; }

    }


    public interface IHeaderParameters
    {
        byte[] abSTX { get; set; }
        DateTime dtTime { get; set; }

    }
}
