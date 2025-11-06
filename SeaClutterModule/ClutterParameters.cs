using RadarSim.Core.Clutter;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SeaClutterModule
{
    public class SeaParameters : IClutterParameters
    {
        /// <summary>
        /// 해상 상태 (0 = 잔잔, 9 = 매우 거침)
        /// </summary>
        public int SeaState { get; set; } = 3;
    }
}
