using RadarSim.Core.Header;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace DopplerSensorModule
{
    public class HeaderParameters : IHeaderParameters
    {
        public byte[] abSTX { get; set; }
        public DateTime dtTime { get; set; }
    }
}
