using RadarSim.Core.Header;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace DopplerSensorModule
{
    public class DopplerSensorModule : IHeaderModel
    {
        private HeaderParameters parameters;
        public void Initialize(IHeaderParameters parameters)
        {
            if (parameters is HeaderParameters headerParams)
            {
                this.parameters = headerParams;
            }
            else
            {
                throw new ArgumentException("Invalid parameter type for SeaClutterModel");
            }
        }

        /// <summary>
        /// Generates the file header.
        /// </summary>
        /// <param name="dTime">The d time.</param>
        /// <param name="nSequence">The n sequence.</param>
        /// <param name="anArg">CH, MODE, MFCWProfile, IQ Indentifier, COLLECTION MODE</param>
        /// <returns></returns>
        public byte[] GenerateFileHeader(double dTime, int nSequence, int[] anArg)
        {
            byte[] abyHeader = new byte[128];
            int nHeaderIndex = 0;
            int nIndexArg = 0;
            //STX
            Array.Copy(parameters.abSTX, 0, abyHeader, nHeaderIndex, parameters.abSTX.Length);
            nHeaderIndex += parameters.abSTX.Length;

            //CH
            ushort usConv = (ushort)anArg[nIndexArg++];
            Array.Copy(BitConverter.GetBytes(usConv), 0, abyHeader, nHeaderIndex, 2);
            nHeaderIndex += 2;

            //MODE
            usConv = (ushort)anArg[nIndexArg++];
            Array.Copy(BitConverter.GetBytes(usConv), 0, abyHeader, nHeaderIndex, 2);
            nHeaderIndex += 2;

            //MFCW Profile
            usConv = (ushort)anArg[nIndexArg++];
            Array.Copy(BitConverter.GetBytes(usConv), 0, abyHeader, nHeaderIndex, 2);
            nHeaderIndex += 2;

            //I or Q Identifier
            usConv = (ushort)anArg[nIndexArg++];
            Array.Copy(BitConverter.GetBytes(usConv), 0, abyHeader, nHeaderIndex, 2);
            nHeaderIndex += 2;

            //DRCTRL COUNTER MSB, DRCTRL COUNTER LSB
            byte[] abySeq = BitConverter.GetBytes(nSequence);
            Array.Copy(abySeq, 2, abyHeader, nHeaderIndex, 2);
            nHeaderIndex += 2;
            Array.Copy(abySeq, 0, abyHeader, nHeaderIndex, 2);
            nHeaderIndex += 2;

            DateTime dtCur = parameters.dtTime.AddSeconds(dTime * nSequence);
            //TIME HOUR
            usConv = (ushort)dtCur.TimeOfDay.Hours;
            Array.Copy(BitConverter.GetBytes(usConv), 0, abyHeader, nHeaderIndex, 2);
            nHeaderIndex += 2;

            //TIME MINUTE
            usConv = (ushort)dtCur.TimeOfDay.Minutes;
            Array.Copy(BitConverter.GetBytes(usConv), 0, abyHeader, nHeaderIndex, 2);
            nHeaderIndex += 2;

            //TIME SECOND
            usConv = (ushort)dtCur.TimeOfDay.Seconds;
            Array.Copy(BitConverter.GetBytes(usConv), 0, abyHeader, nHeaderIndex, 2);
            nHeaderIndex += 2;

            //TIME MILLI-SEC
            usConv = (ushort)dtCur.TimeOfDay.Milliseconds;
            Array.Copy(BitConverter.GetBytes(usConv), 0, abyHeader, nHeaderIndex, 2);
            nHeaderIndex += 2;

            //EVEN / ODD FLAG
            usConv = (ushort)(nSequence % 2);
            Array.Copy(BitConverter.GetBytes(usConv), 0, abyHeader, nHeaderIndex, 2);
            nHeaderIndex += 2;

            //RESERVED(4)
            nHeaderIndex += 4;

            //COLLECTION MODE
            usConv = (ushort)anArg[nIndexArg++];
            Array.Copy(BitConverter.GetBytes(usConv), 0, abyHeader, nHeaderIndex, 2);
            //nHeaderIndex += 2;

            snap = new SnapShot(dTime, nSequence, dtCur, anArg, abyHeader);
            return abyHeader;
        }
        private SnapShot snap;
        public SnapShot GetSnapshot()
        {
            return snap;
        }
        
    }
}
