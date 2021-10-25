//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.BaxterHrc
{
    public class JointStateServiceRequest : Message
    {
        public const string RosMessageName = "baxter_hrc/JointStateService";

        public Std.String request;

        public JointStateServiceRequest()
        {
            this.request = new Std.String();
        }

        public JointStateServiceRequest(Std.String request)
        {
            this.request = request;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(request.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.request.Deserialize(data, offset);

            return offset;
        }

        public override string ToString()
        {
            return "JointStateServiceRequest: " +
            "\nrequest: " + request.ToString();
        }
    }
}
