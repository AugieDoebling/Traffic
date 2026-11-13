using Traffic.CommonData;
using Unity.Entities;
using Unity.Mathematics;

namespace Traffic.Components.LaneConnections
{
    public struct Connector : IComponentData
    {
        public Entity edge;
        public Entity node;
        public int laneIndex;
        /// <summary>
        /// Connector on the roundabout ring only (see RoundaboutUtils): lane index of the path node owned by the node (lane | segment &lt;&lt; 8)
        /// </summary>
        public int pathLaneIndex;
        public int2 carriagewayAndGroupIndex;
        public float3 position;
        public float3 lanePosition;
        public float3 direction;
        public VehicleGroup vehicleGroup;
        public ConnectorType connectorType;
    }
}
