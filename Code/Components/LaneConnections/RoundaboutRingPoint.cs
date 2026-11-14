using Unity.Entities;
using Unity.Mathematics;

namespace Traffic.Components.LaneConnections
{
    /// <summary>
    /// Lane end placed on the roundabout ring, written by the lane system to the roundabout node every time its lanes are generated.
    /// Used by the Lane Connector tool to build connectors: the tool cannot recover identities of ring lane ends from generated lanes.
    /// Every lane end of the ring is listed twice: as a source of the following ring segment and as a target of the preceding one.
    /// Not serialized.
    /// </summary>
    [InternalBufferCapacity(0)]
    public struct RoundaboutRingPoint : IBufferElementData
    {
        /// <summary>anchor edge, part of the lane end identity (see RoundaboutUtils)</summary>
        public Entity edge;
        /// <summary>flagged lane index, part of the lane end identity (see RoundaboutUtils)</summary>
        public int laneIndex;
        /// <summary>lane index of the path node owned by the roundabout node (lane | segment &lt;&lt; 8)</summary>
        public int pathLaneIndex;
        /// <summary>identifier of the ring segment (non-zero), ring lanes connect source with target of the same segment</summary>
        public int segment;
        public float3 position;
        /// <summary>travel direction</summary>
        public float3 direction;
        public Entity lanePrefab;
        public bool isSource;
    }
}
