using Game.Pathfind;
using Traffic.Components.LaneConnections;
using Unity.Entities;

namespace Traffic.Systems.Helpers
{
    /// <summary>
    /// Helpers for lane connectors placed on the roundabout ring.
    /// A ring connector is a lane end owned by the node itself (not by any connected edge):
    /// the end of an entry lane (ring target) or the start of an exit lane (ring source).
    /// It is stored as a <see cref="Connector"/> with <c>edge == node</c> and
    /// <c>laneIndex</c> = full path-node lane index (lane | segment &lt;&lt; 8).
    /// </summary>
    public static class RoundaboutUtils
    {
        /// <summary>
        /// Marks ring target connectors, a single ring point can be both the end of an entry lane
        /// and the start of an exit lane so the two connectors need different lane index
        /// </summary>
        public const int RingTargetFlag = 1 << 16;

        public static bool IsRingConnector(in Connector connector)
        {
            return connector.edge == connector.node;
        }

        public static int EncodeRingLaneIndex(ushort pathLaneIndex, bool isTarget)
        {
            return isTarget ? pathLaneIndex | RingTargetFlag : pathLaneIndex;
        }

        public static bool IsOwnedBy(PathNode pathNode, Entity owner)
        {
            return pathNode.OwnerEquals(new PathNode(owner, 0));
        }
    }
}
