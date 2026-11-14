using Colossal.Entities;
using Game;
using Game.Common;
using Game.Net;
using Game.Prefabs;
using Game.Tools;
using Traffic.Components;
using Traffic.Components.LaneConnections;
using Traffic.Systems.Helpers;
using Unity.Collections;
using Unity.Entities;
using Edge = Game.Net.Edge;

namespace Traffic.Systems.LaneConnections
{
    /// <summary>
    /// Removes lane connections which do not match the type of the intersection anymore.
    /// Connections at the roundabout are stored in a different way than at the regular intersection (see <see cref="RoundaboutUtils"/>)
    /// and cannot be converted when the roundabout is added or removed.
    /// Tools creating the Temp node are handled by <see cref="SyncCustomLaneConnectionsSystem"/>, this system covers
    /// changes applied directly to the node (e.g. bulldozing the roundabout object).
    /// Has to run before the lane system, the node is going to be regenerated in the same frame.
    /// </summary>
    public partial class RoundaboutConnectionsCleanupSystem : GameSystemBase
    {
        private EntityQuery _query;

        protected override void OnCreate()
        {
            base.OnCreate();
            _query = SystemAPI.QueryBuilder()
                .WithAll<ModifiedLaneConnections, Node, Updated>()
                .WithNone<Temp, Deleted>()
                .Build();
            RequireForUpdate(_query);
        }

        protected override void OnUpdate()
        {
            NativeArray<Entity> nodes = _query.ToEntityArray(Allocator.Temp);
            NativeList<Entity> invalidNodes = new NativeList<Entity>(Allocator.Temp);
            for (int i = 0; i < nodes.Length; i++)
            {
                if (TryGetRoundaboutState(nodes[i], out bool isRoundabout) && !IsMatchingConnectionData(nodes[i], isRoundabout))
                {
                    invalidNodes.Add(nodes[i]);
                }
            }

            for (int i = 0; i < invalidNodes.Length; i++)
            {
                Entity node = invalidNodes[i];
                NativeArray<ModifiedLaneConnections> connections = EntityManager.GetBuffer<ModifiedLaneConnections>(node, true).ToNativeArray(Allocator.Temp);
                Logger.Info($"Roundabout state of the node {node} does not match stored lane connections, removing {connections.Length} modified lane connections");
                for (int j = 0; j < connections.Length; j++)
                {
                    Entity modifiedConnections = connections[j].modifiedConnections;
                    if (modifiedConnections != Entity.Null && EntityManager.Exists(modifiedConnections))
                    {
                        EntityManager.AddComponent<Deleted>(modifiedConnections);
                    }
                }
                connections.Dispose();
                EntityManager.RemoveComponent<ModifiedLaneConnections>(node);
                EntityManager.RemoveComponent<ModifiedConnections>(node);
            }
            invalidNodes.Dispose();
            nodes.Dispose();
        }

        private bool TryGetRoundaboutState(Entity node, out bool isRoundabout)
        {
            isRoundabout = false;
            if (!EntityManager.TryGetBuffer(node, true, out DynamicBuffer<ConnectedEdge> connectedEdges))
            {
                return false;
            }
            for (int i = 0; i < connectedEdges.Length; i++)
            {
                Entity edgeEntity = connectedEdges[i].m_Edge;
                if (EntityManager.HasComponent<Deleted>(edgeEntity) ||
                    !EntityManager.TryGetComponent(edgeEntity, out Edge edge) ||
                    !EntityManager.TryGetComponent(edgeEntity, out Composition composition))
                {
                    continue;
                }
                bool isStart = edge.m_Start == node;
                if (!isStart && edge.m_End != node)
                {
                    continue;
                }
                if (EntityManager.TryGetComponent(isStart ? composition.m_StartNode : composition.m_EndNode, out NetCompositionData nodeComposition))
                {
                    isRoundabout = (nodeComposition.m_Flags.m_General & CompositionFlags.General.Roundabout) != 0;
                    return true;
                }
            }
            return false;
        }

        /// <summary>
        /// Tests if stored connections can be used at the intersection: only connections with the lane end
        /// on the ring at the roundabout, none of them at the regular intersection
        /// </summary>
        private bool IsMatchingConnectionData(Entity node, bool isRoundabout)
        {
            DynamicBuffer<ModifiedLaneConnections> connections = EntityManager.GetBuffer<ModifiedLaneConnections>(node, true);
            for (int i = 0; i < connections.Length; i++)
            {
                ModifiedLaneConnections connection = connections[i];
                if (RoundaboutUtils.IsRing(connection.laneIndex))
                {
                    if (!isRoundabout)
                    {
                        return false;
                    }
                    continue;
                }
                if (connection.modifiedConnections != Entity.Null &&
                    EntityManager.TryGetBuffer(connection.modifiedConnections, true, out DynamicBuffer<GeneratedConnection> generatedConnections))
                {
                    for (int j = 0; j < generatedConnections.Length; j++)
                    {
                        bool isRingConnection = RoundaboutUtils.IsRing(generatedConnections[j].laneIndexMap.x | generatedConnections[j].laneIndexMap.y);
                        if (isRingConnection != isRoundabout)
                        {
                            return false;
                        }
                    }
                }
            }
            return true;
        }
    }
}
