using Colossal.Mathematics;
using Game.Net;
using Game.Pathfind;
using Game.Prefabs;
using Traffic.CommonData;
using Traffic.Components;
using Traffic.Components.LaneConnections;
using Unity.Burst.Intrinsics;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using CarLane = Game.Net.CarLane;
using Edge = Game.Net.Edge;
using LaneConnection = Traffic.Components.LaneConnections.LaneConnection;
using SubLane = Game.Net.SubLane;

namespace Traffic.Systems.LaneConnections
{
    public partial class GenerateConnectorsSystem
    {
#if WITH_BURST
        [Unity.Burst.BurstCompile]
#endif
        private struct GenerateConnectionLanesJob : IJobChunk
        {            
            [ReadOnly] public ComponentTypeHandle<EditIntersection> editIntersectionType;
            [ReadOnly] public ComponentLookup<CarLane> carLaneData;
            [ReadOnly] public ComponentLookup<Composition> compositionData;
            [ReadOnly] public ComponentLookup<Curve> curveData;
            [ReadOnly] public ComponentLookup<Lane> laneData;
            [ReadOnly] public ComponentLookup<MasterLane> masterLaneData;
            [ReadOnly] public ComponentLookup<Edge> edgeData;
            [ReadOnly] public ComponentLookup<Node> nodeData;
            [ReadOnly] public ComponentLookup<Roundabout> roundaboutData;
            [ReadOnly] public BufferLookup<ConnectedEdge> connectedEdgesBuffer;
            [ReadOnly] public BufferLookup<NetCompositionLane> prefabCompositionLaneBuffer;
            [ReadOnly] public BufferLookup<SubLane> subLanesBuffer;
            [ReadOnly] public NativeParallelHashMap<NodeEdgeLaneKey, Entity> connectorsList;
            public EntityCommandBuffer commandBuffer;
 
            public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask) {
                NativeArray<EditIntersection> editIntersections = chunk.GetNativeArray(ref editIntersectionType);
                Logger.DebugConnections($"GenerateConnectionLanesJob Connectors: {connectorsList.Count()}");
                NativeParallelMultiHashMap<Entity, Connection> connections = new (8, Allocator.Temp);
                
                for (int i = 0; i < editIntersections.Length; i++)
                {
                    EditIntersection editIntersection = editIntersections[i];
                    Entity node = editIntersection.node;
                    if (nodeData.HasComponent(node) && subLanesBuffer.HasBuffer(node))
                    {
                        DynamicBuffer<ConnectedEdge> connectedEdges = connectedEdgesBuffer[node];
                        DynamicBuffer<SubLane> subLanes = subLanesBuffer[node];
                        
#if EXPERIMENTAL_ROUNDABOUTS
                        bool isRoundabout = roundaboutData.HasComponent(node);
#endif
                        Logger.DebugConnections($"SubLanes: {subLanes.Length}");
                        foreach (SubLane subLane in subLanes)
                        {
                            Entity subLaneEntity = subLane.m_SubLane;
                            if ((subLane.m_PathMethods & (PathMethod.Road | PathMethod.Track | PathMethod.Bicycle)) == 0 || masterLaneData.HasComponent(subLaneEntity))
                            {
                                continue;
                            }
                            Lane lane = laneData[subLaneEntity];
                            Entity sourceEdge = Helpers.NetUtils.FindEdge(connectedEdges, lane.m_StartNode);
                            Entity targetEdge = sourceEdge;
                            if (!lane.m_StartNode.OwnerEquals(lane.m_EndNode))
                            {
                                targetEdge = Helpers.NetUtils.FindEdge(connectedEdges, lane.m_EndNode);
                            }
                            bool isUnsafe = false;
                            bool isForbidden = false;
                            if (carLaneData.HasComponent(subLaneEntity))
                            {
                                CarLane carLane = carLaneData[subLaneEntity];
                                isUnsafe = (carLane.m_Flags & CarLaneFlags.Unsafe) != 0;
                                isForbidden = (carLane.m_Flags & CarLaneFlags.Forbidden) != 0;
                            }
#if EXPERIMENTAL_ROUNDABOUTS
                            if (isRoundabout && (sourceEdge == Entity.Null || targetEdge == Entity.Null))
                            {
                                AddRingConnection(node, subLane, lane, sourceEdge, targetEdge, isUnsafe, isForbidden);
                                continue;
                            }
#endif
                            if (sourceEdge == Entity.Null || targetEdge == Entity.Null)
                            {
                                continue;
                            }

                            Edge sourceEdgeData = edgeData[sourceEdge];
                            Edge targetEdgeData = edgeData[targetEdge];
                            bool2 isEdgeEndMap = new bool2(sourceEdgeData.m_End.Equals(node), targetEdgeData.m_End.Equals(node));
                            if (Helpers.NetUtils.GetAdditionalLaneDetails(sourceEdge, targetEdge, new int2(lane.m_StartNode.GetLaneIndex() & 0xff,lane.m_EndNode.GetLaneIndex() & 0xff), isEdgeEndMap, ref compositionData, ref prefabCompositionLaneBuffer, out float3x2 lanePositionMap, out int4 carriagewayWithGroupMap))
                            {
                                Bezier4x3 bezier = curveData[subLaneEntity].m_Bezier;
                                Logger.DebugConnections($"Adding connection (subLane: {subLaneEntity}): idx[{lane.m_StartNode.GetLaneIndex() & 0xff}->{lane.m_EndNode.GetLaneIndex() & 0xff}] edge:[{sourceEdge}=>{targetEdge}] | unsafe?: {isUnsafe}, methods: {subLane.m_PathMethods}");
                                Connection connection = new Connection(lane, bezier, lanePositionMap, carriagewayWithGroupMap, subLane.m_PathMethods, sourceEdge, targetEdge, isUnsafe, isForbidden);
                                connections.Add(sourceEdge, connection);
                            }
                        }

                        foreach (ConnectedEdge connectedEdge in connectedEdges)
                        {
                            Entity edge = connectedEdge.m_Edge;
                            if (connections.ContainsKey(edge))
                            {
                                foreach (Connection connection in connections.GetValuesForKey(edge))
                                {
                                    // TODO REDESIGN, GET RID OF LANE_CONNECTION buffer
                                    int sourceConnectorIndex = connection.sourceNode.GetLaneIndex() & 0xff;
                                    if (connectorsList.TryGetValue(new NodeEdgeLaneKey(node.Index, edge.Index, sourceConnectorIndex), out Entity sourceConnector))
                                    {
                                        Entity e = commandBuffer.CreateEntity();
                                        DynamicBuffer<Connection> connectionsBuffer = commandBuffer.AddBuffer<Connection>(e);
                                        Logger.DebugConnections($"Adding connection to buffer ({e}): idx[{connection.sourceNode.GetLaneIndex() & 0xff}->{connection.targetNode.GetLaneIndex() & 0xff}] edge[{connection.sourceEdge}=>{connection.targetEdge}]");
                                        connectionsBuffer.Add(connection);
                                        Logger.DebugConnections($"Detected connection n[{node}] e[{edge}] idx[{sourceConnectorIndex}] | connector: {sourceConnector} | e: {e}");
                                        commandBuffer.AppendToBuffer(sourceConnector, new LaneConnection() { connection = e });
                                    }
                                }
                            }
                        }
                        connections.Clear();
                    }
                }
                connections.Dispose();
            }

            /// <summary>
            /// Adds a connection for the roundabout entry lane (edge lane -> ring) or exit lane (ring -> edge lane).
            /// The lane end placed on the ring is owned by the node, see <see cref="Helpers.RoundaboutUtils"/>.
            /// Lanes running along the ring (both ends owned by the node) are skipped
            /// </summary>
            private void AddRingConnection(Entity node, SubLane subLane, Lane lane, Entity sourceEdge, Entity targetEdge, bool isUnsafe, bool isForbidden)
            {
                bool startOnRing = sourceEdge == Entity.Null && Helpers.RoundaboutUtils.IsOwnedBy(lane.m_StartNode, node);
                bool endOnRing = targetEdge == Entity.Null && Helpers.RoundaboutUtils.IsOwnedBy(lane.m_EndNode, node);
                if (startOnRing == endOnRing)
                {
                    return;
                }
                Entity edge = startOnRing ? targetEdge : sourceEdge;
                if (edge == Entity.Null)
                {
                    return;
                }
                int edgeLaneIndex = (startOnRing ? lane.m_EndNode.GetLaneIndex() : lane.m_StartNode.GetLaneIndex()) & 0xff;
                if (!Helpers.NetUtils.GetLaneEndDetails(edge, edgeLaneIndex, edgeData[edge].m_End.Equals(node), ref compositionData, ref prefabCompositionLaneBuffer, out float3 lanePosition, out int2 carriagewayAndGroup))
                {
                    return;
                }

                NodeEdgeLaneKey sourceConnectorKey;
                float3x2 lanePositionMap;
                int4 carriagewayWithGroupMap;
                if (startOnRing)
                {
                    // exit lane, the source connector is placed on the ring
                    sourceConnectorKey = new NodeEdgeLaneKey(node.Index, node.Index, Helpers.RoundaboutUtils.EncodeRingLaneIndex(lane.m_StartNode.GetLaneIndex(), isTarget: false));
                    lanePositionMap = new float3x2(float3.zero, lanePosition);
                    carriagewayWithGroupMap = new int4(-1, -1, carriagewayAndGroup.x, carriagewayAndGroup.y);
                }
                else
                {
                    // entry lane, the source connector is the edge lane end
                    sourceConnectorKey = new NodeEdgeLaneKey(node.Index, edge.Index, edgeLaneIndex);
                    lanePositionMap = new float3x2(lanePosition, float3.zero);
                    carriagewayWithGroupMap = new int4(carriagewayAndGroup.x, carriagewayAndGroup.y, -1, -1);
                }

                if (connectorsList.TryGetValue(sourceConnectorKey, out Entity sourceConnector))
                {
                    Connection connection = new Connection(lane, curveData[subLane.m_SubLane].m_Bezier, lanePositionMap, carriagewayWithGroupMap, subLane.m_PathMethods, startOnRing ? node : sourceEdge, endOnRing ? node : targetEdge, isUnsafe,
                        isForbidden);
                    Entity e = commandBuffer.CreateEntity();
                    DynamicBuffer<Connection> connectionsBuffer = commandBuffer.AddBuffer<Connection>(e);
                    connectionsBuffer.Add(connection);
                    Logger.DebugConnections($"Adding ring connection ({e}) n[{node}] {(startOnRing ? "exit" : "entry")} idx[{lane.m_StartNode.GetLaneIndex()}->{lane.m_EndNode.GetLaneIndex()}] edge[{edge}] | connector: {sourceConnector}");
                    commandBuffer.AppendToBuffer(sourceConnector, new LaneConnection() { connection = e });
                }
            }
        }
    }
}
