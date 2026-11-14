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
            [ReadOnly] public ComponentLookup<Connector> connectorData;
            [ReadOnly] public BufferLookup<RoundaboutRingPoint> ringPointBuffer;
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
            /// Adds a connection for the roundabout lane: entry (edge lane -> ring), exit (ring -> edge lane) or the lane running along the ring.
            /// Identity of the lane end placed on the ring is read from ring points provided by the lane system,
            /// the lane end which is not listed there is the middle point of the bypass lane
            /// </summary>
            private void AddRingConnection(Entity node, DynamicBuffer<RoundaboutRingPoint> ringPoints, SubLane subLane, Lane lane, Entity sourceEdge, Entity targetEdge, bool startOnRing, bool endOnRing, bool isUnsafe, bool isForbidden)
            {
                if (!startOnRing && !endOnRing)
                {
                    return;
                }
                Entity sourceEntity = sourceEdge;
                Entity targetEntity = targetEdge;
                int2 laneIndexMap = new int2(lane.m_StartNode.GetLaneIndex() & 0xff, lane.m_EndNode.GetLaneIndex() & 0xff);
                float3x2 lanePositionMap = default;
                int2 sourceCarriagewayAndGroup = int2.zero;
                int2 targetCarriagewayAndGroup = int2.zero;
                if (startOnRing)
                {
                    int pathLaneIndex = lane.m_StartNode.GetLaneIndex();
                    if (!TryFindRingPoint(ringPoints, pathLaneIndex, true, out sourceEntity, out laneIndexMap.x))
                    {
                        if (endOnRing || targetEdge == Entity.Null)
                        {
                            return;
                        }
                        sourceEntity = targetEdge;
                        laneIndexMap.x = Helpers.RoundaboutUtils.EncodeExit(pathLaneIndex, true);
                    }
                }
                else if (sourceEdge == Entity.Null ||
                    !Helpers.NetUtils.GetLaneEndDetails(sourceEdge, laneIndexMap.x, edgeData[sourceEdge].m_End.Equals(node), ref compositionData, ref prefabCompositionLaneBuffer, out lanePositionMap.c0, out sourceCarriagewayAndGroup))
                {
                    return;
                }
                if (endOnRing)
                {
                    int pathLaneIndex = lane.m_EndNode.GetLaneIndex();
                    if (!TryFindRingPoint(ringPoints, pathLaneIndex, false, out targetEntity, out laneIndexMap.y))
                    {
                        if (startOnRing || sourceEdge == Entity.Null)
                        {
                            return;
                        }
                        targetEntity = sourceEdge;
                        laneIndexMap.y = Helpers.RoundaboutUtils.EncodeEntry(pathLaneIndex, true);
                    }
                }
                else if (targetEdge == Entity.Null ||
                    !Helpers.NetUtils.GetLaneEndDetails(targetEdge, laneIndexMap.y, edgeData[targetEdge].m_End.Equals(node), ref compositionData, ref prefabCompositionLaneBuffer, out lanePositionMap.c1, out targetCarriagewayAndGroup))
                {
                    return;
                }

                if (sourceEntity != Entity.Null && targetEntity != Entity.Null &&
                    connectorsList.TryGetValue(new NodeEdgeLaneKey(node.Index, sourceEntity.Index, laneIndexMap.x), out Entity sourceConnector))
                {
                    Connection connection = new Connection(lane, curveData[subLane.m_SubLane].m_Bezier, lanePositionMap, new int4(sourceCarriagewayAndGroup, targetCarriagewayAndGroup), subLane.m_PathMethods, sourceEntity, targetEntity, isUnsafe,
                        isForbidden);
                    connection.laneIndexMap = laneIndexMap;
                    Entity e = commandBuffer.CreateEntity();
                    DynamicBuffer<Connection> connectionsBuffer = commandBuffer.AddBuffer<Connection>(e);
                    connectionsBuffer.Add(connection);
                    Logger.DebugConnections($"Adding ring connection ({e}) n[{node}] idx[{lane.m_StartNode.GetLaneIndex()}->{lane.m_EndNode.GetLaneIndex()}] identity[{sourceEntity}:{laneIndexMap.x} -> {targetEntity}:{laneIndexMap.y}] | connector: {sourceConnector}");
                    commandBuffer.AppendToBuffer(sourceConnector, new LaneConnection() { connection = e });
                }
            }

            private static bool TryFindRingPoint(DynamicBuffer<RoundaboutRingPoint> ringPoints, int pathLaneIndex, bool isSource, out Entity edge, out int laneIndex)
            {
                for (int i = 0; i < ringPoints.Length; i++)
                {
                    RoundaboutRingPoint ringPoint = ringPoints[i];
                    if (ringPoint.pathLaneIndex == pathLaneIndex && ringPoint.isSource == isSource)
                    {
                        edge = ringPoint.edge;
                        laneIndex = ringPoint.laneIndex;
                        return true;
                    }
                }
                edge = Entity.Null;
                laneIndex = -1;
                return false;
            }

            /// <summary>
            /// Fallback, used when ring points have not been provided by the lane system yet.
            /// Adds a connection for the roundabout entry lane (edge lane -> ring) or exit lane (ring -> edge lane).
            /// The lane end placed on the ring is owned by the node, see <see cref="Helpers.RoundaboutUtils"/>.
            /// Lanes running along the ring (both ends owned by the node) are skipped
            /// </summary>
            private void AddRingConnection(Entity node, SubLane subLane, Lane lane, Entity sourceEdge, Entity targetEdge, bool isUnsafe, bool isForbidden)
            {
                bool startOnRing = sourceEdge == Entity.Null && Helpers.RoundaboutUtils.IsOwnedBy(lane.m_StartNode, node);
                bool endOnRing = targetEdge == Entity.Null && Helpers.RoundaboutUtils.IsOwnedBy(lane.m_EndNode, node);
                if (ringPointBuffer.TryGetBuffer(node, out DynamicBuffer<RoundaboutRingPoint> ringPoints) && !ringPoints.IsEmpty)
                {
                    // lane ends of the ring provided by the lane system, lanes running along the ring are supported
                    AddRingConnection(node, ringPoints, subLane, lane, sourceEdge, targetEdge, startOnRing, endOnRing, isUnsafe, isForbidden);
                    return;
                }
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

                // the lane end placed on the ring is identified by the same edge and flagged lane index
                int ringPathLaneIndex = startOnRing ? lane.m_StartNode.GetLaneIndex() : lane.m_EndNode.GetLaneIndex();
                int ringLaneIndex = startOnRing ? Helpers.RoundaboutUtils.EncodeExit(ringPathLaneIndex) : Helpers.RoundaboutUtils.EncodeEntry(ringPathLaneIndex);
                if (!connectorsList.TryGetValue(new NodeEdgeLaneKey(node.Index, edge.Index, ringLaneIndex), out Entity ringConnector) ||
                    !connectorData.TryGetComponent(ringConnector, out Connector ringConnectorData) ||
                    ringConnectorData.pathLaneIndex != ringPathLaneIndex)
                {
                    // not a lane of the ring, middle point of the bypass lane
                    ringLaneIndex |= Helpers.RoundaboutUtils.RingBypassFlag;
                }
                int2 laneIndexMap;
                float3x2 lanePositionMap;
                int4 carriagewayWithGroupMap;
                if (startOnRing)
                {
                    // exit lane, the source connector is placed on the ring
                    laneIndexMap = new int2(ringLaneIndex, edgeLaneIndex);
                    lanePositionMap = new float3x2(float3.zero, lanePosition);
                    carriagewayWithGroupMap = new int4(0, 0, carriagewayAndGroup.x, carriagewayAndGroup.y);
                }
                else
                {
                    // entry lane, the source connector is the edge lane end
                    laneIndexMap = new int2(edgeLaneIndex, ringLaneIndex);
                    lanePositionMap = new float3x2(lanePosition, float3.zero);
                    carriagewayWithGroupMap = new int4(carriagewayAndGroup.x, carriagewayAndGroup.y, 0, 0);
                }

                if (connectorsList.TryGetValue(new NodeEdgeLaneKey(node.Index, edge.Index, laneIndexMap.x), out Entity sourceConnector))
                {
                    Connection connection = new Connection(lane, curveData[subLane.m_SubLane].m_Bezier, lanePositionMap, carriagewayWithGroupMap, subLane.m_PathMethods, edge, edge, isUnsafe, isForbidden);
                    connection.laneIndexMap = laneIndexMap;
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
