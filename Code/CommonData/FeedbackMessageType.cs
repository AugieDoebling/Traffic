namespace Traffic.CommonData
{
    public enum FeedbackMessageType
    {
        WarnResetForbiddenTurnUpgrades,
        WarnForbiddenTurnApply,
        WarnResetPrioritiesTrafficLightsApply,
        WarnResetPrioritiesRoundaboutApply,
        WarnResetPrioritiesChangeApply,
        WarnRoadBuilderApply,
        WarnResetLaneConnectionsRoundaboutChange,
        ErrorLaneConnectorNotSupported,
        ErrorPrioritiesNotSupported,
        ErrorPrioritiesRemoveTrafficLights,
        ErrorApplyRoundabout,
        ErrorHasRoundabout,
    }
}
