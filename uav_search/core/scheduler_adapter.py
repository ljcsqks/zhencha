from __future__ import annotations

from uav_search.core.contracts import ControlCommand
from uav_search.core.contracts import DecisionOutput as ContractDecisionOutput
from uav_search.core.contracts import Observation
from uav_search.core.scheduler import Scheduler


class SchedulerAlgorithmAdapter:
    """Adapter that exposes the legacy Scheduler through the new algorithm boundary.

    The adapter intentionally reuses the same Scheduler instance for the whole run.
    Observation is used to synchronize external inputs and command acknowledgements;
    task allocation, search state, and confirmation state stay inside Scheduler.
    """

    def __init__(self, scheduler: Scheduler, default_command_ttl_s: float | None = None) -> None:
        self.scheduler = scheduler
        self.default_command_ttl_s = default_command_ttl_s
        self.last_command_acks = []

    def decide(self, observation: Observation) -> ContractDecisionOutput:
        self.last_command_acks = list(observation.command_acks)
        ack_commands = self.scheduler.handle_command_acks(observation.command_acks)
        ack_events_handled = self.scheduler.pop_ack_events_handled()
        for event in observation.events:
            self.scheduler.event_manager.emit(event)

        decision = self.scheduler.regular_cycle(now=observation.time_s)
        decision_commands = [*ack_commands, *decision.commands]
        commands = [
            ControlCommand.from_decision(command, issued_at=observation.time_s, ttl_s=self.default_command_ttl_s)
            for command in decision_commands
        ]
        executable_commands = [
            command
            for command in commands
            if not (command.metadata.get("advisory") is True or command.metadata.get("effect") == "none")
        ]
        advisory_commands = [command for command in commands if command not in executable_commands]
        advisory_summary = _summarize_advisories(advisory_commands)
        self.scheduler.remember_control_commands(executable_commands)
        return ContractDecisionOutput(
            commands=executable_commands,
            task_summary=self.scheduler.task_status_snapshot(),
            target_summary=self.scheduler.target_metrics_snapshot(),
            metrics_updates={
                "global_coverage": decision.global_coverage,
                "priority_coverage": decision.priority_coverage,
                "decision_latency_ms": decision.decision_latency_ms,
            },
            debug={
                "events_handled": [*ack_events_handled, *decision.events_handled],
                "assignments": list(decision.assignments),
                "conflict_advisories": [],
                "conflict_summary": advisory_summary,
            },
        )

    def update_after_step(self, now: float) -> ContractDecisionOutput:
        return ContractDecisionOutput(
            commands=[],
            task_summary=self.scheduler.task_status_snapshot(),
            target_summary=self.scheduler.target_metrics_snapshot(),
            debug={"events_handled": [], "assignments": []},
        )


def _summarize_advisories(commands: list[ControlCommand]) -> dict[str, object]:
    by_uav: dict[str, int] = {}
    by_reason: dict[str, int] = {}
    for command in commands:
        by_uav[command.uav_id] = by_uav.get(command.uav_id, 0) + 1
        reason = command.reason or "unspecified"
        by_reason[reason] = by_reason.get(reason, 0) + 1
    return {"count": len(commands), "by_uav": by_uav, "by_reason": by_reason}
