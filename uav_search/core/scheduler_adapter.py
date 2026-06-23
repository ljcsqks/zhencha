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
        self.scheduler.handle_command_acks(observation.command_acks)
        for event in observation.events:
            self.scheduler.event_manager.emit(event)

        decision = self.scheduler.regular_cycle(now=observation.time_s)
        commands = [
            ControlCommand.from_decision(command, issued_at=observation.time_s, ttl_s=self.default_command_ttl_s)
            for command in decision.commands
        ]
        self.scheduler.remember_control_commands(commands)
        return ContractDecisionOutput(
            commands=commands,
            task_summary=self.scheduler.task_status_snapshot(),
            target_summary=self.scheduler.target_metrics_snapshot(),
            metrics_updates={
                "global_coverage": decision.global_coverage,
                "priority_coverage": decision.priority_coverage,
                "decision_latency_ms": decision.decision_latency_ms,
            },
            debug={"events_handled": list(decision.events_handled), "assignments": list(decision.assignments)},
        )

    def update_after_step(self, now: float) -> ContractDecisionOutput:
        commands, events_handled = self.scheduler.update_after_step(now=now)
        control_commands = [
            ControlCommand.from_decision(command, issued_at=now, ttl_s=self.default_command_ttl_s)
            for command in commands
        ]
        self.scheduler.remember_control_commands(control_commands)
        return ContractDecisionOutput(
            commands=control_commands,
            task_summary=self.scheduler.task_status_snapshot(),
            target_summary=self.scheduler.target_metrics_snapshot(),
            debug={"events_handled": list(events_handled), "assignments": []},
        )
