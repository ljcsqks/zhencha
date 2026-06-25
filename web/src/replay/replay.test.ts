import { describe, expect, it } from "vitest";
import { parseReplayPayload, replayStepToState } from "./replay";

describe("replay helpers", () => {
  it("parses exported replay payloads with summary and steps", () => {
    const replay = parseReplayPayload({
      run_id: "run_1",
      scenario_name: "demo_search_3uav",
      summary: { final_coverage: 0.95 },
      steps: [
        {
          time_s: 1,
          global_coverage: 0.2,
          priority_coverage: 0.5,
          uavs: [],
          commands: [],
          command_acks: [],
          events: [],
        },
      ],
    });

    expect(replay.run_id).toBe("run_1");
    expect(replay.steps).toHaveLength(1);
    expect(replay.summary.final_coverage).toBe(0.95);
  });

  it("converts a replay step to a read-only simulation state", () => {
    const state = replayStepToState(
      {
        time_s: 2,
        global_coverage: 0.4,
        priority_coverage: 0.6,
        uavs: [{ id: "uav_01", position: { x: 1, y: 1 }, status: "SEARCHING", battery: 1 }],
        commands: [{ command_id: "cmd_1", command: "FOLLOW_PATH", uav_id: "uav_01" }],
        command_acks: [{ command_id: "cmd_1", status: "running" }],
        events: ["event_1"],
      },
      "run_1",
      "demo",
      7,
    );

    expect(state.running).toBe(false);
    expect(state.tick).toBe(7);
    expect(state.run_id).toBe("replay_run_1");
    expect(state.commands[0].command_id).toBe("cmd_1");
  });
});
