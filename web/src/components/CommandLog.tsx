import type { CommandLogEntry } from "../hooks/simulationState";

interface Props {
  entries: CommandLogEntry[];
  selectedCommandId?: string;
  onSelectCommand(commandId: string | undefined): void;
  onClearLogs(): void;
}

export function CommandLog({ entries, selectedCommandId, onSelectCommand, onClearLogs }: Props) {
  const rows = entries.slice(-80).reverse();
  return (
    <section className="panel log-panel">
      <div className="panel-heading">
        <h2>Command Log</h2>
        <button onClick={onClearLogs}>Clear logs</button>
      </div>
      <div className="table-wrap">
        <table>
          <thead>
            <tr>
              <th>time</th>
              <th>command_id</th>
              <th>command</th>
              <th>uav</th>
              <th>task</th>
              <th>type</th>
              <th>ack</th>
              <th>progress</th>
              <th>reason</th>
              <th>issued</th>
              <th>updated</th>
            </tr>
          </thead>
          <tbody>
            {rows.map((entry, index) => (
              <tr
                key={`${entry.command_id}-${index}`}
                className={`${entry.advisory || !entry.executable ? "advisory-row" : ""} ${selectedCommandId === entry.command_id ? "selected-row" : ""}`}
                onClick={() => onSelectCommand(selectedCommandId === entry.command_id ? undefined : entry.command_id)}
              >
                <td>{fmt(entry.time_s)}</td>
                <td className="mono">{entry.command_id}</td>
                <td>{entry.command}</td>
                <td>{entry.uav_id || "-"}</td>
                <td>{entry.task_id || "-"}</td>
                <td>{entry.advisory || !entry.executable ? "advisory" : "exec"}</td>
                <td>{entry.ack_status || "-"}</td>
                <td>{typeof entry.progress === "number" ? entry.progress.toFixed(2) : "-"}</td>
                <td>{entry.reason || "-"}</td>
                <td>{fmt(entry.issued_at)}</td>
                <td>{fmt(entry.updated_at)}</td>
              </tr>
            ))}
          </tbody>
        </table>
        {rows.length === 0 && <span className="empty">No commands yet.</span>}
      </div>
    </section>
  );
}

function fmt(value?: number | null): string {
  return typeof value === "number" ? value.toFixed(1) : "-";
}
