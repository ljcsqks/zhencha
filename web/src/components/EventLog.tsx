import type { EventRecord, SimulationState } from "../types/sim";

interface Props {
  state?: SimulationState;
  eventLog: EventRecord[];
}

export function EventLog({ state, eventLog }: Props) {
  return (
    <section className="panel log-panel">
      <h2>Event Log</h2>
      <div className="event-stream">
        <EventGroup title="pending" events={state?.pending_events || []} />
        <EventGroup title="recent" events={state?.recent_events || []} />
        <EventGroup title="history" events={eventLog.slice(-80).reverse()} />
        {(state?.pending_events?.length || 0) + (state?.recent_events?.length || 0) + eventLog.length === 0 && (
          <span className="empty">No events yet.</span>
        )}
      </div>
    </section>
  );
}

function EventGroup({ title, events }: { title: string; events: EventRecord[] }) {
  if (events.length === 0) {
    return null;
  }
  return (
    <div className="event-group">
      <h3>{title}</h3>
      {events.map((event, index) => (
        <article key={`${title}-${event.event_id}-${index}`} className={`event-row ${event.status || "queued"}`}>
          <div>
            <strong>{event.type}</strong>
            <span>{event.status || "queued"}</span>
          </div>
          <code>{event.event_id}</code>
          <small>
            queued {fmt(event.queued_at_s)} / handled {fmt(event.handled_at_s)}
          </small>
        </article>
      ))}
    </div>
  );
}

function fmt(value?: number | null): string {
  return typeof value === "number" ? value.toFixed(1) : "-";
}
