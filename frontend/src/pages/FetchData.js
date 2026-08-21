import { useEffect, useState } from "react";
import RobotStateHistory from "../components/RobotStateHistory";
import { useApiClient } from "../context/ApiClientContext";

export default function FetchData() {
  const apiClient = useApiClient();
  const [sessions, setSessions] = useState([]);
  const [selectedSession, setSelectedSession] = useState("");
  const [loadingSessions, setLoadingSessions] = useState(true);

  useEffect(() => {
    let active = true;
    (async () => {
      try {
        const data = await apiClient.getSessions();
        if (active) {
          setSessions(data || []);
          if (data && data.length > 0) {
            setSelectedSession(data[0].id);
          }
        }
      } catch (e) {
        console.error("Failed to load sessions", e);
      } finally {
        if (active) setLoadingSessions(false);
      }
    })();
    return () => {
      active = false;
    };
  }, [apiClient]);

  return (
    <div>
      <div style={{ marginTop: "2rem" }}>
        <h2>Robot State History</h2>
        <div style={{ marginBottom: "1rem" }}>
          <h3>Select Session:</h3>
          {loadingSessions ? (
            <p><em>Loading sessions...</em></p>
          ) : sessions.length === 0 ? (
            <p>No sessions available</p>
          ) : (
            <table className="table table-striped">
              <thead>
                <tr>
                  <th>Session ID</th>
                  <th>Session Code</th>
                  <th>Last Status</th>
                  <th>Action</th>
                </tr>
              </thead>
              <tbody>
                {sessions.map((session) => (
                  <tr key={session.id}>
                    <td>{session.id}</td>
                    <td>{session.sessionCode}</td>
                    <td>{session.laststate?.status}</td>
                    <td>
                      <button
                        onClick={() => setSelectedSession(session.id)}
                        style={{
                          background: "none",
                          border: "none",
                          color: "#007bff",
                          cursor: "pointer",
                          textDecoration: "underline",
                          padding: 0,
                        }}
                      >
                        Select
                      </button>
                    </td>
                  </tr>
                ))}
              </tbody>
            </table>
          )}
        </div>
        {selectedSession && <RobotStateHistory sessionId={selectedSession} />}
      </div>
    </div>
  );
}
