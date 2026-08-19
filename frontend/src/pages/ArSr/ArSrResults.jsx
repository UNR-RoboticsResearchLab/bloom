import { useState, useEffect } from "react";
import { useNavigate } from "react-router-dom";

const API = process.env.REACT_APP_API_BASE_URL || "http://bloom-server-dev:5000/";

export default function ArSrResults() {
  const [sessions, setSessions]   = useState([]);
  const [loading, setLoading]     = useState(true);
  const [error, setError]         = useState(null);
  const navigate                  = useNavigate();

  useEffect(() => {
    fetch(`${API}api/arsr/sessions`, { credentials: "include" })
      .then(r => { if (!r.ok) throw new Error(r.statusText); return r.json(); })
      .then(data => { setSessions(data); setLoading(false); })
      .catch(e => { setError(e.message); setLoading(false); });
  }, []);

  if (loading) return <p style={{ padding: 40, color: "#888" }}>Loading…</p>;
  if (error)   return <p style={{ padding: 40, color: "#ef4444" }}>Error: {error}</p>;

  return (
    <div style={{ maxWidth: 900, margin: "40px auto", padding: "0 16px" }}>
      <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center", marginBottom: 28 }}>
        <div>
          <h2 style={{ fontWeight: 800, fontSize: "1.6rem", marginBottom: 4 }}>RSR Sessions</h2>
          <p style={{ color: "#888", fontSize: "0.9rem" }}>{sessions.length} session{sessions.length !== 1 ? "s" : ""}</p>
        </div>
        <button style={btnStyle} onClick={() => navigate("/arsr/study")}>+ New Session</button>
      </div>

      {sessions.length === 0
        ? <p style={{ color: "#aaa", textAlign: "center", padding: 60 }}>No sessions yet.</p>
        : (
          <table style={{ width: "100%", borderCollapse: "collapse" }}>
            <thead>
              <tr style={{ borderBottom: "2px solid #e5e5e5" }}>
                {["Date", "Participant", "Age (mo)", "Score", "Result", "Percentile", ""].map(h => (
                  <th key={h} style={thStyle}>{h}</th>
                ))}
              </tr>
            </thead>
            <tbody>
              {sessions.map(s => (
                <tr key={s.id}
                  onClick={() => navigate(`/arsr/sessions/${s.id}`)}
                  onMouseEnter={e => e.currentTarget.style.background = "#fafaf9"}
                  onMouseLeave={e => e.currentTarget.style.background = ""}
                  style={{ cursor: "pointer", borderBottom: "1px solid #f0f0f0" }}>
                  <td style={tdStyle}>{new Date(s.createdAt).toLocaleDateString()}</td>
                  <td style={{ ...tdStyle, fontWeight: 600 }}>{s.participantCode || "—"}</td>
                  <td style={tdStyle}>{s.ageMonths}</td>
                  <td style={{ ...tdStyle, fontWeight: 700 }}>{s.totalScore ?? "—"}</td>
                  <td style={tdStyle}>
                    <span style={{
                      padding: "3px 10px", borderRadius: 999, fontSize: "0.75rem", fontWeight: 700,
                      background: s.result === "Pass" ? "#d1fae5" : s.result === "Fail" ? "#fee2e2" : "#f3f4f6",
                      color: s.result === "Pass" ? "#065f46" : s.result === "Fail" ? "#b91c1c" : "#6b7280",
                    }}>
                      {s.result ?? "N/A"}
                    </span>
                  </td>
                  <td style={tdStyle}>{s.percentile}th</td>
                  <td style={tdStyle}>
                    <button style={{ ...btnSmall }} onClick={e => { e.stopPropagation(); navigate(`/arsr/sessions/${s.id}`); }}>
                      View →
                    </button>
                  </td>
                </tr>
              ))}
            </tbody>
          </table>
        )
      }
    </div>
  );
}

const btnStyle  = { padding: "10px 18px", background: "#111", color: "#fff",
                    border: "none", borderRadius: 8, fontSize: "0.85rem",
                    fontWeight: 700, cursor: "pointer" };
const btnSmall  = { padding: "5px 12px", background: "#f3f4f6", color: "#111",
                    border: "none", borderRadius: 6, fontSize: "0.8rem",
                    fontWeight: 600, cursor: "pointer" };
const thStyle   = { textAlign: "left", padding: "8px 12px", fontWeight: 700,
                    fontSize: "0.73rem", textTransform: "uppercase", color: "#888" };
const tdStyle   = { padding: "12px 12px", fontSize: "0.9rem" };
