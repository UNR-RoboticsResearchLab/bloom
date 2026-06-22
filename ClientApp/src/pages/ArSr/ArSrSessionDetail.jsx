import { useState, useEffect } from "react";
import { useParams, useNavigate } from "react-router-dom";

const API = process.env.REACT_APP_API_BASE_URL || "http://bloom-server-dev:5000/";

export default function ArSrSessionDetail() {
  const { sessionId }             = useParams();
  const navigate                  = useNavigate();
  const [session, setSession]     = useState(null);
  const [loading, setLoading]     = useState(true);
  const [error, setError]         = useState(null);
  const [expanded, setExpanded]   = useState(null);

  useEffect(() => {
    fetch(`${API}api/arsr/sessions/${sessionId}`, { credentials: "include" })
      .then(r => { if (!r.ok) throw new Error(r.statusText); return r.json(); })
      .then(data => { setSession(data); setLoading(false); })
      .catch(e => { setError(e.message); setLoading(false); });
  }, [sessionId]);

  if (loading) return <p style={{ padding: 40, color: "#888" }}>Loading…</p>;
  if (error)   return <p style={{ padding: 40, color: "#ef4444" }}>Error: {error}</p>;
  if (!session) return null;

  const passColor = session.result === "Pass" ? "#d1fae5"
                  : session.result === "Fail" ? "#fee2e2" : "#f3f4f6";
  const passText  = session.result === "Pass" ? "#065f46"
                  : session.result === "Fail" ? "#b91c1c" : "#6b7280";

  return (
    <div style={{ maxWidth: 800, margin: "40px auto", padding: "0 16px" }}>
      <button style={backBtn} onClick={() => navigate("/arsr/results")}>← Back</button>

      {/* Header */}
      <div style={{ background: "#fff", border: "1.5px solid #e5e5e5", borderRadius: 14,
                    padding: 32, marginBottom: 20 }}>
        <div style={{ display: "flex", justifyContent: "space-between", alignItems: "flex-start", flexWrap: "wrap", gap: 12 }}>
          <div>
            <h2 style={{ fontWeight: 800, fontSize: "1.5rem", marginBottom: 4 }}>
              Session — {session.participantCode || "Unknown"}
            </h2>
            <p style={{ color: "#888", fontSize: "0.85rem" }}>
              {new Date(session.createdAt).toLocaleString()} · Age {session.ageMonths} months · {session.percentile}th percentile
            </p>
            {session.administratorNotes && (
              <p style={{ marginTop: 10, fontSize: "0.88rem", color: "#555" }}>
                {session.administratorNotes}
              </p>
            )}
          </div>
          <div style={{ textAlign: "center" }}>
            <div style={{ fontSize: "2.2rem", fontWeight: 800 }}>{session.totalScore ?? "—"}</div>
            <div style={{ fontSize: "0.75rem", color: "#888", marginBottom: 6 }}>Total Score</div>
            <span style={{ padding: "4px 14px", borderRadius: 999, fontWeight: 700, fontSize: "0.85rem",
                           background: passColor, color: passText }}>
              {session.result ?? "N/A"}
            </span>
          </div>
        </div>
      </div>

      {/* Score summary bar */}
      <div style={{ background: "#fff", border: "1.5px solid #e5e5e5", borderRadius: 14,
                    padding: "18px 24px", marginBottom: 20, display: "flex", gap: 20, flexWrap: "wrap" }}>
        {[2, 1, 0].map(score => {
          const count = session.sentenceResults.filter(r => r.score === score).length;
          return (
            <div key={score} style={{ display: "flex", alignItems: "center", gap: 8 }}>
              <span style={{ fontWeight: 800, fontSize: "1.3rem",
                             color: score === 2 ? "#22c55e" : score === 1 ? "#f59e0b" : "#ef4444" }}>
                {count}
              </span>
              <span style={{ fontSize: "0.8rem", color: "#888" }}>
                {score === 2 ? "perfect (2 pts)" : score === 1 ? "minor errors (1 pt)" : "errors (0 pts)"}
              </span>
            </div>
          );
        })}
      </div>

      {/* Sentence-by-sentence results */}
      <div style={{ background: "#fff", border: "1.5px solid #e5e5e5", borderRadius: 14, overflow: "hidden" }}>
        {session.sentenceResults.map((r, idx) => {
          const isOpen   = expanded === r.sentenceNumber;
          const editData = r.editScript ? tryParse(r.editScript) : null;
          return (
            <div key={r.sentenceNumber} style={{ borderBottom: idx < session.sentenceResults.length - 1 ? "1px solid #f0f0f0" : "none" }}>
              <div style={{ display: "flex", alignItems: "center", padding: "14px 20px",
                            cursor: "pointer", gap: 14 }}
                onClick={() => setExpanded(isOpen ? null : r.sentenceNumber)}>
                <span style={{ width: 28, height: 28, borderRadius: 6, display: "flex",
                               alignItems: "center", justifyContent: "center", fontSize: "0.75rem",
                               fontWeight: 700, flexShrink: 0,
                               background: r.score === 2 ? "#d1fae5" : r.score === 1 ? "#fef3c7" : "#fee2e2",
                               color: r.score === 2 ? "#065f46" : r.score === 1 ? "#92400e" : "#b91c1c" }}>
                  {r.sentenceNumber}
                </span>
                <div style={{ flex: 1, minWidth: 0 }}>
                  <div style={{ fontSize: "0.88rem", color: "#555", marginBottom: 2,
                                whiteSpace: "nowrap", overflow: "hidden", textOverflow: "ellipsis" }}>
                    {r.groundTruth}
                  </div>
                  <div style={{ fontSize: "0.82rem", color: r.response ? "#111" : "#bbb",
                                whiteSpace: "nowrap", overflow: "hidden", textOverflow: "ellipsis" }}>
                    {r.response || "No response recorded"}
                  </div>
                </div>
                <div style={{ display: "flex", gap: 12, alignItems: "center", flexShrink: 0 }}>
                  <span style={{ fontSize: "0.75rem", color: "#888" }}>{r.errors} error{r.errors !== 1 ? "s" : ""}</span>
                  <span style={{ fontWeight: 800, fontSize: "1rem",
                                 color: r.score === 2 ? "#22c55e" : r.score === 1 ? "#f59e0b" : "#ef4444" }}>
                    {r.score} pt{r.score !== 1 ? "s" : ""}
                  </span>
                  <span style={{ color: "#aaa" }}>{isOpen ? "▲" : "▼"}</span>
                </div>
              </div>

              {isOpen && (
                <div style={{ padding: "0 20px 16px", borderTop: "1px solid #f9f9f7" }}>
                  <Row label="Ground truth" value={r.groundTruth} />
                  <Row label="Response"     value={r.response || "—"} />
                  {editData && <EditScriptView data={editData} />}
                </div>
              )}
            </div>
          );
        })}
      </div>
    </div>
  );
}

function Row({ label, value }) {
  return (
    <div style={{ display: "flex", gap: 12, marginTop: 10, fontSize: "0.85rem" }}>
      <span style={{ color: "#888", fontWeight: 600, minWidth: 100, flexShrink: 0 }}>{label}</span>
      <span style={{ color: "#111" }}>{value}</span>
    </div>
  );
}

function EditScriptView({ data }) {
  const sections = ["Insertions", "Deletions", "Substitutions", "Swaps"];
  const hasAny   = sections.some(k => data[k]?.length > 0);
  if (!hasAny) return <Row label="Edits" value="None" />;

  return (
    <div style={{ marginTop: 10 }}>
      <span style={{ color: "#888", fontWeight: 600, fontSize: "0.85rem" }}>Edit script</span>
      <div style={{ marginTop: 6, display: "flex", flexWrap: "wrap", gap: 6 }}>
        {sections.flatMap(type =>
          (data[type] ?? []).map((item, i) => (
            <span key={`${type}-${i}`} style={{
              padding: "3px 9px", borderRadius: 6, fontSize: "0.75rem", fontWeight: 600,
              background: type === "Insertions" ? "#dbeafe" : type === "Deletions" ? "#fee2e2"
                        : type === "Substitutions" ? "#fef3c7" : "#f3f4f6",
              color: type === "Insertions" ? "#1d4ed8" : type === "Deletions" ? "#b91c1c"
                   : type === "Substitutions" ? "#92400e" : "#374151",
            }}>
              {type.slice(0, -1)}: {JSON.stringify(item)}
            </span>
          ))
        )}
      </div>
    </div>
  );
}

function tryParse(str) {
  try { return JSON.parse(str); } catch { return null; }
}

const backBtn = { background: "none", border: "none", color: "#888", cursor: "pointer",
                  fontSize: "0.85rem", padding: "0 0 16px", fontWeight: 600 };
