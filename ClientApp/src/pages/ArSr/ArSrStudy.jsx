import { useState, useRef, useEffect, useCallback } from "react";

const API = process.env.REACT_APP_API_BASE_URL || "http://bloom-server-dev:5000/";

const SENTENCES_COUNT = 16;

export default function ArSrStudy() {
  const [enrollments, setEnrollments]       = useState([]);
  const [selectedEnrollment, setSelected]   = useState(null);
  const [ageMonths, setAgeMonths]           = useState("");
  const [percentile, setPercentile]         = useState(5);
  const [sentences, setSentences]           = useState([]);
  const [currentSentence, setCurrent]       = useState(0); // 1-indexed, 0 = not started
  const [markers, setMarkers]               = useState({});
  const [phase, setPhase]                   = useState("setup"); // setup | session | submitting | done
  const [result, setResult]                 = useState(null);
  const [error, setError]                   = useState(null);
  const [isRecording, setIsRecording]       = useState(false);
  const [sentenceAudio, setSentenceAudio]   = useState(null); // per-sentence stimulus audio

  const mediaRecorderRef = useRef(null);
  const audioChunksRef   = useRef([]);
  const sessionStartRef  = useRef(null);
  const sentenceStartRef = useRef(null);
  const audioBlobRef     = useRef(null);

  useEffect(() => {
    fetch(`${API}api/arsr/sentences`, { credentials: "include" })
      .then(r => r.json()).then(setSentences).catch(console.error);
    fetch(`${API}api/arsr/enrollments`, { credentials: "include" })
      .then(r => r.json()).then(setEnrollments).catch(console.error);
  }, []);

  // ── Recording helpers ───────────────────────────────────────────────────

  const startSessionRecording = useCallback(async () => {
    const stream = await navigator.mediaDevices.getUserMedia({ audio: true });
    const mr     = new MediaRecorder(stream, { mimeType: "audio/webm" });
    audioChunksRef.current = [];

    mr.ondataavailable = e => { if (e.data.size > 0) audioChunksRef.current.push(e.data); };
    mr.onstop = () => {
      audioBlobRef.current = new Blob(audioChunksRef.current, { type: "audio/webm" });
    };

    mediaRecorderRef.current = mr;
    mr.start(100); // collect chunks every 100ms
    sessionStartRef.current = Date.now();
    setIsRecording(true);
  }, []);

  const stopSessionRecording = useCallback(() => {
    mediaRecorderRef.current?.stop();
    setIsRecording(false);
  }, []);

  // ── Sentence flow ───────────────────────────────────────────────────────

  const startSession = async () => {
    if (!selectedEnrollment || !ageMonths) {
      setError("Select a participant and enter age in months.");
      return;
    }
    setError(null);
    await startSessionRecording();
    setPhase("session");
    setCurrent(1);
  };

  const markSentenceStart = () => {
    sentenceStartRef.current = Date.now() - sessionStartRef.current;
    // Play the stimulus audio if available
    const s = sentences.find(s => s.id === currentSentence);
    if (s?.audio_url) {
      const audio = new Audio(`${API}${s.audio_url}`);
      setSentenceAudio(audio);
      audio.play();
    }
  };

  const markSentenceEnd = () => {
    const start_ms = sentenceStartRef.current ?? 0;
    const end_ms   = Date.now() - sessionStartRef.current;
    setMarkers(prev => ({
      ...prev,
      [currentSentence]: { sentence_id: currentSentence, start_ms, end_ms },
    }));
    setSentenceAudio(null);
  };

  const nextSentence = () => {
    if (currentSentence < SENTENCES_COUNT) {
      setCurrent(n => n + 1);
      sentenceStartRef.current = null;
    }
  };

  const finishSession = async () => {
    stopSessionRecording();
    // Wait a tick for MediaRecorder.onstop to fire
    await new Promise(r => setTimeout(r, 200));
    setPhase("submitting");

    const form = new FormData();
    form.append("session_audio", audioBlobRef.current, "session.webm");
    form.append("enrollmentId",  selectedEnrollment.id);
    form.append("ageMonths",     ageMonths);
    form.append("percentile",    percentile);
    form.append("markers",       JSON.stringify(Object.values(markers)));

    try {
      const res = await fetch(`${API}api/arsr/sessions`, {
        method: "POST",
        credentials: "include",
        body: form,
      });
      if (!res.ok) throw new Error(await res.text());
      const data = await res.json();
      setResult(data);
      setPhase("done");
    } catch (e) {
      setError(e.message);
      setPhase("session");
      await startSessionRecording();
    }
  };

  const reset = () => {
    setPhase("setup");
    setCurrent(0);
    setMarkers({});
    setResult(null);
    setError(null);
    setSelected(null);
    setAgeMonths("");
    audioBlobRef.current    = null;
    sessionStartRef.current = null;
  };

  // ── Render ───────────────────────────────────────────────────────────────

  const currentText = sentences.find(s => s.id === currentSentence)?.text ?? "";
  const marked      = markers[currentSentence];

  return (
    <div style={{ maxWidth: 640, margin: "40px auto", padding: "0 16px" }}>
      <h2 style={{ fontWeight: 800, fontSize: "1.6rem", marginBottom: 4 }}>AutoRSR Screening</h2>
      <p style={{ color: "#666", marginBottom: 28 }}>16-item Redmond Sentence Recall</p>

      {error && (
        <div style={{ background: "#fee2e2", border: "1px solid #fca5a5", borderRadius: 8,
                      padding: "12px 16px", marginBottom: 20, color: "#b91c1c" }}>
          {error}
        </div>
      )}

      {/* ── Setup ── */}
      {phase === "setup" && (
        <div style={{ background: "#fff", border: "1.5px solid #e5e5e5", borderRadius: 14, padding: 32 }}>
          <label style={labelStyle}>Participant</label>
          <select style={inputStyle} value={selectedEnrollment?.id ?? ""}
            onChange={e => setSelected(enrollments.find(en => en.id === e.target.value) ?? null)}>
            <option value="">Select participant…</option>
            {enrollments.map(en => (
              <option key={en.id} value={en.id}>{en.participantCode} — {en.condition}</option>
            ))}
          </select>

          <label style={{ ...labelStyle, marginTop: 16 }}>Age (months)</label>
          <input style={inputStyle} type="number" min={60} max={119}
            value={ageMonths} onChange={e => setAgeMonths(e.target.value)}
            placeholder="e.g. 72" />

          <label style={{ ...labelStyle, marginTop: 16 }}>Percentile cutoff</label>
          <select style={inputStyle} value={percentile}
            onChange={e => setPercentile(Number(e.target.value))}>
            {[5, 10, 15].map(p => <option key={p} value={p}>{p}th</option>)}
          </select>

          <button style={{ ...btnStyle, marginTop: 24 }} onClick={startSession}>
            Start Session
          </button>
        </div>
      )}

      {/* ── Session ── */}
      {phase === "session" && (
        <div style={{ background: "#fff", border: "1.5px solid #e5e5e5", borderRadius: 14, padding: 32 }}>
          <div style={{ display: "flex", justifyContent: "space-between", marginBottom: 20 }}>
            <span style={{ fontSize: "0.8rem", fontWeight: 700, textTransform: "uppercase",
                           color: "#888", letterSpacing: "0.06em" }}>
              Sentence {currentSentence} / {SENTENCES_COUNT}
            </span>
            {isRecording && (
              <span style={{ color: "#ef4444", fontSize: "0.8rem", fontWeight: 700 }}>
                ● Recording
              </span>
            )}
          </div>

          <div style={{ background: "#f9f9f7", borderRadius: 10, padding: "20px 24px",
                        fontSize: "1.15rem", lineHeight: 1.6, marginBottom: 24, minHeight: 72 }}>
            {currentText}
          </div>

          <div style={{ display: "flex", gap: 10, flexWrap: "wrap" }}>
            <button style={{ ...btnStyle, flex: 1, background: marked ? "#d1fae5" : "#111",
                             color: marked ? "#065f46" : "#fff" }}
              onClick={markSentenceStart} disabled={!!marked}>
              ▶ Play &amp; Mark Start
            </button>
            <button style={{ ...btnStyle, flex: 1, background: marked ? "#d1fae5" : "#e5e7eb",
                             color: marked ? "#065f46" : "#111" }}
              onClick={markSentenceEnd} disabled={!sentenceStartRef.current || !!marked}>
              ■ Mark End
            </button>
          </div>

          <div style={{ display: "flex", gap: 10, marginTop: 12 }}>
            {currentSentence < SENTENCES_COUNT
              ? <button style={{ ...btnStyle, flex: 1, background: "#e5e7eb", color: "#111" }}
                  onClick={nextSentence} disabled={!marked}>
                  Next →
                </button>
              : <button style={{ ...btnStyle, flex: 1 }} onClick={finishSession} disabled={!marked}>
                  Finish &amp; Analyze
                </button>
            }
          </div>

          <div style={{ marginTop: 20 }}>
            <div style={{ fontSize: "0.75rem", color: "#aaa", marginBottom: 8 }}>
              Marked: {Object.keys(markers).length} / {SENTENCES_COUNT}
            </div>
            <div style={{ display: "flex", gap: 4, flexWrap: "wrap" }}>
              {Array.from({ length: SENTENCES_COUNT }, (_, i) => i + 1).map(n => (
                <div key={n} style={{
                  width: 28, height: 28, borderRadius: 6, display: "flex",
                  alignItems: "center", justifyContent: "center", fontSize: "0.7rem",
                  fontWeight: 700, cursor: "pointer",
                  background: markers[n] ? "#22c55e" : n === currentSentence ? "#111" : "#e5e7eb",
                  color: (markers[n] || n === currentSentence) ? "#fff" : "#666",
                }} onClick={() => setCurrent(n)}>
                  {n}
                </div>
              ))}
            </div>
          </div>
        </div>
      )}

      {/* ── Submitting ── */}
      {phase === "submitting" && (
        <div style={{ textAlign: "center", padding: 60 }}>
          <div style={{ fontSize: "2rem", marginBottom: 12 }}>⏳</div>
          <p style={{ color: "#666" }}>Transcribing and scoring… this may take a minute.</p>
        </div>
      )}

      {/* ── Done ── */}
      {phase === "done" && result && (
        <div style={{ background: "#fff", border: "1.5px solid #e5e5e5", borderRadius: 14, padding: 32 }}>
          <div style={{ textAlign: "center", marginBottom: 24 }}>
            <div style={{
              display: "inline-block", padding: "6px 18px", borderRadius: 999, fontWeight: 700,
              fontSize: "1rem", marginBottom: 8,
              background: result.result === "Pass" ? "#d1fae5" : result.result === "Fail" ? "#fee2e2" : "#f3f4f6",
              color: result.result === "Pass" ? "#065f46" : result.result === "Fail" ? "#b91c1c" : "#374151",
            }}>
              {result.result ?? "N/A"}
            </div>
            <div style={{ fontSize: "2.5rem", fontWeight: 800 }}>{result.totalScore}</div>
            <div style={{ color: "#888", fontSize: "0.9rem" }}>Total Score</div>
          </div>

          <table style={{ width: "100%", borderCollapse: "collapse", fontSize: "0.85rem" }}>
            <thead>
              <tr style={{ borderBottom: "2px solid #e5e5e5" }}>
                <th style={thStyle}>#</th>
                <th style={thStyle}>Ground Truth</th>
                <th style={thStyle}>Response</th>
                <th style={thStyle}>Errors</th>
                <th style={thStyle}>Score</th>
              </tr>
            </thead>
            <tbody>
              {result.sentenceResults?.map(r => (
                <tr key={r.sentenceNumber} style={{ borderBottom: "1px solid #f0f0f0" }}>
                  <td style={tdStyle}>{r.sentenceNumber}</td>
                  <td style={tdStyle}>{r.groundTruth}</td>
                  <td style={{ ...tdStyle, color: r.response ? "#111" : "#bbb" }}>
                    {r.response || "—"}
                  </td>
                  <td style={{ ...tdStyle, textAlign: "center" }}>{r.errors}</td>
                  <td style={{
                    ...tdStyle, textAlign: "center", fontWeight: 700,
                    color: r.score === 2 ? "#22c55e" : r.score === 1 ? "#f59e0b" : "#ef4444",
                  }}>{r.score}</td>
                </tr>
              ))}
            </tbody>
          </table>

          <button style={{ ...btnStyle, marginTop: 24 }} onClick={reset}>
            New Session
          </button>
        </div>
      )}
    </div>
  );
}

const labelStyle = { display: "block", fontSize: "0.82rem", fontWeight: 600,
                     marginBottom: 6, color: "#111" };
const inputStyle  = { width: "100%", padding: "10px 13px", border: "1.5px solid #e5e5e5",
                      borderRadius: 8, fontSize: "0.95rem", fontFamily: "inherit",
                      outline: "none", background: "#fff" };
const btnStyle    = { padding: "12px 20px", background: "#111", color: "#fff",
                      border: "none", borderRadius: 8, fontSize: "0.9rem",
                      fontWeight: 700, cursor: "pointer", fontFamily: "inherit" };
const thStyle     = { textAlign: "left", padding: "8px 10px", fontWeight: 700,
                      fontSize: "0.75rem", textTransform: "uppercase", color: "#888" };
const tdStyle     = { padding: "8px 10px", verticalAlign: "top" };
