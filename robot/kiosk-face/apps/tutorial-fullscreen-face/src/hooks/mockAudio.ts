// Synthesizes short, silent playback fixtures for mock RSR speech
// (?robotId=mock) so the real audio + viseme playback pipeline
// (useStaticVisemePlayback) can be exercised with no bloom backend, no
// Azure TTS credentials, and no checked-in audio binaries.

function writeAsciiString(view: DataView, offset: number, value: string) {
  for (let i = 0; i < value.length; i++) {
    view.setUint8(offset + i, value.charCodeAt(i));
  }
}

export function buildSilentWavBlobUrl(durationMs: number): string {
  const sampleRate = 8000;
  const numSamples = Math.ceil((durationMs / 1000) * sampleRate);
  const dataSize = numSamples * 2; // 16-bit mono
  const buffer = new ArrayBuffer(44 + dataSize);
  const view = new DataView(buffer);

  writeAsciiString(view, 0, "RIFF");
  view.setUint32(4, 36 + dataSize, true);
  writeAsciiString(view, 8, "WAVE");
  writeAsciiString(view, 12, "fmt ");
  view.setUint32(16, 16, true); // fmt chunk size
  view.setUint16(20, 1, true); // PCM
  view.setUint16(22, 1, true); // mono
  view.setUint32(24, sampleRate, true);
  view.setUint32(28, sampleRate * 2, true); // byte rate
  view.setUint16(32, 2, true); // block align
  view.setUint16(34, 16, true); // bits per sample
  writeAsciiString(view, 36, "data");
  view.setUint32(40, dataSize, true);
  // Remaining bytes default to 0 (silence).

  return URL.createObjectURL(new Blob([buffer], { type: "audio/wav" }));
}

// A fixed rotation of Azure viseme ids covering most mouth-shape segments
// (see azureVisemeMapping.ts), spread evenly across the clip so mock
// playback visibly cycles the mouth even though there's no real phoneme
// timing behind it.
const MOCK_VISEME_CYCLE = [21, 4, 18, 6, 10, 14, 1, 20];

export function buildMockVisemeTimelineBlobUrl(durationMs: number): string {
  const events: { timestamp_ms: number; viseme_id: number }[] = [
    { timestamp_ms: 0, viseme_id: 0 },
  ];
  const steps = MOCK_VISEME_CYCLE.length;
  MOCK_VISEME_CYCLE.forEach((visemeId, i) => {
    events.push({
      timestamp_ms: Math.round(((i + 1) / (steps + 1)) * durationMs),
      viseme_id: visemeId,
    });
  });
  events.push({ timestamp_ms: durationMs, viseme_id: 0 });

  return URL.createObjectURL(
    new Blob([JSON.stringify(events)], { type: "application/json" }),
  );
}

export function estimateSpeechDurationMs(text: string): number {
  const wordCount = text.trim().split(/\s+/).filter(Boolean).length || 1;
  return Math.min(4000, Math.max(1200, wordCount * 350));
}
