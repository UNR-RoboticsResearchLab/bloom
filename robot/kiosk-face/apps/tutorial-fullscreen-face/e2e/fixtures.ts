export type VisemeEvent = { timestamp_ms: number; viseme_id: number };

export const AUDIO_DURATION_MS = 1200;

// Same example values used in robot/face_display/test_face.py and
// test_viseme_timing.py, just tightened to fit a short fixture clip:
// silence -> "sh"(21) -> "e"(4) -> "f"(18) -> silence.
export const VISEME_TIMELINE: VisemeEvent[] = [
  { timestamp_ms: 0, viseme_id: 0 },
  { timestamp_ms: 250, viseme_id: 21 },
  { timestamp_ms: 500, viseme_id: 4 },
  { timestamp_ms: 800, viseme_id: 18 },
  { timestamp_ms: 1100, viseme_id: 0 },
];

// Builds a minimal, silent PCM WAV file in memory so the test doesn't need a
// checked-in audio binary. Real playback (HTMLAudioElement) still exercises
// the actual timing/ended-event path the app relies on.
export function buildSilentWav(durationMs: number): Buffer {
  const sampleRate = 8000;
  const numSamples = Math.ceil((durationMs / 1000) * sampleRate);
  const dataSize = numSamples * 2; // 16-bit mono
  const buffer = Buffer.alloc(44 + dataSize);

  buffer.write("RIFF", 0, "ascii");
  buffer.writeUInt32LE(36 + dataSize, 4);
  buffer.write("WAVE", 8, "ascii");
  buffer.write("fmt ", 12, "ascii");
  buffer.writeUInt32LE(16, 16); // fmt chunk size
  buffer.writeUInt16LE(1, 20); // PCM
  buffer.writeUInt16LE(1, 22); // mono
  buffer.writeUInt32LE(sampleRate, 24);
  buffer.writeUInt32LE(sampleRate * 2, 28); // byte rate
  buffer.writeUInt16LE(2, 32); // block align
  buffer.writeUInt16LE(16, 34); // bits per sample
  buffer.write("data", 36, "ascii");
  buffer.writeUInt32LE(dataSize, 40);
  // Remaining bytes default to 0 (silence).

  return buffer;
}
