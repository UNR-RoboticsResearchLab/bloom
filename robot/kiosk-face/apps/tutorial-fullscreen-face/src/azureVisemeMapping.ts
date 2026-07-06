// Maps Azure Cognitive Services Speech viseme IDs (0-21, see Microsoft's
// published "viseme ID and phonemes" table) to this rig's (hugo_rigged.glb)
// viseme pose segments. The segment vocabulary and several of the phoneme
// groupings mirror apps/tutorial-agent-face/src/visemeMapping.ts, which maps
// the same rig from Amazon Polly's viseme codes instead.
//
// This mapping is a best-effort phonetic approximation — Azure groups
// several IPA phonemes per viseme ID, and the rig only exposes 15 mouth
// shapes, so some IDs collapse onto the same segment. It should be spot
// checked visually against the rig once real synthesized audio is available.
const FACE_VISEME_SEGMENTS = [
  "pose_pzzzfnvy",
  "p",
  "t",
  "t_2",
  "s",
  "f",
  "k",
  "i",
  "r",
  "u",
  "a",
  "e",
  "e_2",
  "o",
  "o_2",
] as const;

export type FaceVisemeSegment = (typeof FACE_VISEME_SEGMENTS)[number];

export type ResolvedFaceViseme = {
  segment: FaceVisemeSegment | null;
  isSilence: boolean;
  sourceId: number;
};

type AzureMapping = {
  segment: FaceVisemeSegment | null;
  isSilence?: boolean;
};

// id -> example IPA phonemes (per Microsoft's Azure TTS viseme table) -> segment
const AZURE_VISEME_ID_TO_SEGMENT: Record<number, AzureMapping> = {
  0: { segment: null, isSilence: true }, // silence
  1: { segment: "a" }, // æ, ə, ʌ
  2: { segment: "a" }, // ɑ
  3: { segment: "o_2" }, // ɔ
  4: { segment: "e" }, // ɛ, ʊ
  5: { segment: "r" }, // ɝ
  6: { segment: "i" }, // j, i, ɪ
  7: { segment: "u" }, // w, u
  8: { segment: "o" }, // o
  9: { segment: "a" }, // aʊ
  10: { segment: "o_2" }, // ɔɪ
  11: { segment: "a" }, // aɪ
  12: { segment: "pose_pzzzfnvy" }, // h
  13: { segment: "r" }, // ɹ
  14: { segment: "r" }, // l
  15: { segment: "s" }, // s, z
  16: { segment: "s" }, // ʃ, tʃ, dʒ, ʒ
  17: { segment: "t_2" }, // ð
  18: { segment: "f" }, // f, v
  19: { segment: "t" }, // d, t, n, θ
  20: { segment: "k" }, // k, g, ŋ
  21: { segment: "p" }, // p, b, m
};

const reportedUnknown = new Set<number>();

export function mapAzureViseme(visemeId: number): ResolvedFaceViseme {
  const mapping = AZURE_VISEME_ID_TO_SEGMENT[visemeId];
  if (!mapping) {
    if (!reportedUnknown.has(visemeId)) {
      console.warn(`[face] Unmapped Azure viseme id: ${visemeId}. Falling back to neutral.`);
      reportedUnknown.add(visemeId);
    }
    return { segment: null, isSilence: false, sourceId: visemeId };
  }
  return {
    segment: mapping.segment,
    isSilence: mapping.isSilence ?? false,
    sourceId: visemeId,
  };
}

export { FACE_VISEME_SEGMENTS };
