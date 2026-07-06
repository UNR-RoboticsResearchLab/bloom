import { test, expect } from "@playwright/test";
import { buildSilentWav, VISEME_TIMELINE, AUDIO_DURATION_MS } from "./fixtures";
import { mapAzureViseme } from "../src/azureVisemeMapping";

// Drives the real kiosk-face app in a real browser end to end (poll -> fetch
// sentence -> play audio -> run the viseme timeline -> acknowledge), but
// intercepts the bloom backend calls at the network boundary so no .NET
// backend, database, Azure TTS, or physical robot needs to be running. See
// robot/kiosk-face/README.md.
//
// The rig's live morph-target weights live inside @vizij/render's internal
// animatable store, keyed by an opaque id assigned at GLB-import time (not
// the human-readable "rig/<face>/visemes/<segment>.weight" path) — there's
// no supported way to read "is viseme X currently the active mouth shape"
// from outside without depending on that internal id scheme. Instead this
// asserts on the boundary this app actually owns: the right sentence gets
// fetched, the audio plays for its real duration (not aborted early by an
// error), every viseme id in the timeline maps to a known segment (no
// "Unmapped Azure viseme id" warning), and the command is acknowledged only
// after playback actually finished.
test("kiosk face polls, plays, and lip-syncs a queued RSR sentence without a live backend", async ({
  page,
}) => {
  const robotId = "test-robot";
  const wav = buildSilentWav(AUDIO_DURATION_MS);
  const sentenceText = "The mock sentence used for e2e testing.";
  let pendingServed = false;
  let acknowledgedAt: number | null = null;

  // apiBaseUrl (localhost:9999) is a different origin than the app
  // (localhost:5173), so every intercepted response needs a CORS header or
  // the browser's fetch() calls get blocked client-side even though the
  // request itself never leaves Playwright's interception layer.
  const corsHeaders = { "Access-Control-Allow-Origin": "*" };

  // useRsrPolling fetches this once to look up sentence text for the
  // subtitle bar (pending commands only carry a sentenceId).
  await page.route("**/api/rsr-speech/sentences", async (route) => {
    await route.fulfill({
      status: 200,
      contentType: "application/json",
      headers: corsHeaders,
      body: JSON.stringify([
        {
          id: 1,
          text: sentenceText,
          audioUrl: "/fixtures/sentence.wav",
          visemeUrl: "/fixtures/sentence.visemes.json",
        },
      ]),
    });
  });

  await page.route(`**/api/rsr-speech/${robotId}/pending`, async (route) => {
    if (route.request().method() === "DELETE") {
      acknowledgedAt = Date.now();
      await route.fulfill({
        status: 200,
        contentType: "application/json",
        headers: corsHeaders,
        body: JSON.stringify({ message: "acknowledged" }),
      });
      return;
    }

    if (pendingServed) {
      await route.fulfill({ status: 204, headers: corsHeaders, body: "" });
      return;
    }
    pendingServed = true;
    await route.fulfill({
      status: 200,
      contentType: "application/json",
      headers: corsHeaders,
      body: JSON.stringify({
        commandId: "cmd-1",
        sentenceId: 1,
        audioUrl: "/fixtures/sentence.wav",
        visemeUrl: "/fixtures/sentence.visemes.json",
      }),
    });
  });

  let audioRequestedAt: number | null = null;
  await page.route("**/fixtures/sentence.wav", async (route) => {
    audioRequestedAt = Date.now();
    await route.fulfill({
      status: 200,
      contentType: "audio/wav",
      headers: corsHeaders,
      body: wav,
    });
  });

  let visemeTimelineRequested = false;
  await page.route("**/fixtures/sentence.visemes.json", async (route) => {
    visemeTimelineRequested = true;
    await route.fulfill({
      status: 200,
      contentType: "application/json",
      headers: corsHeaders,
      body: JSON.stringify(VISEME_TIMELINE),
    });
  });

  const unmappedVisemeWarnings: string[] = [];
  const playbackErrors: string[] = [];
  page.on("console", (msg) => {
    const text = msg.text();
    if (text.includes("Unmapped Azure viseme id")) unmappedVisemeWarnings.push(text);
    if (text.includes("[face] Failed to play sentence audio")) playbackErrors.push(text);
    if (text.includes("[face] Failed to load viseme timeline")) playbackErrors.push(text);
    if (text.includes("[face] RSR speech poll failed")) playbackErrors.push(text);
  });

  // Chromium blocks new Audio(url).play() with no user gesture by default;
  // the launch args in playwright.config.ts disable that for this suite, but
  // this addInitScript instruments the Audio element itself so the test can
  // observe the real play()/ended timing without any app source changes.
  await page.addInitScript(() => {
    const NativeAudio = window.Audio;
    const events: { type: string; t: number }[] = [];
    (window as unknown as { __audioEvents: typeof events }).__audioEvents = events;
    window.Audio = class extends NativeAudio {
      constructor(src?: string) {
        super(src);
        events.push({ type: "created", t: Date.now() });
        this.addEventListener("play", () => events.push({ type: "play", t: Date.now() }));
        this.addEventListener("ended", () => events.push({ type: "ended", t: Date.now() }));
        this.addEventListener("error", () => events.push({ type: "error", t: Date.now() }));
      }
    };
  });

  await page.goto(`/?robotId=${robotId}`);
  await page.waitForSelector(".canvas-wrapper", { timeout: 45_000 });

  // Sentence + viseme timeline should get fetched, and the ack should follow
  // only once the audio actually finishes playing (not fail instantly).
  await expect.poll(() => visemeTimelineRequested, { timeout: 10_000 }).toBe(true);

  // The subtitle bar should show the sentence's text while it's playing...
  await expect(page.locator(".subtitle")).toHaveText(sentenceText, { timeout: 5_000 });

  await expect.poll(() => acknowledgedAt, { timeout: 10_000 }).not.toBeNull();

  // ...and clear once playback finishes.
  await expect(page.locator(".subtitle")).toHaveCount(0);

  expect(audioRequestedAt).not.toBeNull();
  const playbackMs = acknowledgedAt! - audioRequestedAt!;
  // Real playback of the fixture clip takes ~AUDIO_DURATION_MS; an
  // immediately-rejected/errored play() would acknowledge in a fraction of
  // that, so this distinguishes "played the file" from "gave up instantly".
  expect(playbackMs).toBeGreaterThan(AUDIO_DURATION_MS * 0.7);

  const audioEvents = await page.evaluate(
    () => (window as unknown as { __audioEvents: { type: string }[] }).__audioEvents,
  );
  expect(audioEvents.map((e) => e.type)).toContain("play");
  expect(audioEvents.map((e) => e.type)).toContain("ended");
  expect(audioEvents.map((e) => e.type)).not.toContain("error");

  // Every viseme id used by the fixture timeline must resolve to a known
  // mouth-shape segment (sanity check reusing the app's own mapping table,
  // and a guard against silent regressions in azureVisemeMapping.ts).
  for (const event of VISEME_TIMELINE) {
    const { segment, isSilence } = mapAzureViseme(event.viseme_id);
    expect(segment !== null || isSilence).toBe(true);
  }
  expect(unmappedVisemeWarnings).toEqual([]);
  expect(playbackErrors).toEqual([]);
});
