import { defineConfig, devices } from "@playwright/test";

const PORT = 5173;
const baseURL = `http://localhost:${PORT}`;

// Runs the real dev server + a real (headless) browser, but the specs
// intercept the app's HTTP calls at the network boundary (page.route()) so
// no bloom backend, database, or physical robot needs to be running.
// See e2e/rsr-face.spec.ts and robot/kiosk-face/README.md.
export default defineConfig({
  testDir: "./e2e",
  // Cold dev-server starts (WASM init, GLB parse, ~300+ rig inputs staged)
  // can take well over 20s before the canvas is ready; give it room.
  timeout: 60_000,
  fullyParallel: false,
  workers: 1,
  retries: 0,
  use: {
    baseURL,
    trace: "retain-on-failure",
  },
  webServer: {
    command: "pnpm run dev -- --port " + PORT + " --strictPort",
    url: baseURL,
    reuseExistingServer: !process.env.CI,
    env: {
      VITE_BLOOM_API_URL: "http://localhost:9999",
    },
    timeout: 30_000,
  },
  projects: [
    {
      name: "chromium",
      use: {
        ...devices["Desktop Chrome"],
        launchOptions: {
          args: [
            "--use-gl=swiftshader",
            "--enable-webgl",
            "--ignore-gpu-blocklist",
            // The app calls new Audio(url).play() with no user gesture;
            // Chromium blocks that by default outside a real user session.
            "--autoplay-policy=no-user-gesture-required",
          ],
        },
      },
    },
  ],
});
