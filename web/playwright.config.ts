import { defineConfig, devices } from "@playwright/test";

const backend = "E:\\anaconda\\python.exe -m uvicorn uav_search.server.app:app --host 127.0.0.1 --port 8000";
const frontend = "npm run dev -- --port 5173";

export default defineConfig({
  testDir: "./e2e",
  timeout: 60_000,
  use: {
    baseURL: "http://127.0.0.1:5173",
    trace: "retain-on-failure",
  },
  webServer: [
    {
      command: backend,
      cwd: "..",
      url: "http://127.0.0.1:8000/api/health",
      reuseExistingServer: true,
      timeout: 90_000,
    },
    {
      command: frontend,
      cwd: ".",
      url: "http://127.0.0.1:5173",
      reuseExistingServer: true,
      timeout: 90_000,
    },
  ],
  projects: [
    {
      name: "chromium",
      use: { ...devices["Desktop Chrome"] },
    },
  ],
});
