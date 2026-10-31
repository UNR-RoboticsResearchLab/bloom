import React from "react";
import { render } from "@testing-library/react";

import { ApiClientProvider, useApiClient } from "./ApiClientContext";

function BaseUrl() {
  return <span data-testid="base">{useApiClient().baseUrl}</span>;
}

describe("ApiClientProvider API address", () => {
  const saved = { url: process.env.REACT_APP_API_BASE_URL, port: process.env.REACT_APP_API_PORT };
  beforeEach(() => {
    delete process.env.REACT_APP_API_BASE_URL;
    delete process.env.REACT_APP_API_PORT;
  });
  afterAll(() => {
    process.env.REACT_APP_API_BASE_URL = saved.url;
    process.env.REACT_APP_API_PORT = saved.port;
    if (saved.url === undefined) delete process.env.REACT_APP_API_BASE_URL;
    if (saved.port === undefined) delete process.env.REACT_APP_API_PORT;
  });

  const baseFor = () => render(<ApiClientProvider><BaseUrl /></ApiClientProvider>).getByTestId("base").textContent;

  it("uses the page's own host on port 8080 by default", () => {
    expect(baseFor()).toBe("http://localhost:8080");
  });

  it("uses REACT_APP_API_PORT for the port, keeping the page's host", () => {
    process.env.REACT_APP_API_PORT = "18080";
    expect(baseFor()).toBe("http://localhost:18080");
  });

  it("lets an explicit REACT_APP_API_BASE_URL win", () => {
    process.env.REACT_APP_API_PORT = "18080";
    process.env.REACT_APP_API_BASE_URL = "https://api.example.edu";
    expect(baseFor()).toBe("https://api.example.edu");
  });
});
