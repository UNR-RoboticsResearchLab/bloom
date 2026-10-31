import React, { createContext, useContext } from "react";
import ApiClient from "../interfaces/ApiClient";

const ApiClientContext = createContext(null);

export const ApiClientProvider = ({ children }) => {
  // REACT_APP_API_BASE_URL is baked into the bundle at build/start time and
  // served identically to every browser, so a hardcoded value (e.g.
  // "http://localhost:8080") only works from the machine running Docker --
  // any other machine's "localhost" resolves to itself, not the backend.
  // Falling back to the page's own hostname (same one the browser used to
  // reach this app) lets it work from any LAN/remote client automatically,
  // matching the backend's CORS policy which already allows any LAN origin.
  // REACT_APP_API_PORT changes only the port (8080 by default), for a dev stack whose API is published elsewhere.
  // A production build is served by the backend itself (usually behind a TLS proxy), so it calls
  // the page's own origin ("" = same origin, <site>/api/...). An explicitly empty
  // REACT_APP_API_BASE_URL means the same in any build.
  const apiPort = process.env.REACT_APP_API_PORT || "8080";
  const apiBase =
    process.env.REACT_APP_API_BASE_URL ??
    (process.env.NODE_ENV === "production"
      ? ""
      : `${window.location.protocol}//${window.location.hostname}:${apiPort}`);
  const client = new ApiClient(apiBase);

  return (
    <ApiClientContext.Provider value={client}>
      {children}
    </ApiClientContext.Provider>
  );
};

export const useApiClient = () => {
  const context = useContext(ApiClientContext);
  if (!context) {
    throw new Error("useApiClient must be used within an ApiClientProvider");
  }
  return context;
};
