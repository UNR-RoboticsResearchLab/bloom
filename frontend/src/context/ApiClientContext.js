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
  const apiBase =
    process.env.REACT_APP_API_BASE_URL ||
    `${window.location.protocol}//${window.location.hostname}:8080`;
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
