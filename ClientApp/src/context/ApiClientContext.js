import React, { createContext, useContext } from "react";
import ApiClient from "../interfaces/ApiClient";

const ApiClientContext = createContext(null);

export const ApiClientProvider = ({ children }) => {
  const apiBase = process.env.REACT_APP_API_BASE_URL || "";
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
