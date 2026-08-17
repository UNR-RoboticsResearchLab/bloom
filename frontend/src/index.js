import "bootstrap/dist/css/bootstrap.min.css";
import React from "react";
import { createRoot } from "react-dom/client";
import { BrowserRouter } from "react-router-dom";
import { ThemeProvider } from "@material-tailwind/react";

import App from "./App";
import * as serviceWorkerRegistration from "./serviceWorkerRegistration";
import reportWebVitals from "./reportWebVitals";

import './index.css';
import "./custom.css";
import { ApiClientProvider } from "./context/ApiClientContext";
import { RobotPairingProvider } from "./context/RobotPairingContext";

const baseUrl = document.getElementsByTagName("base")[0]?.getAttribute("href") || "/";
const container = document.getElementById("root");
const root = createRoot(container);

root.render(
  <ThemeProvider>
    <BrowserRouter basename={baseUrl}>
      <ApiClientProvider>
        <RobotPairingProvider>
          <App />
        </RobotPairingProvider>
      </ApiClientProvider>
    </BrowserRouter>
  </ThemeProvider>
);

// keep default CRA behavior
serviceWorkerRegistration.unregister();
reportWebVitals();
