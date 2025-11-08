// src/App.js
import { Routes, Route } from "react-router-dom";

import Layout from "./components/Layout";
import SignIn from "./components/SignIn";
import SignUp from "./components/SignUp";
import { RequireAuth, RequireRole } from "./components/RouteGuards";

import Home from "./pages/Home";
import FetchData from "./pages/FetchData";
import Counter from "./pages/Counter";
import ForgotPassword from "./pages/ForgotPassword";

import AdminDashboard from "./components/dashboard/AdminDashboard";
import TeacherDashboard from "./components/dashboard/TeacherDashboard";
import StudentDashboard from "./components/dashboard/StudentDashboard";
import SlpDashboard from "./components/dashboard/SlpDashboard";

import "./custom.css";

export default function App() {

  const apiBase = process.env.REACT_APP_API_BASE_URL || "http://bloom-server-dev:5000/";


  return (
    <Layout>
      <Routes>
        {/* Public */}
        <Route path="/" element={<Home />} />
        <Route path="/counter" element={<Counter />} />
        <Route path="/fetch-data" element={<FetchData />} />
        <Route path="/forgot-password" element={<ForgotPassword />} />
        <Route path="/sign-in" element={<SignIn />} />
        <Route path="/sign-up" element={<SignUp />} />

        {/* Protected */}
        <Route element={<RequireAuth />}>
          <Route path="/dashboard">
            <Route element={<RequireRole allow={["admin"]} />}>
              <Route path="admin" element={<AdminDashboard />} />
            </Route>
            <Route element={<RequireRole allow={["teacher"]} />}>
              <Route path="teacher" element={<TeacherDashboard />} />
            </Route>
            <Route element={<RequireRole allow={["student"]} />}>
              <Route path="student" element={<StudentDashboard />} />
            </Route>
            <Route element={<RequireRole allow={["slp"]} />}>
              <Route path="slp" element={<SlpDashboard />} />
            </Route>
          </Route>
        </Route>

        {/* Fallback */}
        <Route path="*" element={<div className="p-3">Not found</div>} />
      </Routes>
    </Layout>
  );
}
