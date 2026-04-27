// src/App.js
import { Routes, Route } from "react-router-dom";

import Layout  from "./components/Layout";
import SignIn from "./pages/SignIn";
import SignUp from "./pages/SignUp";
import { RequireAuth, RequireRole } from "./components/RouteGuards";
import About from "./pages/About";
import Home from "./pages/Home";

import AdminDashboard from "./components/dashboard/AdminDashboard";
import TeacherDashboard from "./components/dashboard/TeacherDashboard";
import StudentDashboard from "./components/dashboard/StudentDashboard";
import SlpDashboard from "./components/dashboard/SlpDashboard";

import Lessons from "./pages/Lessons";
import Lesson from "./pages/Lesson";
import LessonHistory from "./pages/LessonHistory";

import Students from "./pages/Students";
import Student from "./pages/Student";

import ForgotPassword from "./pages/ForgotPassword";

import FetchData from "./pages/FetchData";
import Counter from "./pages/Counter";

// import "./custom.css";

import AddStudentCard from "./pages/AddStudentCard";
import AddLessonCard from "./pages/AddLessonCard";

import LessonView from "./pages/LessonView";
import EditAccount from "./pages/EditAccount";

export default function App() {

  const apiBase = process.env.REACT_APP_API_BASE_URL ?? "http://localhost:5000";


  return (
    <Layout>
      <Routes>
        {/* Public */}
        <Route path="/" element={<Home />} />
        <Route path="/counter" element={<Counter />} />
        <Route path="/fetch-data" element={<FetchData />} />
        {/* <Route path="/forgot-password" element={<ForgotPassword />} /> */}
        <Route path="/sign-in" element={<SignIn />} />
        <Route path="/sign-up" element={<SignUp />} />
        <Route path="/about" element={<About />} />
        <Route path="/lessons" element={<Lessons />} />
        <Route path="/students" element={<Students />} />
        <Route path="/student/:studentId" element={<Student />} />
        <Route path="/lesson/:lessonId" element={<Lesson />} />
        <Route path="/lesson-view" element={<LessonView />} />
        <Route path="/lesson-history/:sessionId" element={<LessonHistory />} />
        
        <Route path="/add-student" element={<AddStudentCard />} />

        <Route path="/admin" element={<AdminDashboard />} />
        <Route path="/teacher" element={<TeacherDashboard />} />
        <Route path="/student" element={<StudentDashboard />} />
        <Route path="/slp" element={<SlpDashboard />} />

        {/* Protected */}
        <Route element={<RequireAuth />}>
          <Route path="/account" element={<EditAccount />} />
          <Route path="/dashboard">
            <Route element={<RequireRole allow={["admin"]} />}>
              <Route path="admin" element={<AdminDashboard />} />
              <Route path="admin/add-lesson" element={<AddLessonCard />} />
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
