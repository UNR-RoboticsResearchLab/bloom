// remove BrowserRouter import
import { Routes, Route } from 'react-router-dom';
import Layout from './components/Layout';
import Home from './components/Home';
import FetchData from './components/FetchData';
import Counter from './components/Counter';
import SignIn from './components/SignIn';
import SignUp from './components/SignUp';
import './custom.css';

export default function App() {
  return (
    <Layout>
      <Routes>
        <Route path="/" element={<Home />} />
        <Route path="/counter" element={<Counter />} />
        <Route path="/fetch-data" element={<FetchData />} />
        <Route path="/sign-in" element={<SignIn />} />
        <Route path="/sign-up" element={<SignUp />} />
        {/* <Route path="/forgot-password" element={<ForgotPassword />} />
        <Route path="/dashboard" element={<Dashboard />} /> */}
        <Route path="*" element={<div className="p-3">Not found</div>} />
      </Routes>
    </Layout>
  );
}
