import { useState } from "react";
import {
  Navbar,
  NavbarBrand,
  Nav,
  NavItem,
  NavLink,
  Container,
} from "reactstrap";
import { Link } from "react-router-dom";
import { getSession, dashboardPathForRole } from "../utils/auth";
import { useRobotPairing } from "../context/RobotPairingContext";
import { PairRobotCard } from "../pages/PairRobotCard";
import "../styles/components/nav-menu.css";

function getInitials(name) {
  if (!name) return "";
  return name
    .split(" ")
    .filter(Boolean)
    .map((word) => word[0])
    .join("")
    .toUpperCase();
}

export default function NavMenu() {
  const user = getSession();
  const isLoggedIn = !!user;
  const displayName = user?.fullName || user?.name || user?.userName || user?.email;
  const initials = getInitials(displayName);
  const homePath = isLoggedIn
    ? dashboardPathForRole(user.role?.toLowerCase())
    : "/";

  const role = user?.role?.toLowerCase();
  const canPairRobot = isLoggedIn && role !== "student";

  const { isPaired, robotCode, unpair } = useRobotPairing();
  const [showPairRobotCard, setShowPairRobotCard] = useState(false);
  const [isUnpairing, setIsUnpairing] = useState(false);

  async function handleQuickUnpair(e) {
    e.stopPropagation();
    setIsUnpairing(true);
    try {
      await unpair();
    } finally {
      setIsUnpairing(false);
    }
  }

  return (
    <header>
      <Navbar className="navbar-expand-sm border-bottom box-shadow mb-3" light>
        <Container className="d-flex justify-content-between align-items-center">
          <NavbarBrand tag={Link} to="/" className="text-dark fw-bold">
            bloom
          </NavbarBrand>

          <Nav className="d-flex flex-row gap-3 align-items-center">
            <NavItem>
              <NavLink tag={Link} to={homePath} className="text-dark">
                Home
              </NavLink>
            </NavItem>

            <NavItem>
              <NavLink tag={Link} to="/about" className="text-dark">
                About
              </NavLink>
            </NavItem>

            {isLoggedIn && (<NavItem>
              <NavLink tag={Link} to="/browse-lessons" className="text-dark">
                Lessons
              </NavLink>
            </NavItem>
          )}

            {isLoggedIn && (<NavItem>
              <NavLink tag={Link} to="/rsr-assessment" className="text-dark">
                RSR
              </NavLink>
            </NavItem>
          )}

            {canPairRobot && (
              <NavItem>
                <div
                  className={`inline-flex items-center rounded-full border text-sm font-medium shadow-sm transition ${
                    isPaired
                      ? "border-green-300 bg-green-50 text-green-700 py-1 pl-3 pr-1"
                      : "border-gray-300 bg-white text-gray-700 px-3 py-1.5 hover:bg-gray-50"
                  }`}
                >
                  <button
                    type="button"
                    onClick={() => setShowPairRobotCard(true)}
                    className={`inline-flex items-center gap-2 border-0 bg-transparent p-0 ${
                      isPaired ? "hover:opacity-80" : ""
                    }`}
                  >
                    <span
                      className={`h-2 w-2 rounded-full ${isPaired ? "bg-green-500" : "bg-gray-400"}`}
                    />
                    {isPaired ? `Robot · ${robotCode}` : "Pair Robot"}
                  </button>
                  {isPaired && (
                    <button
                      type="button"
                      onClick={handleQuickUnpair}
                      disabled={isUnpairing}
                      title="Unpair robot"
                      aria-label="Unpair robot"
                      className="ml-1 rounded-full border-0 bg-transparent p-1 text-green-600 hover:bg-green-100 hover:text-green-800 disabled:opacity-50"
                    >
                      ×
                    </button>
                  )}
                </div>
              </NavItem>
            )}

            {!isLoggedIn ? (
              <NavItem>
                <NavLink tag={Link} to="/sign-in" className="text-dark">
                  Sign in
                </NavLink>
              </NavItem>
            ) : (
              <NavItem>
                <div className="inline-flex h-10 w-10 items-center justify-center rounded-full border border-gray-900 bg-white">
                  <span className="text-xl font-semibold text-gray-900">
                    {initials}
                  </span>
                </div>
              </NavItem>
            )}
          </Nav>
        </Container>
      </Navbar>

      {showPairRobotCard && (
        <div className="fixed inset-0 z-50 flex items-center justify-center">
          <div
            className="absolute inset-0 bg-black/40"
            onClick={() => setShowPairRobotCard(false)}
          />
          <div className="relative z-10 w-full max-w-xl px-4">
            <PairRobotCard
              onCancel={() => setShowPairRobotCard(false)}
              onPaired={() => setShowPairRobotCard(false)}
              onUnpaired={() => setShowPairRobotCard(false)}
            />
          </div>
        </div>
      )}
    </header>
  );
}