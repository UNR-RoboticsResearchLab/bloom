import {
  Navbar,
  NavbarBrand,
  Nav,
  NavItem,
  NavLink,
  Container,
} from "reactstrap";
import { Link } from "react-router-dom";
import { getSession } from "../utils/auth";
import "./NavMenu.css";

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
  const initials = getInitials(user?.name);

  return (
    <header>
      <Navbar className="navbar-expand-sm border-bottom box-shadow mb-3" light>
        <Container className="d-flex justify-content-between align-items-center">
          <NavbarBrand tag={Link} to="/" className="text-dark fw-bold">
            bloom
          </NavbarBrand>

          <Nav className="d-flex flex-row gap-3 align-items-center">
            <NavItem>
              <NavLink tag={Link} to="/" className="text-dark">
                Home
              </NavLink>
            </NavItem>

            <NavItem>
              <NavLink tag={Link} to="/about" className="text-dark">
                About
              </NavLink>
            </NavItem>

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
    </header>
  );
}