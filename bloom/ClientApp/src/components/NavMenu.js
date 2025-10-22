import {
  Navbar,
  NavbarBrand,
  Nav,
  NavItem,
  NavLink,
  Container,
} from "reactstrap";
import { Link } from "react-router-dom";
import "./NavMenu.css";

export default function NavMenu() {

  return (
    <header>
      <Navbar className="navbar-expand-sm navbar-toggleable-sm ng-white border-bottom box-shadow mb-3" light><Container>
        <Nav> 
          <NavbarBrand tag={Link} to="/">
            bloom
          </NavbarBrand>         
          <NavItem>
            <NavLink tag={Link} to="/" className="text-dark">Home</NavLink>
          </NavItem>
          <NavItem>
            <NavLink tag={Link} to="/counter" className="text-dark">Counter</NavLink>
          </NavItem>
          <NavItem>
            <NavLink tag={Link} to="/fetch-data" className="text-dark">Fetch data</NavLink>
          </NavItem>
          <NavItem>
            <NavLink tag={Link} to="/sign-in" className="text-dark">Sign in</NavLink>
          </NavItem>
          </Nav>
        </Container>
      </Navbar>
    </header>
  );
}
