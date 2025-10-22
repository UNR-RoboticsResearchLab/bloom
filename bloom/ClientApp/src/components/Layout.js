import { Container } from 'reactstrap';
import NavMenu from './NavMenu';

export default function Layout({ children }) {
  return (
    <div>
      <NavMenu />
      <Container className="mt-3">
        {children}
      </Container>
    </div>
  );
}
