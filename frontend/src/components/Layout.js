import { Container } from 'reactstrap';
import NavMenu from './NavMenu';
import { useLocation } from 'react-router-dom';

export default function Layout({ children }) {
  const { pathname } = useLocation();
  if ((pathname.startsWith('/rsr-study/child/') || pathname === '/rsr-study/login')) return <>{children}</>;
  return (
    <div>
      <NavMenu />
      <Container className="mt-3">
        {children}
      </Container>
    </div>
  );
}
