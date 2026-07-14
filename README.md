# Bloom

Senior Projects 25-26 - Team 24

Bloom is a speech-therapy platform for children built around a physical robot companion. It has three parts:

- **Web app** — an ASP.NET Core 9 backend + React frontend used by SLPs (speech-language pathologists) and students to manage accounts, lessons, and live robot sessions.
- **Robot driver** — a ROS 2 (Jazzy) stack that runs on the physical robot: conversation (Azure STT/LLM/TTS), the animated face, and motion (Blossom-style motor rig).
- **AutoRSR** — a speech-recognition microservice the backend talks to over HTTP (see [Configuration](#configuration)).

## Repo Structure

```
bloom/
├── backend/               ASP.NET Core 9 Web API (also serves the built React app)
│   ├── Controllers/       REST endpoints (accounts, lessons, robot sessions, RSR, ...)
│   ├── Services/          Business logic (lesson orchestration, robot, RSR, ...)
│   ├── Repositories/      Data access
│   ├── Models/             EF Core entities + DTOs
│   ├── Migrations/        EF Core migrations
│   └── appsettings*.json  Connection strings, JWT config, service URLs
├── frontend/              React app (Create React App), served by the backend in production
│   └── src/
├── robot/                 On-device ROS 2 workspace for the physical robot (not containerized)
│   └── src/
│       ├── bloom_node/    Core C++ ROS 2 driver: lesson orchestration, state, behavior arbitration
│       ├── bloom_msgs/    Shared action/service interfaces (PlayBehavior, PlayKeyframes, ListBehaviors)
│       ├── bloom_face/    Face rendering node (pygame)
│       ├── bloom_speech/  STT/LLM/TTS ROS 2 nodes (Azure)
│       └── blsm_unr/      Git submodule — motor control for the Blossom-style robot body
├── tests/                 xUnit test project for the backend (see tests/README.md)
├── docker-compose.yml         Dev stack (hot-reload backend + frontend, dev DBs)
├── docker-compose.prod.yml    Production stack (backend + DB; frontend is built into the backend image)
├── Dockerfile / Dockerfile.dev   Backend (+ frontend build) images
├── build.sh / deploy.sh       Non-Docker production build + systemd deployment
├── startdocker-dev.sh / startdocker.sh   Docker convenience wrappers
└── test.sh                    Runs backend + frontend tests inside Docker (local or CI)
```

The `robot/` ROS 2 workspace is developed and run independently of the web app (it targets the robot's onboard computer, not a container) — see [robot/README.md](robot/README.md) for robot-specific setup. `robot/src/blsm_unr` is a **git submodule**; after cloning, run:

```bash
git submodule update --init --recursive
```

## Prerequisites

| Component | Requirement |
|---|---|
| Web app (Docker path) | Docker + Docker Compose |
| Web app (non-Docker path) | Node.js, .NET 9.0 SDK, MySQL/MariaDB |
| Robot driver | ROS 2 Jazzy, colcon (see [robot/README.md](robot/README.md)) |

## Development Setup

### With Docker (recommended)

```bash
git clone <this repo>
cd bloom
./startdocker-dev.sh
```

This builds and starts, via `docker-compose.yml`:

| Service | Container | Port(s) |
|---|---|---|
| React (hot-reload) | `bloom-react-dev` | `3000` |
| ASP.NET Core backend | `bloom-server-dev` | `8080` (http), `2443` |
| MariaDB (bloom DB) | `mariadb-dev` | `3306` |
| MariaDB (ArSr DB) | `arsr-db-dev` | `3307` |
| AutoRSR speech service | `auto_rsr` | `5050` |

- App: `http://localhost:3000` (React dev server, proxies API calls to the backend)
- API: `http://localhost:8080`
- To also apply EF Core migrations on startup: `./startdocker-dev.sh --migration`
- To tail logs: `./startdocker-dev.sh --logs {frontend|backend|both}`
- `./startdocker.sh` is a lighter-weight variant that just brings the compose stack up (`--build` to rebuild images, `--migration` to run migrations against `mariadb-dev`) without tailing logs.
- Frontend source under `frontend/src` and `frontend/public` is volume-mounted into `bloom-react-dev`, so edits hot-reload without a rebuild. Backend code changes require a rebuild (`docker compose up -d bloom-server-dev --build`), since `Dockerfile.dev` does a full `dotnet build` at image build time rather than mounting source.

The `auto_rsr` service builds from `./AutoRSR`, which is **not part of this repository** — clone/place it alongside before running the full stack, or comment that service out of `docker-compose.yml` if you don't need RSR locally. It also requests an NVIDIA GPU (`deploy.resources.reservations.devices`); drop that block if you don't have one.

### Without Docker

1. Install Node.js, a MySQL/MariaDB server, and the .NET 9.0 SDK.
2. `git clone` this repo, then `git submodule update --init --recursive`.
3. Create a database and point `backend/appsettings.json` → `ConnectionStrings:DefaultConnection` at it (or override via `backend/appsettings.Development.json` / environment variables — ASP.NET Core config precedence applies).
4. Frontend: `cd frontend && npm install`. Run `npm start` for a standalone dev server, or `npm run build` to produce the static bundle the backend serves in production.
5. Backend: `cd backend && dotnet run` (add `dotnet ef database update` first if the schema isn't up to date).

## Configuration

Backend configuration lives in `backend/appsettings.json` (base) and `backend/appsettings.Development.json` (dev overrides), following standard ASP.NET Core config layering — any key can also be supplied via environment variable (e.g. `Jwt__Key`, `ConnectionStrings__DefaultConnection`). Notable keys:

- `ConnectionStrings:DefaultConnection` / `ProductionConnection` — main Bloom DB
- `ConnectionStrings:DevArsrConnection` / `ProdArsrConnection` — ArSr DB
- `Jwt:Key` / `Issuer` / `Audience` / `ExpireMinutes` — auth token settings (change `Jwt:Key` before any real deployment; the checked-in value is a placeholder)
- `AutoRsr:BaseUrl` — where the backend reaches the AutoRSR microservice

The robot driver has its own credentials, kept out of version control: copy `robot/.env.example` to `robot/.env` and fill in Azure OpenAI / Azure Speech keys. See [robot/README.md](robot/README.md).

## API Documentation

Swagger UI is available once the backend is running: `https://localhost:5000/swagger/index.html` in production, or `http://localhost:8080/swagger/index.html` against the dev containers.

## Testing

```bash
./test.sh          # backend (dotnet test) + frontend (npm test) inside Docker, dev compose
./test.sh --prod   # same, against the prod compose file/images
```

`test.sh` starts the relevant DB, builds the app images, runs both test suites, and tears the stack down afterward (`-v`) — this is also what's intended for CI (Jenkins).

Without Docker:
```bash
dotnet test tests/bloom.Tests.csproj    # backend unit/integration tests — see tests/README.md
cd frontend && npm test                 # frontend tests
```

## Production

### Docker

```bash
docker compose -f docker-compose.prod.yml up -d --build
```

`Dockerfile` is a multi-stage build: it builds the React app, publishes the .NET backend, and copies the React build output into the final image so the backend serves both API and static frontend from one container/port. `docker-compose.prod.yml` brings that image up alongside `mariadb-prod` and `arsr-service`. Backend is published on host port `5000` (mapped from container `8080`).

Before deploying for real: change `Jwt:Key` and the MariaDB passwords away from the checked-in defaults (currently plain values in `appsettings.json` / `docker-compose.prod.yml`).

### Without Docker (systemd)

`build.sh` and `deploy.sh` support deploying directly onto a Linux host running Bloom as a systemd service (`bloom.service`), instead of Docker:

```bash
./build.sh --migration --backup     # dotnet publish + npm run build (+ optional DB backup/migration)
sudo -u www-data ./deploy.sh --prod --migration
```

- `build.sh` builds the React frontend (`npm run build`), publishes the backend (`dotnet publish -c Release`) to `/var/www/bloom-build/net9.0/publish`, and optionally backs up the DB (`--backup`) or runs EF Core migrations (`--migration`) first. `--skip-build` reuses a previous build.
- `deploy.sh` **must run as `www-data`**. It stops `bloom.service`, replaces the contents of the target directory (`/var/www/bloom` with `--prod`, otherwise `/var/www/bloom-dev`) with the fresh publish + frontend build output, optionally runs migrations (`--migration`), and restarts `bloom.service`. `--restart-only` just restarts the existing deployment without redeploying files.
- Logs: `sudo journalctl -u bloom.service -f`.

This path assumes `bloom.service` and the target directories already exist on the host (out of scope of this repo — provisioned separately).

## Robot

The physical robot runs a separate ROS 2 workspace under `robot/`, independent of the Docker/web-app stack above. See [robot/README.md](robot/README.md) for build instructions, and note that `robot/src/blsm_unr` is a git submodule that needs `git submodule update --init --recursive` before it will build.