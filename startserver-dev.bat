@echo off
REM Start script for Bloom development environment (Windows)
REM Usage: startserver.bat [--migration]

SETLOCAL ENABLEDELAYEDEXPANSION

SET COMPOSE_FILE=docker-compose.dev.yml
SET SERVER_CONTAINER=bloom-server-dev
SET REACT_CONTAINER=bloom-react-dev
SET DB_SERVICE=mariadb-dev
SET RUN_MIGRATIONS=false

REM --- Parse arguments ---
IF "%1"=="--migration" SET RUN_MIGRATIONS=true

ECHO Starting Bloom development environment...

REM --- Start database first ---
ECHO Starting database service: %DB_SERVICE%...
docker compose -f %COMPOSE_FILE% up -d %DB_SERVICE%

REM --- Wait for database to be ready ---
ECHO Waiting for database (%DB_SERVICE%:3306) to be ready...
:WAIT_DB
docker-compose -f %COMPOSE_FILE% logs %DB_SERVICE% | findstr /I "ready for connections" >nul
IF ERRORLEVEL 1 (
    TIMEOUT /T 2 >nul
    GOTO WAIT_DB
)
ECHO Database is ready.

REM --- Start React container ---
ECHO Starting react container: %REACT_CONTAINER%...
docker compose -f %COMPOSE_FILE% up -d %REACT_CONTAINER% --build

REM --- Start server container ---
ECHO Starting server container: %SERVER_CONTAINER%...
docker compose -f %COMPOSE_FILE% up -d %SERVER_CONTAINER% --build

REM --- Optionally run migrations ---
IF "%RUN_MIGRATIONS%"=="true" (
    ECHO Running EF Core migrations inside %SERVER_CONTAINER%...
    docker exec %SERVER_CONTAINER% dotnet ef database update
    IF ERRORLEVEL 1 (
        ECHO Migration failed or EF tools not installed.
    )
)

REM --- Tail server logs ---
ECHO Attaching to server logs (Ctrl+C to detach)...
docker logs -f %SERVER_CONTAINER%
